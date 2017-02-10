/*
 * Copyright (C) 2017 Intel Deutschland GmbH, Germany
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "gps_waypoints.h"
#include "../sdkio.h"
#include "../sdk.h"
#include "../util/gpsmath.h"

WaypointExample wpExample;

/* This function demonstrates simple waypoint command generation. To use this command set you
 * strictly require GPS reception. The UAV must be flying and in GPS mode prior to the start of this example.
 * Then, there are two options to start this example:
 * - Set the AUX switch/dial to minimum on your RC, then turn it to maximum
 * - "Mx" menu on Jeti Telemetry equipped RCs: While the UAV is hovering in GPS mode,
 *   enter the menu and press the left arrow once. Information about the next waypoint is displayed.
 *
 * The UAV will fly a 15m by 15m square. A waypoint is calculated from the current position and
 * height and is transmitted to the low level processor. The waypoint status is monitored to switch to
 * the next waypoint after the current one is reached.
 *
 * Feedback for waypoint flight can be found here: sdk.ro.waypoint
 * navStatus defines are found in sdkio.h, see WP_NAVSTAT_*
 *
 * sdk.cmd.wpAbsolute.updated needs to be set after the wpAbsolute data structure has been
 * filled with new data/command. This field is reset by the internal HL logic automatically as soon
 * as the command is sent to LL.
 *
 * Important:
 * In the case of an emergency remember to disable the serial switch to regain manual control!
 */
void ExampleGPSWaypointControl()
{
//  static unsigned char wpExampleState = 0;
  static double originLat, originLon;

  static uint8_t auxState = 2;

  if(sdk.ro.rc.aux < 1600)
  {
    if(auxState == 1)
      wpExample.abortEvent = 1;

    auxState = 0;
  }

  if(sdk.ro.rc.aux > 2400)
  {
    if(auxState == 0)
      wpExample.startEvent = 1;

    auxState = 1;
  }

  // reset static variables if serial interface is off
  if(sdk.ro.serialInterfaceReady == 0 || wpExample.abortEvent)
  {
    auxState = 2;
    wpExample.state = 0;
    sdk.cmd.mode = SDK_CMD_MODE_OFF;
    wpExample.startEvent = 0;
    wpExample.abortEvent = 0;
  }


  switch(wpExample.state)
  {
    case 0:
    {
      if(wpExample.startEvent) // jeti Mx menu or aux channel can start example
      {
        wpExample.startEvent = 0;
        wpExample.state = 1;
      }
    }
    break;

    case 1:
    {
      double lat, lon;
      //calculate and send first waypoint and switch state
      sdk.cmd.mode = SDK_CMD_MODE_GPS_WAYPOINT_ABS;

      //fill waypoint structure
      sdk.cmd.wpAbsolute.maxSpeed = 100;
      sdk.cmd.wpAbsolute.reachedTolerance = 3000; // 3m accuracy
      sdk.cmd.wpAbsolute.timeToStay = 4000; // 4 seconds waiting time at each waypoint

      //use current height and yaw
      sdk.cmd.wpAbsolute.heading = sdk.ro.attitude.yaw; //use current yaw
      sdk.cmd.wpAbsolute.height = sdk.ro.height; //use current height

      originLat = (double)sdk.ro.gps.latitude / 10000000.0;
      originLon = (double)sdk.ro.gps.longitude / 10000000.0;

      //calculate a position 15m north of us
      xy2latlon(originLat, originLon, 0.0, 15.0, &lat, &lon);

      sdk.cmd.wpAbsolute.longitude = lon * 10000000;
      sdk.cmd.wpAbsolute.latitude = lat * 10000000;

      //send waypoint
      sdk.cmd.wpAbsolute.updated = 1;

      wpExample.wpNr = 0;
      wpExample.state = 2;
    }
    break;

    case 2:
    {
      //wait until cmd is processed and sent to LL processor
      if((sdk.cmd.wpAbsolute.updated == 0) && (sdk.ro.waypoint.ackTrigger))
      {
        //check if waypoint was reached and wait time is over
        if(sdk.ro.waypoint.navStatus & (WP_NAVSTAT_REACHED_POS_TIME))
        {
          //new waypoint
          double lat, lon;

          //fill waypoint structure
          sdk.cmd.wpAbsolute.maxSpeed = 100;
          sdk.cmd.wpAbsolute.reachedTolerance = 3000; // 3m accuracy
          sdk.cmd.wpAbsolute.timeToStay = 4000; // 4 seconds waiting time at each waypoint

          //use current height and yaw
          sdk.cmd.wpAbsolute.heading = sdk.ro.attitude.angle[2]; //use current yaw
          sdk.cmd.wpAbsolute.height = sdk.ro.height; //use current height

          //calculate a position 15m north and 15m east of origin
          xy2latlon(originLat, originLon, 15.0, 15.0, &lat, &lon);

          sdk.cmd.wpAbsolute.longitude = lon * 10000000;
          sdk.cmd.wpAbsolute.latitude = lat * 10000000;

          //send waypoint
          sdk.cmd.wpAbsolute.updated = 1;

          wpExample.wpNr++;
          wpExample.state = 3;
        }

        if(sdk.ro.waypoint.navStatus & WP_NAVSTAT_PILOT_ABORT)
        {
          wpExample.state = 0;
        }
      }
    }
    break;

    case 3:
    {
      //wait until cmd is processed and sent to LL processor
      if((sdk.cmd.wpAbsolute.updated == 0) && (sdk.ro.waypoint.ackTrigger))
      {
        //check if waypoint was reached and wait time is over
        if(sdk.ro.waypoint.navStatus & (WP_NAVSTAT_REACHED_POS_TIME))
        {
          //new waypoint
          double lat, lon;

          //fill waypoint structure
          sdk.cmd.wpAbsolute.maxSpeed = 100;
          sdk.cmd.wpAbsolute.reachedTolerance = 3000; // 3m accuracy
          sdk.cmd.wpAbsolute.timeToStay = 4000; // 4 seconds waiting time at each waypoint

          //use current height and yaw
          sdk.cmd.wpAbsolute.heading = sdk.ro.attitude.angle[2]; //use current yaw
          sdk.cmd.wpAbsolute.height = sdk.ro.height; //use current height

          //calculate a position 15m east of origin
          xy2latlon(originLat, originLon, 15.0, 0.0, &lat, &lon);

          sdk.cmd.wpAbsolute.longitude = lon * 10000000;
          sdk.cmd.wpAbsolute.latitude = lat * 10000000;

          //send waypoint
          sdk.cmd.wpAbsolute.updated = 1;

          wpExample.wpNr++;
          wpExample.state = 4;
        }

        if(sdk.ro.waypoint.navStatus & WP_NAVSTAT_PILOT_ABORT)
        {
          wpExample.state = 0;
        }
      }
    }
    break;

    case 4:
    {
      //wait until cmd is processed and sent to LL processor
      if((sdk.cmd.wpAbsolute.updated == 0) && (sdk.ro.waypoint.ackTrigger))
      {
        //check if waypoint was reached and wait time is over
        if(sdk.ro.waypoint.navStatus & (WP_NAVSTAT_REACHED_POS_TIME))
        {
          //fill waypoint structure
          sdk.cmd.wpAbsolute.maxSpeed = 100;
          sdk.cmd.wpAbsolute.reachedTolerance = 3000; // 3m accuracy
          sdk.cmd.wpAbsolute.timeToStay = 4000; // 4 seconds waiting time at each waypoint

          //use current height and yaw
          sdk.cmd.wpAbsolute.heading = sdk.ro.attitude.angle[2]; //use current yaw
          sdk.cmd.wpAbsolute.height = sdk.ro.height; //use current height

          //go to the start point
          sdk.cmd.wpAbsolute.longitude = originLon * 10000000;
          sdk.cmd.wpAbsolute.latitude = originLat * 10000000;

          //send waypoint
          sdk.cmd.wpAbsolute.updated = 1;

          wpExample.wpNr++;
          wpExample.state = 0;
        }

        if(sdk.ro.waypoint.navStatus & WP_NAVSTAT_PILOT_ABORT)
        {
          wpExample.state = 0;
        }
      }
    }
    break;

    default:
      wpExample.state = 0;
      break;
  }
}
