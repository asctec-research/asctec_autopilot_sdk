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

#include "main.h"
#include "util/gpsmath.h"
#include "sdk_telemetry.h"
#include "hal/system.h"
#include <stdint.h>
#include <stdio.h>
#include "sdkio.h"

#include "hal/jeti_telemetry.h"
#include "hal/uart1.h"
#include "ll_hl_comm.h"

SDKData sdk;

#define LL_STATUS_FLIGHT_MODE_MASK         0x07
#define LL_STATUS_SERIAL_INTERFACE_ENABLED 0x20
#define LL_STATUS_SERIAL_INTERFACE_ACTIVE  0x40 //is active when control commands are sent to the LL
#define LL_STATUS_EMERGENCY_MODE           0x80 //when RC link is lost -> serial interface disabled

#define LL_ERROR_COMPASS                 0x0010
#define LL_ERROR_CALIBRATION             0x0100
#define LL_ERROR_GYRO_CALIBRATION        0x0200
#define LL_ERROR_ACC_CALIBRATION         0x0400
#define LL_ERROR_MAGNETIC_FIELD_STRENGTH 0x4000
#define LL_ERROR_MAGNETIC_INCLINATION    0x8000
#define LL_ERROR_MASK                    0xC710

//set waypoint properties with WPPROP_* defines (wpToLL.properties)
#define WPPROP_ABSCOORDS     0x01//if set waypoint is interpreted as absolute coordinates, else relative coordinates
#define WPPROP_HEIGHTENABLED 0x02//set new height at waypoint
#define WPPROP_YAWENABLED    0x04//set new yaw-angle at waypoint
#define WPPROP_AUTOMATICGOTO 0x10//if set, vehicle will not wait for a goto command, but goto this waypoint directly

#define WP_CMD_SINGLE_WP  0x01 //fly to single waypoint
#define WP_CMD_LAUNCH     0x02 //launch to 10m at current position
#define WP_CMD_LAND       0x03 //land at current position
#define WP_CMD_GOHOME     0x04 //come home at current height. Home position is set when motors are started (valid GPS signal mandatory!) or with WP_CMD_SETHOME
#define WP_CMD_SETHOME    0x05 //save current vehicle position as home position
#define WP_CMD_ABORT      0x06 //abort navigation (stops current waypoint flying)

void SDKParseLLData(struct LL_ATTITUDE_DATA* pLL)
{
  unsigned char current_page = pLL->system_flags & 0x03;

  sdk.ro.attitude.angle[0] = pLL->angle_roll*10;
  sdk.ro.attitude.angle[1] = pLL->angle_pitch*10;
  sdk.ro.attitude.angle[2] = pLL->angle_yaw*10;

  sdk.ro.attitude.angularVelocity[0] = pLL->angvel_roll*15;
  sdk.ro.attitude.angularVelocity[1] = pLL->angvel_pitch*15;
  sdk.ro.attitude.angularVelocity[2] = pLL->angvel_yaw*15;

  switch(current_page)
  {
    case 0:
    {
      for(uint8_t i = 0; i < 8; i++)
      {
        sdk.ro.rc.channels[i] = pLL->RC_data[i] * 16;
      }

      sdk.ro.rc.pitch = sdk.ro.rc.channels[0];
      sdk.ro.rc.roll = sdk.ro.rc.channels[1];
      sdk.ro.rc.thrust = sdk.ro.rc.channels[2];
      sdk.ro.rc.yaw = sdk.ro.rc.channels[3];
      sdk.ro.rc.serialSwitch = sdk.ro.rc.channels[4];
      sdk.ro.rc.flightMode = sdk.ro.rc.channels[5];
      sdk.ro.rc.aux = sdk.ro.rc.channels[6];

      sdk.ro.sensors.acc[0] = (pLL->acc_x*981)/100;
      sdk.ro.sensors.acc[1] = (pLL->acc_y*981)/100;
      sdk.ro.sensors.acc[2] = (pLL->acc_z*981)/100;

      sdk.ro.gps.latitude = pLL->latitude_best_estimate;
      sdk.ro.gps.longitude = pLL->longitude_best_estimate;
    }
    break;
    case 1:
    {
      sdk.ro.height = pLL->height;
      sdk.ro.verticalSpeed = pLL->dheight;

      sdk.ro.gps.speedEastWest = pLL->speed_x_best_estimate;
      sdk.ro.gps.speedNorthSouth = pLL->speed_y_best_estimate;

      for(uint8_t i = 0; i < 6; i++)
      {
        sdk.ro.motors.speed[i] = ((int32_t)pLL->motor_data[i + 8])*64;
        sdk.ro.motors.pwm[i] = ((int32_t)pLL->motor_data[i]);
      }
    }
    break;
    case 2:
    {
      sdk.ro.sensors.mag[0] = pLL->mag_x;
      sdk.ro.sensors.mag[1] = pLL->mag_y;
      sdk.ro.sensors.mag[2] = pLL->mag_z;

      // parse flight mode status bits
      sdk.ro.lowLevelError = pLL->flightMode & LL_ERROR_MASK;

      if(pLL->flightMode & LL_STATUS_SERIAL_INTERFACE_ENABLED)
        sdk.ro.serialInterfaceReady = 1;
      else
        sdk.ro.serialInterfaceReady = 0;

      if(pLL->flightMode & LL_STATUS_SERIAL_INTERFACE_ACTIVE)
        sdk.ro.serialInterfaceActive = 1;
      else
        sdk.ro.serialInterfaceActive = 0;

      #define LL_STATUS_FLIGHT_MODE_ATTITUDE_CONTROL   0x01
      #define LL_STATUS_FLIGHT_MODE_HEIGHT_CONTROL     0x02
      #define LL_STATUS_FLIGHT_MODE_POSITION_CONTROL   0x04

      int32_t fMode = 0;
      if(pLL->flightMode & LL_STATUS_FLIGHT_MODE_ATTITUDE_CONTROL)
        fMode |= FLIGHTMODE_ACC;
      if(pLL->flightMode & LL_STATUS_FLIGHT_MODE_HEIGHT_CONTROL)
        fMode |= FLIGHTMODE_HEIGHT;
      if(pLL->flightMode & LL_STATUS_FLIGHT_MODE_POSITION_CONTROL)
        fMode |= FLIGHTMODE_POS;

      if(pLL->flightMode & LL_STATUS_EMERGENCY_MODE)
      {
        sdk.ro.rc.hasLock = 0;
        fMode |= FLIGHTMODE_EMERGENCY;
      }
      else
      {
        sdk.ro.rc.hasLock = 1;
      }

      if(pLL->status2 & 0x01)
        fMode |= FLIGHTMODE_FLYING;

      sdk.ro.flightMode = fMode;

      sdk.ro.sensors.battery = pLL->battery_voltage1;
      sdk.ro.cpuLoad = HL_Status.cpu_load;

      uint8_t slowDataUpChannelSelect = (pLL->status2 >> 1) & 0x7F;
      switch(slowDataUpChannelSelect)
      {
        case SUDC_FLIGHTTIME:
          sdk.ro.flightTime = pLL->slowDataUpChannelDataShort;
          break;
        case SUDC_NAVSTATUS:
          sdk.ro.waypoint.navStatus = pLL->slowDataUpChannelDataShort;
          break;
        case SUDC_DISTTOWP:
          sdk.ro.waypoint.distanceToWp = ((int32_t)pLL->slowDataUpChannelDataShort)*100;
          break;
        case SUDC_WPACKTRIGGER:
          sdk.ro.waypoint.ackTrigger = pLL->slowDataUpChannelDataShort;
          break;
        case SUDC_SENDOMTYPE:
          sdk.ro.isHexcopter = pLL->slowDataUpChannelDataShort;
          break;
        case SUDC_EM_MODE:
          if(sdk.cmd.newEmergencyMode && pLL->slowDataUpChannelDataShort == sdk.cmd.newEmergencyMode)
            sdk.cmd.newEmergencyMode = 0;
          sdk.ro.emergencyMode = pLL->slowDataUpChannelDataShort;
          break;
      }
    }
    break;
    default:
      break;
  }
}

void SDKFillLLCommands(struct LL_CONTROL_INPUT* pCtrl)
{
  pCtrl->system_flags &= ~SF_SDK_DISABLE_MOTORONOFF_BY_STICK;

  switch(sdk.cmd.mode)
  {
    case SDK_CMD_MODE_DIMC:
    {
      pCtrl->system_flags |= SF_HL_CONTROL_ENABLED | SF_NEW_SDK;
      pCtrl->system_flags |= SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL;

      uint16_t revMask = 0;
      for(uint8_t i = 0; i < 6; i++)
      {
        int32_t rpm = sdk.cmd.dimc.rpm[i];

        if(rpm < 0)
        {
          rpm = -rpm;
          revMask |= (1 << i);
        }

        int16_t out = 1;
        if(sdk.ro.isHexcopter)
        {
          // Firefly
          out = (4*rpm-500)/175;
        }
        else
        {
          // Hummingbird or Pelican
          out = (8*rpm-8600)/301;
        }

        if(out < 1)
          out = 1;
        if(out > 200)
          out = 200;

        if(sdk.cmd.dimc.enable[i] == 0)
          out = 0;

        pCtrl->direct_motor_control[i] = out;
      }

      pCtrl->direct_motor_control[6] = 0;
      pCtrl->direct_motor_control[7] = 0;

      pCtrl->ctrl_flags = revMask << 8;
    }
    break;
    case SDK_CMD_MODE_RPY_THRUST:
    {
      pCtrl->system_flags |= SF_HL_CONTROL_ENABLED | SF_NEW_SDK;
      pCtrl->system_flags &= ~(SF_DIRECT_MOTOR_CONTROL | SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL | SF_WAYPOINT_MODE);

      pCtrl->ctrl_flags = 0x0F;
      pCtrl->pitch = (sdk.cmd.RPYThrust.pitchAngle*2048)/52000;
      pCtrl->roll = (sdk.cmd.RPYThrust.rollAngle*2048)/52000;
#if VEHICLE_TYPE == VEHICLE_TYPE_HUMMINGBIRD
      pCtrl->yaw = (sdk.cmd.RPYThrust.yawRate*2048)/300000;
#else
      pCtrl->yaw = (sdk.cmd.RPYThrust.yawRate*2048)/200000;
#endif
      pCtrl->thrust = sdk.cmd.RPYThrust.thrust;
    }
    break;
    case SDK_CMD_MODE_RPY_CLIMBRATE:
    {
      pCtrl->system_flags |= SF_HL_CONTROL_ENABLED | SF_NEW_SDK;
      pCtrl->system_flags &= ~(SF_DIRECT_MOTOR_CONTROL | SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL | SF_WAYPOINT_MODE);

      pCtrl->ctrl_flags = 0x1F;
      pCtrl->pitch = (sdk.cmd.RPYClimbRate.pitchAngle*2048)/52000;
      pCtrl->roll = (sdk.cmd.RPYClimbRate.rollAngle*2048)/52000;
#if VEHICLE_TYPE == VEHICLE_TYPE_HUMMINGBIRD
      pCtrl->yaw = (sdk.cmd.RPYClimbRate.yawRate*2048)/300000;
#else
      pCtrl->yaw = (sdk.cmd.RPYClimbRate.yawRate*2048)/200000;
#endif
      pCtrl->thrust = ((sdk.cmd.RPYClimbRate.climbRate+2000)*4096)/4000;
    }
    break;
    case SDK_CMD_MODE_LOCAL_VEL_WITH_GPS:
    {
      pCtrl->system_flags |= SF_HL_CONTROL_ENABLED | SF_NEW_SDK;
      pCtrl->system_flags &= ~(SF_DIRECT_MOTOR_CONTROL | SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL | SF_WAYPOINT_MODE);

      pCtrl->ctrl_flags = 0x3F;
      pCtrl->pitch = (sdk.cmd.localVelWithGPS.speedForward*2048)/3000;
      pCtrl->roll = (sdk.cmd.localVelWithGPS.speedRight*2048)/3000;
#if VEHICLE_TYPE == VEHICLE_TYPE_HUMMINGBIRD
      pCtrl->yaw = (sdk.cmd.localVelWithGPS.yawRate*2048)/300000;
#else
      pCtrl->yaw = (sdk.cmd.localVelWithGPS.yawRate*2048)/200000;
#endif
      pCtrl->thrust = ((sdk.cmd.localVelWithGPS.climbRate+2000)*4096)/4000;
    }
    break;
    case SDK_CMD_MODE_GPS_WAYPOINT_ABS:
    {
      pCtrl->system_flags |= SF_HL_CONTROL_ENABLED | SF_NEW_SDK;
      pCtrl->system_flags |= SF_WAYPOINT_MODE;

      if(sdk.cmd.wpAbsolute.updated == 1)
      {
        sdk.ro.waypoint.ackTrigger = 0;

        pCtrl->ctrl_flags &= 0x00FF;
        pCtrl->ctrl_flags |= WP_CMD_SINGLE_WP_PART1 << 8;
        pCtrl->pitch = sdk.cmd.wpAbsolute.longitude & 0xFFFF;
        pCtrl->roll = sdk.cmd.wpAbsolute.longitude >> 16;
        pCtrl->thrust = sdk.cmd.wpAbsolute.latitude & 0xFFFF;
        pCtrl->yaw = sdk.cmd.wpAbsolute.latitude >> 16;
        pCtrl->direct_motor_control[0] = sdk.cmd.wpAbsolute.height;
        pCtrl->direct_motor_control[1] = sdk.cmd.wpAbsolute.height >> 8;
        pCtrl->direct_motor_control[2] = sdk.cmd.wpAbsolute.height >> 16;
        pCtrl->direct_motor_control[3] = sdk.cmd.wpAbsolute.height >> 24;
        pCtrl->direct_motor_control[4] = sdk.cmd.wpAbsolute.heading;
        pCtrl->direct_motor_control[5] = sdk.cmd.wpAbsolute.heading >> 8;
        pCtrl->direct_motor_control[6] = sdk.cmd.wpAbsolute.heading >> 16;
        pCtrl->direct_motor_control[7] = sdk.cmd.wpAbsolute.heading >> 24;

        sdk.cmd.wpAbsolute.updated++;
      }
      else if(sdk.cmd.wpAbsolute.updated == 2)
      {
        uint16_t time = sdk.cmd.wpAbsolute.timeToStay/10;
        const uint8_t properties = WPPROP_ABSCOORDS | WPPROP_AUTOMATICGOTO | WPPROP_HEIGHTENABLED | WPPROP_YAWENABLED;

        pCtrl->ctrl_flags &= 0x00FF;
        pCtrl->ctrl_flags |= WP_CMD_SINGLE_WP_PART2 << 8;
        pCtrl->pitch = time;
        pCtrl->roll = 0;
        pCtrl->thrust = sdk.cmd.wpAbsolute.reachedTolerance;
        pCtrl->yaw = 0xAAAA + sdk.cmd.wpAbsolute.heading + sdk.cmd.wpAbsolute.height + time
            + sdk.cmd.wpAbsolute.longitude + sdk.cmd.wpAbsolute.latitude + sdk.cmd.wpAbsolute.maxSpeed
            + sdk.cmd.wpAbsolute.reachedTolerance + properties + 1;
        pCtrl->direct_motor_control[0] = 0;
        pCtrl->direct_motor_control[1] = sdk.cmd.wpAbsolute.maxSpeed;
        pCtrl->direct_motor_control[2] = properties;
        pCtrl->direct_motor_control[3] = 1;
        pCtrl->direct_motor_control[4] = 0;
        pCtrl->direct_motor_control[5] = 0;
        pCtrl->direct_motor_control[6] = 0;
        pCtrl->direct_motor_control[7] = 0;
        sdk.cmd.wpAbsolute.updated = 0;
        sdk.ro.waypoint.navStatus = 0;
      }
      else
      {
    	  pCtrl->ctrl_flags &= 0x00FF;
      }
    }
    break;
    case SDK_CMD_MODE_GPS_WAYPOINT_REL:
    {
      pCtrl->system_flags |= SF_HL_CONTROL_ENABLED | SF_NEW_SDK;
      pCtrl->system_flags |= SF_WAYPOINT_MODE;

      if(sdk.cmd.wpRelative.updated == 1)
      {
        sdk.ro.waypoint.ackTrigger = 0;

        pCtrl->ctrl_flags &= 0x00FF;
        pCtrl->ctrl_flags |= WP_CMD_SINGLE_WP_PART1 << 8;
        pCtrl->pitch = sdk.cmd.wpRelative.east & 0xFFFF;
        pCtrl->roll = sdk.cmd.wpRelative.east >> 16;
        pCtrl->thrust = sdk.cmd.wpRelative.north & 0xFFFF;
        pCtrl->yaw = sdk.cmd.wpRelative.north >> 16;
        pCtrl->direct_motor_control[0] = sdk.cmd.wpRelative.height;
        pCtrl->direct_motor_control[1] = sdk.cmd.wpRelative.height >> 8;
        pCtrl->direct_motor_control[2] = sdk.cmd.wpRelative.height >> 16;
        pCtrl->direct_motor_control[3] = sdk.cmd.wpRelative.height >> 24;
        pCtrl->direct_motor_control[4] = sdk.cmd.wpRelative.heading;
        pCtrl->direct_motor_control[5] = sdk.cmd.wpRelative.heading >> 8;
        pCtrl->direct_motor_control[6] = sdk.cmd.wpRelative.heading >> 16;
        pCtrl->direct_motor_control[7] = sdk.cmd.wpRelative.heading >> 24;

        sdk.cmd.wpRelative.updated++;
      }
      else if(sdk.cmd.wpRelative.updated == 2)
      {
        uint16_t time = sdk.cmd.wpRelative.timeToStay/10;
        const uint8_t properties = WPPROP_AUTOMATICGOTO | WPPROP_HEIGHTENABLED | WPPROP_YAWENABLED;

        pCtrl->ctrl_flags &= 0x00FF;
        pCtrl->ctrl_flags |= WP_CMD_SINGLE_WP_PART2 << 8;
        pCtrl->pitch = time;
        pCtrl->roll = 0;
        pCtrl->thrust = sdk.cmd.wpRelative.reachedTolerance;
        pCtrl->yaw = 0xAAAA + sdk.cmd.wpRelative.heading + sdk.cmd.wpRelative.height + time
            + sdk.cmd.wpRelative.east + sdk.cmd.wpRelative.north + sdk.cmd.wpRelative.maxSpeed
            + sdk.cmd.wpRelative.reachedTolerance + properties + 1;
        pCtrl->direct_motor_control[0] = 0;
        pCtrl->direct_motor_control[1] = sdk.cmd.wpRelative.maxSpeed;
        pCtrl->direct_motor_control[2] = properties;
        pCtrl->direct_motor_control[3] = 1;
        pCtrl->direct_motor_control[4] = 0;
        pCtrl->direct_motor_control[5] = 0;
        pCtrl->direct_motor_control[6] = 0;
        pCtrl->direct_motor_control[7] = 0;
        sdk.cmd.wpRelative.updated = 0;
        sdk.ro.waypoint.navStatus = 0;
      }
      else
      {
    	  pCtrl->ctrl_flags &= 0x00FF;
      }
    }
    break;
    default:
    {
      pCtrl->system_flags &= ~SF_SDK_DISABLE_MOTORONOFF_BY_STICK;
      pCtrl->system_flags &= ~(SF_HL_CONTROL_ENABLED | SF_NEW_SDK);
      pCtrl->system_flags &= ~(SF_DIRECT_MOTOR_CONTROL | SF_DIRECT_MOTOR_CONTROL_INDIVIDUAL | SF_WAYPOINT_MODE);
    }
    break;
  }
}

void SDKPrintROData()
{
  printf("\n-- ATTITUDE --\n");
  printf("Angles:   %8d %8d %8d mdeg\n", sdk.ro.attitude.angle[0], sdk.ro.attitude.angle[1], sdk.ro.attitude.angle[2]);
  printf("Ang. Vel: %8d %8d %8d mdeg/s\n", sdk.ro.attitude.angularVelocity[0], sdk.ro.attitude.angularVelocity[1], sdk.ro.attitude.angularVelocity[2]);

  printf("-- SENSORS --\n");
  printf("Acc: %8d %8d %8d mm/s^2\n", sdk.ro.sensors.acc[0], sdk.ro.sensors.acc[1], sdk.ro.sensors.acc[2]);
  printf("Mag: %8d %8d %8d\n", sdk.ro.sensors.mag[0], sdk.ro.sensors.mag[1], sdk.ro.sensors.mag[2]);
  printf("Height: %8d mm\nVert. Speed: %8d mm/s\n", sdk.ro.height, sdk.ro.verticalSpeed);
  printf("Bat: %8d mV\n", sdk.ro.sensors.battery);

  printf("-- RC --\n");
  printf("Lock: %hu\n", (uint16_t)sdk.ro.rc.hasLock);
  printf("Ch: %4hu, %4hu, %4hu, %4hu, %4hu, %4hu, %4hu, %4hu\n", sdk.ro.rc.channels[0], sdk.ro.rc.channels[1],
          sdk.ro.rc.channels[2], sdk.ro.rc.channels[3], sdk.ro.rc.channels[4], sdk.ro.rc.channels[5],
          sdk.ro.rc.channels[6], sdk.ro.rc.channels[7]);

  printf("-- GPS --\n");
  printf("Latitude:  %10d deg*10^7\nLongitude: %10d deg*10^7\n", sdk.ro.gps.latitude, sdk.ro.gps.longitude);
  printf("Speed. EW: %8d, NS: %8d mm/s\n", sdk.ro.gps.speedEastWest, sdk.ro.gps.speedNorthSouth);
  printf("Sats: %hu, Lock: %hu\n", sdk.ro.gps.raw.numSatellites, (uint16_t)sdk.ro.gps.raw.hasLock);

  printf("-- MOTORS --\n");
  printf("Speed: %6hd %6hd %6hd %6hd %6hd %6hd RPM\n", sdk.ro.motors.speed[0], sdk.ro.motors.speed[1],
      sdk.ro.motors.speed[2], sdk.ro.motors.speed[3], sdk.ro.motors.speed[4], sdk.ro.motors.speed[5]);
  printf("PWM:   %6hd %6hd %6hd %6hd %6hd %6hd\n", sdk.ro.motors.pwm[0], sdk.ro.motors.pwm[1],
      sdk.ro.motors.pwm[2], sdk.ro.motors.pwm[3], sdk.ro.motors.pwm[4], sdk.ro.motors.pwm[5]);

  printf("-- STATUS --\n");
  printf("Flight mode: %04X\n", sdk.ro.flightMode);
  printf("LL Error: 0x%04hX\n", sdk.ro.lowLevelError);
  printf("Serial: ready: %hu, active: %hu\n", (uint16_t)sdk.ro.serialInterfaceReady, (uint16_t)sdk.ro.serialInterfaceActive);
  printf("HL CPU load: %3hu.%1hu%%\n", sdk.ro.cpuLoad/10, sdk.ro.cpuLoad%10);

  printf("-- MISC --\n");
  printf("isHex: %hu\n", (uint16_t)sdk.ro.isHexcopter);
  printf("Flight time: %hu s\n", sdk.ro.flightTime);
  printf("EM Mode: %hu\n", (uint16_t)sdk.ro.emergencyMode);

  printf("-- WAYPOINTS --\n");
  printf("Status: 0x%02hX\n", (uint16_t)sdk.ro.waypoint.navStatus);
  printf("Distance to WP: %d\n", sdk.ro.waypoint.distanceToWp);
  printf("ACK trigger: %hu\n", (uint16_t)sdk.ro.waypoint.ackTrigger);
}

// Sets emergency mode on LowLevel processor. Select one of the EM_ defines as mode option.
// See EM_ defines for details
void SDKSetEmergencyMode(uint8_t mode)
{
  if((mode != EM_SAVE_EXTENDED_WAITING_TIME) && (mode != EM_SAVE) && (mode != EM_RETURN_AT_MISSION_SUMMIT)
      && (mode != EM_RETURN_AT_PREDEFINED_HEIGHT))
    return;

  sdk.cmd.newEmergencyMode = mode;
}
