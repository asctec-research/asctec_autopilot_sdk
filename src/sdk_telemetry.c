/*
 * Copyright (C) 2016 Ascending Technologies GmbH, Germany
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
#include "sdk_telemetry.h"
#include "util/gpsmath.h"
#include "hal/system.h"
#include <string.h>
#include <stdio.h>

#include "hal/jeti_telemetry.h"
#include "ll_hl_comm.h"
#include "sdk.h"
#include "sdkio.h"
#include "util/fastmath.h"

void SDK_jetiAscTecExampleUpdateDisplay(unsigned char state)
{
  char text[33]; // Line	  11111111111111112222222222222222
  switch(state)
  {
    case 0:
      if(sdk.ro.flightMode & FLIGHTMODE_FLYING)
        jetiSetTextDisplay("<-Waypoint Test  EmergencyMode->");
      else
        jetiSetTextDisplay("AscTec JetiTest  EmergencyMode->");
      break;
    case 1:
      if(sdk.ro.emergencyMode == EM_SAVE)
        jetiSetTextDisplay("Current EmMode<>Direct Landing");
      else if(sdk.ro.emergencyMode == EM_SAVE_EXTENDED_WAITING_TIME)
        jetiSetTextDisplay("Current EmMode<>Wait&Land");
      else if(sdk.ro.emergencyMode == EM_RETURN_AT_MISSION_SUMMIT)
        jetiSetTextDisplay("Current EmMode<>Come Home High");
      else
        jetiSetTextDisplay("Current EmMode<>Come Home ");
      break;
    case 2:
      jetiSetTextDisplay("EmMode v=Set  <>Direct Landing");
      break;
    case 3:
      jetiSetTextDisplay("EmMode v=Set  <>Wait&Land");
      break;
    case 4:
      jetiSetTextDisplay("EmMode v=Set  <>Come Home     ");
      break;
    case 5:
      jetiSetTextDisplay("EmMode v=Set  <>Come Home High");
      break;
    case 6:
      snprintf(text, 33, "WP Act. v=Stop  WP# %1i, Dist: %2im", wpExample.wpNr, sdk.ro.waypoint.distanceToWp / 1000);
      jetiSetTextDisplay(text);
      break;
  }
}

void SDK_jetiAscTecExampleKeyChange(unsigned char key)
{
  static unsigned char displayState = 0;

  switch(displayState)
  {
    case 0:
      switch(key)
      {
        case JETI_KEY_UP:
          break;
        case JETI_KEY_DOWN:
          break;
        case JETI_KEY_LEFT:
          if(sdk.ro.flightMode & FLIGHTMODE_FLYING)
          {
            wpExample.startEvent = 1;
            displayState = 6;
          }
          break;
        case JETI_KEY_RIGHT:
          displayState++;
          break;
      }
      break;

    case 1:
      switch(key)
      {
        case JETI_KEY_UP:
          break;
        case JETI_KEY_DOWN:
          break;
        case JETI_KEY_LEFT:
          displayState = 0;
          break;
        case JETI_KEY_RIGHT:
          displayState++;
          break;
      }
      break;

    case 2:
      switch(key)
      {
        case JETI_KEY_UP:
          break;
        case JETI_KEY_DOWN:
          SDKSetEmergencyMode(EM_SAVE);
          displayState = 0;
          break;
        case JETI_KEY_LEFT:
          displayState = 5;
          break;
        case JETI_KEY_RIGHT:
          displayState++;
          break;
      }
      break;

    case 3:
      switch(key)
      {
        case JETI_KEY_UP:
          break;
        case JETI_KEY_DOWN:
          SDKSetEmergencyMode(EM_SAVE_EXTENDED_WAITING_TIME);
          displayState = 0;
          break;
        case JETI_KEY_LEFT:
          displayState--;
          break;
        case JETI_KEY_RIGHT:
          displayState++;
          break;
      }
      break;
    case 4:
      switch(key)
      {
        case JETI_KEY_UP:
          break;
        case JETI_KEY_DOWN:
          SDKSetEmergencyMode(EM_RETURN_AT_PREDEFINED_HEIGHT);
          displayState = 0;
          break;
        case JETI_KEY_LEFT:
          displayState--;
          break;
        case JETI_KEY_RIGHT:
          displayState++;
          break;
      }
      break;

    case 5:
      switch(key)
      {
        case JETI_KEY_UP:
          break;
        case JETI_KEY_DOWN:
          SDKSetEmergencyMode(EM_RETURN_AT_MISSION_SUMMIT);
          displayState = 0;
          break;
        case JETI_KEY_LEFT:
          displayState--;
          break;
        case JETI_KEY_RIGHT:
          displayState = 2;
          break;
      }
      break;

    case 6:
      //switch back when waypoint example is finished
      if(wpExample.state == 0)
        displayState = 0;

      switch(key)
      {
        case JETI_KEY_UP:
          break;
        case JETI_KEY_DOWN:
          displayState = 0;
          wpExample.abortEvent = 1;
          break;
        case JETI_KEY_LEFT:
          break;
        case JETI_KEY_RIGHT:
          break;
      }
      break;
  }

  SDK_jetiAscTecExampleUpdateDisplay(displayState);
}

void SDK_jetiAscTecExampleInit(void)
{
  jetiSetDeviceName("AscTec SDK");
  jetiSetTextDisplay("AscTec Example  ->more Infos");
  jetiInitValue(0, "Pitch", "°");
  jetiSetDecimalPoint(0, 2);
  jetiInitValue(1, "Roll", "°");
  jetiSetDecimalPoint(1, 2);
  jetiInitValue(2, "Yaw", "°");
  jetiSetDecimalPoint(2, 2);
  jetiInitValue(3, "Height", "m");
  jetiSetDecimalPoint(3, 2);
  jetiInitValue(4, "FlightMode", "");
  jetiInitValue(5, "Speed", "m/s");
  jetiSetDecimalPoint(5, 1);
  jetiInitValue(6, "GPS", "%");
  jetiInitValue(7, "BAT", "V");
  jetiSetDecimalPoint(7, 2);
  jetiInitValue(8, "CPU Time", "us");
  jetiInitValue(9, "Lat", "");
  jetiInitValue(10, "Lon", "");
  jetiInitValue(11, "LatRem", "");
  jetiInitValue(12, "LonRem", "");
  jetiSetValue14B(0, 0);
  jetiSetValue14B(1, 0);
  jetiSetValue22B(2, 0);
  jetiSetValue22B(3, 0);
  jetiSetValue6B(4, 0);
  jetiSetValue14B(5, 0);
  jetiSetValue14B(6, 0);
  jetiSetValue22B(7, 0);
  jetiSetValue14B(8, 0);
  jetiSetValue14B(9, 0);
  jetiSetValue14B(10, 0);
  jetiSetValue22B(11, 0);
  jetiSetValue22B(12, 0);

  SDK_jetiAscTecExampleUpdateDisplay(0);
}

void SDK_jetiAscTecExampleRun(void)
{
  int speed;
  int gps_quality;
  unsigned char key;
  static unsigned char first = 0;
  static float voltageFilter = 12.6f;
  //counter for updating the jeti display regularly
  static unsigned char jetiDisplayUpdateCnt = 0;

  if(!first)
  {
    first = 1;
    SDK_jetiAscTecExampleInit();
  }

  jetiSetValue14B(0, sdk.ro.attitude.roll/10);
  jetiSetValue14B(1, sdk.ro.attitude.pitch/10);
  jetiSetValue22B(2, sdk.ro.attitude.yaw/10);
  jetiSetValue22B(3, sdk.ro.height / 10);

  jetiSetValue14B(9, gps.data.latitude / 10000000);
  jetiSetValue14B(10, gps.data.longitude / 10000000);
  jetiSetValue22B(11, (gps.data.latitude / 10) % 1000000);
  jetiSetValue22B(12, (gps.data.longitude / 10) % 1000000);

  if((sdk.ro.flightMode & FLIGHTMODE_POS) && gps.data.hasLock)
    jetiSetValue6B(4, 2);
  else if(sdk.ro.flightMode & FLIGHTMODE_HEIGHT)
    jetiSetValue6B(4, 1);
  else
    jetiSetValue6B(4, 0);

  speed = fast_sqrt(gps.data.speedEastWest * gps.data.speedEastWest
      + gps.data.speedNorthSouth * gps.data.speedNorthSouth);
  speed /= 100;
  jetiSetValue14B(5, speed);

  if(gps.data.hasLock == 0 || (gps.data.numSatellites < 3) || (gps.data.horizontalAccuracy > 10000))
    gps_quality = 0;
  else
    gps_quality = 100 - ((gps.data.horizontalAccuracy - 1500) / 85);
  if(gps_quality > 100)
    gps_quality = 100;

  jetiSetValue14B(6, gps_quality);

  if(sdk.ro.sensors.battery > 5000)
  {
    if(voltageFilter == 12.6f)
      voltageFilter = (float)sdk.ro.sensors.battery / 1000.0f;

    voltageFilter = (9995.0f * voltageFilter + 5.0f * ((float)sdk.ro.sensors.battery / 1000.0f))
        / 10000.0f;
  }

  jetiSetValue22B(7, (int)(voltageFilter * 100.0f));
  if((voltageFilter < 10.5f) && (voltageFilter > 5.0f))
    jetiSetAlarm('U', 0);
  else
    jetiSetAlarm(0, 0);

  jetiSetValue14B(8, HL_Status.cpu_load);

  key = jetiCheckForKeyChange();
  jetiDisplayUpdateCnt++;
  if((jetiDisplayUpdateCnt == 20) || (key))
  {
    SDK_jetiAscTecExampleKeyChange(key);
    jetiDisplayUpdateCnt = 0;
  }
}
