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
#include "hal/system.h"
#include "util/gpsmath.h"
#include "util/declination.h"
#include "sdk_telemetry.h"
#include "hal/ssp.h"
#include <string.h>

#include "hal/jeti_telemetry.h"
#include "ll_hl_comm.h"
#include "sdkio.h"
#include "util/build_info.h"

static struct LL_ATTITUDE_DATA LL_1khz_attitude_data;
static struct LL_CONTROL_INPUT LL_1khz_control_input;
static volatile unsigned char transmitBuildInfoTrigger = 0;

void SSP_data_distribution_HL(void)
{
  unsigned char current_page = LL_1khz_attitude_data.system_flags & 0x03;
  static unsigned char oldKey = 0;

  SDKParseLLData(&LL_1khz_attitude_data);

  if(LL_1khz_attitude_data.system_flags & SF_GPS_NEW)
    gps.newForLL = 0;

  if(!current_page) //page 0
  {
    //system is initialized as soon as values differ from 0
    if(LL_1khz_attitude_data.acc_z && (SYSTEM_initialized < 10))
      SYSTEM_initialized++;
  }
  else if(current_page == 1) //page 1
  {
  }
  else if(current_page == 2)
  {
    unsigned char slowDataUpChannelSelect = (LL_1khz_attitude_data.status2 >> 1) & 0x7F;
    switch(slowDataUpChannelSelect)
    {
      case SUDC_JETIKEYVAL:
        if(oldKey != LL_1khz_attitude_data.slowDataUpChannelDataShort)
          jetiSetKeyChanged(LL_1khz_attitude_data.slowDataUpChannelDataShort);
        oldKey = LL_1khz_attitude_data.slowDataUpChannelDataShort;
        break;
    }
  }
}



int HL2LL_write_cycle(void) //write data to low-level processor
{
  static char pageselect = 0;
  static unsigned char jetiValuePartialSyncPending = 0;
  static unsigned char * transmitPtr;
  static unsigned short transmitCnt = 0;

  if(!SSPDataSentToLL)
    return (0);

  if(SYSTEM_initialized && (!transmitBuildInfoTrigger))
    transmitBuildInfoTrigger = 1;

  //update 1kHz data
  LL_1khz_control_input.system_flags = 0 | pageselect;

  if(gps.newForLL)
    LL_1khz_control_input.system_flags |= SF_GPS_NEW;

  SDKFillLLCommands(&LL_1khz_control_input);

  if(pageselect == 0)
  {
    if(LL_1khz_control_input.system_flags & SF_GPS_NEW)
    {
      //fill struct with 500Hz data
      LL_1khz_control_input.latitude = gps.data.latitude;
      LL_1khz_control_input.longitude = gps.data.longitude;
      LL_1khz_control_input.height = gps.data.height;
      LL_1khz_control_input.speed_x = gps.data.speedEastWest;
      LL_1khz_control_input.speed_y = gps.data.speedNorthSouth;
      LL_1khz_control_input.heading = gps.data.heading;
      LL_1khz_control_input.status = gps.status;
    }
    else
    {
      //default is no command
      LL_1khz_control_input.status = 0;

      if(LL_1khz_control_input.system_flags & SF_GPS_NEW)
      {
        //fill struct with 500Hz data
        LL_1khz_control_input.latitude = gps.data.latitude;
        LL_1khz_control_input.longitude = gps.data.longitude;
        LL_1khz_control_input.height = gps.data.height;
        LL_1khz_control_input.speed_x = gps.data.speedEastWest;
        LL_1khz_control_input.speed_y = gps.data.speedNorthSouth;
        LL_1khz_control_input.heading = gps.data.heading;
        LL_1khz_control_input.status = gps.status;
      }
      else
      {
        static unsigned char jetiSyncState = 0;
        static unsigned char jetiSensorCnt = 0;
        static unsigned char jetiSensorValCnt = 0;
        static unsigned char jetiSensorValUpdateTimeout = 0;

        //default is no command
        LL_1khz_control_input.status = 0;

        //data fields are used for jeti commands
        if(jetiValuePartialSyncPending == 1)
        {
          jetiValuePartialSyncPending = 0;
          LL_1khz_control_input.status = PD_JETI_SETSENSOR2;

          LL_1khz_control_input.height = jetiValues[jetiSensorCnt].active;
          memcpy(&LL_1khz_control_input.latitude, &jetiValues[jetiSensorCnt].unit[0], 4);
          LL_1khz_control_input.speed_x = jetiValues[jetiSensorCnt].unit[4];
          LL_1khz_control_input.speed_y = jetiValues[jetiSensorCnt].varType;
          LL_1khz_control_input.longitude = jetiValues[jetiSensorCnt].value;
          jetiSensorCnt++;
          if(jetiSensorCnt == 15)
            jetiSensorCnt = 0;
          jetiSyncState++;
          jetiSensorValCnt = 0;
          jetiSensorValUpdateTimeout = 0;
        }
        else
        {
          switch(jetiSyncState)
          {
            case 0:
              //sync name
              LL_1khz_control_input.status = PD_JETI_SETNAME;
              memcpy(&LL_1khz_control_input.latitude, &jetiName[0], 4);
              memcpy(&LL_1khz_control_input.longitude, &jetiName[4], 4);
              memcpy(&LL_1khz_control_input.speed_x, &jetiName[8], 2);
              jetiSyncState++;
              break;
            case 1:
              //sync alarm
              LL_1khz_control_input.status = PD_JETI_SETALARM;
              LL_1khz_control_input.speed_x = jetiAlarm;
              LL_1khz_control_input.speed_y = jetiAlarmType;
              jetiSyncState++;
              break;
            case 2:
              //sync complete sensor fields
              if(jetiValues[jetiSensorCnt].active)
              {
                LL_1khz_control_input.status = PD_JETI_SETSENSOR;
                LL_1khz_control_input.height = jetiValues[jetiSensorCnt].active;
                memcpy(&LL_1khz_control_input.latitude, &jetiValues[jetiSensorCnt].name[0], 4);
                memcpy(&LL_1khz_control_input.longitude, &jetiValues[jetiSensorCnt].name[4], 4);
                memcpy(&LL_1khz_control_input.speed_x, &jetiValues[jetiSensorCnt].name[8], 2);
                LL_1khz_control_input.speed_y = jetiValues[jetiSensorCnt].decPoint;
                jetiValuePartialSyncPending = 1;
              }
              else
              {
                jetiSensorCnt++;
                if(jetiSensorCnt == 15)
                  jetiSensorCnt = 0;

                jetiSyncState++;
                jetiSensorValCnt = 0;
                jetiSensorValUpdateTimeout = 0;
              }
              break;
            case 3:
              //sync text
              LL_1khz_control_input.status = PD_JETI_SETTEXT;
              memcpy(&LL_1khz_control_input.latitude, &jetiDisplayText[0], 4);
              memcpy(&LL_1khz_control_input.longitude, &jetiDisplayText[4], 4);
              memcpy(&LL_1khz_control_input.height, &jetiDisplayText[8], 4);
              memcpy(&LL_1khz_control_input.speed_x, &jetiDisplayText[12], 2);
              memcpy(&LL_1khz_control_input.speed_y, &jetiDisplayText[14], 2);
              jetiSyncState++;
              break;
            case 4:
              //sync text2
              LL_1khz_control_input.status = PD_JETI_SETTEXT2;
              memcpy(&LL_1khz_control_input.latitude, &jetiDisplayText[16], 4);
              memcpy(&LL_1khz_control_input.longitude, &jetiDisplayText[20], 4);
              memcpy(&LL_1khz_control_input.height, &jetiDisplayText[24], 4);
              memcpy(&LL_1khz_control_input.speed_x, &jetiDisplayText[28], 2);
              memcpy(&LL_1khz_control_input.speed_y, &jetiDisplayText[30], 2);
              jetiSyncState++;
              break;
            case 5:
              //sync all sensor values for 75ms
              LL_1khz_control_input.status = PD_JETI_UPDATESDATA;
              LL_1khz_control_input.height = jetiSensorValCnt;
              LL_1khz_control_input.latitude = jetiValues[3 * jetiSensorValCnt].value;
              LL_1khz_control_input.longitude = jetiValues[3 * jetiSensorValCnt + 1].value;
              memcpy(&LL_1khz_control_input.speed_y,
                  (void *)(((unsigned char *)(&jetiValues[3 * jetiSensorValCnt + 2].value)) + 2), 2);
              memcpy(&LL_1khz_control_input.speed_x, &jetiValues[3 * jetiSensorValCnt + 2].value, 2);

              jetiSensorValCnt++;
              if(jetiSensorValCnt == 5)
                jetiSensorValCnt = 0;

              if(jetiTriggerTextSync)
              {
                jetiSyncState = 3; //trigger text resync
                jetiTriggerTextSync = 0;
              }

              jetiSensorValUpdateTimeout++;
              if(jetiSensorValUpdateTimeout == 75)
                jetiSyncState = 0;
              break;
          }
        }
      }
    }

    //write data
    SSPWriteToLL(pageselect, (uint8_t*)&LL_1khz_control_input);
    //set pageselect to other page for next cycle
    pageselect = 1;
  }
  else //pageselect=1
  {
    //fill struct with 500Hz data
    LL_1khz_control_input.hor_accuracy = gps.data.horizontalAccuracy;
    LL_1khz_control_input.vert_accuracy = gps.data.verticalAccuracy;
    LL_1khz_control_input.speed_accuracy = gps.data.speedAccuracy;
    LL_1khz_control_input.numSV = gps.data.numSatellites;
    LL_1khz_control_input.battery_voltage_1 = HL_Status.battery_voltage_1;
    LL_1khz_control_input.battery_voltage_2 = 0;
    if(declinationAvailable == 1)
    {
      declinationAvailable = 2;
      LL_1khz_control_input.slowDataChannelSelect = SDC_DECLINATION;
      LL_1khz_control_input.slowDataChannelDataShort = estimatedDeclination;
    }
    else if(declinationAvailable == 2)
    {
      declinationAvailable = 3;
      LL_1khz_control_input.slowDataChannelSelect = SDC_INCLINATION;
      LL_1khz_control_input.slowDataChannelDataShort = estimatedInclination;
    }
    else if(transmitBuildInfoTrigger == 1)
    {
      transmitBuildInfoTrigger = 2;
      transmitCnt = 0;
      transmitPtr = (unsigned char *)&buildInfo.version_major;
      LL_1khz_control_input.slowDataChannelSelect = SDC_BUILDINFO;
      LL_1khz_control_input.slowDataChannelDataShort = transmitCnt;
      LL_1khz_control_input.slowDataChannelDataChar = *transmitPtr;
      transmitPtr++;
      transmitCnt++;
    }
    else if(transmitBuildInfoTrigger == 2)
    {
      LL_1khz_control_input.slowDataChannelSelect = SDC_BUILDINFO;
      LL_1khz_control_input.slowDataChannelDataShort = transmitCnt;
      LL_1khz_control_input.slowDataChannelDataChar = *transmitPtr;
      transmitPtr++;
      transmitCnt++;

      if(transmitCnt == sizeof(buildInfo))
        transmitBuildInfoTrigger = 3;
    }
    else if(sdk.cmd.newEmergencyMode)
    {
      LL_1khz_control_input.slowDataChannelSelect = SDC_EM_MODE;
      LL_1khz_control_input.slowDataChannelDataShort = sdk.cmd.newEmergencyMode;
    }
    else
    {
      LL_1khz_control_input.slowDataChannelDataShort = 0;
      LL_1khz_control_input.slowDataChannelSelect = 0;
      LL_1khz_control_input.slowDataChannelDataChar = 0;
    }

    //write data
    SSPWriteToLL(pageselect, (uint8_t*)&LL_1khz_control_input);
    //set pageselect to other page for next cycle
    pageselect = 0;
  }

  return 1;
}

inline void SSP_rx_handler_HL(unsigned char SPI_rxdata) //rx_handler @ high-level processor
{
  static volatile unsigned char SPI_syncstate = 0;
  static volatile unsigned char SPI_rxcount = 0;
  static volatile unsigned char *SPI_rxptr;
  static volatile unsigned char incoming_page;

  //receive handler
  if(SPI_syncstate == 0)
  {
    if(SPI_rxdata == '>')
      SPI_syncstate++;
    else
      SPI_syncstate = 0;
  }
  else if(SPI_syncstate == 1)
  {
    if(SPI_rxdata == '*')
    {
      SPI_syncstate++;
      SPI_rxptr = (unsigned char *)&LL_1khz_attitude_data;
      SPI_rxcount = 40;
    }
    else
      SPI_syncstate = 0;
  }
  else if(SPI_syncstate == 2)
  {
    if(SPI_rxcount == 26) //14 bytes transmitted => select 500Hz page
    {
      incoming_page = LL_1khz_attitude_data.system_flags & 0x03; //system flags were already received
      if(incoming_page == 1)
        SPI_rxptr += 26;
      else if(incoming_page == 2)
        SPI_rxptr += 52;
    }
    SPI_rxcount--;
    *SPI_rxptr = SPI_rxdata;
    SPI_rxptr++;
    if(SPI_rxcount == 0)
    {
      SPI_syncstate++;
    }
  }
  else if(SPI_syncstate == 3)
  {
    if(SPI_rxdata == '<') //last byte ok => data should be valid
    {
      SSP_data_distribution_HL(); //only distribute data to other structs, if it was received correctly
      //ack data receiption
    }
    SPI_syncstate = 0;
  }
  else
    SPI_syncstate = 0;
}
