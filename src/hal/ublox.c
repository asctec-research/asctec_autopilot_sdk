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

#include "LPC214x.h"
#include "main.h"
#include "util/gpsmath.h"
#include "ublox.h"
#include "uart1.h"

// used by: uBloxReceiveEngine
#define UR_MAX_RETRYS 80
#define MAX_BR_CNT    3

// used by: uBloxReceiveHandler
#define URS_SYNC1   0
#define URS_SYNC2   1
#define URS_CLASS   2
#define URS_ID      3
#define URS_LENGTH1 4
#define URS_LENGTH2 5
#define URS_DATA    6
#define URS_CKA     7
#define URS_CKB     8
#define URS_RAWDATA 9

#define URES_IDLE 0
#define URES_RESET_UBLOX_CONFIG 1
#define URES_WAIT_FOR_DATA 2
#define URES_RESETCONFIG 3
#define URES_ACK_CFG 4
#define URES_ACK_MSG 5
#define URES_CONFIG_DONE 6

#define UBX_UNKNOWN  0
#define UBX_VER_ANT4 1
#define UBX_VER_UB6  2
#define UBX_VER_UB8  3

#define UR_MAX_DATA_LENGTH 256

#define MAX_COMMON_CFG_PACKETS_ANT4 5
#define MAX_COMMON_CFG_PACKETS_UB6 6
const unsigned char urCfgIdListAnt4[MAX_COMMON_CFG_PACKETS_ANT4] =
  { 0x00, 0x08, 0x13, 0x16, 0x1A };
const unsigned char urCfgIdListUB6[MAX_COMMON_CFG_PACKETS_UB6] =
  { 0x00, 0x08, 0x13, 0x16, 0x24, 0x23 };

//CFG-PRT
const unsigned char urCfg00Ant4[] =
  { 0x01, 0x00, 0x00, 0x00, 0xd0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
const unsigned char urCfg00UB6[] =
  { 0x01, 0x12, 0x00, 0x00, 0xc0, 0x08, 0x00, 0x00, 0x00, 0xc2, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

const unsigned char urCfg08[] =
  { 0xc8, 0x00, 0x01, 0x00, 0x00, 0x00 }; // 5Hz

//CFG-ANT
const unsigned char urCfg13[] =
  { 0x0b, 0x00, 0x00, 0x00 };

//CFG-SBAS
const unsigned char urCfg16[] =
  { 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };

#define UBX_NO_OF_MSG_CONFIGS 10
//don't change the last two entries! MON-SCHED is not working with the UB6 chipset and the lest entry activates the raw output on T chipsets. If packages are added, add them in the beginning!
const unsigned char urCfgMsgList[UBX_NO_OF_MSG_CONFIGS][3] =
  {
    { 0x01, 0x02, 1 },
    { 0x01, 0x03, 1 },
    { 0x01, 0x04, 1 },
    { 0x01, 0x06, 1 },
    { 0x01, 0x12, 1 },
    { 0x01, 0x30, 2 },
    { 0x0A, 0x09, 1 },
    { 0x0A, 0x01, 1 },
    { 0x02, 0x10, 1 },
    { 0x02, 0x11, 1 } };

const unsigned char urCfgNav2[] =
  { 0x05, 0x00, 0x00, 0x00, 0x04, 0x03, 0x0A, 0x02, 0x50, 0xc3, 0x00, 0x00, 0x0f, 0x0a, 0x0a, 0x3c, 0x00, 0x01, 0x00,
      0x00, 0xfa, 0x00, 0xfa, 0x00, 0x64, 0x00, 0x2c, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00 };

const unsigned char urCfgNav5[] =
  { 0xFF, 0xFF, 0x06, 0x02, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x0A, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64,
      0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

//max 20 satellites for NEO M8 GPS
const unsigned char urCfgNav5X[] =
  { 0x00, 0x00, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0x03, 0x02, 0x03, 0x14, 0x0A, 0x00, 0x01, 0x01, 0x00, 0x00, 0xF8,
      0x05, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x64, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
      0x00, 0x00 }; //Firmware 7.03

//enable GPS/GLONASS etc.
const unsigned char urCfgGNSS[] =
  {
      //0xB5, 0x62, 0x06, 0x3E, 0x2C, 0x00
      0x00, 0x00, 0x20, 0x05, 0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x00, 0x00, 0x01,
      0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00,
//		0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x00, 0x00, //GLONASS OFF
      0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, //GLONASS ON
      0x01, 0x01 //, 0xFC, 0x11
    };

const unsigned char urCfgPwr[] =
  { 0x09, 0x00 };

GPSData gps = {
  .version = UBX_UNKNOWN,
};

static unsigned int urTimeCnt = 0;
static volatile unsigned char urAckReceived = 0;
static volatile unsigned char urAckClass = 0;
static volatile unsigned char urAckId = 0;
static unsigned char urConfigCnt;
static unsigned char * urCfgIdList;
static volatile unsigned char urConfigMessageReceived = 0;
static volatile unsigned char urVersionCheck = 0;
static unsigned char urRecData[UR_MAX_DATA_LENGTH];
static unsigned char urEngineState = URES_IDLE;
static unsigned short urMsgCnt = 0;

static void sendMessage(unsigned char urClass, unsigned char urId, unsigned char * urData, unsigned char urLength)
{
  unsigned char ckA, ckB;
  unsigned char header[2];

  header[0] = 0xB5;
  header[1] = 0x62;

  ckA = 0;
  ckB = 0;

  ckA += urClass;
  ckB += ckA;

  ckA += urId;
  ckB += ckA;

  ckA += urLength & 0xff;
  ckB += ckA;

  ckA += urLength >> 8;
  ckB += ckA;

  for(int i = 0; i < urLength; i++)
  {
    ckA += urData[i];
    ckB += ckA;
  }

  UART1Send((unsigned char *)&header, 2);
  UART1Send((unsigned char *)&urClass, 1);
  UART1Send((unsigned char *)&urId, 1);

  unsigned char c = urLength;
  UART1Send(&c, 1);

  c = urLength >> 8;
  UART1Send(&c, 1);

  UART1Send(urData, urLength);
  UART1Send(&ckA, 1);
  UART1Send(&ckB, 1);

  //wait until all data is sent out
  while((U1LSR & (1 << 6)) == 0);
}

static void setMessageRate(unsigned char urClass, unsigned char urId, unsigned char rate)
{
  unsigned char data[3];

  data[0] = urClass;
  data[1] = urId;
  data[2] = rate;
  sendMessage(0x06, 0x01, (unsigned char *)&data, 3);
}

static void resetConfiguration(void)
{
  UBX_CFG_CFG ubxCfg;

  ubxCfg.clearMask = 0xffff; //reset complete config except port config
  ubxCfg.loadMask = 0xffff;
  ubxCfg.saveMask = 0;
  ubxCfg.deviceMask = 0x07;

  sendMessage(0x06, 0x09, (unsigned char *)&ubxCfg, sizeof(UBX_CFG_CFG));
}

static void handleMessage(unsigned char urClass, unsigned char urId, unsigned char * urData, unsigned short urLength)
{
  (void)urLength;

  static int sacc_filter = 0;

  urMsgCnt++;

  switch(urClass)
  {
    case 0x01:
    {
      switch(urId)
      {
        case 0x02: //NAV-POSLLH
        {
          UBX_NAV_POSLLH* navPosLLH = (UBX_NAV_POSLLH*)urData;
          gps.data.latitude = navPosLLH->lat;
          gps.data.longitude = navPosLLH->lon;
          gps.data.height = navPosLLH->height;
          gps.data.horizontalAccuracy = navPosLLH->hAcc;
          gps.data.verticalAccuracy = navPosLLH->vAcc;
        }
        break;

        case 0x03: //NAV-STATUS
        {
          UBX_NAV_STATUS* navStatus = (UBX_NAV_STATUS*)urData;
          gps.status = (navStatus->flags << 8) | (navStatus->fixStat << 16) | navStatus->gpsFix;
          if(gps.status == 0x03)
            gps.data.hasLock = 1;
          else
            gps.data.hasLock = 0;
        }
        break;

        case 0x06: //NAV-SOL
        {
          UBX_NAV_SOL* navSol = (UBX_NAV_SOL*)urData;
          gps.time.week = navSol->week;
          gps.time.time_of_week = navSol->iTow;
          gps.data.numSatellites = navSol->numSV;
        }
        break;

        case 0x12: //NAV-VELNED
        {
          UBX_NAV_VELNED* navVelNed = (UBX_NAV_VELNED*)urData;
          gps.data.speedEastWest = navVelNed->velE * 10;
          gps.data.speedNorthSouth = navVelNed->velN * 10;
          gps.data.speedAccuracy = navVelNed->sAcc * 10;
          gps.data.heading = navVelNed->heading / 100;
          sacc_filter = (2 * sacc_filter + navVelNed->sAcc) / 3;
          if((gps.data.horizontalAccuracy > 15000) || (sacc_filter > 230))
          {
            gps.status &= ~0x03;
            gps.data.hasLock = 0;
          }
          gps.newForLL = 1;
          gps.dataUpdated = 1;
          GPS_timeout = 0;
        }
        break;

        case 0x30:
        {
//          struct UBX_NAV_SVINFO_HEADER* navSVInfoHeader = (struct UBX_NAV_SVINFO_HEADER*)urData;
        }
        break;
      }
    }
    break;

    case 0x05: //ACK
    {
      urAckClass = urData[0];
      urAckId = urData[1];

      if((urVersionCheck)&&(urAckClass == 0x06) && (urAckId == 0x24))
      {
        urVersionCheck = 0;
        if(urId == 0x01)
        {
          gps.version = UBX_VER_UB8;
          urCfgIdList = (unsigned char *)&urCfgIdListUB6;
        }
        else
        {
          gps.version = UBX_VER_ANT4;
          urCfgIdList = (unsigned char *)&urCfgIdListAnt4;
        }
      }
      else
      {
        switch(urId)
        {
          case 0x01:
          {
            urAckReceived = 1;
          }
          break;
          case 0x00:
          {
            //always ACK GNSS package, even when GPS is not supporting it => compatibility with units older than M8Q
            if(urAckId == 0x3E)
            {
              urAckReceived = 1;
              gps.version = UBX_VER_UB6;
            }
          }
          break;
        }
      }
    }
    break;

    case 0x0A: //MON
    {
      switch(urId)
      {
        case 0x01: //MON-SCHED
        {
//          struct UBX_MON_SCHED* monSched = (struct UBX_MON_SCHED*)urData;
        }
        break;
        case 0x09: //MON-HW
        {
//          struct UBX_MON_HW* monHw = (struct UBX_MON_HW*)urData;
        }
        break;
      }
    }
    break;
  }
}

void uBloxReceiveHandler(unsigned char recByte)
{
  static unsigned char urState = URS_SYNC1;
  static unsigned char urClass;
  static unsigned char urId;
  static unsigned short urLength;
  static unsigned short urCnt = 0;
  static unsigned char urCkARec, urCkBRec;
  static unsigned char urCkA, urCkB;

  switch(urState)
  {
    case URS_SYNC1:
    {
      if(recByte == 0xB5)
        urState = URS_SYNC2;
    }
    break;
    case URS_SYNC2:
    {
      if(recByte == 0x62)
      {
        urState = URS_CLASS;
        urCkA = 0;
        urCkB = 0;
      }
      else
      {
        urState = URS_SYNC1;
      }
    }
    break;
    case URS_CLASS:
    {
      urClass = recByte;
      urState = URS_ID;

      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
    }
    break;
    case URS_ID:
    {
      urId = recByte;
      urState = URS_LENGTH1;
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
    }
    break;
    case URS_LENGTH1:
    {
      urLength = recByte;
      urState = URS_LENGTH2;
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
    }
    break;
    case URS_LENGTH2:
    {
      urLength |= recByte << 8;
      urCnt = 0;
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;
      if(urLength > UR_MAX_DATA_LENGTH)
        urState = URS_SYNC1;
      else
        urState = URS_DATA;
    }
    break;
    case URS_DATA:
    {
      //update chkSum
      urCkA += recByte;
      urCkB += urCkA;

      if(urCnt >= UR_MAX_DATA_LENGTH)
      {
        urState = URS_SYNC1;
        return;
      }

      urRecData[urCnt++] = recByte;

      if(urCnt == urLength)
        urState = URS_CKA;
    }
    break;
    case URS_CKA:
    {
      urCkARec = recByte;
      urState = URS_CKB;
    }
    break;
    case URS_CKB:
    {
      urCkBRec = recByte;
      if((urCkA == urCkARec) && (urCkB == urCkBRec))
      {
        handleMessage(urClass, urId, &urRecData[0], urLength);
      }
      urState = URS_SYNC1;
    }
    break;
    default:
    {
      urState = URS_SYNC1;
    }
    break;
  }
}

void uBloxReceiveEngine(void)
{
  static unsigned char urTimeOut;
  static unsigned char urReconfigurationRetries = UR_MAX_RETRYS;
  static unsigned int currentBaudrate = 115200;
  static unsigned short startup_timeout = 400;
  const unsigned int baudrateList[MAX_BR_CNT] =
    { 9600, 57600, 115200 };
  static unsigned char baudrateCnt = 0;
  int i;

  urTimeCnt++;

  if(urTimeCnt % 10)
    return;

  if(!urReconfigurationRetries)
  {
    //gps configuration failed completly. Dont' do anything again
    return;
  }

  switch(urEngineState)
  {
    case URES_IDLE:
    {
      //init all vars
      baudrateCnt = 0;
      currentBaudrate = baudrateList[baudrateCnt];

      urMsgCnt = 0;
      urCfgIdList = (unsigned char *)&urCfgIdListAnt4;

      if(startup_timeout)
        startup_timeout--;
      else
        urEngineState = URES_RESET_UBLOX_CONFIG;
    }
    break;
    case URES_RESET_UBLOX_CONFIG:
    {
      //cycle through baudrates and reset config

      //wait until all data is sent out
      while((U1LSR & (1 << 6)) == 0);
      UART1Init(currentBaudrate);

      //clear RX and TX fifos
      U1FCR = 0x03;

      //send reset command three times
      resetConfiguration();
      resetConfiguration();
      resetConfiguration();

      //get next baudrate
      baudrateCnt++;
      if(baudrateCnt < MAX_BR_CNT)
      {
        currentBaudrate = baudrateList[baudrateCnt];
      }
      else
      {
        currentBaudrate = 9600;
        //wait until all data is sent out
        while((U1LSR & (1 << 6)) == 0);
        UART1Init(currentBaudrate);

        //clear RX and TX fifos
        U1FCR = 0x03;

        urEngineState = URES_WAIT_FOR_DATA;

        //poll NAV5 to check if the GPS is a newer or older uBlox
        urVersionCheck = 1;
        sendMessage(0x6, 0x24, (unsigned char *)0, 0);
        urTimeOut = 10;
      }
    }
    break;
    case URES_WAIT_FOR_DATA:
    {
      if(urVersionCheck == 0)
      {
        //see if baudrate is correct. Otherwise transmit port setting and start over
        if(currentBaudrate == 9600)
        {
          for(i = 0; i < 3; i++)
          {
            if(gps.version == UBX_VER_ANT4)
              sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg00Ant4[0], sizeof(urCfg00Ant4));
            else
              sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg00UB6[0], sizeof(urCfg00UB6));
          }
          currentBaudrate = 115200;

          //wait until all data is sent out
          while((U1LSR & (1 << 6)) == 0);
          UART1Init(currentBaudrate);

          //clear RX and TX fifos
          U1FCR = 0x03;
        }

        urEngineState = URES_ACK_CFG;
        urAckReceived = 0;
        urConfigCnt = 1;
        urConfigMessageReceived = 0;
        urTimeOut = 10;

        //set first packet
        sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg08, sizeof(urCfg08));
      }
      else
      {
        if(urTimeOut)
        {
          urTimeOut--;
        }
        else
        {
          urReconfigurationRetries--;

          urMsgCnt = 0;
          urEngineState = URES_WAIT_FOR_DATA;
          urVersionCheck = 1;
          sendMessage(0x6, 0x24, (unsigned char *)0, 0);
          urTimeOut = 10;
        }
      }
    }
    break;
    case URES_ACK_CFG:
    {
      if((urAckReceived)&&(urAckClass == 0x06) && (urAckId == urCfgIdList[urConfigCnt]))
      {
        urAckReceived = 0;
        urReconfigurationRetries = UR_MAX_RETRYS;
        urConfigCnt++;
        if(((gps.version == UBX_VER_UB6) && (urConfigCnt == MAX_COMMON_CFG_PACKETS_UB6 - 1))
            || ((gps.version == UBX_VER_ANT4) && (urConfigCnt == MAX_COMMON_CFG_PACKETS_ANT4))
            || ((gps.version == UBX_VER_UB8) && (urConfigCnt == MAX_COMMON_CFG_PACKETS_UB6)))
        {
          urEngineState = URES_ACK_MSG;
          urConfigCnt = 0;
          urTimeOut = 10;
          setMessageRate(urCfgMsgList[urConfigCnt][0], urCfgMsgList[urConfigCnt][1], urCfgMsgList[urConfigCnt][2]);
        }
        else
        {
          urTimeOut = 10;
          if(urConfigCnt == 1)
            sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg08, sizeof(urCfg08));
          else if(urConfigCnt == 2)
            sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg13, sizeof(urCfg13));
          else if(urConfigCnt == 3)
            sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg16, sizeof(urCfg16));
          else if(urCfgIdList[urConfigCnt] == 0x23)
            sendMessage(0x6, 0x23, (unsigned char *)&urCfgNav5X, sizeof(urCfgNav5X));
          else if(urCfgIdList[urConfigCnt] == 0x24)
            sendMessage(0x6, 0x24, (unsigned char *)&urCfgNav5, sizeof(urCfgNav5));
          else if(urCfgIdList[urConfigCnt] == 0x1A)
            sendMessage(0x6, 0x1A, (unsigned char *)&urCfgNav2, sizeof(urCfgNav2));
          else if(urCfgIdList[urConfigCnt] == 0x3E)
            sendMessage(0x06, 0x3E, (unsigned char *)&urCfgGNSS, sizeof(urCfgGNSS));
          else if(urCfgIdList[urConfigCnt] == 0x11)
            sendMessage(0x06, 0x11, (unsigned char *)&urCfgPwr, sizeof(urCfgPwr));
        }
      }
      else
      {
        if(urTimeOut)
        {
          urTimeOut--;
        }
        else
        {
          urReconfigurationRetries--;
          urAckReceived = 0;
          urTimeOut = 10;
          if(urConfigCnt == 1)
            sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg08, sizeof(urCfg08));
          else if(urConfigCnt == 2)
            sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg13, sizeof(urCfg13));
          else if(urConfigCnt == 3)
            sendMessage(0x6, urCfgIdList[urConfigCnt], (unsigned char *)&urCfg16, sizeof(urCfg16));
          else if(urCfgIdList[urConfigCnt] == 0x23)
            sendMessage(0x6, 0x23, (unsigned char *)&urCfgNav5X, sizeof(urCfgNav5X));
          else if(urCfgIdList[urConfigCnt] == 0x24)
            sendMessage(0x6, 0x24, (unsigned char *)&urCfgNav5, sizeof(urCfgNav5));
          else if(urCfgIdList[urConfigCnt] == 0x1A)
            sendMessage(0x6, 0x1A, (unsigned char *)&urCfgNav2, sizeof(urCfgNav2));
          else if(urCfgIdList[urConfigCnt] == 0x3E)
            sendMessage(0x06, 0x3E, (unsigned char *)&urCfgGNSS, sizeof(urCfgGNSS));
          else if(urCfgIdList[urConfigCnt] == 0x11)
            sendMessage(0x06, 0x11, (unsigned char *)&urCfgPwr, sizeof(urCfgPwr));
        }
      }
    }
    break;

    case URES_ACK_MSG:
    {
      if(urAckReceived)
      {
        urAckReceived = 0;
        urConfigCnt++;
        urReconfigurationRetries = UR_MAX_RETRYS;
        if((gps.version == UBX_VER_UB6) && (urConfigCnt == UBX_NO_OF_MSG_CONFIGS - 2))
          urConfigCnt++;

        //skip raw message configuration here always. raw message is enabled in URES_CONFIG_DONE on request
        if((urConfigCnt == UBX_NO_OF_MSG_CONFIGS - 1))
          urConfigCnt++;

        if(urConfigCnt == UBX_NO_OF_MSG_CONFIGS)
        {
          urEngineState = URES_CONFIG_DONE;
        }
        else
        {
          urEngineState = URES_ACK_MSG;
          urConfigMessageReceived = 0;
          urTimeOut = 10;
          setMessageRate(urCfgMsgList[urConfigCnt][0], urCfgMsgList[urConfigCnt][1], urCfgMsgList[urConfigCnt][2]);
        }
      }
      else
      {
        if(urTimeOut)
        {
          urTimeOut--;
        }
        else
        {
          urReconfigurationRetries--;
          setMessageRate(urCfgMsgList[urConfigCnt][0], urCfgMsgList[urConfigCnt][1], urCfgMsgList[urConfigCnt][2]);
          urTimeOut = 10;
          urAckReceived = 0;
          urEngineState = URES_ACK_MSG;
        }
      }
    }
    break;

    case URES_CONFIG_DONE:
      break;
  }
}
