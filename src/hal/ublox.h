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

#pragma once

#include <stdint.h>

#pragma pack(push,1)

typedef struct _UBX_NAV_SOL
{
  unsigned int iTow;
  unsigned int fTow;
  short week;
  unsigned char gpsFix;
  unsigned char flags;
  int ecefX;
  int ecefY;
  int ecefZ;
  unsigned int pAcc;
  int ecefVX;
  int ecefVY;
  int ecefVZ;
  unsigned int sAcc;
  unsigned short pDOP;
  unsigned char reserved1;
  unsigned char numSV;
  unsigned int reserved2;
} UBX_NAV_SOL;

typedef struct _UBX_NAV_POSLLH
{
  unsigned int iTow;
  int lon;
  int lat;
  int height;
  int hMSL;
  int hAcc;
  int vAcc;
} UBX_NAV_POSLLH;

typedef struct _UBX_NAV_VELNED
{
  unsigned int iTow;
  int velN;
  int velE;
  int velD;
  unsigned int speed;
  unsigned int gSpeed;
  int heading;
  unsigned int sAcc;
  unsigned int cAcc;
} UBX_NAV_VELNED;

typedef struct _UBX_NAV_STATUS
{
  unsigned int iTow;
  unsigned char gpsFix;
  unsigned char flags;
  unsigned char fixStat;
  unsigned char flags2;
  unsigned int ttff;
  unsigned int msss;
} UBX_NAV_STATUS;

typedef struct _UBX_MON_VER_HEADER
{
  char swVersion[30];
  char hwVersion[10];
  char romVersion[30];
} UBX_MON_VER_HEADER;

typedef struct _UBX_MON_SCHED
{
  unsigned char reserved[16];
  unsigned short stackUsed;
  unsigned short stackAv;
  unsigned short cpuLoad;
  unsigned char fullSlots;
  unsigned char partSlots;
} UBX_MON_SCHED;

typedef struct _UBX_MON_HW
{
  unsigned int pinSel;
  unsigned int pinBank;
  unsigned int pinDir;
  unsigned int pinVal;

  unsigned short noisePerMs;
  unsigned short agcCnt;
  unsigned char aStatus;
  unsigned char aPower;
  unsigned char flags;
  unsigned char reserved1;
  unsigned int usedMast;
  unsigned char vp[25];
  unsigned char jamInd;
  unsigned short reserved3;
  unsigned int pinIrq;
  unsigned int pullH;
  unsigned int pullL;
} UBX_MON_HW;

typedef struct _UBX_NAV_SVINFO_HEADER
{
  unsigned int iTow;
  unsigned char numCh;
  unsigned char globalFlags;
  unsigned short reserved2;
} UBX_NAV_SVINFO_HEADER;

typedef struct _UBX_NAV_SVINFO_SAT
{
  unsigned char channel;
  unsigned char svId;
  unsigned char flags;
  unsigned char quality;
  unsigned char cNo;
  char elev; //in deg
  short azim; // in deg
  int prRes; //pseudo range residual
} UBX_NAV_SVINFO_SAT;

typedef struct _UBX_CFG_CFG
{
  unsigned int clearMask;
  unsigned int saveMask;
  unsigned int loadMask;
  unsigned char deviceMask;
} UBX_CFG_CFG;

typedef struct _SVINFO_HEADER
{
  unsigned char numCh;
  unsigned char globalFlags;
} SVINFO_HEADER;

typedef struct _SVINFO
{
  unsigned char channel;
  unsigned char svId;
  unsigned char flags;
  unsigned char quality;
  unsigned char cNo;
  char elev; //in deg
  short azim; // in deg
  int prRes; //pseudo range residual
} SVINFO;

typedef struct _GPS_HW_STATUS
{
  unsigned char antennaStatus;
  unsigned char antennaPower;
  unsigned short agcMonitor;
  unsigned short noiseLevel;

  unsigned short stackUsed;
  unsigned short stackAv;
  unsigned short cpuLoad;
  unsigned char fullSlots;
  unsigned char partSlots;
} GPS_HW_STATUS;

#pragma pack(pop)

typedef struct _GPSRawData
{
  int32_t latitude;   // [deg*10^7]
  int32_t longitude;  // [deg*10^7]
  int32_t height; // [mm]
  int32_t speedEastWest; // [mm/s]
  int32_t speedNorthSouth; // [mm/s]
  int32_t heading; // [deg*1000]
  uint16_t horizontalAccuracy; // [mm]
  uint16_t verticalAccuracy; // [mm]
  uint16_t speedAccuracy; // [mm/s]
  uint16_t numSatellites;
  uint8_t hasLock;
} GPSRawData;

typedef struct _GPSTime
{
  uint32_t time_of_week;  //[ms] (1 week = 604,800 s)
  uint32_t week;    // starts from beginning of year 1980
} GPSTime;

typedef struct _GPSData
{
  uint8_t version;
  uint8_t dataUpdated;
  uint8_t status;
  uint8_t newForLL;

  GPSRawData data;
  GPSTime time;
} GPSData;

extern GPSData gps;

void uBloxReceiveHandler(unsigned char recByte);
void uBloxReceiveEngine(void); //call with 200ms
