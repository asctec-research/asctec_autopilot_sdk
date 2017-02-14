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

#include "ext_com.h"
#include "util/crc16.h"
#include "hal/uart0.h"
#include "hal/sys_time.h"
#include "asctec_uav_msgs/message_definitions.h"
#include "asctec_uav_msgs/transport_definitions.h"
#include "sdkio.h"
#include "sdk.h"
#include <math.h>
#include <string.h>

ExtCom extCom;

static void msgImu();
static void msgVehicleStatus();
static void msgRcData();
static void msgMotorState();
static void msgGpsData();
static void msgFilteredSensorData();

typedef void(*ExtTxFunc)();

typedef struct _MsgTxConfig
{
  uint32_t msgId;
  ExtTxFunc pTxFunc;
  uint16_t div;
  uint16_t cnt;
} MsgTxConfig;

static MsgTxConfig wireCfg[] = {
  { MESSAGE_ID_IMU,                  &msgImu,                2,   0 },
  { MESSAGE_ID_VEHICLE_STATUS,       &msgVehicleStatus,      50,  0 },
  { MESSAGE_ID_RC_DATA,              &msgRcData,             50,  0 },
  { MESSAGE_ID_MOTOR_STATE,          &msgMotorState,         3,   0 },
  { MESSAGE_ID_GPS_DATA,             &msgGpsData,            200, 0 },
  { MESSAGE_ID_FILTERED_SENSOR_DATA, &msgFilteredSensorData, 3,   0 },
};

int16_t ExtComSend(void* _pData, uint32_t dataSize)
{
  if(dataSize > EXT_COM_MAX_MSG_SIZE)
  {
    ++extCom.txStat.oversized;
    return 2;
  }

  uint32_t bytesWritten;
  uint16_t seq = extCom.seq++;
  uint16_t crc = CRC16Checksum(_pData, dataSize);
  crc = CRC16ChecksumFeed(crc, &seq, EXT_COM_HEADER_SIZE);

  COBSState cState;
  COBSStartEncode(&cState, dataSize + EXT_COM_CHECKSUM_SIZE + EXT_COM_HEADER_SIZE, extCom.procBuffer, EXT_COM_MAX_ENCODED_MSG_SIZE);
  COBSFeedEncodeBlock(&cState, _pData, dataSize);
  COBSFeedEncodeBlock(&cState, &seq, EXT_COM_HEADER_SIZE);
  COBSFeedEncodeBlock(&cState, &crc, EXT_COM_CHECKSUM_SIZE);
  COBSFinalizeEncode(&cState, &bytesWritten);

  extCom.procBuffer[bytesWritten++] = 0;  // Insert packet delimiter

  if(FifoWrite(&uart0.txFifo, extCom.procBuffer, bytesWritten))
  {
    ++extCom.txStat.noMem;
    return 1;
  }

  ++extCom.txStat.good;
  return 0;
}

int16_t ExtComSendMessage(TransportHeader* pHeader, void* pData, uint32_t dataSize)
{
  if(dataSize + sizeof(TransportHeader) > EXT_COM_MAX_MSG_SIZE)
    return 1;

  memcpy(extCom.sendBuffer, pHeader, sizeof(TransportHeader));
  memcpy(extCom.sendBuffer+sizeof(TransportHeader), pData, dataSize);

  return ExtComSend(extCom.sendBuffer, sizeof(TransportHeader)+dataSize);
}

static int16_t handleExtMsg(uint8_t* pData, uint32_t dataSize)
{
  TransportHeader header;

  if(dataSize < sizeof(TransportHeader))
  {
    // something went wrong, msg is too small
    return 1;
  }
  else
  {
    memcpy(&header, pData, sizeof(TransportHeader));
    pData += sizeof(TransportHeader);
    dataSize -= sizeof(TransportHeader);
  }

  switch(header.id)
  {
    case MESSAGE_ID_COMMAND_MOTOR_SPEED:
    {
      CommandMotorSpeed* pCmd = (CommandMotorSpeed*)pData;

      sdk.cmd.mode = SDK_CMD_MODE_DIMC;

      for(uint8_t i = 0; i < 6; i++)
      {
        sdk.cmd.dimc.rpm[i] = pCmd->commandRpm[i];

        if(pCmd->commandRpm[i] == 0)
          sdk.cmd.dimc.enable[i] = 0;
        else
          sdk.cmd.dimc.enable[i] = 1;
      }

      extCom.cmdTimeout = 0;
    }
    break;
    case MESSAGE_ID_COMMAND_ROLL_PITCH_YAWRATE_THRUST:
    {
      CommandRollPitchYawrateThrust* pCmd = (CommandRollPitchYawrateThrust*)pData;

      sdk.cmd.mode = SDK_CMD_MODE_RPY_THRUST;
      sdk.cmd.RPYThrust.rollAngle = (int32_t)(pCmd->roll*180.0f/((float)M_PI)*1000.0f);
      sdk.cmd.RPYThrust.pitchAngle = (int32_t)(pCmd->pitch*180.0f/((float)M_PI)*1000.0f);
      sdk.cmd.RPYThrust.yawRate = (int32_t)(pCmd->yawRate*180.0f/((float)M_PI)*1000.0f);
      sdk.cmd.RPYThrust.thrust = (uint32_t)((pCmd->thrust/MAX_THRUST)*4095.0f);

      extCom.cmdTimeout = 0;
    }
    break;
    case MESSAGE_ID_COMMAND_ROLL_PITCH_YAWRATE_CLIMBRATE:
    {
      CommandRollPitchYawrateClimbRate* pCmd = (CommandRollPitchYawrateClimbRate*)pData;

      sdk.cmd.mode = SDK_CMD_MODE_RPY_CLIMBRATE;
      sdk.cmd.RPYClimbRate.rollAngle = (int32_t)(pCmd->roll*180.0f/((float)M_PI)*1000.0f);
      sdk.cmd.RPYClimbRate.pitchAngle = (int32_t)(pCmd->pitch*180.0f/((float)M_PI)*1000.0f);
      sdk.cmd.RPYClimbRate.yawRate = (int32_t)(pCmd->yawRate*180.0f/((float)M_PI)*1000.0f);
      sdk.cmd.RPYClimbRate.climbRate = (int32_t)((pCmd->climbRate)*1000.0f);

      extCom.cmdTimeout = 0;
    }
    break;
    case MESSAGE_ID_COMMAND_GPS_WAYPOINT:
    {
      CommandGpsWaypoint* pCmd = (CommandGpsWaypoint*)pData;

      sdk.cmd.mode = SDK_CMD_MODE_GPS_WAYPOINT_ABS;
      sdk.cmd.wpAbsolute.latitude = pCmd->latitude;
      sdk.cmd.wpAbsolute.longitude = pCmd->longitude;
      sdk.cmd.wpAbsolute.height = (int32_t)(pCmd->height*1000.0f);
      sdk.cmd.wpAbsolute.heading = (int32_t)(pCmd->heading*1000.0f);
      sdk.cmd.wpAbsolute.reachedTolerance = 3000;
      sdk.cmd.wpAbsolute.timeToStay = 1;
      if(pCmd->maxSpeed > 2.0f)
        sdk.cmd.wpAbsolute.maxSpeed = 100;
      else
        sdk.cmd.wpAbsolute.maxSpeed = pCmd->maxSpeed*0.5f*100.0f;
      sdk.cmd.wpAbsolute.updated = 1;

      extCom.cmdTimeout = 0;
    }
    break;
    case MESSAGE_ID_SYSTEM_UPTIME:
    {
      TransportHeader upHeader;
      SystemUpTime up;

      upHeader.flags = 0;
      upHeader.id = MESSAGE_ID_SYSTEM_UPTIME;
      upHeader.ackId = 0;

      up.timestampUs = SysTimeLongUSec();

      ExtComSendMessage(&upHeader, &up, sizeof(SystemUpTime));
    }
    break;
    case MESSAGE_ID_CONFIG_SET_MESSAGE_RATE_DIVISOR:
    {
      const uint32_t partSize = sizeof(uint32_t)+sizeof(uint16_t);
      uint32_t numParts = dataSize/partSize;

      for(uint32_t i = 0; i < numParts; i++)
      {
        uint32_t msgId = *((uint32_t*)pData);
        pData += sizeof(uint32_t);
        uint16_t div = *((uint16_t*)pData);
        pData += sizeof(uint16_t);

        for(uint16_t i = 0; i < sizeof(wireCfg) / sizeof(wireCfg[0]); i++)
        {
          if(wireCfg[i].msgId == msgId)
          {
            wireCfg[i].div = div;
          }
        }
      }
    }
    break;
    default:
    {
      SDKProcessUserMsg(&header, pData, dataSize);
    }
    break;
  }

  if(header.flags & TRANSPORT_FLAG_ACK_REQUEST)
  {
    TransportHeader ackHeader;
    ackHeader.id = header.id;
    ackHeader.flags = TRANSPORT_FLAG_ACK_RESPONSE;
    ackHeader.ackId = header.ackId;

    ExtComSend(&ackHeader, sizeof(TransportHeader));
  }

  return 0;
}

// dataSize is without packet delimiter '0'
static void processCompleteMsg(uint8_t* pData, uint32_t dataSize)
{
  uint32_t bytesWritten;
  if(COBSDecode(pData, dataSize, extCom.procBuffer, EXT_COM_MAX_ENCODED_MSG_SIZE, &bytesWritten) == 0)
  {
    if(bytesWritten > EXT_COM_CHECKSUM_SIZE + EXT_COM_HEADER_SIZE)
    {
      uint16_t crc = CRC16Checksum(extCom.procBuffer, bytesWritten - EXT_COM_CHECKSUM_SIZE);
      if(crc == *((uint16_t*)&extCom.procBuffer[bytesWritten - EXT_COM_CHECKSUM_SIZE]))
      {
        // data now complete in extCom.procBuffer. Size: bytesWritten - EXT_COM_CHECKSUM_SIZE - EXT_COM_HEADER_SIZE
        ++extCom.rxStat.good;

        handleExtMsg(extCom.procBuffer, bytesWritten - EXT_COM_CHECKSUM_SIZE - EXT_COM_HEADER_SIZE);
      }
      else
      {
        ++extCom.rxStat.crcFail;
      }
    }
    else
    {
      ++extCom.rxStat.crcFail;
    }
  }
  else
  {
    ++extCom.rxStat.decodeFail;
  }
}

static void eulerRPY2Quaternion(const float* pRPY, Quaternion* pQuat)
{
  float t0 = cosf(pRPY[2]*0.5f);
  float t1 = sinf(pRPY[2]*0.5f);
  float t2 = cosf(pRPY[0]*0.5f);
  float t3 = sinf(pRPY[0]*0.5f);
  float t4 = cosf(pRPY[1]*0.5f);
  float t5 = sinf(pRPY[1]*0.5f);

  pQuat->w = t0 * t2 * t4 + t1 * t3 * t5;
  pQuat->x = t0 * t3 * t4 - t1 * t2 * t5;
  pQuat->y = t0 * t2 * t5 + t1 * t3 * t4;
  pQuat->z = t1 * t2 * t4 - t0 * t3 * t5;
}

static void msgImu()
{
  TransportHeader header;
  Imu imu;

  header.id = MESSAGE_ID_IMU;
  header.flags = 0;
  header.ackId = 0;

  imu.timestampUs = SysTimeLongUSec();
  imu.angularVelocity.x = sdk.ro.attitude.angularVelocity[0]*0.001f*((float)M_PI)/180.0f;
  imu.angularVelocity.y = sdk.ro.attitude.angularVelocity[1]*0.001f*((float)M_PI)/180.0f;
  imu.angularVelocity.z = sdk.ro.attitude.angularVelocity[2]*0.001f*((float)M_PI)/180.0f;
  imu.linearAcceleration.x = sdk.ro.sensors.acc[0]*0.001f;
  imu.linearAcceleration.y = sdk.ro.sensors.acc[1]*0.001f;
  imu.linearAcceleration.z = sdk.ro.sensors.acc[2]*0.001f;

  float rpy[3];
  rpy[0] = sdk.ro.attitude.roll*0.001f*((float)M_PI)/180.0f;
  rpy[1] = sdk.ro.attitude.pitch*0.001f*((float)M_PI)/180.0f;
  rpy[2] = sdk.ro.attitude.yaw*0.001f*((float)M_PI)/180.0f;
  eulerRPY2Quaternion(rpy, &imu.attitude);

  ExtComSendMessage(&header, &imu, sizeof(Imu));
}

static void msgVehicleStatus()
{
  TransportHeader header;

  header.id = MESSAGE_ID_VEHICLE_STATUS;
  header.flags = 0;
  header.ackId = 0;

  VehicleStatus vStatus;
  vStatus.timestampUs = SysTimeLongUSec();
  vStatus.batteryVoltageMv = sdk.ro.sensors.battery;
  vStatus.cpuLoadPerMill = sdk.ro.cpuLoad;
  vStatus.flightMode = sdk.ro.flightMode;
  vStatus.flightTimeMs = ((int32_t)sdk.ro.flightTime)*1000;
  vStatus.safetyPilotState = 0;
  vStatus.vehicleType = VEHICLE_TYPE;

  ExtComSendMessage(&header, &vStatus, sizeof(VehicleStatus));
}

static void msgRcData()
{
  TransportHeader header;

  header.id = MESSAGE_ID_RC_DATA;
  header.flags = 0;
  header.ackId = 0;

  RcData rc;
  rc.timestampUs = SysTimeLongUSec();
  rc.hasLock = sdk.ro.rc.hasLock;
  rc.aux = sdk.ro.rc.aux*8;
  rc.stickPitch = sdk.ro.rc.pitch*8;
  rc.stickRoll = sdk.ro.rc.roll*8;
  rc.stickThrust = sdk.ro.rc.thrust*8;
  rc.stickYaw = sdk.ro.rc.yaw*8;
  rc.switchExternalCommand = sdk.ro.rc.serialSwitch*8;
  rc.switchMode = sdk.ro.rc.flightMode*8;
  rc.switchPowerOnOff = -1;

  ExtComSendMessage(&header, &rc, sizeof(RcData));
}

static void msgMotorState()
{
  TransportHeader header;

  header.id = MESSAGE_ID_MOTOR_STATE;
  header.flags = 0;
  header.ackId = 0;

  MotorState motor;
  motor.timestampUs = SysTimeLongUSec();
  memset(motor.commadedRpm, 0, sizeof(motor.commadedRpm));
  memcpy(motor.commadedRpm, sdk.cmd.dimc.rpm, sizeof(sdk.cmd.dimc.rpm));
  memset(motor.measuredRpm, 0, sizeof(motor.measuredRpm));
  memcpy(motor.measuredRpm, sdk.ro.motors.speed, sizeof(sdk.ro.motors.speed));

  ExtComSendMessage(&header, &motor, sizeof(MotorState));
}

static void msgGpsData()
{
  TransportHeader header;

  header.id = MESSAGE_ID_GPS_DATA;
  header.flags = 0;
  header.ackId = 0;

  GpsData gps;
  memset(&gps, 0, sizeof(GpsData));
  gps.latitude = sdk.ro.gps.latitude;
  gps.longitude = sdk.ro.gps.longitude;
  gps.height = sdk.ro.height;
  gps.speed.x = sdk.ro.gps.speedEastWest*0.001f;
  gps.speed.y = sdk.ro.gps.speedNorthSouth*0.001f;
  gps.speed.z = sdk.ro.verticalSpeed*0.001f;
  gps.heading = sdk.ro.gps.raw.heading*0.001f;
  gps.horizontalAccuracy = sdk.ro.gps.raw.horizontalAccuracy;
  gps.verticalAccuracy = sdk.ro.gps.raw.verticalAccuracy;
  gps.speedAccuracy = sdk.ro.gps.raw.speedAccuracy;
  gps.numSatellites = sdk.ro.gps.raw.numSatellites;
  gps.status = sdk.ro.gps.raw.hasLock ? 0x03 : 0x00;

  ExtComSendMessage(&header, &gps, sizeof(GpsData));
}

static void msgFilteredSensorData()
{
  TransportHeader header;

  header.id = MESSAGE_ID_FILTERED_SENSOR_DATA;
  header.flags = 0;
  header.ackId = 0;

  FilteredSensorData data;
  data.acc.x = sdk.ro.sensors.acc[0]*0.001f;
  data.acc.y = sdk.ro.sensors.acc[1]*0.001f;
  data.acc.z = sdk.ro.sensors.acc[2]*0.001f;
  data.gyro.x = sdk.ro.attitude.angularVelocity[0]*0.001f*((float)M_PI)/180.0f;
  data.gyro.y = sdk.ro.attitude.angularVelocity[1]*0.001f*((float)M_PI)/180.0f;
  data.gyro.z = sdk.ro.attitude.angularVelocity[2]*0.001f*((float)M_PI)/180.0f;
  data.mag.x = sdk.ro.sensors.mag[0]*48.0f/2500.0f;
  data.mag.y = sdk.ro.sensors.mag[1]*48.0f/2500.0f;
  data.mag.z = sdk.ro.sensors.mag[2]*48.0f/2500.0f;
  data.baroHeight = sdk.ro.height*0.001f;

  ExtComSendMessage(&header, &data, sizeof(FilteredSensorData));
}

void ExtComSpinOnce()
{
  uint8_t data;

  ++extCom.cmdTimeout;

  if(extCom.cmdTimeout > 200)
  {
    extCom.cmdTimeout = 200;

    if(extCom.active)
    {
      sdk.cmd.mode = SDK_CMD_MODE_OFF;
      extCom.active = 0;
    }
  }

  if(extCom.cmdTimeout < 200)
  {
    extCom.active = 1;
  }

  // check RX data
  while(FifoGet(&uart0.rxFifo, &data) == 0)
  {
    if(data == 0)
    {
      // Delimiter found => message complete
      processCompleteMsg(extCom.rxProcBuffer, extCom.rxProcUsed);

      extCom.rxProcUsed = 0;
    }
    else
    {
      extCom.rxProcBuffer[extCom.rxProcUsed++] = data;

      if(extCom.rxProcUsed == EXT_COM_MAX_ENCODED_MSG_SIZE)
      {
        // we just filled the whole rx buffer without a delimiter => discard all data
        extCom.rxProcUsed = 0;
        ++extCom.rxStat.oversized;
      }
    }
  }

  // do regular transmissions
  for(uint16_t i = 0; i < sizeof(wireCfg) / sizeof(wireCfg[0]); i++)
  {
    ++wireCfg[i].cnt;

    if(wireCfg[i].cnt >= wireCfg[i].div && wireCfg[i].div > 0)
    {
      wireCfg[i].cnt = 0;
      (*wireCfg[i].pTxFunc)();
    }
  }
}
