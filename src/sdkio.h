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

#pragma once

#include <stdint.h>
#include "hal/ublox.h"
#include "ll_hl_comm.h"
#include "asctec_uav_msgs/message_definitions.h"

#define SDK_CMD_MODE_OFF                0
#define SDK_CMD_MODE_DIMC               1
#define SDK_CMD_MODE_RPY_THRUST         2
#define SDK_CMD_MODE_RPY_CLIMBRATE      3
#define SDK_CMD_MODE_LOCAL_VEL_WITH_GPS 4
#define SDK_CMD_MODE_GPS_WAYPOINT_ABS   5
#define SDK_CMD_MODE_GPS_WAYPOINT_REL   6

#define WP_NAVSTAT_REACHED_POS      0x01  // vehicle has entered a radius of WAYPOINT.pos_acc and time to stay is not necessarily over
#define WP_NAVSTAT_REACHED_POS_TIME 0x02  // vehicle is within a radius of WAYPOINT.pos_acc and time to stay is over
#define WP_NAVSTAT_20M              0x04  // vehicle within a 20m radius of the waypoint
#define WP_NAVSTAT_PILOT_ABORT      0x08  // waypoint navigation aborted by safety pilot (any stick was moved)

//Emergency modes (these are the emergency procedures which will automatically be activated if the data link between R/C and UAV is lost).
//Go to the AscTec Wiki for further information about the procedures!
#define EM_SAVE                        0x01 //"direct landing"
#define EM_SAVE_EXTENDED_WAITING_TIME  0x02 //"wait and land"
#define EM_RETURN_AT_PREDEFINED_HEIGHT 0x04 //"come home"
#define EM_RETURN_AT_MISSION_SUMMIT    0x08 //"come home high"

typedef struct _SDKData
{
  struct _READONLY
  {
    // Update rate: 1000Hz
    struct
    {
      //angles derived by integration of gyro_outputs, drift compensated by data fusion; -90000..+90000 pitch(nick) and roll, 0..360000 yaw; 1000 = 1 degree
      union
      {
        int32_t angle[3]; // roll, pitch, yaw [deg*1000]
        struct
        {
          int32_t roll;
          int32_t pitch;
          int32_t yaw;
        };
      };

      //angular velocities, bias free
      union
      {
        int32_t angularVelocity[3]; // [deg/s*1000]
        struct
        {
          int32_t angularVelocityRoll;
          int32_t angularVelocityPitch;
          int32_t angularVelocityYaw;
        };
      };
    } attitude;

    // Update rate of all following data: 333Hz
    struct
    {
      // acc-sensor outputs, calibrated, body frame coordinate system
      int32_t acc[3]; // [m/s^2*1000]
      // magnetic field sensors output, offset free and scaled to +-2500 = +- earth field strength (determined during compass calibration procedure)
      // vehicle frame
      int32_t mag[3];
      uint16_t battery; // [mV]
    } sensors;

    int32_t height; // set to zero when motors are switched on! [mm]
    int32_t verticalSpeed; // differentiated height [mm/s]

    struct
    {
      // RC value range: 0 - 4080
      uint16_t pitch;
      uint16_t roll;
      uint16_t thrust;
      uint16_t yaw;
      uint16_t serialSwitch;
      uint16_t flightMode;
      uint16_t aux;

      uint16_t channels[8];

      uint8_t hasLock; // 1 if link to RC is established, 0 otherwise
    } rc;

    struct
    {
      // this fused data is only available when GPS mode is active!
      int32_t latitude;   // [deg*10^7]
      int32_t longitude;  // [deg*10^7]
      int32_t speedEastWest; // [mm/s]
      int32_t speedNorthSouth; // [mm/s]

      // this raw data is always present
      GPSRawData raw;
    } gps;

    struct
    {
      int16_t speed[6]; // [RPM]
      int16_t pwm[6]; // 0 - 200
    } motors;

    int32_t flightMode; // uses FlightMode from messages packet
    uint8_t serialInterfaceActive;
    uint8_t serialInterfaceReady;
    uint16_t lowLevelError;

    uint16_t cpuLoad; // 0 - 1000, 1000 means CPU usage at maximum

    // Update rate of the following is variable (slow update channel)
    uint8_t isHexcopter;
    uint16_t flightTime;  // [s], ~120Hz
    uint8_t emergencyMode; // ~2Hz

    struct
    {
      uint8_t navStatus; // ~60Hz
      int32_t distanceToWp; // ~60Hz, [mm]
      uint8_t ackTrigger; // set by LL if command is accepted
    } waypoint;
  } ro;

  struct _WRITEONLY
  {
    uint8_t mode;

    struct _DirectIndividualMotorCommands
    {
      // Normal operation are always positive RPM values.
      // RPM can be negative on the Firefly and will reverse motor turning direction.
      int16_t rpm[6]; // 1100 - 8600 on Hummingbird/Pelican, 1250 - 10000 on Firefly
      uint8_t enable[6]; // set to 1 to enable a motor, 0 stops the motor
    } dimc;

    struct _RollPitchYawRateThrust
    {
      int32_t pitchAngle; // [deg*1000], Range: -52,000 to 52,000
      int32_t rollAngle;  // [deg*1000], Range: -52,000 to 52,000
      int32_t yawRate;    // [deg/s*1000], Range: -200,000 to 200,000 (-300,000 to 300,000 for Hummingbird)
      uint32_t thrust;    // 0 to 4095, pseudo unit
    } RPYThrust;

    struct _RollPitchYawRateClimbRate
    {
      int32_t pitchAngle; // [deg*1000], Range: -52,000 to 52,000
      int32_t rollAngle;  // [deg*1000], Range: -52,000 to 52,000
      int32_t yawRate;    // [deg/s*1000], Range: -200,000 to 200,000 (-300,000 to 300,000 for Hummingbird)
      int32_t climbRate;  // [mm/s], Range: -2,000 to 2,000
    } RPYClimbRate;

    struct _LocalVelocityWithGPS
    {
      // TODO: verfiy directions
      int32_t speedForward; // [mm/s], Range: -3,000 to 3,000
      int32_t speedRight;   // [mm/s], Range: -3,000 to 3,000
      int32_t yawRate;      // [deg/s*1000], Range: -200,000 to 200,000 (-300,000 to 300,000 for Hummingbird)
      int32_t climbRate;    // [mm/s], Range: -2,000 to 2,000
    } localVelWithGPS;

    struct _WaypointAbsolute
    {
      int32_t latitude;     // [deg*10^7]
      int32_t longitude;    // [deg*10^7]
      int32_t height;       // [mm] above "motor switch on"-point
      int32_t heading;      // [deg*1000]
      uint8_t maxSpeed;     // in percent => 0..100
      uint32_t timeToStay;  // [ms] Range: 0 to 655,000
      uint16_t reachedTolerance; // [mm] position accuracy to consider a waypoint reached
      uint8_t updated;      // set to 1 when you have set new values
    } wpAbsolute;

    struct _WaypointRelative
    {
      int32_t east;         // displacement to the east [mm]
      int32_t north;        // displacement to the north [mm]
      int32_t height;       // [mm] above "motor switch on"-point
      int32_t heading;      // [deg*1000]
      uint8_t maxSpeed;     // in percent => 0..100
      uint32_t timeToStay;  // [ms] Range: 0 to 655,000
      uint16_t reachedTolerance; // [mm] position accuracy to consider a waypoint reached
      uint8_t updated;      // set to 1 when you have set new values
    } wpRelative;

    uint8_t newEmergencyMode;
  } cmd;
} SDKData;

extern SDKData sdk;

void SDKParseLLData(struct LL_ATTITUDE_DATA* pLL);
void SDKFillLLCommands(struct LL_CONTROL_INPUT* pCtrl);
void SDKSetEmergencyMode(uint8_t mode);
void SDKPrintROData();
