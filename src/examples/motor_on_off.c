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

#include "motor_on_off.h"
#include "../sdkio.h"

/**
 * This example shows you how to turn on and off the motors via SDK code.
 * This is especially useful if you want to implement some auto-takeoff or landing feature.
 *
 * Motors are enabled exactly the same way you would do with the RC. A minimum thrust and
 * full negaitve yaw will start the motors. The same with full positive yaw will stop them.
 * There must be a neutral command after motors started/stopped. Going from full negative to
 * full positive yaw will not work.
 *
 * Warning: Motors will turn automatically with this example as soon as the serial switch is ON!
 */
void ExampleMotorsOnOff()
{
  static uint8_t state = 0;
  static uint16_t timer;

  sdk.cmd.mode = SDK_CMD_MODE_RPY_THRUST;

  switch(state)
  {
    case 0: // start motors, minimum thrust + full yaw
    {
      sdk.cmd.RPYThrust.pitchAngle = 0;
      sdk.cmd.RPYThrust.rollAngle = 0;
#if VEHICLE_TYPE == VEHICLE_TYPE_HUMMINGBIRD
      sdk.cmd.RPYThrust.yawRate = -299999;
#else
      sdk.cmd.RPYThrust.yawRate = -199999;
#endif
      sdk.cmd.RPYThrust.thrust = 0;

      state++;
    }
    break;
    case 1: // check if all motors are spinning
    {
      uint8_t numMotors = 4;
#if VEHICLE_TYPE == VT_FIREFLY
      numMotors = 6;
#endif

      uint8_t allRunning = 1;
      for(uint8_t i = 0; i < numMotors; i++)
      {
        if(sdk.ro.motors.speed[i] < 1000)
        {
          allRunning = 0;
          break;
        }
      }

      if(allRunning)
      {
        // reset thrust + yaw when motors are spinning
        sdk.cmd.RPYThrust.pitchAngle = 0;
        sdk.cmd.RPYThrust.rollAngle = 0;
        sdk.cmd.RPYThrust.yawRate = 0;
        sdk.cmd.RPYThrust.thrust = 0;

        timer = 0;
        state++;
      }
    }
    break;
    case 2: // wait 2s
    {
      timer++;
      if(timer > 2000)
        state++;
    }
    break;
    case 3: // turn off motors
    {
      sdk.cmd.RPYThrust.pitchAngle = 0;
      sdk.cmd.RPYThrust.rollAngle = 0;
#if VEHICLE_TYPE == VEHICLE_TYPE_HUMMINGBIRD
      sdk.cmd.RPYThrust.yawRate = 299999;
#else
      sdk.cmd.RPYThrust.yawRate = 199999;
#endif
      sdk.cmd.RPYThrust.thrust = 0;

      timer = 0;
      state++;
    }
    break;
    case 4: // wait 500ms, then set yaw to zero
    {
      timer++;
      if(timer > 500)
      {
        sdk.cmd.RPYThrust.pitchAngle = 0;
        sdk.cmd.RPYThrust.rollAngle = 0;
        sdk.cmd.RPYThrust.yawRate = 0;
        sdk.cmd.RPYThrust.thrust = 0;

        timer = 0;
        state++;
      }
    }
    break;
    case 5: // wait 4s, motor rotation will decay
    {
      timer++;
      if(timer > 4000)
        state = 0;
    }
    break;
  }
}
