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

#include "sdk.h"
#include "sdkio.h"
#include "sdk_telemetry.h"
#include "util/gpsmath.h"
#include "config.h"
#include "terminal.h"
#include "hal/sys_time.h"

#include "examples/motor_commands.h"
#include "examples/attitude_commands.h"
#include "examples/gps_waypoints.h"
#include "examples/motor_on_off.h"
#include "examples/terminal_print.h"

/******** SDK in general ************
 * You can find further information about the AscTec SDK in our AscTec Wiki: http://wiki.asctec.de
 *
 * SDKMainloop() is triggered @ 1kHz.
 *
 * All user-accessable data resides in the global "sdk" structure (defined in sdkio.h)
 *
 * sdk.ro.* fields are read-only for user access. They are updated regularly from the
 * Low-Level processor and other sensors. Not all sensors are updated at 1kHz.
 *
 * sdk.cmd.* fields are the primary input for user control of the UAV via the High-Level processor.
 */

void SDKInit(void)
{
  // Implement initialization of your variables or commands here.
}

void SDKMainloop(void)
{
  //write your own C-code within this function

  //you can select an example by using ONE of the functions below.
  //CAUTION! Read the code of the examples before you test them on your UAV!
//  ExampleDirectIndividualMotorCommands(); // checked
//  ExampleRollPitchYawRateThrustCommands(); // checked
//  ExampleMotorsOnOff();  // checked
  ExampleRegularTerminalPrint(); // checked

  ExampleGPSWaypointControl();

  //jeti telemetry can always be activated. You may deactivate this call if you don't use the AscTec Telemetry package.
  SDK_jetiAscTecExampleRun();
}

// Custom messages, that are not handled internally are passed on to this function
void SDKProcessUserMsg(const TransportHeader* pHeader, const uint8_t* pData, uint32_t dataSize)
{
  (void)pHeader;
  (void)pData;
  (void)dataSize;
}

