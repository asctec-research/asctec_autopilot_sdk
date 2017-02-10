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

#include "attitude_commands.h"
#include "../sdkio.h"

/**
 * This example shows simple attitude commands.
 * As soon as the serial switch is ON the motors will
 * be set to 20% thrust. Don't worry, this is not enough to take off.
 */
void ExampleRollPitchYawRateThrustCommands(void)
{
  sdk.cmd.mode = SDK_CMD_MODE_RPY_THRUST;

  sdk.cmd.RPYThrust.pitchAngle = 0;
  sdk.cmd.RPYThrust.rollAngle = 0;
  sdk.cmd.RPYThrust.yawRate = 0;
  sdk.cmd.RPYThrust.thrust = 800; // 20% thrust
}
