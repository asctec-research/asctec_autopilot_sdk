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

#include "motor_commands.h"
#include "../sdkio.h"

/*
 * The following example shows direct motor command usage by mapping
 * the thrust stick directly to motor outputs.
 * Warning: Do NOT try to fly!!!
 */
void ExampleDirectIndividualMotorCommands(void)
{
  sdk.cmd.mode = SDK_CMD_MODE_DIMC;

  for(uint8_t i = 0; i < 6; i++)
  {
    // RPM scaled to 1500 - 5596
    sdk.cmd.dimc.rpm[i] = 1500 + sdk.ro.rc.thrust;
    sdk.cmd.dimc.enable[i] = 1;
  }

  // disable motor 0 when the AUX channel from the RC is over 2500
  if(sdk.ro.rc.aux > 2500)
  {
    sdk.cmd.dimc.enable[0] = 0;
  }
  else
  {
    sdk.cmd.dimc.enable[0] = 1;
  }
}
