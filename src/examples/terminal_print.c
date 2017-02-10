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

#include "terminal_print.h"
#include "../sdkio.h"

/**
 * This example simply prints out all information in the sdk.ro.* fields
 * in a human readable format every 100 calls (=10Hz).
 *
 * For the printing to work, UART0 has to be configured in terminal mode.
 * To do so, please open config.h and uncomment the line:
 * #define UART0_FUNCTION UART0_FUNCTION_TERMINAL
 */
void ExampleRegularTerminalPrint()
{
  static uint32_t cnt = 0;

  ++cnt;
  if(cnt == 100) // print out all SDK read-only data every 100ms
  {
    cnt = 0;

    SDKPrintROData();
  }
}
