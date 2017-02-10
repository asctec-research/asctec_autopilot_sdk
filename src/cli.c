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

#include "terminal.h"

#include "hal/buzzer.h"
#include "sdkio.h"
#include <string.h>
#include <inttypes.h>

void CLIEscCallback(VT100Result* pResult)
{
  switch(pResult->code)
  {
    case CURSOR_UP:
    {
      TerminalPrint("CURSOR_UP\r\n");
    }
      break;
    case CURSOR_DOWN:
    {
      TerminalPrint("CURSOR_DOWN\r\n");
    }
      break;
    case CURSOR_BACKWARD:
    {
      TerminalPrint("CURSOR_BACKWARD\r\n");
    }
      break;
    case CURSOR_FORWARD:
    {
      TerminalPrint("CURSOR_FORWARD\r\n");
    }
      break;
    case FUNCTION_KEY:
    {
      TerminalPrint("FUNCTION_KEY: %hu\r\n", pResult->param1);

      if(pResult->param1 == 11) // F1
      {
        SDKPrintROData();
      }

      if(pResult->param1 == 12) // F2
      {
      }
    }
      break;
  }
}

void CLICmdCallback()
{
  if(TerminalCmpCmd("version"))
  {
    TerminalPrint("SDK Version: 4.0.0\r\n");
  }

  if(TerminalCmpCmd("hello"))
  {
    TerminalPrint("Hello World!\r\n");
  }

  uint16_t u16;
  if(TerminalScanCmd(1, "test %hu", &u16))
  {
    TerminalPrint("Number: %hu\r\n", u16);
  }

  if(TerminalCmpCmd("time"))
  {
//    int64_t time = SysTimeLongUSec();
//    TerminalPrint("SysTime: %u\r\n", SysTimeUSec());
//    TerminalPrint("SysTimeLong: %u%010u\r\n", (uint32_t)(time >> 32), (uint32_t)(time & 0xFFFFFFFF));
  }
}
