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

#include "util/fifo.h"

#define TERMINAL_INPUT_BUFFER_SIZE 80
#define TERMINAL_OUTPUT_BUFFER_SIZE 200

typedef struct _VT100Result
{
  uint16_t code;
  uint16_t param1;
  uint16_t param2;
} VT100Result;

enum VT100Sequence
{
  CURSOR_UP = 1,
  CURSOR_DOWN,
  CURSOR_FORWARD,
  CURSOR_BACKWARD,
  FUNCTION_KEY,
};

enum VT100FunctionKeys
{
  FK_HOME = 1,
  FK_INSERT,
  FK_DELETE,
  FK_END,
  FK_PAGE_UP,
  FK_PAGE_DOWN,
  FK_F1 = 11,
  FK_F2,
  FK_F3,
  FK_F4,
  FK_F5,
  FK_F6 = 17,
  FK_F7,
  FK_F8,
  FK_F9,
  FK_F10,
  FK_F11 = 23,
  FK_F12,
};

typedef void (*TerminalCmdCb)();
typedef void (*TerminalEscCb)(VT100Result*);

typedef struct _Terminal
{
  char inputBuffer[TERMINAL_INPUT_BUFFER_SIZE];
  char outputBuffer[TERMINAL_OUTPUT_BUFFER_SIZE];
  uint16_t inBufPos;

  uint8_t escapeMode;

  TerminalCmdCb cmdCb;
  TerminalEscCb escCb;

  Fifo* pInFifo;
  Fifo* pOutFifo;
} Terminal;

extern Terminal terminal;

void TerminalInit(Fifo* pInFifo, Fifo* pOutFifo, TerminalCmdCb cmdCb, TerminalEscCb escCb);
void TerminalSpinOnce();
int32_t TerminalPrint(const char* fmt, ...);
int16_t TerminalCmpCmd(const char* cmd);
int16_t TerminalScanCmd(int32_t numArgs, const char* fmt, ...);
