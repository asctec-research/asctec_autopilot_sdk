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

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

typedef struct _VT100Sequence
{
  const char* pSeq;
  uint16_t code;
  uint16_t default1;
  uint16_t default2;
} VT100Sequence;

static VT100Sequence sequences[] = {
  { "[\1A", CURSOR_UP, 1, 0 },
  { "[\1B", CURSOR_DOWN, 1, 0 },
  { "[\1C", CURSOR_FORWARD, 1, 0 },
  { "[\1D", CURSOR_BACKWARD, 1, 0 },
  { "[\1~", FUNCTION_KEY, 0xFF, 0 },
};

Terminal terminal;

void TerminalInit(Fifo* pInFifo, Fifo* pOutFifo, TerminalCmdCb cmdCb, TerminalEscCb escCb)
{
  terminal.pInFifo = pInFifo;
  terminal.pOutFifo = pOutFifo;
  terminal.cmdCb = cmdCb;
  terminal.escCb = escCb;
}

static VT100Result compareSeq(char* pIn, VT100Sequence* pVT100Seq)
{
  const char* pSeq = pVT100Seq->pSeq;
  VT100Result result;
  result.code = 0;
  result.param1 = 0xFFFF;
  result.param2 = 0xFFFF;

  while(1)
  {
    if(*pIn == 0 && *pSeq == 0)
    {
      result.code = pVT100Seq->code;
      return result;
    }

    if(*pIn == 0)
      break;

    if(*pSeq == 0)
      break;

    if(*pSeq == 1)	// (optional) parameter 1 to read
    {
      char* pNewIn;

      result.param1 = strtol(pIn, &pNewIn, 10);
      if(pIn == pNewIn)
      {
        // parameter not found
        result.param1 = pVT100Seq->default1;
      }

      pIn = pNewIn;
    }
    else if(*pSeq == 2)	// (optional) parameter 2 to read
    {
      char* pNewIn;

      result.param2 = strtol(pIn, &pNewIn, 10);
      if(pIn == pNewIn)
      {
        // parameter not found
        result.param2 = pVT100Seq->default2;
      }

      pIn = pNewIn;
    }
    else
    {
      if(*pSeq != *pIn)
        return result;

      ++pIn;
    }

    ++pSeq;
  }

  return result;
}

static uint8_t findEscapeSequence(VT100Result* pResult)
{
  VT100Result result;
  uint16_t numSequences = sizeof(sequences) / sizeof(sequences[0]);

  for(uint16_t i = 0; i < numSequences; i++)
  {
    result = compareSeq(terminal.inputBuffer, sequences + i);
    if(result.code != 0)
    {
      *pResult = result;
      return 1;
    }
  }

  return 0;
}

void TerminalSpinOnce()
{
  uint8_t data;
  VT100Result vtResult;

  if(FifoGet(terminal.pInFifo, &data) < 0)
    return;

  if(data > 31 && data < 127)	// readable ASCII char?
  {
    if(terminal.inBufPos < TERMINAL_INPUT_BUFFER_SIZE)
    {
      terminal.inputBuffer[terminal.inBufPos++] = data;
    }

    if(terminal.escapeMode)
    {
      if(findEscapeSequence(&vtResult))
      {
        // sequence complete
        if(terminal.escCb)
          (*terminal.escCb)(&vtResult);

        terminal.inBufPos = 0;
        memset(terminal.inputBuffer, 0, TERMINAL_INPUT_BUFFER_SIZE);

        terminal.escapeMode = 0;
      }
    }
    else
    {
      FifoPut(terminal.pOutFifo, data);
    }
  }

  if(data == 127)	// backspace
  {
    if(terminal.inBufPos > 0)
    {
      terminal.inputBuffer[terminal.inBufPos] = 0;
      --terminal.inBufPos;
      FifoPut(terminal.pOutFifo, data);
    }
  }

  if(data == '\n' || data == '\r')
  {
    FifoPut(terminal.pOutFifo, '\r');
    FifoPut(terminal.pOutFifo, '\n');

    // command complete, process
    if(terminal.cmdCb)
      (*terminal.cmdCb)();

    terminal.inBufPos = 0;
    memset(terminal.inputBuffer, 0, TERMINAL_INPUT_BUFFER_SIZE);

    terminal.escapeMode = 0;
  }

  if(data == 27)	// escape character?
  {
    terminal.inBufPos = 0;
    memset(terminal.inputBuffer, 0, TERMINAL_INPUT_BUFFER_SIZE);

    terminal.escapeMode = 1;
  }
}

int32_t TerminalPrint(const char* fmt, ...)
{
  va_list args;
  int32_t bytesWritten;

  va_start(args, fmt);
  bytesWritten = vsnprintf(terminal.outputBuffer, TERMINAL_OUTPUT_BUFFER_SIZE, fmt, args);
  va_end(args);

  for(int32_t i = 0; i < bytesWritten; i++)
  {
    FifoPut(terminal.pOutFifo, terminal.outputBuffer[i]);
  }

  return bytesWritten;
}

int16_t TerminalCmpCmd(const char* cmd)
{
  int16_t result = 0;

  if(strcmp(terminal.inputBuffer, cmd) == 0)
    result = 1;

  return result;
}

int16_t TerminalScanCmd(int32_t numArgs, const char* fmt, ...)
{
  va_list args;
  int32_t storedFields = 0;

  va_start(args, fmt);
  storedFields = vsscanf(terminal.inputBuffer, fmt, args);
  va_end(args);

  if(storedFields == numArgs)
    return 1;

  return 0;
}
