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
#include "util/fifo.h"

#define UART0_BUFFER_SIZE 1024

typedef struct _UART0Data
{
  uint8_t txBuf[UART0_BUFFER_SIZE];
  uint8_t rxBuf[UART0_BUFFER_SIZE];

  Fifo txFifo;
  Fifo rxFifo;
} UART0Data;

extern UART0Data uart0;

void UART0Init(uint32_t baud);
void UART0InitIRQ();
void UART0SpinOnce();
void UART0WriteChar(uint8_t ch);
uint8_t UART0ReadChar(void);
