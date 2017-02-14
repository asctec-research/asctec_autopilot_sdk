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

#include "LPC214x.h"
#include "hal/system.h"
#include "main.h"
#include "irq.h"
#include "util/gpsmath.h"
#include "ublox.h"
#include "string.h"
#include "../sdkio.h"
#include "uart1.h"

#define RBREAD 0
#define RBWRITE 1
#define RBFREE  2

static void uart1IRQ(void) __irq
{
  IENABLE;
  uint32_t iir = U1IIR;
  // Handle UART interrupt
  switch((iir >> 1) & 0x7)
  {
    case 1:
      // THRE interrupt (disabled)
      break;
    case 2:
      // RX interrupt
      uBloxReceiveHandler(U1RBR);
      break;
    case 3:
      // RLS interrupt (disabled)
      break;
    case 6:
      // CTI interrupt (disabled)
      break;
  }
  IDISABLE;
  VICVectAddr = 0; // Acknowledge Interrupt
}

void UART1Init(uint32_t baud)
{
  uint32_t divisor = peripheralClockFrequency() / (16 * baud);

  U1LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U1DLL = divisor & 0xFF;
  U1DLM = (divisor >> 8) & 0xFF;
  U1LCR &= ~0x80; /* Disable DLAB */
  U1FCR = 1;
}

void UART1InitIRQ()
{
  install_irq( UART1_INT, (void *)uart1IRQ);
  U1IER = 1; // enable RX data available interrupt
}

void UART1WriteChar(uint8_t ch)
{
  while((U1LSR & 0x20) == 0);
  U1THR = ch;
}

uint8_t UART1ReadChar(void)
{
  while((U1LSR & 0x01) == 0);
  return U1RBR;
}

void UART1Send(uint8_t* pBuffer, uint16_t length)
{
  uint16_t cnt = 0;

  while(length--)
  {
    while(!(U1LSR & 0x20)); //wait until U1THR is empty
    U1THR = pBuffer[cnt++];
  }
}
