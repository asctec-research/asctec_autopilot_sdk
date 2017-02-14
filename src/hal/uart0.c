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

#include "uart0.h"

#include "LPC214x.h"
#include "irq.h"
#include "system.h"

UART0Data uart0;

static void uart0IRQ(void) __irq
{
  // Read IIR to clear interrupt and find out the cause
  IENABLE;
  uint32_t iir = U0IIR;

  // Handle UART interrupt
  switch((iir >> 1) & 0x7)
  {
    case 1:
    {
      // THRE interrupt
      uint16_t txBytes = FifoBytesUsed(&uart0.txFifo);
      if(txBytes > 16)
        txBytes = 16;

      for(uint8_t i = 0; i < txBytes; i++)
      {
        uint8_t c;
        FifoGet(&uart0.txFifo, &c);
        U0THR = c;
      }

      if(txBytes < 2)
        U0IER &= ~0x02; // disable THRE IRQ
    }
    break;
    case 2:
    {
      // RDA interrupt
      uint8_t c = U0RBR;
      FifoPut(&uart0.rxFifo, c);
    }
    break;
    case 3:
      // RLS interrupt (disabled)
      break;
    case 6:
      // CTI interrupt (disabled)
      break;
  }
  IDISABLE;
  VICVectAddr = 0;    // Acknowledge Interrupt
}

void UART0Init(uint32_t baud)
{
  FifoInit(&uart0.txFifo, uart0.txBuf, UART0_BUFFER_SIZE);
  FifoInit(&uart0.rxFifo, uart0.rxBuf, UART0_BUFFER_SIZE);

  uint32_t divisor = peripheralClockFrequency() / (16 * baud);

  //UART0
  U0LCR = 0x83; /* 8 bit, 1 stop bit, no parity, enable DLAB */
  U0DLL = divisor & 0xFF;
  U0DLM = (divisor >> 8) & 0xFF;
  U0LCR &= ~0x80; /* Disable DLAB */
  U0FCR = 1;
}

void UART0InitIRQ()
{
  install_irq(UART0_INT, (void *)uart0IRQ);
  U0IER = 1; // enable THRE and RX interrupt
}

//Write to UART0
void UART0WriteChar(uint8_t ch)
{
  FifoPut(&uart0.txFifo, ch);
}

uint8_t UART0ReadChar(void)
{
  while((U0LSR & 0x01) == 0);
  return U0RBR;
}

void UART0SpinOnce()
{
  if(U0LSR & 0x40) // check TEMT
  {
    // Transmitter and shift register empty
    uint16_t txBytes = FifoBytesUsed(&uart0.txFifo);
    if(txBytes > 16)
      txBytes = 16;

    for(uint8_t i = 0; i < txBytes; i++)
    {
      uint8_t c;
      FifoGet(&uart0.txFifo, &c);
      U0THR = c;
    }

    if(txBytes > 1)
      U0IER |= 0x02; // enable THRE IRQ
  }
}
