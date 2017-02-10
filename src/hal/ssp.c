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

#include "LPC214x.h"			/* LPC21XX Peripheral Registers	*/
#include "irq.h"
#include "ssp.h"

#include "../ll_hl_comm.h"

/* SPI read and write buffer size */
#define FIFOSIZE  8

/* SPI Status register */
#define SSPSR_TFE 1 << 0
#define SSPSR_TNF 1 << 1
#define SSPSR_RNE 1 << 2
#define SSPSR_RFF 1 << 3
#define SSPSR_BSY 1 << 4

/* SPI 1 CR0 register */
#define SSPCR0_DSS  1 << 0
#define SSPCR0_FRF  1 << 4
#define SSPCR0_CPOL 1 << 6
#define SSPCR0_CPHA 1 << 7
#define SSPCR0_SCR  1 << 8

/* SPI 1 CR1 register */
#define SSPCR1_LBM  1 << 0
#define SSPCR1_SSE  1 << 1
#define SSPCR1_MS 1 << 2
#define SSPCR1_SOD  1 << 3

/* SPI 1 Interrupt Mask Set/Clear register */
#define SSPIMSC_RORIM 1 << 0
#define SSPIMSC_RTIM  1 << 1
#define SSPIMSC_RXIM  1 << 2
#define SSPIMSC_TXIM  1 << 3

/* SPI 1 Interrupt Status register */
#define SSPRIS_RORRIS 1 << 0
#define SSPRIS_RTRIS  1 << 1
#define SSPRIS_RXRIS  1 << 2
#define SSPRIS_TXRIS  1 << 3

/* SPI 1 Masked Interrupt register */
#define SSPMIS_RORMIS 1 << 0
#define SSPMIS_RTMIS  1 << 1
#define SSPMIS_RXMIS  1 << 2
#define SSPMIS_TXMIS  1 << 3

/* SPI 1 Interrupt clear register */
#define SSPICR_RORIC  1 << 0
#define SSPICR_RTIC 1 << 1

static char SPIWRData[128];
static int CurrentTxIndex;
static unsigned int SPIWR_num_bytes;

volatile uint8_t SSPDataSentToLL = 1;

void SSPHandler() __irq
{
  int regValue;
  unsigned short input_data;

  IENABLE; /* handles nested interrupt */

  regValue = SSPMIS;
  if(regValue & SSPMIS_RORMIS) /* Receive overrun interrupt */
  {
    SSPICR = SSPICR_RORIC; /* clear interrupt */
  }
  if(regValue & SSPMIS_RTMIS) /* Receive timeout interrupt */
  {
    SSPICR = SSPICR_RTIC; /* clear interrupt */
  }

  if(regValue & SSPMIS_RXMIS) /* Rx at least half full */
  {
    /* receive until it's empty */
    while( SSPSR & SSPSR_RNE)
    {
      input_data = SSPDR;

      SSP_rx_handler_HL(input_data & 0xFF);
      SSP_rx_handler_HL(input_data >> 8);

    }
  }

  if(regValue & SSPMIS_TXMIS) /* Tx at least half empty */
  {
    /* transmit until it's full */
    while((SSPSR & SSPSR_TNF))
    {
      if(CurrentTxIndex < SPIWR_num_bytes)
      {
        SSPDR = SPIWRData[CurrentTxIndex] | (SPIWRData[CurrentTxIndex + 1] << 8);
        CurrentTxIndex += 2;
      }
      else
      {
        CurrentTxIndex = 0;
        SPIWR_num_bytes = 0;
        SSPDataSentToLL = 1;
        SSPDR = 0;
      }
    }
  }

  IDISABLE;
  VICVectAddr = 0; /* Acknowledge Interrupt */
}

void SSPInit()
{
  /* Set DSS data to 8-bit, Frame format SPI, CPOL = 0, CPHA = 0, and SCR is 3 */
  SSPCR0 = 0x040F;

  /* SSPCPSR clock prescale register, master mode, minimum divisor is 0x02 */
  SSPCPSR = 0x1B;

  for(uint8_t i = 0; i < FIFOSIZE; i++)
  {
    SSPDR; /* clear the RxFIFO */
  }

  /*all ints deactivated*/
  SSPIMSC = 0;

  /* Device select as master, SSP Enabled */
  SSPCR1 = 0x00;  // | SSPCR1_SSE;
}

void SSPInitIRQ()
{
  //SSP interrupt
  install_irq( SPI1_INT, (void *)SSPHandler);
  /* Set SSPINMS registers to enable interrupts */
  /* enable all interrupts, Rx overrun, Rx timeout, RX FIFO half full int,
   TX FIFO half empty int */
  SSPIMSC = SSPIMSC_TXIM | SSPIMSC_RXIM | SSPIMSC_RORIM; // | SSPIMSC_RTIM;
  /* SSP Enabled */
  SSPCR1 |= SSPCR1_SSE;
}

void SSPWriteToLL(uint8_t page, uint8_t* dataptr)
{
  uint16_t i;
  uint16_t spi_chksum;

  //initialize syncbytes
  SPIWRData[0] = '>';
  SPIWRData[1] = '*';

  spi_chksum = 0xAAAA;

  if(!page)
  {
    for(i = 2; i < 40; i++)
    {
      SPIWRData[i] = *dataptr++;
      spi_chksum += SPIWRData[i];
    }
  }
  else
  {
    for(i = 2; i < 22; i++)
    {
      SPIWRData[i] = *dataptr++;
      spi_chksum += SPIWRData[i];
    }
    dataptr += 18;
    for(i = 22; i < 40; i++)
    {
      SPIWRData[i] = *dataptr++;
      spi_chksum += SPIWRData[i];
    }
  }

  SPIWRData[40] = spi_chksum; //chksum LSB
  SPIWRData[41] = (spi_chksum >> 8); //chksum MSB

  SPIWR_num_bytes = 42;
  SSPDataSentToLL = 0;
}
