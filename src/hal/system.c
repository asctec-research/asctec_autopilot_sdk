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
#include "system.h"
#include "main.h"
#include "type.h"
#include "irq.h"
#include "hal/ssp.h"
#include "hal/adc.h"
#include "uart0.h"
#include "uart1.h"
#include "sys_time.h"

#define EXT_NCS 7   //CS outputs on P0

static void PLLInit(void);
static void TIM0Init(void);
static void IRQInit(void);
static void PortsInit(void);
static void SPIInit(void);
static void PWMInit(void);

void init(void)
{
  MAMCR = 0x02;  //Memory Acceleration enabled
  MAMTIM = 0x04;
  VPBDIV = 0x01;  //0x01: peripheral frequency == cpu frequency, 0x00: per. freq. = crystal freq.

  PLLInit();
  PortsInit();
//  UART0Initialize(57600);	//debug / command
  UART0Init(921600); //debug / command
  UART1Init(115200);	//57600 Servo / GPS, 38400 "indoor GPS"
  SPIInit();
  SSPInit();
  TIM0Init();
//  I2CInit(I2CMASTER);
  PWMInit();
  ADC0Init();
  SysTimeInit();
  IRQInit();
}

void IRQInit(void)
{
  init_VIC();

  //Timer0 interrupt
  install_irq( TIMER0_INT, (void *)timer0ISR);

  UART1InitIRQ();
  UART0InitIRQ();

  //I2C0 interrupt
//  install_irq( I2C0_INT, (void *) I2C0MasterHandler );
//  I20CONSET = I2CONSET_I2EN;

  SSPInitIRQ();

  SysTimeInitIRQ();
}

void PortsInit(void)
{
  /* PINSEL0
   *
   * PORT0:
   * P0.0: TXD0 -> 01
   * P0.1: RXD0 -> 01
   * P0.2: SCO0 -> 01
   * P0.3: SDA0 -> 01
   * Byte0_sel = 0b01010101 = 0x55
   *
   * SPI0 => power board connection
   * P0.4: SCK0 -> 01
   * P0.5: MISO0 -> 01
   * P0.6: MOSI0 -> 01
   * P0.7: LL_NCS/IO_out -> 00
   * or: PWM2 -> 10
   * Byte1_sel = 0x00010101 = 0x15
   * Byte0_io_dir = 0x80
   *
   * P0.8: TXD1 -> 01
   * P0.9: RXD1 -> 01
   * P0.10: IO_in -> 00
   * P0.11: SCL1 -> 11
   * or Falcon8: IO_out -> 00
   * Byte2_sel = 0b11000101 = 0xC5
   *
   * P0.12: IO_in -> 00
   * P0.13: IO_in -> 00
   * P0.14: SDA1 -> 11
   * or IO_out (CS SD-Card) => SD_Logging
   * P0.15: IO_in -> 00
   * Byte3_sel = 0b00110000 = 0x30
   * Byte1_io_dir = 0x00
   * or SD_Logging => Byte1_io_dir=0x40
   */

  PINSEL0 = 0x30C51555;

  /* PINSEL1
   *
   * P0.16: IO_in -> 00
   * P0.17: SCK1 -> 10
   * P0.18: MISO1 -> 10
   * P0.19: MOSI1-> 10
   * Byte0: 0b10101000 = 0xA8
   *
   * P0.20: SSEL1 -> 10
   * P0.21: PWM5 -> 01
   * P0.22: IO_in -> 00
   * P0.23: IO_in -> 00
   * Byte1: 0b00000110 = 0x06
   * Byte2_io_dir: 0x30 //0x11
   *
   * P0.24: 00
   * P0.25: VOLTAGE_2: -> 01
   * or IO_in (FALCON) -> 00
   * P0.26: 00
   * P0.27: 00
   * Byte2: 0b00000100 = 0x04
   *
   * P0.28: CURRENT_2: -> 01
   * P0.29: VOLTAGE_1: -> 01
   * P0.30: CURRENT_1: -> 01
   * P0.31: IO_in -> 00
   * Byte3: 0b00010101 = 0x15
   * Byte3_io_dir=0x00
   */
  PINSEL1 = 0x150406A8;

  PINSEL2 = 0x00000004;

  IODIR0 = 0x0030B480;
  IODIR0 |= (1 << 22); // CTS pin of UART (P0.22)

  IOSET0 = (1 << EXT_NCS) | (1 << 11); //all nCS high
  IOSET0 = (1 << 22);
  //IOSET0 = (1<<LL_nCS);	//CS LL_Controller

  /* P1.16: IO_1/IO_out	=> FET for camera power supply
   * P1.17: Beeper/IO_out
   * .
   * .
   * P1.24: LED1/IO_out
   * P1.25: LED2/IO_out
   *
   */

  IODIR1 = 0x03030000;
  IOSET1 = ((1 << 24) | (1 << 16)); //turn off LED1, turn beeper off
}

void TIM0Init(void)
{
  T0TC = 0;
  T0TCR = 0x0;    //Reset timer0
  T0MCR = 0x3;    //Interrupt on match MR0 and reset counter
  T0PR = 0;
  T0PC = 0;     //Prescale Counter = 0
  T0MR0 = peripheralClockFrequency() / ControllerCyclesPerSecond - 1; // /200 => 200 Hz Period
  T0TCR = 0x1;   //Set timer0
}

void PWMInit(void)
{
  //  match_counter = 0;
  //  PINSEL0 = 0x000A800A;	/* set GPIOs for all PWMs */
  //  PINSEL1 = 0x00000400;
  PWMTCR = TCR_RESET; /* Counter Reset */

  PWMPR = 0x00; /* count frequency:Fpclk */
  PWMMCR = PWMMR0R; /* interrupt on PWMMR0, reset on PWMMR0, reset
   TC if PWM0 matches */
  PWMMR0 = 1179648;
  PWMMR5 = 88470;

  /* all PWM latch enabled */
  PWMLER = LER5_EN;

  /* All single edge, all enable */
  PWMPCR = PWMENA1 | PWMENA2 | PWMENA3 | PWMENA4 | PWMENA5 | PWMENA6;
  PWMTCR = TCR_CNT_EN | TCR_PWM_EN; /* counter enable, PWM enable */
}

void SPIInit(void)
{
  S0SPCCR = 0x04; //30 clock-cycles (~60MHz) = 1 SPI cycle => SPI @ 2MHz
  S0SPCR = 0x20;  //LPC is Master
}

void PLLInit(void)
{
  PLLCFG = 0x23;    //0b00100011; => M=4,0690; P=2;
  PLLCON = 0x03;    //PLLE=1, PLLC=1 => PLL enabled as system clock

  PLLFEED = 0xAA;
  PLLFEED = 0x55;
}

unsigned int processorClockFrequency(void)
{
  return 58982400;
}

unsigned int peripheralClockFrequency(void)
{
  unsigned int divider = 1;

  switch(VPBDIV & 3)
  {
    case 0:
      divider = 4;
      break;
    case 1:
      divider = 1;
      break;
    case 2:
      divider = 2;
      break;
  }

  return processorClockFrequency() / divider;
}
