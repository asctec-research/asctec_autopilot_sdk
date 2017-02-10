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

#include "sys_time.h"
#include "../LPC214x.h"
#include "../win_arm/irq.h"

volatile int64_t sysTimeLong;

static void timer1IRQ(void) __irq
{
  T1IR = 0x01;      //Clear the timer 1 interrupt
  IENABLE;

  sysTimeLong += 1000000;

  IDISABLE;
  VICVectAddr = 0;    // Acknowledge Interrupt
}

void SysTimeInit()
{
  T1TC = 0;
  T1TCR = 0x0;    //Reset timer0
  T1MCR = 0x3;    //Interrupt on match MR0 and reset counter
  T1PR = 0;
  T1PC = 0;     //Prescale Counter = 0
  T1MR0 = CPU_CLOCK_HZ-1; // 1Hz Period
  T1TCR = 0x1;   //Set timer1
}

void SysTimeInitIRQ()
{
  install_irq(TIMER1_INT, (void *)timer1IRQ);
}

int64_t SysTimeLongUSec()
{
  int64_t time = sysTimeLong;
  int64_t timer = T1TC;

  return time + (timer*1000000)/CPU_CLOCK_HZ;
}
