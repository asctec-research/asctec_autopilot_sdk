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
#include "main.h"
#include "hal/system.h"
#include "irq.h"
#include "hal/i2c1.h"
#include "util/gpsmath.h"
#include "hal/adc.h"
#include "hal/buzzer.h"
#include "hal/ublox.h"
#include "hal/pelican_ptu.h"
#include "util/declination.h"
#include "fastmath.h"
#include "hal/led.h"
#include "hal/uart0.h"
#include "terminal.h"
#include "cli.h"
#include "ext_com.h"
#include "hal/sys_time.h"
#include <string.h>

#include "hal/uart1.h"
#include "hal/uart1.h"
#include "ll_hl_comm.h"
#include "sdk.h"
#include "sdkio.h"
#include "util/build_info.h"

struct HL_STATUS HL_Status;

volatile unsigned int GPS_timeout = 0;
volatile char SYSTEM_initialized = 0;

static volatile uint8_t mainloopTrigger = 0;

void timer0ISR(void) __irq
{
  T0IR = 0x01;      //Clear the timer 0 interrupt
  IENABLE;

  mainloopTrigger = 1;

  IDISABLE;
  VICVectAddr = 0;		// Acknowledge Interrupt
}

int main(void)
{
  uint32_t vbat1 = 12000; //battery_voltage (lowpass-filtered)
  uint8_t cnt100Hz = 0;

  init();
  BuzzerEnable(0);

  TerminalInit(&uart0.rxFifo, &uart0.txFifo, &CLICmdCallback, &CLIEscCallback);

  //initialize AscTec Firefly LED fin on I2C1 (not necessary on AscTec Hummingbird or Pelican)
  I2C1Init();
  I2C1SetRGBLed(255, 0, 0);

  generateBuildInfo();

  LEDGreenEnable(1);

  PTU_init();	//initialize camera PanTiltUnit

  mainloopTrigger = 0;
  while(mainloopTrigger == 0)
    asm volatile("nop");

  uint32_t maxIdleIncrements = 0;
  mainloopTrigger = 0;

  while(mainloopTrigger == 0)
  {
    ++maxIdleIncrements;
  }

  SDKInit();

  uint32_t idleIncrements = 0;

  while(1)
  {
    // triggered at 1kHz
    if(mainloopTrigger)
    {
      mainloopTrigger = 0;

      ++cnt100Hz;

      if(GPS_timeout < ControllerCyclesPerSecond)
      {
        GPS_timeout++;
      }
      else if(GPS_timeout == ControllerCyclesPerSecond)
      {
        GPS_timeout = ControllerCyclesPerSecond + 1;
        gps.data.hasLock = 0;
        gps.data.numSatellites = 0;
        sdk.ro.gps.raw.hasLock = 0;
        sdk.ro.gps.raw.numSatellites = 0;
      }

      //battery monitoring
      uint32_t batRaw = ADC0GetRawValue(ADC0_CH_BAT);
      vbat1 = (vbat1 * 14 + (batRaw * 9872 / 579)) / 15;	//voltage in mV

      HL_Status.battery_voltage_1 = vbat1;

      mainloop();

      // triggered at 100Hz
      if(cnt100Hz == 10)
      {
        cnt100Hz = 0;

        BuzzerHandler(HL_Status.battery_voltage_1, gps.data.hasLock,
            sdk.ro.rc.flightMode > 3500, sdk.ro.lowLevelError);
      }

      uint32_t newLoad = 1000-(idleIncrements*1000)/maxIdleIncrements;
      uint32_t prevLoad = HL_Status.cpu_load;

      HL_Status.cpu_load = (10*newLoad + 990*prevLoad)/1000;
//      HL_Status.cpu_load = newLoad;

      idleIncrements = 0;
    }
    else
    {
      ++idleIncrements;
    }
  }

  return 0;
}

void mainloop() //mainloop is triggered at 1 kHz
{
  static unsigned char led_cnt = 0, led_state = 1;
  static int Firefly_led_fin_cnt = 0;

  //blink red led if no GPS lock available
  led_cnt++;
  if(gps.data.hasLock)
  {
    LEDRedEnable(0);
  }
  else
  {
    if(led_cnt == 150)
    {
      LEDRedEnable(1);
    }
    else if(led_cnt == 200)
    {
      led_cnt = 0;
      LEDRedEnable(0);
    }
  }

  //after first lock, determine magnetic inclination and declination
  if(SYSTEM_initialized)
  {
    if((!declinationAvailable) && (gps.data.horizontalAccuracy < 10000) && gps.data.hasLock
        && (gps.dataUpdated)) //make sure GPS lock is valid
    {
      int status;
      estimatedDeclination = getDeclination(gps.data.latitude, gps.data.longitude, gps.data.height / 1000, 2014, &status);
      if(estimatedDeclination < -32000)
        estimatedDeclination = -32000;
      if(estimatedDeclination > 32000)
        estimatedDeclination = 32000;
      declinationAvailable = 1;
    }
  }

  //toggle green LED and update SDK input struct when GPS data packet is received
  if(gps.dataUpdated)
  {
    if(led_state)
    {
      led_state = 0;
      LEDGreenEnable(0);
    }
    else
    {
      LEDGreenEnable(1);
      led_state = 1;
    }

    memcpy(&sdk.ro.gps.raw, &gps.data, sizeof(GPSRawData));

    gps.dataUpdated = 0;
  }

  //handle gps data reception
  uBloxReceiveEngine();

  //run SDK mainloop. Please put all your data handling / controller code in sdk.c
  SDKMainloop();

  //write data to transmit buffer for immediate transfer to LL processor
  HL2LL_write_cycle();

  //control pan-tilt-unit ("cam option 4" @ AscTec Pelican and AscTec Firefly)
  PTU_update();

#if UART0_FUNCTION == UART0_FUNCTION_TERMINAL
  TerminalSpinOnce();
#else
  ExtComSpinOnce();
#endif

  UART0SpinOnce();

  //Firefly LED
  if(SYSTEM_initialized && sdk.ro.isHexcopter)
  {
    if(++Firefly_led_fin_cnt == 10)
    {
      Firefly_led_fin_cnt = 0;
      fireFlyLedHandler(gps.status);
    }
  }
}
