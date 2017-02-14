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

#include "buzzer.h"

#include "../ll_hl_comm.h"
#include "main.h"
#include "system.h"
#include "util/gpsmath.h"
#include "LPC214x.h"
#include "i2c1.h"
#include "led.h"

//Buzzer active defines
#define BU_INIT             0x01
#define BU_ERROR_GYRO       0x02
#define BU_ERROR_ACC        0x04
#define BU_ERROR_ADC        0x08
#define BU_BATTERY          0x10
#define BU_COMPASS_FAILURE  0x20
#define BU_WARNING_MAG_FS   0x40
#define BU_WARNING_MAG_INC  0x80
#define BU_GPS_BEEP         0x100

#define BUZZER_WARNING_INIT_BEEP  0x01
#define BUZZER_WARNING_GPS_BEEP   0x02

#define BUZZ_LENGTH               5	//50 ms
#define BUZZ_PAUSE                5 //50 ms
#define BUZZ_INTERVAL             200 //2 s
#define BUZZ_MAG_WARNING_TIMEOUT  500 //mag warning for 5 seconds only
#define BUZZ_NR_OF_WARNINGS       9 //total number of different buzzer signals (see BU_ defines above)

#define ENABLED_WARNINGS            (BUZZER_WARNING_INIT_BEEP | BUZZER_WARNING_GPS_BEEP)
#define ALARM_BAT_WARN_VOLTAGE_HIGH 10700
#define ALARM_BAT_WARN_VOLTAGE_LOW  10100

void BuzzerHandler(unsigned int vbat, uint8_t gpsLock, uint8_t rcInGPSMode, uint16_t llError)	//needs to be triggered at 100 Hz
{
  unsigned int buz_active = 0;
  static unsigned short error_cnt_mag_fs;
  static unsigned short error_cnt_mag_inc;
  static unsigned short error_cnt_compass;

  unsigned int buz_priority = 0;
  static unsigned short buz_cnt = 0;

  static unsigned int bat_div = 5;
  static int bat_cnt = 0, bat_warning = 0;
  static char bat_warning_enabled = 0;

  unsigned char i;

  if(++buz_cnt >= BUZZ_INTERVAL)
    buz_cnt = 0;

  //battery warning
  if(++bat_cnt == 100)
  {
    bat_cnt = 0;
    bat_div = (ALARM_BAT_WARN_VOLTAGE_HIGH - ALARM_BAT_WARN_VOLTAGE_LOW) / 100;
  }
  if(vbat < ALARM_BAT_WARN_VOLTAGE_LOW)
    vbat = ALARM_BAT_WARN_VOLTAGE_LOW;

  if(vbat < ALARM_BAT_WARN_VOLTAGE_HIGH)	//decide if it's really an empty battery
  {
    if(bat_warning < ControllerCyclesPerSecond / 5)
      bat_warning++;
    else
      bat_warning_enabled = 1;
  }
  else
  {
    if(bat_warning > 10)
      bat_warning -= 2;
    else
    {
      bat_warning_enabled = 0;
      buz_active &= ~BU_BATTERY;
    }
  }
  if(bat_warning_enabled)
  {
    if(bat_cnt > ((vbat - ALARM_BAT_WARN_VOLTAGE_LOW) / bat_div))
      buz_active |= BU_BATTERY; //Beeper on
    else
      buz_active &= ~BU_BATTERY; //Beeper off
    buz_priority |= BU_BATTERY;
  }
  else
  {
    buz_active &= ~BU_BATTERY;
    buz_priority &= ~BU_BATTERY;
  }

  if(ENABLED_WARNINGS & BUZZER_WARNING_GPS_BEEP)
  {
    if((gpsLock == 0) && rcInGPSMode)	//no lock and in GPS mode
    {
      buz_priority |= BU_GPS_BEEP;
      if(buz_cnt < 5)
        buz_active |= BU_GPS_BEEP;
      else
        buz_active &= ~BU_GPS_BEEP;
    }
    else
    {
      buz_active &= ~BU_GPS_BEEP;
      buz_priority &= ~BU_GPS_BEEP;
    }
  }

  //gyro error
  if((llError & FM_CALIBRATION_ERROR_GYROS) && (SYSTEM_initialized))
  {
    buz_priority |= BU_ERROR_GYRO;
    if(buz_cnt < 155)
      buz_active |= BU_ERROR_GYRO;
    else if(buz_cnt < 160)
      buz_active &= ~BU_ERROR_GYRO;
    else if(buz_cnt < 165)
      buz_active |= BU_ERROR_GYRO;
    else
      buz_active &= ~BU_ERROR_GYRO;
  }
  else
  {
    buz_priority &= ~BU_ERROR_GYRO;
    buz_active &= ~BU_ERROR_GYRO;
  }

  //ACC error
  if((llError & FM_CALIBRATION_ERROR_ACC) && (SYSTEM_initialized))
  {
    buz_priority |= BU_ERROR_ACC;
    if(buz_cnt < 145)
      buz_active |= BU_ERROR_ACC;
    else if(buz_cnt < 150)
      buz_active &= ~BU_ERROR_ACC;
    else if(buz_cnt < 155)
      buz_active |= BU_ERROR_ACC;
    else if(buz_cnt < 160)
      buz_active &= ~BU_ERROR_ACC;
    else if(buz_cnt < 165)
      buz_active |= BU_ERROR_ACC;
    else
      buz_active &= ~BU_ERROR_ACC;
  }
  else
  {
    buz_priority &= ~BU_ERROR_ACC;
    buz_active &= ~BU_ERROR_ACC;
  }

  //ADC error
  if((llError & FM_ADC_STARTUP_ERROR) && (SYSTEM_initialized))
  {
    buz_priority |= BU_ERROR_ADC;
    if(buz_cnt < 135)
      buz_active |= BU_ERROR_ADC;
    else if(buz_cnt < 140)
      buz_active &= ~BU_ERROR_ADC;
    else if(buz_cnt < 145)
      buz_active |= BU_ERROR_ADC;
    else if(buz_cnt < 150)
      buz_active &= ~BU_ERROR_ADC;
    else if(buz_cnt < 155)
      buz_active |= BU_ERROR_ADC;
    else if(buz_cnt < 160)
      buz_active &= ~BU_ERROR_ADC;
    else if(buz_cnt < 165)
      buz_active |= BU_ERROR_ADC;
    else
      buz_active &= ~BU_ERROR_ADC;
  }
  else
  {
    buz_priority &= ~BU_ERROR_ADC;
    buz_active &= ~BU_ERROR_ADC;
  }

  //compass failure: warn 3 seconds only
  if((llError & FM_COMPASS_FAILURE) && (SYSTEM_initialized)&&(error_cnt_compass++ < 400))
  {
    buz_priority |= BU_COMPASS_FAILURE;
    if(buz_cnt % 100 < 5)
      buz_active |= BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 10)
      buz_active &= ~BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 15)
      buz_active |= BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 20)
      buz_active &= ~BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 25)
      buz_active |= BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 30)
      buz_active &= ~BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 35)
      buz_active |= BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 40)
      buz_active &= ~BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 45)
      buz_active |= BU_COMPASS_FAILURE;
    else if(buz_cnt % 100 < 50)
      buz_active &= ~BU_COMPASS_FAILURE;
    else
      buz_active &= ~BU_COMPASS_FAILURE;
  }
  else
  {
    buz_priority &= ~BU_COMPASS_FAILURE;
    buz_active &= ~BU_COMPASS_FAILURE;
  }

  //mag fieldstrength warning: warn 3 times only
  if((llError & FM_MAG_FIELD_STRENGTH_ERROR)
      && (SYSTEM_initialized)&&(error_cnt_mag_fs++ < 400))
  {
    buz_priority |= BU_WARNING_MAG_FS;
    if(buz_cnt % 100 < 5)
      buz_active |= BU_WARNING_MAG_FS;
    else if(buz_cnt % 100 < 10)
      buz_active &= ~BU_WARNING_MAG_FS;
    else if(buz_cnt % 100 < 15)
      buz_active |= BU_WARNING_MAG_FS;
    else if(buz_cnt % 100 < 20)
      buz_active &= ~BU_WARNING_MAG_FS;
    else if(buz_cnt % 100 < 25)
      buz_active |= BU_WARNING_MAG_FS;
    else if(buz_cnt % 100 < 30)
      buz_active &= ~BU_WARNING_MAG_FS;
    else
      buz_active &= ~BU_WARNING_MAG_FS;
  }
  else
  {
    buz_priority &= ~BU_WARNING_MAG_FS;
    buz_active &= ~BU_WARNING_MAG_FS;
  }

  //mag inclination warning: warn 3 times only
  if((llError & FM_MAG_INCLINATION_ERROR) && (SYSTEM_initialized)&&(error_cnt_mag_inc++ < 400))
  {
    buz_priority |= BU_WARNING_MAG_INC;
    if(buz_cnt % 100 < 5)
      buz_active |= BU_WARNING_MAG_INC;
    else if(buz_cnt % 100 < 10)
      buz_active &= ~BU_WARNING_MAG_INC;
    else if(buz_cnt % 100 < 15)
      buz_active |= BU_WARNING_MAG_INC;
    else if(buz_cnt % 100 < 20)
      buz_active &= ~BU_WARNING_MAG_INC;
    else if(buz_cnt % 100 < 25)
      buz_active |= BU_WARNING_MAG_INC;
    else if(buz_cnt % 100 < 30)
      buz_active &= ~BU_WARNING_MAG_INC;
    else if(buz_cnt % 100 < 35)
      buz_active |= BU_WARNING_MAG_INC;
    else if(buz_cnt % 100 < 40)
      buz_active &= ~BU_WARNING_MAG_INC;
    else
      buz_active &= ~BU_WARNING_MAG_INC;
  }
  else
  {
    buz_priority &= ~BU_WARNING_MAG_INC;
    buz_active &= ~BU_WARNING_MAG_INC;
  }

  if(ENABLED_WARNINGS & BUZZER_WARNING_INIT_BEEP)
  {
    if(!SYSTEM_initialized)
    {
      buz_priority |= BU_INIT;

      if(buz_cnt % 100 < 5)
      {
        buz_active |= BU_INIT;
        I2C1SetRGBLed(0, 0, 255);
      }
      else if(buz_cnt % 100 < 10)
      {
        buz_active &= ~BU_INIT;
        I2C1SetRGBLed(0, 0, 0);
      }
      else if(buz_cnt % 100 < 15)
      {
        buz_active |= BU_INIT;
        I2C1SetRGBLed(0, 0, 255);
      }
      else if(buz_cnt % 100 < 30)
      {
        buz_active &= ~BU_INIT;
        I2C1SetRGBLed(0, 0, 0);
      }
      else if(buz_cnt % 100 < 35)
      {
        buz_active |= BU_INIT;
        I2C1SetRGBLed(0, 0, 255);
      }
      else
      {
        buz_active &= ~BU_INIT;
        I2C1SetRGBLed(0, 0, 0);
      }
    }
    else
    {
      buz_active &= ~BU_INIT;
      buz_priority &= ~BU_INIT;
    }
  }

  //buzzer control
  for(i = 0; i < BUZZ_NR_OF_WARNINGS; i++)
  {
    if(buz_priority & (1 << i))
    {
      buz_active &= (1 << i);
      i = BUZZ_NR_OF_WARNINGS;
    }
  }

  if(buz_active)
    BuzzerEnable(1);
  else
    BuzzerEnable(0);
}

void BuzzerEnable(unsigned char offon)
{

  if(offon)	//beeper on
  {
    IOSET1 = (1 << 17);
  }
  else
  {
    IOCLR1 = (1 << 17);
  }
}
