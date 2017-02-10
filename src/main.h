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

extern void mainloop(void);
extern void timer0ISR(void);

extern volatile unsigned int GPS_timeout;
extern volatile char SYSTEM_initialized;

#define ControllerCyclesPerSecond 	1000

#if (ControllerCyclesPerSecond%50)
#error "Use a multiple of 50 for ControllerCyclesPerSecond"
#endif

//packet descriptors
#define PD_IMURAWDATA               0x01
#define PD_LLSTATUS                 0x02
#define PD_IMUCALCDATA              0x03
#define PD_HLSTATUS                 0x04

#define PD_CTRLOUT                  0x11
#define PD_FLIGHTPARAMS             0x12
#define PD_CTRLCOMMANDS             0x13
#define PD_CTRLINTERNAL             0x14
#define PD_RCDATA                   0x15
#define PD_CTRLSTATUS               0x16

#define PD_WAYPOINT                 0x20
#define PD_CURRENTWAY               0x21
#define PD_NMEADATA                 0x22
#define PD_GPSDATA                  0x23

#define PD_CAMERACOMMANDS           0x30
#define PD_RO_ALL_DATA              0x90

#define PD_JETI_SETNAME             0xA0
#define PD_JETI_SETSENSOR           0xA1
#define PD_JETI_UPDATESDATA         0xA2
#define PD_JETI_SETALARM            0xA3
#define PD_JETI_SETTEXT             0xA4
#define PD_JETI_RCDATA              0xA5
#define PD_JETI_SETSENSOR2          0xA6
#define PD_JETI_SETTEXT2            0xA7
#define PD_JETI_UPDATESENSORBLOCK_1 0xA8
#define PD_JETI_UPDATESENSORBLOCK_2 0xA9

//system status defines for buzzer handling
#define FM_COMPASS_FAILURE          0x10
#define FM_CALIBRATION_ERROR        0x100
#define FM_CALIBRATION_ERROR_GYROS  0x200
#define FM_CALIBRATION_ERROR_ACC    0x400
#define FM_ADC_STARTUP_ERROR        0x800
#define FM_MAG_FIELD_STRENGTH_ERROR 0x4000
#define FM_MAG_INCLINATION_ERROR    0x8000
