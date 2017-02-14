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

#pragma once

//JETI ERROR CODES
//are returned by most functions and indicate parameter range problems.
//In case of the JETI_ERROR_STRING_* return values the function executed anyway, but the string got truncated
//In any other case the function aborted

//no error
#define JETI_NO_ERROR				0x00
//id not <15
#define JETI_ERROR_ID_RANGE			0x01
//id wasn't active. either execute jetiInitValue() or jetiActicateValue()
#define JETI_ERROR_ID_NOT_ACTIVE	0x02
//string length errors
#define JETI_ERROR_STRING_NAME		0x13
#define JETI_ERROR_STRING_DESC		0x14
#define JETI_ERROR_STRING_UNIT		0x15
#define JETI_ERROR_STRING_TEXT		0x16
//value range or conversion problems
#define JETI_ERROR_VALUE_RANGE		0x20
#define JETI_ERROR_VALUE_CONV_LAT	0x21
#define JETI_ERROR_VALUE_CONV_LON	0x21
//decimal point range error (must be 0..3)
#define JETI_ERROR_DECPOINT_RANGE	0x30
//time date range errors
#define JETI_ERROR_YEAR	 			0x41
#define JETI_ERROR_MONTH			0x42
#define JETI_ERROR_DAY				0x43
#define JETI_ERROR_HOUR	 			0x44
#define JETI_ERROR_MINUTE			0x45
#define JETI_ERROR_SECOND			0x46
//alarm range errors. alarm type must be 0 or 1
#define JETI_ERROR_ALARM_RANGE		0x50
#define JETI_ERROR_ALARM_TYPE		0x51

//high nibble stores size in bytes
#define JETI_VART_6B ((1<<4)|0)
#define JETI_VART_14B ((2<<4)|1)
#define JETI_VART_22B ((3<<4)|4)
#define JETI_VART_DATETIME ((3<<4)|5)
#define JETI_VART_30B ((4<<4)|8)
#define JETI_VART_GPSCOORD ((4<<4)|9)

#define JETI_KEY_UP					0x0D
#define JETI_KEY_DOWN				0x0B
#define JETI_KEY_LEFT				0x07
#define JETI_KEY_RIGHT				0x0E

//jeti value struct
struct __attribute__((packed)) JETI_VALUE
{
  unsigned char active;
  unsigned char name[10];
  unsigned char unit[5];
  unsigned char varType;
  unsigned char decPoint;
  int value;
};

//ids are 0 to 14
//set alarm to 'A' to 'Z'. Alarmtype is usually 0
extern unsigned char jetiSetAlarm(unsigned char alarm, unsigned alarmType);
//sets device name
extern unsigned char jetiSetDeviceName(const char * name);
//activates a telemetry value
extern unsigned char jetiActivateValue(unsigned char id);
//deactivates a telemetry value
extern unsigned char jetiDeactivateValue(unsigned char id);
//sets position of the decimalpoint of an id. decimalPoint has range 0..3
extern unsigned char jetiSetDecimalPoint(unsigned char id, unsigned char decimalPoint);
//setvalue with 30bit + sign range
extern unsigned char jetiSetValue30B(unsigned char id, int value);
//setvalue with 22bit + sign range
extern unsigned char jetiSetValue22B(unsigned char id, int value);
//setvalue with 14bit + sign range
extern unsigned char jetiSetValue14B(unsigned char id, int value);
//setvalue with 6bit + sign range
extern unsigned char jetiSetValue6B(unsigned char id, int value);
//set value to time
extern unsigned char jetiSetValueTime(unsigned char id, unsigned char hours, unsigned char minutes,
    unsigned char seconds);
//set value to date
extern unsigned char jetiSetValueDate(unsigned char id, unsigned char day, unsigned char month, unsigned short year);
//change text display. display is 2 lines with 16 bytes each
extern unsigned char jetiSetTextDisplay(const char * text);
//init value with description(10 chars) and unit(3 chars)
extern unsigned char jetiInitValue(unsigned char id, const char * description, const char * unit);
//is called by LL_HL_Comm when jeti key state changed
extern void jetiSetKeyChanged(unsigned char key);
//user function to check for a key change. Returns JETI_KEY_ value or 0 for no change
extern unsigned char jetiCheckForKeyChange(void);

extern unsigned char jetiName[10];
extern struct JETI_VALUE jetiValues[15];
extern unsigned char jetiDisplayText[33];
extern unsigned char jetiAlarm;
extern unsigned char jetiAlarmType;
extern unsigned char jetiTriggerTextSync;
