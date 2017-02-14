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

//	Camera Pan Tilt Unit (PTU) control
#include "main.h"
#include "hal/system.h"
#include "pelican_ptu.h"

#include "../ll_hl_comm.h"
#include "../sdkio.h"
#include "LPC214x.h"

struct CAMERA_PTU CAMERA_ptu;
struct CAMERA_COMMANDS CAMERA_Commands;

int PTU_cam_angle_roll_offset = 0;
int PTU_cam_angle_pitch_offset = 0;

unsigned char PTU_enable_plain_ch7_to_servo = 0; // channel 7 is mapped plain to 1-2ms servo output

void PTU_init(void)
{
#ifdef HUMMINGBIRD_ROLL_SERVO
#ifndef HUMMINGBIRD_ROLL_SERVO_ON_SSEL0
  PINSEL0&=~0x01;
  PINSEL0|=0x02;
#else
  PINSEL0 |= 0x8000;
  PINSEL0 &= ~0x4000;
#endif
#endif

#ifdef PELICAN_PTU
  CAMERA_ptu.servo_pitch_offset = 61500;
  CAMERA_ptu.servo_pitch_scale = 54853;
  CAMERA_ptu.servo_pitch_min = 46000;
  CAMERA_ptu.servo_pitch_max = 128000;

  CAMERA_ptu.servo_roll_offset = 82000;	//74000;
  CAMERA_ptu.servo_roll_scale = 115000;	//=90° 110000
  CAMERA_ptu.servo_roll_min = 46000;	//53900;
  CAMERA_ptu.servo_roll_max = 128000;	//94500;
#endif
}

void PTU_update(void)
{
  static int ptu_cnt = 0;
  if(++ptu_cnt > 9)	//generate 100Hz
  {
    ptu_cnt = 0;

    if(PTU_enable_plain_ch7_to_servo)
    {
      int value;

      value = 88473 + (((int)sdk.ro.rc.channels[4] - 2048) * 29491) / 2048;

      PWMMR5 = value;
      PWMLER = LER5_EN | LER1_EN | LER2_EN;

      return;
    }

    int angle_pitch, angle_roll;

    PTU_update_middle_positions_by_stick();

#ifdef CAMMOUNT_XCONFIG	//rotate pitch/roll tiltcompensation for 45°
#ifndef CAM_FACING_FRONT_RIGHT
    angle_pitch = sdk.ro.attitude.angle[1]*707/1000 + sdk.ro.attitude.angle[0]*707/1000;
    angle_roll = sdk.ro.attitude.angle[0]*707/1000 - sdk.ro.attitude.angle[1]*707/1000;
#else
    angle_roll = sdk.ro.attitude.angle[0] * 707 / 1000 + sdk.ro.attitude.angle[1] * 707 / 1000;
    angle_pitch = -sdk.ro.attitude.angle[1] * 707 / 1000 + sdk.ro.attitude.angle[0] * 707 / 1000;
#endif
#else
    angle_pitch = sdk.ro.attitude.angle[1];
    angle_roll = sdk.ro.attitude.angle[0];
#endif

    static int cam_angle_pitch = 0;
#ifdef SET_CAMERA_ANGLE_INCREMENTAL
    if(LL_1khz_attitude_data.RC_data[4]>192) cam_angle_pitch+=200;
    else if(LL_1khz_attitude_data.RC_data[4]<64) cam_angle_pitch-=200;
    if(cam_angle_pitch>55000) cam_angle_pitch=55000;
    if(cam_angle_pitch<-55000) cam_angle_pitch=-55000;
#else
    //example for camera to react on serial interface ON/OFF switch:
    if(sdk.ro.rc.serialSwitch > 3000)
      CAMERA_Commands.desired_angle_pitch = 90000;
    else
      CAMERA_Commands.desired_angle_pitch = 0;
    //<-example

    cam_angle_pitch = CAMERA_Commands.desired_angle_pitch;
    if(cam_angle_pitch > 90000)
      cam_angle_pitch = 90000;
    else if(cam_angle_pitch < 0)
      cam_angle_pitch = 0;
#endif

    if(CAMERA_Commands.status & 0x02)	//no tilt compensation
    {
      SERVO_pitch_move(
          (CAMERA_OFFSET_HUMMINGBIRD_PITCH + cam_angle_pitch + PTU_cam_angle_pitch_offset)
              * HUMMINGBIRD_SERVO_DIRECTION_PITCH);
      SERVO_roll_move(
          (CAMERA_OFFSET_HUMMINGBIRD_ROLL + CAMERA_Commands.desired_angle_roll + PTU_cam_angle_roll_offset)
              * HUMMINGBIRD_SERVO_DIRECTION_ROLL);
    }
    else
    {
      SERVO_pitch_move(
          (CAMERA_OFFSET_HUMMINGBIRD_PITCH + cam_angle_pitch + angle_pitch + PTU_cam_angle_pitch_offset)
              * HUMMINGBIRD_SERVO_DIRECTION_PITCH);
      SERVO_roll_move(
          (CAMERA_OFFSET_HUMMINGBIRD_ROLL + CAMERA_Commands.desired_angle_roll + angle_roll + PTU_cam_angle_roll_offset)
              * HUMMINGBIRD_SERVO_DIRECTION_ROLL);
    }
  }
}

void SERVO_pitch_move(int angle)
{
  unsigned int value;
  value = CAMERA_ptu.servo_pitch_offset + (angle / 10) * CAMERA_ptu.servo_pitch_scale / 9000;	//9000

  if(value > CAMERA_ptu.servo_pitch_max)
    value = CAMERA_ptu.servo_pitch_max;
  else if(value < CAMERA_ptu.servo_pitch_min)
    value = CAMERA_ptu.servo_pitch_min;

  PWMMR5 = value;
  PWMLER = LER5_EN | LER1_EN | LER2_EN;
}

void SERVO_roll_move(int angle)
{
  int value;
  value = CAMERA_ptu.servo_roll_offset + (angle / 10) * CAMERA_ptu.servo_roll_scale / 9000;	//9000

  if(value > CAMERA_ptu.servo_roll_max)
    value = CAMERA_ptu.servo_roll_max;
  else if(value < CAMERA_ptu.servo_roll_min)
    value = CAMERA_ptu.servo_roll_min;

#ifdef HUMMINGBIRD_ROLL_SERVO_ON_SSEL0
  PWMMR2 = value;
#else
  PWMMR1 = value;
#endif
  PWMLER = LER5_EN | LER1_EN | LER2_EN;
}

void PTU_update_middle_positions_by_stick(void)
{
#define THRESHOLD 300
  //set roll offset
  int roll_offset_inc = 0;
  static unsigned char roll_offset_changed = 0;
  static int reset_timeout_roll = 0;
  if(!(sdk.ro.flightMode & FLIGHTMODE_FLYING)) //flying?
  {
    if(reset_timeout_roll)
    {
      reset_timeout_roll--;
    }
    else
    {
      if(sdk.ro.rc.pitch > 3500)	//pitch stick at maximum
      {
        //use roll-stick to change roll servo offset
        if(sdk.ro.rc.roll > 2048 + THRESHOLD)
          roll_offset_inc = (sdk.ro.rc.roll - 2048 - THRESHOLD) / 20;
        else if(sdk.ro.rc.roll < 2048 - THRESHOLD)
          roll_offset_inc = (sdk.ro.rc.roll - 2048 + THRESHOLD) / 20;
        else
          roll_offset_inc = 0;

        PTU_cam_angle_roll_offset += roll_offset_inc;	//(LL_1khz_attitude_data.RC_data[3]-128)*5;
        if(PTU_cam_angle_roll_offset > 20000)
        {
          PTU_cam_angle_roll_offset = 0;
          reset_timeout_roll = 100;
        }
        if(PTU_cam_angle_roll_offset < -20000)
        {
          PTU_cam_angle_roll_offset = 0;
          reset_timeout_roll = 100;
        }
        if(roll_offset_inc)
          roll_offset_changed = 1;
      }
      else
      {
        if(roll_offset_changed)
        {
          roll_offset_changed = 0;

          // TODO: re-implement?
//          lpc_aci_SavePara();
//          lpc_aci_WriteParatoFlash();
        }
      }
    }
  }

  //set pitch offset
  int pitch_offset_inc = 0;
  static unsigned char pitch_offset_changed = 0;
  static int reset_timeout_pitch;
  if(!(sdk.ro.flightMode & FLIGHTMODE_FLYING)) //flying?
  {
    if(reset_timeout_pitch)
    {
      reset_timeout_pitch--;
    }
    else
    {
      if(sdk.ro.rc.thrust > 3500)	//thrust stick at maximum
      {
        //use pitch-stick to change pitch servo offset
        if(sdk.ro.rc.pitch > 2048 + THRESHOLD)
          pitch_offset_inc = (sdk.ro.rc.pitch - 2048 - THRESHOLD) / 20;
        else if(sdk.ro.rc.pitch < 2048 - THRESHOLD)
          pitch_offset_inc = (sdk.ro.rc.pitch - 2048 + THRESHOLD) / 20;
        else
          pitch_offset_inc = 0;

        PTU_cam_angle_pitch_offset -= pitch_offset_inc;	//(LL_1khz_attitude_data.RC_data[3]-128)*5;
        if(PTU_cam_angle_pitch_offset > 20000)
        {
          PTU_cam_angle_pitch_offset = 0;
          reset_timeout_pitch = 100;
        }
        if(PTU_cam_angle_pitch_offset < -20000)
        {
          PTU_cam_angle_pitch_offset = 0;
          reset_timeout_pitch = 100;
        }
        if(pitch_offset_inc)
          pitch_offset_changed = 1;
      }
      else
      {
        if(pitch_offset_changed)
        {
          pitch_offset_changed = 0;

          // TODO: re-implement?
//          lpc_aci_SavePara();
//          lpc_aci_WriteParatoFlash();
        }
      }
    }
  }
}
