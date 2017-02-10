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

#define HUMMINGBIRD_ROLL_SERVO	//generate roll servo output and use HL serial 0 TX for PWM 1
#define HUMMINGBIRD_ROLL_SERVO_ON_SSEL0	//SSEL0 is used for Roll servo, serial 0 TX stays TX pin!
#define CAMMOUNT_XCONFIG // turn roll/pitch commands for camera compensation by 45°
#define PELICAN_PTU //roll/nick servo offset and scales for standard Pelican camera mount, pitch dir -1, roll dir 1
#define CAM_FACING_FRONT_RIGHT	//define if camera is facing to the front right, for front left leave undefined

//Servo angle offsets in degrees*1000 => change this value if camera mount is not leveled
#define CAMERA_OFFSET_HUMMINGBIRD_PITCH	0
#define CAMERA_OFFSET_HUMMINGBIRD_ROLL	0

#define HUMMINGBIRD_SERVO_DIRECTION_PITCH 1 //1: servo mounted on right hand side of camera, -1: left
#define HUMMINGBIRD_SERVO_DIRECTION_ROLL  -1

void PTU_init(void);
void PTU_update(void);
void PTU_update_middle_positions_by_stick(void);
void SERVO_pitch_move(int);
void SERVO_roll_move(int);

extern int PTU_cam_angle_roll_offset;
extern int PTU_cam_angle_pitch_offset;
extern unsigned char PTU_enable_plain_ch7_to_servo; // =1->channel 7 is mapped plain to 1-2ms servo output

extern unsigned char PTU_cam_option_4_version;

//Pan Tilt Unit Data
struct CAMERA_PTU
{
  int servo_roll_offset;
  int servo_pitch_offset;

  int servo_roll_scale;
  int servo_pitch_scale;

  int servo_pitch_min;
  int servo_pitch_max;

  int servo_roll_min;
  int servo_roll_max;
};

extern struct CAMERA_PTU CAMERA_ptu;

struct CAMERA_COMMANDS
{
  unsigned short status;     //0x01 => camera power on; 0x00 => camera power off
  short chksum;  //status + desired_angle_pitch + desired_angle_roll;
  int desired_angle_pitch;	//desired angles in 1/1000th degree
  int desired_angle_roll;
};

extern struct CAMERA_COMMANDS CAMERA_Commands;
