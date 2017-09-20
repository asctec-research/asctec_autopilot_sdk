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

#include "asctec_uav_msgs/message_definitions.h"

// Adjust this line to match your UAV type:
#define VEHICLE_TYPE VEHICLE_TYPE_HUMMINGBIRD
//#define VEHICLE_TYPE VEHICLE_TYPE_PELICAN
//#define VEHICLE_TYPE VEHICLE_TYPE_FIREFLY

#define UART0_FUNCTION_TERMINAL 0
#define UART0_FUNCTION_COMM     1

// select either a command line terminal on UART0 or a data communication protocol:
//#define UART0_FUNCTION UART0_FUNCTION_TERMINAL
#define UART0_FUNCTION UART0_FUNCTION_COMM

// 921600 is recommended for a wired connection, use 57600 baud when using an XBee
#define UART0_BAUDRATE 921600

// EXT_COM
#define EXT_COM_MAX_MSG_SIZE 128


#if VEHICLE_TYPE == VEHICLE_TYPE_HUMMINGBIRD
#define MAX_THRUST 20.0f
#else
#define MAX_THRUST 36.0f
#endif
