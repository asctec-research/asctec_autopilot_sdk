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

#include <stdint.h>
#include "asctec_uav_msgs/transport_definitions.h"

typedef struct _WaypointExample
{
  uint8_t wpNr;
  uint8_t startEvent;
  uint8_t abortEvent;
  uint16_t state;
} WaypointExample;

extern WaypointExample wpExample;

// SDKInit gets called once during startup.
void SDKInit(void);

// SDKMainloop is called regularly at 1kHz
void SDKMainloop(void);

void SDKProcessUserMsg(const TransportHeader* pHeader, const uint8_t* pData, uint32_t dataSize);
