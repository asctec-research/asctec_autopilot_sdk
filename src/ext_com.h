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

#include "config.h"
#include "util/cobs.h"
#include "asctec_uav_msgs/transport_definitions.h"
#include <stdint.h>

#define EXT_COM_HEADER_SIZE sizeof(uint16_t)
#define EXT_COM_CHECKSUM_SIZE sizeof(uint16_t)
#define EXT_COM_MAX_ENCODED_MSG_SIZE (COBSMaxStuffedSize(EXT_COM_MAX_MSG_SIZE+EXT_COM_CHECKSUM_SIZE+EXT_COM_HEADER_SIZE)+1)

typedef struct _ExtCom
{
  uint8_t procBuffer[EXT_COM_MAX_ENCODED_MSG_SIZE];

  uint8_t rxProcBuffer[EXT_COM_MAX_ENCODED_MSG_SIZE];
  uint16_t rxProcUsed;

  uint8_t sendBuffer[EXT_COM_MAX_MSG_SIZE];

  struct
  {
    uint32_t good;
    uint32_t decodeFail;
    uint32_t crcFail;
    uint32_t oversized;
  } rxStat;

  struct
  {
    uint32_t good;
    uint32_t noMem;
    uint32_t oversized;
  } txStat;

  uint16_t seq;

  uint16_t cmdTimeout;

  uint8_t active;
} ExtCom;

extern ExtCom extCom;

void ExtComSpinOnce();
int16_t ExtComSend(void* _pData, uint32_t dataSize);
int16_t ExtComSendMessage(TransportHeader* pHeader, void* pData, uint32_t dataSize);
