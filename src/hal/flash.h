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

#include <stdint.h>

#define FLASH_PAGE_SIZE 256
#define FLASH_MAX_PAGES 4

#pragma pack(push, 1)

typedef struct _FlashPage
{
  uint8_t id;
  uint8_t data[FLASH_PAGE_SIZE-1];
} FlashPage;

#pragma pack(pop)

typedef struct _Flash
{
  FlashPage pageBackup[FLASH_MAX_PAGES];
} Flash;

extern Flash flash;

int16_t FlashRead(uint8_t id, void* pDest, uint8_t size);
void    FlashWrite(uint8_t id, const void* pData, uint8_t size);
