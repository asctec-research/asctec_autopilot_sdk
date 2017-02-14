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

#include "flash.h"

#include "LPC214x.h"
#include <string.h>

#define SECTOR_NUMBER 14
#define FLASH_ADDR 0x00038000
#define FLASH_SIZE 0x8000
#define SYSCLK_KHZ 58982

Flash flash;

static uint8_t wrBuf[FLASH_PAGE_SIZE] __attribute__((aligned (4)));

#define IAP_LOCATION      0x7ffffff1
typedef void (*IAP)(uint32_t[], uint32_t[]);
static IAP iap_entry = (IAP)IAP_LOCATION;

uint32_t erase()
{
  uint32_t commandIAP[5];
  uint32_t resultIAP[3];
  uint32_t enabledIRQs;

  enabledIRQs = VICIntEnable;  //disable all interrupts
  VICIntEnClr = enabledIRQs;

  commandIAP[0] = 50;          //prepare sectors for erase
  commandIAP[1] = SECTOR_NUMBER;
  commandIAP[2] = SECTOR_NUMBER;
  iap_entry(commandIAP, resultIAP);

  commandIAP[0] = 52;          //erase sectors
  commandIAP[1] = SECTOR_NUMBER;
  commandIAP[2] = SECTOR_NUMBER;
  commandIAP[3] = SYSCLK_KHZ;
  iap_entry(commandIAP, resultIAP);

  commandIAP[0] = 53;          //blankcheck sectors
  commandIAP[1] = SECTOR_NUMBER;
  commandIAP[2] = SECTOR_NUMBER;
  iap_entry(commandIAP, resultIAP);

  VICIntEnable = enabledIRQs;  //restore interrupt enable register

  return resultIAP[0];
}

static uint32_t write(uint32_t addr, uint8_t id, const void* pData, uint8_t size)
{
  uint32_t commandIAP[5];
  uint32_t resultIAP[3];
  uint32_t enabledIRQs;

  memset(wrBuf, 0xFF, FLASH_PAGE_SIZE);
  wrBuf[0] = id;
  memcpy(&wrBuf[1], pData, size);

  enabledIRQs = VICIntEnable;  //disable all interrupts
  VICIntEnClr = enabledIRQs;

  commandIAP[0] = 50;          //prepare sectors for write
  commandIAP[1] = SECTOR_NUMBER;
  commandIAP[2] = SECTOR_NUMBER;
  iap_entry(commandIAP, resultIAP);

  commandIAP[0] = 51;          //copy RAM to flash/eeprom
  commandIAP[1] = addr;
  commandIAP[2] = (uint32_t)wrBuf;
  commandIAP[3] = FLASH_PAGE_SIZE;
  commandIAP[4] = SYSCLK_KHZ;
  iap_entry(commandIAP, resultIAP);

  commandIAP[0] = 56;          //compare RAM and flash/eeprom
  commandIAP[1] = (uint32_t)wrBuf;
  commandIAP[2] = addr;
  commandIAP[3] = FLASH_PAGE_SIZE;
  iap_entry(commandIAP, resultIAP);

  VICIntEnable = enabledIRQs;  //restore interrupt enable register

  return resultIAP[0];
}

static void cleanup()
{
  uint8_t usedPages = 0;

  memset(flash.pageBackup, 0xFF, sizeof(FlashPage)*FLASH_MAX_PAGES);

  for(uint32_t addr = FLASH_ADDR+FLASH_SIZE-FLASH_PAGE_SIZE; addr >= FLASH_ADDR; addr += FLASH_PAGE_SIZE)
  {
    FlashPage* pPage = (FlashPage*)addr;

    if(pPage->id == 0xFF)
      continue;

    // we have found a page with content, check if we already backed it up
    uint8_t alreadySafe = 0;
    for(uint8_t i = 0; i < usedPages; i++)
    {
      if(flash.pageBackup[i].id == pPage->id)
      {
        alreadySafe = 1;
        break;
      }
    }

    if(alreadySafe)
      continue;

    // found a page which is not backed up
    memcpy(&flash.pageBackup[usedPages], (void*)addr, sizeof(FlashPage));
    ++usedPages;

    // just for safety, at no time more than FLASH_MAX_PAGES pages should be used
    if(usedPages >= FLASH_MAX_PAGES)
      break;
  }

  // now erase flash
  erase();

  // restore saved data
  for(uint8_t i = 0; i < usedPages; i++)
  {
    write(FLASH_ADDR+i*FLASH_PAGE_SIZE, flash.pageBackup[i].id, flash.pageBackup[i].data, FLASH_PAGE_SIZE-1);
  }
}

void FlashWrite(uint8_t id, const void* pData, uint8_t size)
{
  // check if content has changed
  for(uint32_t addr = FLASH_ADDR+FLASH_SIZE-FLASH_PAGE_SIZE; addr >= FLASH_ADDR; addr += FLASH_PAGE_SIZE)
  {
    FlashPage* pPage = (FlashPage*)addr;

    if(pPage->id == id)
    {
      if(memcmp(pPage->data, pData, size) == 0)
        return;
    }
  }

  // find free page
  for(uint32_t addr = FLASH_ADDR; addr < FLASH_ADDR+FLASH_SIZE; addr += FLASH_PAGE_SIZE)
  {
    FlashPage* pPage = (FlashPage*)addr;

    if(pPage->id == 0xFF) // unprogrammed?
    {
      write(addr, id, pData, size);

      return;
    }
  }

  // no free page found => erase flash and start from the beginning
  cleanup();

  FlashWrite(id, pData, size);
}

int16_t FlashRead(uint8_t id, void* pDest, uint8_t size)
{
  for(uint32_t addr = FLASH_ADDR+FLASH_SIZE-FLASH_PAGE_SIZE; addr >= FLASH_ADDR; addr += FLASH_PAGE_SIZE)
  {
    FlashPage* pPage = (FlashPage*)addr;

    if(pPage->id == id)
    {
      memcpy(pDest, pPage->data, size);
      return 0;
    }
  }

  return -1;
}
