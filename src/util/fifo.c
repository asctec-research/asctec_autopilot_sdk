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

#include "fifo.h"

void FifoInit(Fifo* pFifo, uint8_t* pData, uint16_t size)
{
  pFifo->pData = pData;
  pFifo->size = size;
  pFifo->writePos = 0;
  pFifo->readPos = 0;
}

int16_t FifoPut(Fifo* pFifo, uint8_t data)
{
  if((pFifo->writePos+1)%pFifo->size == pFifo->readPos)
    return -1;

  pFifo->pData[pFifo->writePos++] = data;
  pFifo->writePos %= pFifo->size;

  return 0;
}

int16_t FifoGet(Fifo* pFifo, uint8_t* pData)
{
  if(pFifo->readPos == pFifo->writePos)
    return -1;

  *pData = pFifo->pData[pFifo->readPos++];
  pFifo->readPos %= pFifo->size;

  return 0;
}

uint16_t FifoBytesUsed(Fifo* pFifo)
{
  if(pFifo->readPos <= pFifo->writePos)
  {
    return pFifo->writePos - pFifo->readPos;
  }
  else
  {
    return pFifo->size - pFifo->readPos + pFifo->writePos;
  }
}

uint16_t FifoBytesFree(Fifo* pFifo)
{
  return pFifo->size - FifoBytesUsed(pFifo);
}

int16_t FifoWrite(Fifo* pFifo, void* _pData, uint16_t dataSize)
{
  uint8_t* pData = (uint8_t*)_pData;

  if(FifoBytesFree(pFifo) < dataSize)
    return -1;

  for(uint16_t i = 0; i < dataSize; i++)
  {
    FifoPut(pFifo, pData[i]);
  }

  return 0;
}
