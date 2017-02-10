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
 *
 * +++ Consistent Overhead Byte Stuffing with Zero Pair and Zero Run Elimination. +++
 */

#pragma once

#include <stdint.h>

#define ERROR_COBS_NOT_ENOUGH_MEMORY				0x3000
#define ERROR_COBS_ZERO_SIZE						0x3001

typedef struct _COBSState
{
	uint8_t code;
	uint8_t* pOut;
	const uint8_t* pOutOrig;
	uint8_t* pCode;
} COBSState;

/**
 * Calculate maximum size of stuffed data in worst-case.
 */
#define COBSMaxStuffedSize(size) ((size)+(size)/208+1)

/**
 * Stuff bytes and remove zeros.
 *
 * @param pIn Data to be stuffed.
 * @param sizeIn Input data size.
 * @param pOut Stuffed output data.
 * @param sizeOut Maximum output size.
 * @param pBytesWritten Actual size of stuffed data, pass 0 if not needed.
 */
int16_t COBSEncode(const uint8_t *pIn, uint32_t sizeIn, uint8_t *pOut, uint32_t sizeOut,
		uint32_t* pBytesWritten);

/**
 * Unstuff COBS data.
 *
 * @param pIn Stuffed data.
 * @param sizeIn Stuffed data size.
 * @param pOut Unstuffed data.
 * @param sizeOut Maximum output size.
 * @param pBytesWritten Actual unstuffed data size.
 */
int16_t COBSDecode(const uint8_t *pIn, uint32_t sizeIn,
		uint8_t *pOut, uint32_t sizeOut, uint32_t* pBytesWritten);

int16_t	COBSStartEncode(COBSState* pState, uint32_t sizeIn, uint8_t* pOut, uint32_t sizeOut);
void	COBSFeedEncode(COBSState* pState, uint8_t c);
void	COBSFeedEncodeBlock(COBSState* pState, const void* _pData, uint32_t dataSize);
void	COBSFinalizeEncode(COBSState* pState, uint32_t* pBytesWritten);
