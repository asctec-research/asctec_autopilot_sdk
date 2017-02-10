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

#include "LPC214x.h"                        /* LPC21xx definitions */
#include "type.h"
#include "target.h"
#include "adc.h"

#define ADC_OFFSET         0x10
#define ADC_INDEX          4

#define ADC_DONE           0x80000000
#define ADC_OVERRUN        0x40000000

#define VOLTAGE_1          2
#define VOLTAGE_2          4
#define CURRENT_1          3
#define CURRENT_2          1

#define ADC_CLK 1000000    // set to 1Mhz

void ADC0Init()
{
  // CLKDIV = Fpclk / 1000000 - 1, burst mode on (continuous conversion)
  // power-up (normal operation), 10 Bit result
  AD0CR = (( Fpclk / ADC_CLK - 1) << 8) | (1 << 16) | (1 << 21);

  // enable measurements
  AD0CR |= (1 << VOLTAGE_1) | (1 << VOLTAGE_2) | (1 << CURRENT_1) | (1 << CURRENT_2);
}

uint32_t ADC0GetRawValue(uint8_t index)
{
  if(index > 5)
    return 0;

  uint32_t channelVal;
  uint32_t regVal = *(volatile uint32_t*)(AD0_BASE_ADDR + ADC_OFFSET + ADC_INDEX * index);

  if((regVal & (ADC_OVERRUN | ADC_DONE)) == 0)
    channelVal = 0;
  else
    channelVal = (regVal >> 6) & 0x3FF;

  return channelVal;
}
