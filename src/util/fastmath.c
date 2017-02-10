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

#include "fastmath.h"

const unsigned int lut_sqrt[] =
  { 1000, 1414, 1732, 2000, 2236, 2449, 2645, 2828, 3000, 3162, 3316, 3464, 3605, 3741, 3872, 4000, 4123, 4242, 4358,
      4472, 4582, 4690, 4795, 4898, 5000, 5099, 5196, 5291, 5385, 5477, 5567, 5656, 5744, 5830, 5916, 6000, 6082, 6164,
      6244, 6324, 6403, 6480, 6557, 6633, 6708, 6782, 6855, 6928, 7000, 7071, 7141, 7211, 7280, 7348, 7416, 7483, 7549,
      7615, 7681, 7745, 7810, 7874, 7937, 8000, 8062, 8124, 8185, 8246, 8306, 8366, 8426, 8485, 8544, 8602, 8660, 8717,
      8774, 8831, 8888, 8944, 9000, 9055, 9110, 9165, 9219, 9273, 9327, 9380, 9433, 9486, 9539, 9591, 9643, 9695, 9746,
      9797, 9848, 9899, 9949, 10000 };

unsigned int fast_sqrt(unsigned int x)
{
  unsigned int div = 1;
  unsigned int mul = 1;
  unsigned int pointSelect;
  unsigned int result;
  unsigned int x_1000;

  //scale to region

  if(x == 0)
    return 0;

  while(x > 100000000)
  {
    x /= 100;
    mul *= 10;
  }

  while(x < 1000000)
  {
    x *= 100;
    div *= 10;
  }

  //select closest sample point
  pointSelect = x / 1000000;

  x_1000 = x - pointSelect * 1000000;
  x_1000 /= 1000;
  //interpolate
  result = lut_sqrt[pointSelect - 1] + ((lut_sqrt[pointSelect] - lut_sqrt[pointSelect - 1]) * x_1000) / 1000;

  result *= mul;
  result /= div;
  return result;
}
