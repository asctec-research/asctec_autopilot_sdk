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

#include "build_info.h"

struct BUILD_INFO buildInfo;

void generateBuildInfo()
{
  static const char months[12][3] =
    { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dez" };
  char dateStr[11] = __DATE__;
  int m, d, y;
  int i, z;

  m = 1;

  for(z = 0; z < 11; z++)
  {
    int cnt = 0;

    for(i = 0; i < 3; i++)
    {
      if(months[z][i] == dateStr[i])
        cnt++;
    }

    if(cnt == 3)
    {
      m = z + 1;
      break;
    }
  }

  for(z = 0; z < 11; z++)
  {
    if(dateStr[z] < 48)
      dateStr[z] = 48;
  }

  d = (dateStr[4] - 48) * 10 + (dateStr[5] - 48);
  y = (dateStr[7] - 48) * 1000 + (dateStr[8] - 48) * 100 + (dateStr[9] - 48) * 10 + (dateStr[10] - 48);

  buildInfo.build_date = y + m * 10000 + d * 1000000;
  buildInfo.configuration = __BUILD_CONFIG;
  buildInfo.build_number = 0;
  buildInfo.svn_modified = 0;
  buildInfo.svn_revision = 0;
  buildInfo.version_major = __VERSION_MAJOR;
  buildInfo.version_minor = __VERSION_MINOR;
}
