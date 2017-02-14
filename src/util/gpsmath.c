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

#include "gpsmath.h"
#include "math.h"
#include "../sdkio.h"

#define MEAN_EARTH_DIAMETER	12756274.0
#define UMR 0.017453292519943295769236907684886		//PI/180
#define PI  3.1415926535897932384626433832795

void xy2latlon(double lat0, double lon0, double X, double Y, double *lat, double *lon)//X: East, Y: North in m; lat0,lon0: Reference coordinates; lat,lon: current GPS measurement
{
  *lat = lat0 + Y / MEAN_EARTH_DIAMETER * 360. / PI;
  *lon = lon0 + X / MEAN_EARTH_DIAMETER * 360. / PI / cos(lat0 * UMR);
}
