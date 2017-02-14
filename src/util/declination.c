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

#include "LPC214x.h"
#include "main.h"
#include "declination.h"
#include <math.h>
#include "string.h"

volatile int estimatedDeclination;
volatile int estimatedInclination;
volatile unsigned char declinationAvailable;

static void E0000(int IENTRY, int *maxdeg, float alt, float glat, float glon, float time, float *dec, float *dip,
    float *ti, float *gv)
{
  static float cd[169] =
    { 0.0, 8.0, -15.1, 0.4, -2.5, -2.8, -0.7, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, -20.9, 10.6, -7.8, -2.6, 2.8, 0.7, 0.4,
        -0.1, 0.3, 0.0, 0.0, 0.0, 0.0, -23.2, -14.6, -0.8, -1.2, -7.0, -3.2, -0.3, -0.3, -0.4, 0.0, 0.0, 0.0, 0.0, 5.0,
        -7.0, -0.6, -6.5, 6.2, -1.1, 2.3, 1.1, 0.3, 0.0, 0.0, 0.0, 0.0, 2.2, 1.6, 5.8, 0.1, -3.8, 0.1, -2.1, 0.6, -0.3,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.7, 2.1, 4.8, -1.1, -0.8, -0.6, 0.5, 0.2, 0.0, 0.0, 0.0, 0.0, -0.6, -1.9, -0.4, -0.5,
        -0.3, 0.7, 1.4, -0.4, 0.4, 0.0, 0.0, 0.0, 0.0, 0.6, 0.4, 0.2, 0.3, -0.8, -0.2, 0.1, 0.6, -0.7, 0.0, 0.0, 0.0,
        0.0, -0.2, 0.1, 0.3, 0.4, 0.1, -0.2, 0.4, 0.4, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  static float c[169] =
    { 0.0, -29556.8, -2340.6, 1335.4, 919.8, -227.4, 73.2, 80.1, 24.9, 5.6, -2.3, 2.8, -2.4, 5079.8, -1671.7, 3046.9,
        -2305.1, 798.1, 354.6, 69.7, -74.5, 7.7, 9.9, -6.3, -1.6, -0.4, -2594.7, -516.7, 1657.0, 1246.7, 211.3, 208.7,
        76.7, -1.4, -11.6, 3.5, 1.6, -1.7, 0.2, -199.9, 269.3, -524.2, 674.0, -379.4, -136.5, -151.2, 38.5, -6.9, -7.0,
        -2.6, 1.7, 0.8, 281.5, -226.0, 145.8, -304.7, 100.0, -168.3, -14.9, 12.4, -18.2, 5.1, 0.0, -0.1, -0.3, 42.4,
        179.8, -123.0, -19.5, 103.6, -14.1, 14.6, 9.5, 10.0, -10.8, 3.1, 0.1, 1.1, -20.3, 54.7, 63.6, -63.4, -0.1, 50.4,
        -86.3, 5.7, 9.2, -1.3, 0.4, -0.7, -0.5, -61.5, -22.4, 7.2, 25.4, 11.0, -26.4, -5.1, 1.8, -11.6, 8.8, 2.1, 0.7,
        0.4, 11.2, -21.0, 9.6, -19.8, 16.1, 7.7, -12.9, -0.2, -5.2, -6.7, 3.9, 1.8, -0.3, -20.1, 12.9, 12.6, -6.7, -8.1,
        8.0, 2.9, -7.9, 6.0, -9.1, -0.1, 0.0, -0.3, 2.4, 0.2, 4.4, 4.8, -6.5, -1.1, -3.4, -0.8, -2.3, -7.9, -2.3, 1.1,
        -0.1, 0.3, 1.2, -0.8, -2.5, 0.9, -0.6, -2.7, -0.9, -1.3, -2.0, -1.2, 4.1, -0.3, -0.4, 0.3, 2.4, -2.6, 0.6, 0.3,
        0.0, 0.0, 0.3, -0.9, -0.4, 0.8, -0.1 };
  static int maxord, n, m, j, D1, D2, D3, D4;
  static float tc[13][13], dp[13][13], snorm[169], sp[13], cp[13], fn[13], fm[13], pp[13], k[13][13], pi, dtr, a, b, re,
      a2, b2, c2, a4, b4, c4, epoch, flnmj, otime, oalt, olat, olon, dt, rlon, rlat, srlon, srlat, crlon, crlat, srlat2,
      crlat2, q, q1, q2, ct, st, r2, r, d, ca, sa, aor, ar, br, bt, bp, bpp, par, temp1, temp2, parp, bx, by, bz, bh;
  static float *p = snorm;

  if(IENTRY == 0)
  {
    // GEOMAG

    /* INITIALIZE CONSTANTS */
    maxord = *maxdeg;
    sp[0] = 0.0;
    cp[0] = *p = pp[0] = 1.0;
    dp[0][0] = 0.0;
    a = 6378.137;
    b = 6356.7523142;
    re = 6371.2;
    a2 = a * a;
    b2 = b * b;
    c2 = a2 - b2;
    a4 = a2 * a2;
    b4 = b2 * b2;
    c4 = a4 - b4;

    /* CONVERT SCHMIDT NORMALIZED GAUSS COEFFICIENTS TO UNNORMALIZED */
    epoch = 2005;

    *snorm = 1.0;
    for(n = 1; n <= maxord; n++)
    {
      *(snorm + n) = *(snorm + n - 1) * (float)(2 * n - 1) / (float)n;
      j = 2;
      for(m = 0, D1 = 1, D2 = (n - m + D1) / D1; D2 > 0; D2--, m += D1)
      {
        k[m][n] = (float)(((n - 1) * (n - 1)) - (m * m)) / (float)((2 * n - 1) * (2 * n - 3));
        if(m > 0)
        {
          flnmj = (float)((n - m + 1) * j) / (float)(n + m);
          *(snorm + n + m * 13) = *(snorm + n + (m - 1) * 13) * sqrt(flnmj);
          j = 1;
          c[n * 13 + m - 1] = *(snorm + n + m * 13) * c[n * 13 + m - 1];
          cd[n * 13 + m - 1] = *(snorm + n + m * 13) * cd[n * 13 + m - 1];
        }
        c[m * 13 + n] = *(snorm + n + m * 13) * c[m * 13 + n];
        cd[m * 13 + n] = *(snorm + n + m * 13) * cd[m * 13 + n];
      }
      fn[n] = (float)(n + 1);
      fm[n] = (float)n;
    }
    k[1][1] = 0.0;

    otime = oalt = olat = olon = -1000.0;
  }
  else
  {
    // GEOMG1

    dt = time - epoch;
    if(otime < 0.0 && (dt < 0.0 || dt > 5.0))
    {
      //Warning! date after maximum livespan model
    }

    pi = 3.14159265359;
    dtr = pi / 180.0;
    rlon = glon * dtr;
    rlat = glat * dtr;
    srlon = sin(rlon);
    srlat = sin(rlat);
    crlon = cos(rlon);
    crlat = cos(rlat);
    srlat2 = srlat * srlat;
    crlat2 = crlat * crlat;
    sp[1] = srlon;
    cp[1] = crlon;

    /* CONVERT FROM GEODETIC COORDS. TO SPHERICAL COORDS. */
    if(alt != oalt || glat != olat)
    {
      q = sqrt(a2 - c2 * srlat2);
      q1 = alt * q;
      q2 = ((q1 + a2) / (q1 + b2)) * ((q1 + a2) / (q1 + b2));
      ct = srlat / sqrt(q2 * crlat2 + srlat2);
      st = sqrt(1.0 - (ct * ct));
      r2 = (alt * alt) + 2.0 * q1 + (a4 - c4 * srlat2) / (q * q);
      r = sqrt(r2);
      d = sqrt(a2 * crlat2 + b2 * srlat2);
      ca = (alt + d) / r;
      sa = c2 * crlat * srlat / (r * d);
    }
    if(glon != olon)
    {
      for(m = 2; m <= maxord; m++)
      {
        sp[m] = sp[1] * cp[m - 1] + cp[1] * sp[m - 1];
        cp[m] = cp[1] * cp[m - 1] - sp[1] * sp[m - 1];
      }
    }
    aor = re / r;
    ar = aor * aor;
    br = bt = bp = bpp = 0.0;
    for(n = 1; n <= maxord; n++)
    {
      ar = ar * aor;
      for(m = 0, D3 = 1, D4 = (n + m + D3) / D3; D4 > 0; D4--, m += D3)
      {
        /*
         COMPUTE UNNORMALIZED ASSOCIATED LEGENDRE POLYNOMIALS
         AND DERIVATIVES VIA RECURSION RELATIONS
         */
        if(alt != oalt || glat != olat)
        {
          if(n == m)
          {
            *(p + n + m * 13) = st * *(p + n - 1 + (m - 1) * 13);
            dp[m][n] = st * dp[m - 1][n - 1] + ct * *(p + n - 1 + (m - 1) * 13);
            goto S50;
          }
          if(n == 1 && m == 0)
          {
            *(p + n + m * 13) = ct * *(p + n - 1 + m * 13);
            dp[m][n] = ct * dp[m][n - 1] - st * *(p + n - 1 + m * 13);
            goto S50;
          }
          if(n > 1 && n != m)
          {
            if(m > n - 2)
              *(p + n - 2 + m * 13) = 0.0;
            if(m > n - 2)
              dp[m][n - 2] = 0.0;
            *(p + n + m * 13) = ct * *(p + n - 1 + m * 13) - k[m][n] * *(p + n - 2 + m * 13);
            dp[m][n] = ct * dp[m][n - 1] - st * *(p + n - 1 + m * 13) - k[m][n] * dp[m][n - 2];
          }
        }
        S50:
        /*
         TIME ADJUST THE GAUSS COEFFICIENTS
         */
        if(time != otime)
        {
          tc[m][n] = c[m * 13 + n] + dt * cd[m * 13 + n];
          if(m != 0)
            tc[n][m - 1] = c[n * 13 + m - 1] + dt * cd[n * 13 + m - 1];
        }
        /*
         ACCUMULATE TERMS OF THE SPHERICAL HARMONIC EXPANSIONS
         */
        par = ar * *(p + n + m * 13);
        if(m == 0)
        {
          temp1 = tc[m][n] * cp[m];
          temp2 = tc[m][n] * sp[m];
        }
        else
        {
          temp1 = tc[m][n] * cp[m] + tc[n][m - 1] * sp[m];
          temp2 = tc[m][n] * sp[m] - tc[n][m - 1] * cp[m];
        }
        bt = bt - ar * temp1 * dp[m][n];
        bp += (fm[m] * temp2 * par);
        br += (fn[n] * temp1 * par);
        /*
         SPECIAL CASE:  NORTH/SOUTH GEOGRAPHIC POLES
         */
        if(st == 0.0 && m == 1)
        {
          if(n == 1)
            pp[n] = pp[n - 1];
          else
            pp[n] = ct * pp[n - 1] - k[m][n] * pp[n - 2];
          parp = ar * pp[n];
          bpp += (fm[m] * temp2 * parp);
        }
      }
    }
    if(st == 0.0)
      bp = bpp;
    else
      bp /= st;
    /*
     ROTATE MAGNETIC VECTOR COMPONENTS FROM SPHERICAL TO
     GEODETIC COORDINATES
     */
    bx = -bt * ca - br * sa;
    by = bp;
    bz = bt * sa - br * ca;
    /*
     COMPUTE DECLINATION (DEC), INCLINATION (DIP) AND
     TOTAL INTENSITY (TI)
     */
    bh = sqrt((bx * bx) + (by * by));
    *ti = sqrt((bh * bh) + (bz * bz));
    *dec = atan2(by, bx) / dtr;
    *dip = atan2(bz, bh) / dtr;
    /*
     COMPUTE MAGNETIC GRID VARIATION IF THE CURRENT
     GEODETIC POSITION IS IN THE ARCTIC OR ANTARCTIC
     (I.E. GLAT > +55 DEGREES OR GLAT < -55 DEGREES)

     OTHERWISE, SET MAGNETIC GRID VARIATION TO -999.0
     */
    *gv = -999.0;
    if(fabs(glat) >= 55.)
    {
      if(glat > 0.0 && glon >= 0.0)
        *gv = *dec - glon;
      if(glat > 0.0 && glon < 0.0)
        *gv = *dec + fabs(glon);
      if(glat < 0.0 && glon >= 0.0)
        *gv = *dec + glon;
      if(glat < 0.0 && glon < 0.0)
        *gv = *dec - fabs(glon);
      if(*gv > +180.0)
        *gv -= 360.0;
      if(*gv < -180.0)
        *gv += 360.0;
    }
    otime = time;
    oalt = alt;
    olat = glat;
    olon = glon;
  }
}

void geomag(int *maxdeg)
{
  E0000(0, maxdeg, 0.0, 0.0, 0.0, 0.0, NULL, NULL, NULL, NULL);
}

void geomg1(float alt, float glat, float glon, float time, float *dec, float *dip, float *ti, float *gv)
{
  E0000(1, NULL, alt, glat, glon, time, dec, dip, ti, gv);
}

//gets declination from lat/lon/year/height above sea level
//status bits:
//MAG_WEAK = 0x01 => weak magnetic field
//MAG_VERYWEAK = 0x02 => very weak magnetic field
//MAG_ERROR = 0x03 => error, returned 0
int getDeclination(int lat, int lon, int height, int year, int *status)
{
  int warn_H, warn_H_strong, warn_P;
  static int maxdeg;
  static float altm, dlat, dlon;
  static float alt, time, dec, dip, ti, gv;
  static float dec1, dip1, ti1;
  char decd[5];
  float h1;
  float rTd = 0.017453292;

  maxdeg = 12;
  warn_H = 0;
  warn_H_strong = 0;
  warn_P = 0;

  geomag(&maxdeg);

  dlat = (float)lat / 10000000;
  dlon = (float)lon / 10000000;
  altm = height;

  alt = altm / 1000;

//time: 1.0=1 year
  time = year;

  geomg1(alt, dlat, dlon, time, &dec, &dip, &ti, &gv);
  dec1 = dec;
  dip1 = dip;
  ti1 = ti;

  /*COMPUTE X, Y, Z, AND H COMPONENTS OF THE MAGNETIC FIELD*/

  h1 = ti1 * (cos((dip1 * rTd)));

  /* deal with geographic and magnetic poles */
  if(h1 < 100.0) /* at magnetic poles */
  {
    dec1 = 0.0;
    strcpy(decd, "(VOID)");
    /* while rest is ok */
  }

  if(h1 < 1000.0)
  {
    warn_H = 0;
    warn_H_strong = 1;
  }
  else if(h1 < 5000.0 && !warn_H_strong)
  {
    warn_H = 1;
  }

  if(90.0 - fabs(dlat) <= 0.001) /* at geographic poles */
  {
    dec1 = 0.0;
    warn_P = 1;
    warn_H = 0;
    warn_H_strong = 0;
    /* while rest is ok */
  }

  *status = 0;

  if(warn_H)
  {
    *status |= 0x01;
  }

  if(warn_H_strong)
  {
    *status |= 0x02;
  }

  if(warn_P)
  {
    *status |= 0x04;
    return 0;
  }

  estimatedInclination = (int)((float)dip1 * 10.);
  if(estimatedInclination < 0)
    estimatedInclination *= -1;

  return (int)((float)dec1 * 1000);
}
