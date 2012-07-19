/*
 *  Copyright (c) 2003 by Matt Cross <matt@dragonflyhollow.org>
 *
 *  This file is part of the firemarshalbill package.
 *
 *  Firemarshalbill is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Firemarshalbill is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Firemarshalbill; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Routines for doing math with 16.16 fixed point numbers.
 */

#include <stdio.h>
#include "f16_16.h"
#include "robot_trace.h"

f16_16
mult_f16_16(f16_16 a, f16_16 b)
{
  int a_frac;
  int a_int;
  int b_frac;
  int b_int;
  int sign = 1;

  if (a < 0) {
    sign = -sign;
    a = -a;
  }
  if (b < 0) {
    sign = -sign;
    b = -b;
  }
  a_frac = a & 0x0000ffff;
  a_int  = (a & 0xffff0000) / 65536;
  b_frac = b & 0x0000ffff;
  b_int  = (b & 0xffff0000) / 65536;

  return (((unsigned)(a_frac * b_frac) >> 16) +
	  (a_frac * b_int) +
	  (a_int * b_frac) +
	  ((a_int * b_int) << 16)) * sign;
}

/* Loses some precision in the fractional part of the divide, but may
   be good enough. */
f16_16
div_f16_16(f16_16 a, f16_16 b)
{
  long long all = (long long)a << 16;
  long long bll = (long long)b;

  if (bll == 0) {
    TRACE_LOG2(ROBOT, F16_16_DIV_ZERO, a, b);
    return (0x7fffffff);
  } else {
    return (f16_16)((all + bll/2)/bll);
  }
}

void
print_f16_16(f16_16 a)
{
  if (a < 0) {
    printf ("-");
    a = -a;
  }
  printf("%ld.%04ld", a / 65536, ((a % 65536) * 10000) / 65536);
}

double
double_from_f16_16(f16_16 a)
{
  return ((double)a / 65536);
}

f16_16
f16_16_from_double(double a)
{
  return ((f16_16)(a * 65536));
}
