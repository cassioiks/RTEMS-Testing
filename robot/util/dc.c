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

/* Simple program used to calculate tables for sharp gp2d02 distance sensors. */

#include <stdio.h>
#include <math.h>

#define D0
/* #define D1 */
/* #define D2 */
/* #define D3 */
/* #define D4 */

#define DO_DIST_TBL

#ifdef D0
int min = 57;
double K1 = 1000;
double K2 = 1.5; /* 1.4 */
double K3 = 29; /* 35 */
#endif

#ifdef D1
int min = 95;
double K1 = 1000;
double K2 = 1.15; /* 1.5 */
double K3 = 74; /* 70 */
#endif

#ifdef D2
int min = 90;
double K1 = 1000;
double K2 = 1.475; /* 1.6 */
double K3 = 62; /* 61 */
#endif

int main(void)
{
  int i;
  double dist;

  printf ("K1: %.2lf K2:%.2lf K3 %.3lf\n", K1, K2, K3);

  for (i=0; i<256; i++)
    {
      dist = K2 / (tan((i - K3)/K1));
      printf ("%02x: %.2lfcm (%.2lfin)\n", i, dist, dist/2.54);
    }

#ifdef DO_DIST_TBL
  printf ("\nsigned char dist_tbl[255] =\n{\n");
  for (i=0; i<255; i++)
    {
      if (i < min)
	dist = -1;
      else
	dist = (K2 / (tan((i - K3)/K1))) / 2.54 * 10;
      switch (i%10)
	{
	case 0:
	  printf ("  /* %3d */ %3d, ", i, (int)dist);
	  break;

	case 9:
	  printf("%3d,\n", (int)dist);
	  break;

	default:
	  printf("%3d, ", (int)dist);
	}
    }
  printf("\n};\n\n");
#endif /* DO_DIST_TBL */

}
