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

#define DO_DIST_TBL
#define MAX_ITER 1000

int main(void)
{
  int i;
  double dist;
  double k1, k2, k2_high, k2_lo, k3;
  double x_3, x_6, x_18;
  char buf[80];

  printf ("Enter reading at 3 inches: ");
  fgets(buf, sizeof(buf), stdin);
  sscanf(buf, "%lf", &x_3);
  printf ("Enter reading at 6 inches: ");
  fgets(buf, sizeof(buf), stdin);
  sscanf(buf, "%lf", &x_6);
  printf ("Enter reading at 18 inches: ");
  fgets(buf, sizeof(buf), stdin);
  sscanf(buf, "%lf", &x_18);

  /* Iterate to find a good value for K2.  Derive K1 and K3 using K2
     and the readings at 3" and 18", then see what we get for 6". */
  k2_high = 100;
  k2_lo = 0.01;

  for (i=0; i<MAX_ITER; i++) {
    k2 = (k2_lo + k2_high) / 2;

    /* These were derived from the base equation for calculating the
       distance. */
    k1 = (x_18 - x_3) / (atan (k2 / 18.0) - atan (k2 / 3.0));

    k3 = x_3 - (atan(k2/3.0) * k1);

    dist = k2 / (tan((x_6 - k3)/k1));

    if (fabs(dist - 6.0) < 0.01)
      /* good enough */
      break;

    if (dist < 6.0) {
      /* k2 is too low. */
      k2_lo = k2;
    } else {
      /* k2 is too high. */
      k2_high = k2;
    }
  }

  if (i == MAX_ITER) {
    printf ("After 1000 iterations, not converging!\n");
  }

  printf ("k1: %.2lf k2:%.2lf k3 %.3lf\n", k1, k2, k3);

  for (i=0; i<256; i++)
    {
      dist = k2 / (tan((i - k3)/k1));
      printf ("%02x: %.2lf\n", i, dist);
    }

#ifdef DO_DIST_TBL
  printf ("\nsigned short dist_tbl[255] =\n{\n");
  for (i=0; i<255; i++)
    {
      dist = (k2 / (tan((i - k3)/k1)));
      if ((dist < 2.5) || (dist > 25.0))
	dist = -1;
      else
	dist *= 10;
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
