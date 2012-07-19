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

/* #define METHOD1 */
#define METHOD2

int main(void)
{
  int i;
  double dist;
  double x_3, x_18;
  double kg, ko;
  char buf[80];

  printf ("Enter reading at 3 inches: ");
  fgets(buf, sizeof(buf), stdin);
  sscanf(buf, "%lf", &x_3);
  printf ("Enter reading at 18 inches: ");
  fgets(buf, sizeof(buf), stdin);
  sscanf(buf, "%lf", &x_18);

  /* method 1: */
#ifdef METHOD1
  kg = (x_18 - x_3) * 18.0 * 3.0 / (3.0 - 18.0);
  ko = (18.0 * x_18 - 3.0 * x_3) / (18.0 - 3.0);
#endif

  /* method 2: */
#ifdef METHOD2
  ko = (18.0 * x_18 - 3.0 * x_3) / (x_3 - x_18);
  kg = x_3 * x_18 * (3.0 - 18.0) / (x_18 - x_3);
#endif

  for (i=0; i<256; i++)
    {
#ifdef METHOD1
      dist = kg / (i - ko);
#endif

#ifdef METHOD2
      dist = kg / i - ko;
#endif

      printf ("%02x: %.2lfcm (%.2lfin)\n", i, dist*2.54, dist);
    }

#ifdef DO_DIST_TBL
  printf ("\nsigned short dist_tbl[255] =\n{\n");
  for (i=0; i<255; i++)
    {
      if (i <= ko)
	dist = -1;
      else {
#ifdef METHOD1
	dist = kg / (i - ko);
#endif

#ifdef METHOD2
	dist = kg / i - ko;
#endif
      }
      if ((dist < 3.0) || (dist > 25.0))
	dist = -1;
      if (dist > 0)
	dist *= 10.0;
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
