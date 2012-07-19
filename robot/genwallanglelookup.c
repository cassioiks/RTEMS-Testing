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

/* Generates a lookup table for the robot to use to calculate it's
 * angle to a wall using the reading from a pair of distance sensors.
 * Feed into the table the difference between the distance sensors,
 * and get out an angle.
 *
 * Distances are measured in deci-inches, posible values are -250 to
 * 250.  To access the table, ensure that it is within this range, and
 * add 250.  For sensors on the right, do rear minus front, on the
 * left do front minus rear.
 *
 * The output of the table is an angle indicating the number of
 * degrees clockwise from parallel to the wall the robot is (negative
 * indicates counterclockwise).  The values ar in fixed 24.8 format.
 */

#include <stdio.h>
#include <math.h>

#define DIST_BETWEEN_SENSORS 40 /* in deci inches */

int
main(void)
{
  int dist;
  double ratio, angle;
  int f24_8_angle;

  printf ("int angle_lookup_table[] =\n{\n");
  for (dist=-250; dist <= 250; dist++)
    {
      ratio = (double)dist / (double)DIST_BETWEEN_SENSORS;
      angle = atan(ratio) * 180 / M_PI;
      f24_8_angle = (int) (angle * 256 + 0.5);

      switch (dist % 10) {
      case 0:
	printf ("  /* %4d */ %d, ", dist, f24_8_angle);
	break;

      case 9:
      case -1:
	printf ("%d,\n", f24_8_angle);
	break;

      default:
	printf ("%d, ", f24_8_angle);
      }
    }
  printf ("};\n");
}
