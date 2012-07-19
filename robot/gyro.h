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
 * Gyro API
 *
 * API for reading a Gyro attached to an A2D port on an MRM board.
 */

#ifndef _GYRO_H
#define _GYRO_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

#define GYRO_X	0	/* X-axis gyro - gives tilt of platform. */
#define GYRO_Z	1	/* Z-axis gyro - gives heading of platform. */

#if 0
/* simple filter gyro board: */
#define GYRO_X_NEUTRAL_DEFAULT	13986693 /* 213.42 as a 16.16 fixed */
#define GYRO_Z_NEUTRAL_DEFAULT	7818903 /* 119.30 as a 16.16 fixed */
#else
/* complex filter gyro board: */

#define GYRO_X_NEUTRAL_DEFAULT	8056995 /* 122.93 as a 16.16 fixed */
#define GYRO_Z_NEUTRAL_DEFAULT	7818903 /* 119.30 as a 16.16 fixed */
#endif

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the gyro library.  Returns 0 on success, non-zero on
   error. */
int gyro_init(void);

/* Returns the current reading of the gyro.  Return value is a 16.16
   fixed point number in degrees per second.  The 'gyro' parameter
   should be either GYRO_X or GYRO_Z. */
int gyro_read(int gyro);

/* Reads the current neutral settings of the gyros, as 16.16 fixed
   point numbers. */
void gyro_read_neutral(int *gyro_x_neutral_p, int *gyro_z_neutral_p);

/* Sets the current neutral settings of the gyros, as 16.16 fixed
   point numbers. */
void gyro_set_neutral(int gyro_x_neutral_new, int gyro_z_neutral_new);

/* Calculate the current neutral settings of the gyro.  The gyro
   _must_ be kept still while this is going on.  It will take 10
   seconds.  NOTE: Only one thread is allowed to call this routine at
   a time! */
void gyro_calibrate(void);

#endif /* _GYRO_H */
