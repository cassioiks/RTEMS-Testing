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
 * Kalman filter
 *
 * A kalman filter to filter the output from a gyro sensor and
 * accelerometer to calculate the actual current tilt of the robot.
 */

#ifndef _KALMAN_H
#define _KALMAN_H

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the kalman filter task. */
void kalman_init(void);

/* Get the current tilt of the platform.  Output is degrees as a 16.16
   number.  Note that a forward tilt is a positive number, and a
   backwards tilt is a negative number. */
int kalman_read(void);

/* Get the most recent 'theta_m' reading (which is the accelerometer
   reading converted to degrees). */
int kalman_read_theta_m(void);

/* Get the most recent gyro only reading (which is the integrated gyro
   readings). */
int kalman_read_gyro_only(void);

#endif /* _KALMAN_H */
