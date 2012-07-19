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
 * Accelerometer API
 */

#ifndef _ACCEL_H
#define _ACCEL_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

#define ACCEL_TPU_PIN	9

#define ACCEL_0G	20230
#define ACCEL_1G	5130

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the accelerometer. */
void accel_init (void);

/* Read the most recent raw reading from the accelerometer. */
unsigned int accel_read_raw (void);

/* Read the most recent accelerometer reading.  Output is G's as a
   16.16 fixed point number.  Note that it may be negative - a
   positive reading indicates a forward tilt of the platform. */
int accel_read (void);

#endif
