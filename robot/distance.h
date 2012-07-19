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
 * Distance Sensor API
 */

#ifndef _DISTANCE_H
#define _DISTANCE_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

/* Constants used to specify which sensor to to read. */
#define DISTANCE_FRONT	0
#define DISTANCE_LEFT	1
#define DISTANCE_RIGHT	2
#define DISTANCE_LEFT_REAR	3
#define DISTANCE_RIGHT_REAR	4

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the distance sensor task.  Returns 0 on success,
   non-zero on error. */
int distance_init(void);

/* Read a distance sensor.  The return value is a 32-bit unsigned
   number specifying centi-inches (inches * 10).  A negative return
   value indicates that the closest object is out of range. */
long distance_read(int which_sensor);

/* Read the raw value from a distance sensor.  Returns a number in the
   range 0 to 255. */
unsigned long distance_read_raw(int which_sensor);

#endif /* _DISTANCE_H */
