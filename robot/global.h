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
 * Global Variables
 */

#ifndef _GLOBAL_H
#define _GLOBAL_H

#include <bsp.h>
#include "f16_16.h"

/**********************************************************************/
/* Constants */
/**********************************************************************/

/* Servo-related constants. */
#define FLAME_CAP_SERVO		14
#define FLAME_CAP_UP		90
#define FLAME_CAP_DOWN		60

#define FAN_SERVO		15
#define FAN_SERVO_STOPPED	-40
#define FAN_SERVO_RUNNING	-20 /* actually really slow, can go up to +20 */

/* Bits on port E */
#define PORTE_MOTOR_ON		0x01 /* output */
#define PORTE_BALANCE_ON	0x08 /* input */
/* #define PORTE_TONE_DETECT	0x20 */ /* input */
#define PORTE_BUTTON0		0x40
#define PORTE_BUTTON1		0x80

/* A minimally strong IR phototransistor reading to indicate a candle
   position. */
#define MIN_READING 0x20

/* A really strong IR phototransistor reading - indicating the candle
   is really close (IE right in front of us). */
#define STRONG_READING 0x800

#define M_PI		3.14159265358979323846
#define M_PI_2		1.57079632679489661923

/* compass calibration values */
#define COMPASS_CAL_X 5
#define COMPASS_CAL_Y 13

/* Approximate number of motor encoder steps in an inch of motion of
   the robot. */
#define MOT_STEPS_PER_INCH	61

/* Width of wheel base, in steps. */
#define MOT_WHEEL_BASE		410

/* Distance between front and rear distance sensors, in deci-inches.
 * They ar 3 13/16" apart, which is about 3.8" or 38 deci-inches.
 */
#define DIST_SENSOR_SEPARATION	38

/* max heading correction to avoid hitting or getting too far from a
   wall. */
#define DIR_MAX_CORRECT 15

/* heading correction when too close to a wall - a turn of DIR_CORRECT
   degrees will be accomplished every DIR_CORRECT_STEPS motor
   steps. */
#define DIR_CORRECT 4
#define DIR_CORRECT_STEPS MOT_STEPS_PER_INCH

#define DIR_CORRECT_OK(m0,m1,m0_lwa,m1_lwa) \
	(((m0.pos - m0_lwa) > DIR_CORRECT_STEPS) && \
	 ((m1.pos - m1_lwa) > DIR_CORRECT_STEPS))

/**********************************************************************/
/* Macros */
/**********************************************************************/

#define MAX(a,b)	((a) > (b) ? (a) : (b))
#define MIN(a,b)	((a) < (b) ? (a) : (b))

/**********************************************************************/
/* Globals */
/**********************************************************************/

/* Number of ticks per second. */
rtems_interval ticks_per_sec;

/**********************************************************************/
/* Functions */
/**********************************************************************/

static inline int
fixup_angle_f16_16(f16_16 angle)
{
  if (angle < 0)
    return ((360*65536) - ((-angle) % (360*65536)));
  else
    return (angle % (360*65536));
}

static inline int
fixup_angle_24_8(int angle)
{
  if (angle < 0)
    return ((360*256) - ((-angle) % (360*256)));
  else
    return (angle % (360*256));
}

static inline int
fixup_angle(int angle)
{
  if (angle < 0)
    return (360 - ((-angle) % 360));
  else
    return (angle % 360);
}

/* Returns the absolute difference between two headings in degress. */
static inline int
abs_dir_diff(int dir1, int dir2)
{
  int diff1, diff2;

  diff1 = fixup_angle(dir1 - dir2);
  diff2 = fixup_angle(dir2 - dir1);

  if (diff1 < diff2)
    return diff1;
  else
    return diff2;
}

/* Returns the absolute difference between two headings in degress. */
static inline int
abs_dir_diff_24_8(int dir1, int dir2)
{
  int diff1, diff2;

  diff1 = fixup_angle_24_8(dir1 - dir2);
  diff2 = fixup_angle_24_8(dir2 - dir1);

  if (diff1 < diff2)
    return diff1;
  else
    return diff2;
}

/* Returns the absolute difference between two headings in degress. */
static inline int
abs_dir_diff_f16_16(f16_16 dir1, f16_16 dir2)
{
  f16_16 diff1, diff2;

  diff1 = fixup_angle_f16_16(dir1 - dir2);
  diff2 = fixup_angle_f16_16(dir2 - dir1);

  if (diff1 < diff2)
    return diff1;
  else
    return diff2;
}

static inline int
leftmost_angle(int a1, int a2)
{
  int diff1, diff2;

  diff1 = fixup_angle(a1 - a2);
  diff2 = fixup_angle(a2 - a1);

  /* diff1 is how many degrees we would have to add to a2 to get a1,
     diff2 is how many degrees we would have to add to a1 to get a2.
     If diff1 is smaller, then a2 is to the left of a1. */
  if (diff1 < diff2)
    return a2;
  else
    return a1;
}


#endif /* _GLOBAL_H */
