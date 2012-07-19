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
 * High-level robot control code
 */

#ifndef _ROBOT_H
#define _ROBOT_H

/**********************************************************************/
/* Globals */
/**********************************************************************/

/* Overshoot/undershoot correction for turns.  Averaged and adjusted
   automatically.  Note that this is a 24.8 number. */
extern long heading_movement_factor;

/* Default acceleration and velocity. */
extern long robot_acc;
extern long robot_vel;

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Turn the robot to face a specific heading. */
int robot_turn_to(int heading);

/* Calculate the distance & andle to the candle by reading the two
   infrared arrays.  Returns 1 if the candle is out of range, 0
   on success (with distp & anglep filled in).  distp is in
   deci-inches; anglep is in degrees. */
int find_candle(int *distp, int *anglep, int *highest_readingl,
		int *highest_readingr);


/* Routine to find and put out the candle.  Should only be called when
   it has been detected that the candle is in the vicinity.  (IE the
   flame sensor returns >2).  Returns 0 if it put out the fire,
   non-zero on failure. */
int put_out_fire(void);

/* Routine to run through the maze, find the candle, put it out, and
   return back home. */
int run_maze(void);

#if 0
/* A little to the left; a little to the right... */
extern void shimmy(int sideways_dist, int angle, int speed, int *dist_leftp,
		   int *movingp, uint32 *m0startposp, uint32 *m1startposp);
#endif

/* test routine, run by 'demo 4' command. */
int doit(void);

#endif /* _ROBOT_H */
