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
 * Motor Controller API
 */

#ifndef _MOTOR_H
#define _MOTOR_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

/* Motor number of left motor. */
#define MOT_LEFT		0

/* Motor number of right motor. */
#define MOT_RIGHT		1

/**********************************************************************/
/* Types */
/**********************************************************************/

typedef unsigned int uint32;
typedef signed int int32;

/* All velocities are in steps/tick as a 24.8 number */
typedef int32 mot_velocity_t;

/* All accelerations are in steps/tick/tick as a 24.8 number */
typedef int32 mot_accel_t;

typedef struct mot_status
{
  uint32 pos;
  mot_velocity_t velocity;
  mot_accel_t accel;
  int stopped;
  int heading_stopped;
  uint32 tick;
  int emergency;
} mot_status_t;

/* If 'emergency' is set in the mot_status_t, the motor task detected
   a dangerous overbalance/position condition, stopped everything, and
   is trying to correct.  The higher level task needs to wait for the
   emergency condition to clear, and then try to resume where it left
   off. */

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the motor controller.  Returns 0 on success, non-zero
   on error. */
int mot_init(void);

/* Get the PID loop constants.  Values are in 24.8 format. */
void mot_get_pid(int32 *kp, int32 *kd, int32 *ki);

/* Set the PID loop constants.  Values are in 24.8 format. */
void mot_set_pid(int32 kp, int32 kd, int32 ki);

/* All of the motor control is based on 10ms ticks.  This gets the
   current tick count. */
uint32 mot_get_ticks(void);

/* Get the motor status. */
void mot_get_status(mot_status_t *mot);

/* Ramp the chassis up (or down) to velocity 'v' using acceleration 'a'
   starting at time 'tick'.
   NOTE: a and v are in 24.8 format. */
int mot_set_vel(mot_accel_t a, mot_velocity_t v, uint32 tick);

/* Return how many steps it will take to stop given a certain velocity
   and acceleration. */
int mot_calc_stop_dist(mot_accel_t a, mot_velocity_t v);

/* Move the chassis forward (or backward) 's' steps, using
   acceleration 'a' and max velocity 'v', starting at time 'tick'.
   NOTE: all values should be positive, except velocity can be negative
   to indicate a backwards move.
   NOTE: a and v are in 24.8 format. */
int mot_move(int32 s, mot_accel_t a, mot_velocity_t v, uint32 tick);

/* Turn on or off balancing - the motor task will monitor the gyro
   output and keep the bot on it's feet.  If 'on' is 0, then balancing
   will be turned off; otherwise it will be turned on. */
int mot_balance(int on);

/* Returns non-zero if the robot is balancing now. */
int mot_balancing(void);

/* Get the balancing PID loop constants.  Values are in 24.8 format. */
void mot_get_bal_pid(int32 *kp, int32 *kd, int32 *ki);

/* Set the PID loop constants.  Values are in 24.8 format. */
void mot_set_bal_pid(int32 kp, int32 kd, int32 ki);

/* Dump out the PID trace to the console. */
void mot_dump_pid_trace(void);

/* Get the current heading of the robot (24.8 number in degrees). */
int mot_get_heading(void);

/* Set a heading to turn to (24.8 number in degrees and degrees/s). */
int mot_set_heading(int new_heading, int heading_vel);

/* Receive a heading update from the distance task.  This gives us an
   estimate of our heading relative to one or both walls to our sides.
   0 degrees is lined up with the wall.  It will only give us an angle
   above 270 or below 90; we will only consider angles above 315 and
   below 45, as we should be lined up with a wall most of the time.
   We will then have to compare that to the quadrant we are in, and if
   we think this update is valid, we update our heading.  Note the
   angle is an integer from 0 to 359. */
void mot_heading_update(int update_angle);

/* Get the heading maintenance PID loop constants.  Values are in 16.16
   format. */
void mot_get_hd_pid(int32 *kp, int32 *kd, int32 *ki);

/* Set the heading maintenance PID loop constants.  Values are in
   16.16 format. */
void mot_set_hd_pid(int32 kp, int32 kd, int32 ki);

#endif /* _MOTOR_H */
