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

#include <bsp.h>
#include <stdio.h>
#include <stdlib.h>
#include "motor.h"
#include "distance.h"
#include "global.h"
#include "mcp3208.h"
#include "servo.h"
#include "flame.h"
#include "fastint.h"
#include "f16_16.h"
#include "robot_trace.h"
#include <math.h>
#include <sim.h>

/**********************************************************************/
/* Types */
/**********************************************************************/

typedef enum stop_condition
{
  NOT_STOPPED,
  WALL_IN_FRONT,
  WALL_ON_LEFT,
  WALL_ON_RIGHT,
  NO_WALL_ON_LEFT,
  NO_WALL_ON_RIGHT,
  NO_WALL_ON_LEFT_OR_RIGHT,
  DIST_COMPLETE
} stop_condition_t;

/**********************************************************************/
/* Constants */
/**********************************************************************/

/* When we we are within this many degrees of the heading, that is
   as good as we can get. */
#define HEADING_CLOSE_ENOUGH 3

/* When we we are within this many deci-inches of the target distance,
   that is as good as we can get. */
#define DIST_CLOSE_ENOUGH 10

/* Number of ticks in a full circle (IE distance between wheels *
   3.14159) */
#define ROBOT_TURN_CIRCUMFERENCE 1270

/* Amount of 'fudge factor' to add in put_out_fire after found the
   candle via a full scan. */
#define SCAN_FUDGE_FACTOR 0

/* Distance from candle that we're aiming for in deci-inches. */
#define CANDLE_DIST 100

/* Means don't go a specific distance (when passed to drive_straight). */
#define FOREVER -1

/**********************************************************************/
/* Globals */
/**********************************************************************/

/* Overshoot/undershoot correction for turns.  Averaged and adjusted
   automatically.  Note that this is a 24.8 number. */
long heading_movement_factor = 0x100;

/* Default acceleration and velocity. */
long robot_acc = 0x001; /* accel is 1/256 step per tick, so it takes
			   it one second to go from 0 steps/tick to 1
			   step/tick. */
long robot_vel = 0x100; /* 1 step/tick, 250 steps/sec, or about
			   4 inches/sec */

/**********************************************************************/
/* Functions */
/**********************************************************************/

int
wait_for_mot_stopped(int seconds)
{
  int retries;
  mot_status_t ms;
  int emergency = 0;

  for (retries=0; retries<seconds*10; retries++) {
    rtems_task_wake_after(ticks_per_sec/10);
    mot_get_status(&ms);
    if (ms.emergency)
      emergency = 1;
    if ((ms.stopped) && (!ms.emergency))
      return emergency;
  }

  /* we timed out! */
  TRACE_LOG1(ROBOT, STOP_TIMEOUT, seconds);
  return -1;
}

int
stop_motors(void)
{
  mot_status_t ms;
  int s;

  TRACE_LOG0 (ROBOT, STOP_MOTORS);
  mot_get_status(&ms);
  s = mot_calc_stop_dist(robot_acc, ms.velocity);
  mot_move(s, robot_acc, 0, mot_get_ticks()+1);
  return wait_for_mot_stopped(5);
}

int
wait_for_mot_heading_stopped(int seconds)
{
  int retries;
  mot_status_t ms;
  int emergency = 0;

  for (retries=0; retries<seconds*10; retries++) {
    rtems_task_wake_after(ticks_per_sec/10);
    mot_get_status(&ms);
    if (ms.emergency)
      emergency = 1;
    if ((ms.heading_stopped) && (!ms.emergency))
      return emergency;
  }

  /* we timed out! */
  TRACE_LOG1(ROBOT, STOP_HD_TIMEOUT, seconds);
  return -1;
}

int
get_balance(void)
{
  int retval;

  stop_motors();
  retval = wait_for_mot_heading_stopped(5);
  rtems_task_wake_after(ticks_per_sec);

  return retval;
}

/* Turn the robot to face a specific heading. */
int
robot_turn_to(int heading)
{
  int heading_vel = 45 * 256;
  int retval;

  TRACE_LOG4(ROBOT, TURN_TO, heading/256, (heading%256)*100/256,
	     heading_vel/256, (heading_vel%256)*100/256);

  do {
    stop_motors();
    mot_set_heading (heading, heading_vel);

    retval = wait_for_mot_heading_stopped(9);
  } while (retval > 0); /* retry on emergencies. */

  return 0;
}

/* 62.5 deci-inches as a 16.16 fixed number. */
#define EYE_DIST_F16_16		4096000

int
calc_flame_pos(int angle_l, int angle_r,
	       int *px, int *py)
{
  f16_16 cl;
  f16_16 cr;
  f16_16 sinval, cosval;

  if (angle_l == 0)
    cl = 0;
  else
    {
      sinval = isinof[90 - angle_l] << 2;
      cosval = icosof[90 - angle_l] << 2;
      /* div_f16_16(sinval,cosval) forms the equivalent of tan(90 - angle_l) */
      cl = div_f16_16(-1 * 65536, div_f16_16(sinval, cosval));
    }

  if (angle_r == 0)
    cr = 0;
  else
    {
      sinval = isinof[(90 + angle_r) % 360] << 2;
      cosval = icosof[(90 + angle_r) % 360] << 2;
      /* div_f16_16(sinval,cosval) forms the equivalent of tan(90 + angle_r) */
      cr = div_f16_16(1 * 65536, div_f16_16(sinval, cosval));
    }

  if (cl == cr) /* both sensors see flame at the same angle?? */
    return -1;

  *px = round_f16_16 (div_f16_16 (mult_f16_16 (-EYE_DIST_F16_16/2, (cr + cl)),
				  (cr - cl)));
  *py = round_f16_16 (div_f16_16 (-EYE_DIST_F16_16, (cl - cr)));

  return 0;
}

/* Read one of the infrared phototransistor arrays to determine the
   angle to the candle.  Fills in anglep with a relative angle in
   degrees and returns 0 on success; returns non-zero on failure.
   The left sensor is number 0; the right sensor is number 1. */

int robot_ir_angles[8] = { -27, -20, -12, -4, 4, 12, 20, 27 };

int
read_array(int sensor, int *anglep, int *highest_reading)
{
  int chans[8];
  unsigned short results[8];
  unsigned sums[8];
  unsigned avg;
  int hi, hi2;
  int hi_idx, hi2_idx;
  int num_samples = 4;
  int angle;
  int i, j;

  /* Init. */
  for (i=0; i<8; i++)
    {
      sums[i] = 0;
      chans[i] = i;
    }

  /* Poll all IR phototransistors num_samples times. */
  for (i=0; i<num_samples; i++)
    {
      mcp3208_read(sensor, 4, chans, results);
      mcp3208_read(sensor, 4, chans+4, results+4);

      for (j=0; j<8; j++)
	sums[j] += results[j];
    }

  /* Calculate results. */
  hi = hi2 = 0;
  hi_idx = hi2_idx = -1;
  avg = 0;
  for (i=0; i<8; i++)
    {
      sums[i] = (sums[i] + num_samples/2) / num_samples;

      avg += sums[i];
      if (sums[i] > hi)
	{
	  hi2 = hi;
	  hi2_idx = hi_idx;
	  hi = sums[i];
	  hi_idx = i;
	}
      else if (sums[i] > hi2)
	{
	  hi2 = sums[i];
	  hi2_idx = i;
	}
    }
  avg = (avg+4)/8;

  if ((hi2_idx >= 0) &&
      (abs(hi_idx - hi2_idx) == 1))
    {
      angle = (robot_ir_angles[hi_idx] * hi +
	       robot_ir_angles[hi2_idx] * hi2) / (hi + hi2);
    }
  else
    {
      angle = robot_ir_angles[hi_idx];
    }

  if ((hi > (avg * 3 / 2)) &&
      (hi > MIN_READING))
    {
      *highest_reading = hi;
      *anglep = angle;
      return 0;
    }
  else
    {
      *highest_reading = hi;
      return 1;
    }
}

/* Calculate the distance & angle to the candle by reading the two
   infrared arrays.  Returns 1 if the candle is out of range to the
   left, 2 if the candle is out of range to the right, -1 if out of
   range and can't tell which side, 0 on success (with distp & anglep
   filled in).  distp is in deci-inches; anglep is in degrees. */
int
find_candle(int *distp, int *anglep, int *highest_readingl,
	    int *highest_readingr)
{
  int anglel, angler, ret0, ret1;
  int real_anglel, real_angler;
  int x, y;

  ret0 = read_array(0, &anglel, highest_readingl);
  ret1 = read_array(1, &angler, highest_readingr);

  if ((ret0 != 0) || (ret1 != 0))
    {
      if (ret0 == 0) {
	TRACE_LOG1(ROBOT, FIND_CANDLE_RET, 1);
	return 1; /* left array can see it, right array can't. */
      } else if (ret1 == 0) {
	TRACE_LOG1(ROBOT, FIND_CANDLE_RET, 2);
	return 2; /* right array can see it, left array can't. */
      } else {
	TRACE_LOG1(ROBOT, FIND_CANDLE_RET, -1);
	return -1; /* neither array can see it. */
      }
    }

  /* Both arrays can see the candle - calculate distance.  NOTE: both
     arrays are turned inwars at about a 7 degree angle. */
  real_anglel = fixup_angle(anglel + 7);
  real_angler = fixup_angle(angler - 7);
  if (calc_flame_pos(real_anglel, real_angler, &x, &y) != 0)
    {
      /* The only way this can happen is if both angles are 0.  Assume
         the candle is just really far straight ahead... */
      *distp = 500;
      *anglep = 0;
      TRACE_LOG6 (ROBOT, FIND_CANDLE, real_anglel, real_angler, x, y, *distp, *anglep);
      return 0;
    }
  else
    {
      *distp = (int) (sqrti(x*x + y*y));
      *anglep = 90 - fastatan2(y, x);
      TRACE_LOG6 (ROBOT, FIND_CANDLE, real_anglel, real_angler, x, y, *distp, *anglep);

      if (abs(*anglep)>45) {
	/* should not be possible!  something went wrong with our
	   calculations.  Just tell caller we could not see the
	   candle. */
	TRACE_LOG0(ROBOT, FIND_CANDLE_BADCALC);
	return -1;
      }
      return 0;
    }
}

/* Routine to find and put out the candle.  Should only be called when
   it has been detected that the candle is in the vicinity.  (IE the
   flame sensor returns >2).  Returns 0 if it put out the fire,
   non-zero on failure. */
int
put_out_fire(void)
{
  int dist, angle, hi_l, hi_r;
  int best_angle, highest;
  int best_angle_flame, highest_flame, flame;
  int retval;
  int heading, curheading;
  uint32 ticks;
  int num_nocandle_checks;
  mot_status_t m0;
  uint32 m0pos_orig = 0;
  uint32 dist_to_move = 0;
  int moving;
  long front_dist, left_dist, right_dist;
  int uv_only;
  int i;
  int turn_done;
  int had_emergency = 0;

  TRACE_LOG0 (ROBOT, POF);

  /* Lower the shutter over the UVTRON. */
  servo_set(FLAME_CAP_SERVO, FLAME_CAP_DOWN);

  /* Sleep for a second to let flame sensor history clear itself */
  rtems_task_wake_after(ticks_per_sec);

  while (1)
    {
      /* Get initial heading. */
      stop_motors();
      rtems_task_wake_after(ticks_per_sec/2);

      heading = mot_get_heading();
      TRACE_LOG2 (ROBOT, POF_INIT_HD, heading/256, (heading%256)*100/256);
      best_angle = heading;
      best_angle_flame = heading;

      /* Scan in a complete circle looking for the candle. */
      highest = -1;
      highest_flame = -1;
      ticks = mot_get_ticks();
      for (i=0; i <4; i++)
	{
	  heading = fixup_angle_24_8(heading - 90*256);
	  mot_set_heading (heading, 30*256);

	  turn_done = 0;

	  do {
	    *PORTC ^= 0x10; /* flash green LED. */

	    curheading = mot_get_heading();
	    TRACE_LOG2(ROBOT, HEADING, curheading/256,
		       (curheading%256)*100/256);
	    retval = find_candle(&dist, &angle, &hi_l, &hi_r);

	    if (retval == 0)
	      {
		if (hi_l > hi_r)
		  hi_l = hi_r;

		if (hi_l > highest)
		  {
		    best_angle = fixup_angle_24_8(curheading + angle*256);
		    highest = hi_l;
		    TRACE_LOG3 (ROBOT, POF_NEW_BEST, hi_l,
				best_angle/256, (best_angle%256)*100/256);

#if 0
		    /* what was this trying to do? */
		    if ((hi_l > STRONG_READING) && (hi_r > STRONG_READING))
		      break;
#endif
		  }
	      }

	    curheading = mot_get_heading();
	    TRACE_LOG2(ROBOT, HEADING, curheading/256,
		       (curheading%256)*100/256);
	    retval = flame_read(0);
	    if (retval > highest_flame)
	      {
		best_angle_flame = curheading;
		highest_flame = retval;
		TRACE_LOG3 (ROBOT, POF_NEW_BEST_UV, highest_flame,
			    best_angle_flame/256,
			    (best_angle_flame%256)*100/256);
	      }

	    mot_get_status(&m0);
	    if (m0.heading_stopped && !m0.emergency && !had_emergency) {
	      turn_done = 1;
	    } else if (m0.emergency) {
	      had_emergency = 1;
	    } else if (!m0.emergency && had_emergency) {
	      /* resume turning after emergency */
	      mot_set_heading (heading, 30*256);
	    }
	  } while (!turn_done);
	}
      *PORTC |= 0x10; /* turn off green LED. */
      ticks = mot_get_ticks();

      if ((highest == -1) && (highest_flame == 0))
	{
	  /* Failed to find candle in a complete sweep! */
	  TRACE_LOG0 (ROBOT, POF_FAILED);
	  servo_set(FLAME_CAP_SERVO, FLAME_CAP_UP); /* Return shutter away from UVTRON. */
	  return 1;
	}

      if (highest == -1)
	{
	  /* So far, we've only seen the candle with the UV sensor -
             use that angle. */
	  best_angle = best_angle_flame;
	}

      TRACE_LOG0 (ROBOT, HUNTING_CANDLE);
	      
      /* Turn to the best angle to find the candle & approach it until
         it is between 8 and 9 inches away.  But keep an eye on the
         front distance sensor - if it indicates we're getting closer
         to something than what the infrared arrays indicate, then
         we're probably chasing a shadow - back up 6 inches & try
         again. */
      robot_turn_to(best_angle);

      num_nocandle_checks = 0;
      moving = 0;
      while (1) /* hunting that wascawwy candle! */
	{
	  /* Wait for any emergencies to clear. */
	  do {
	    mot_get_status(&m0);
	    if (m0.emergency) {
	      moving = 0;
	      rtems_task_wake_after(ticks_per_sec/10);
	    }
	  } while (m0.emergency);

	  retval = find_candle(&dist, &angle, &hi_l, &hi_r);
	  flame = flame_read(0);
	  TRACE_LOG1 (ROBOT, FLAME_READ, flame);
	  if ((retval != 0) && ((flame < (highest_flame-2)) || (flame == 0)))
	    {
	      num_nocandle_checks++;
	      TRACE_LOG1 (ROBOT, NUM_NOCANDLE, num_nocandle_checks);
	      if (num_nocandle_checks > 5)
		break; /* We lost the candle! */
	    }
	  else
	    {
	      if (retval != 0)
		{
		   /* we only see the candle with the UV sensor; so we
		      have angle but no distance.  Pretend distance is
		      24 inches. */
		  angle = 0;
		  uv_only = 1;
		  dist = 240;
		}
	      else
		uv_only = 0;

	      num_nocandle_checks = 0;
	      front_dist = distance_read(DISTANCE_FRONT) - 30;
	      left_dist = distance_read(DISTANCE_LEFT) - 30;
	      right_dist = distance_read(DISTANCE_RIGHT) - 30;

	      TRACE_LOG4 (ROBOT, HUNT_STATE,
			  uv_only, front_dist, left_dist, right_dist);

	      if (abs(angle) > 20)
		{
		  stop_motors();
		  moving = 0;
		  rtems_task_wake_after(ticks_per_sec/2);

		  best_angle = fixup_angle_24_8(mot_get_heading() + angle*256);

		  robot_turn_to(best_angle);
		}
	      else
		{
		  if (angle != 0) {
		    best_angle = fixup_angle_24_8(mot_get_heading() +
						  angle*256);
		    TRACE_LOG2 (ROBOT, HEAD_ADJ,
				best_angle/256, (best_angle%256)*100/256);
		    mot_set_heading(best_angle, 60*256);
		  }
		}

	      if (dist < (CANDLE_DIST + 10))
		{
		  /* We're at the candle.  Put it out! */
		  TRACE_LOG0 (ROBOT, ATTEMPT_TO_EXTINGUISH);
		  stop_motors();
		  moving = 0;
		  rtems_task_wake_after(ticks_per_sec/2);

		  servo_set(FAN_SERVO,FAN_SERVO_RUNNING);
		  rtems_task_wake_after(ticks_per_sec/2);
		  servo_set(FAN_SERVO,FAN_SERVO_STOPPED);

		  rtems_task_wake_after(ticks_per_sec);
		  if (flame_read(0) < 2)
		    {
		      /* We did it! */
		      TRACE_LOG0 (ROBOT, FIRE_OUT);
		      servo_set(FLAME_CAP_SERVO, FLAME_CAP_UP); /* Return shutter away from UVTRON. */
		      return 0;
		    }

		  TRACE_LOG0 (ROBOT, ATTEMPT_FAILED);
		  /* Wait for any emergencies to clear. */
		  do {
		    mot_get_status(&m0);
		    if (m0.emergency) {
		      rtems_task_wake_after(ticks_per_sec/10);
		      moving = 0;
		    }
		  } while (m0.emergency);

		  /* Back up a few inches & try again. */
		  ticks = mot_get_ticks();
		  mot_move(MOT_STEPS_PER_INCH * 2, robot_acc,
			   -robot_vel, ticks+1);
		  wait_for_mot_stopped(5);
		  moving = 0;
		  
		}
	      else if (((front_dist > 0) &&
			(front_dist < (CANDLE_DIST - 40)) &&
			(front_dist < (dist - 30)) &&
			(!uv_only)) ||
		       ((front_dist > 0) && (front_dist < 20)))
		{
		  /* The front distance sensor either reports a
                     reading at least 3 inches shorter than what the
                     infrared arrays report (IE we're chasing a
                     reflection) or we are within 2 inches of a wall.
                     Back up 6 inches & try again.  First, stop &
                     regain our equilibrium where we are. */
		  TRACE_LOG0 (ROBOT, REFLECTION);
		  stop_motors();
		  rtems_task_wake_after(ticks_per_sec/2);

		  ticks = mot_get_ticks();
		  mot_move(MOT_STEPS_PER_INCH * 6, robot_acc,
			   -robot_vel, ticks+1);
		  wait_for_mot_stopped(10);
		  moving = 0;
		  break;
		}
	      else if ((dist > 120) &&
		       (left_dist > 0) &&
		       (left_dist < 60))
		{
		  /* Getting too close to left wall - move out a
		     bit. */
		  TRACE_LOG0 (ROBOT, TOO_CLOSE_LEFT);
		  stop_motors();
		  moving = 0;
		  rtems_task_wake_after(ticks_per_sec/2);

		  best_angle = fixup_angle_24_8(mot_get_heading() + angle*256);

		  robot_turn_to(fixup_angle_24_8(best_angle + 90*256));

		  ticks = mot_get_ticks();
		  mot_move(MOT_STEPS_PER_INCH * 6, robot_acc, robot_vel/2,
			   ticks+1);
		  wait_for_mot_stopped(10);

		  robot_turn_to(best_angle);
		  moving = 0;
		  continue;
		}
	      else if ((dist > 120) &&
		       (right_dist > 0) &&
		       (right_dist < 60))
		{
		  /* Getting too close to right wall - move out a
		     bit. */
		  TRACE_LOG0 (ROBOT, TOO_CLOSE_RIGHT);
		  stop_motors();
		  moving = 0;
		  rtems_task_wake_after(ticks_per_sec/2);

		  best_angle = fixup_angle_24_8(mot_get_heading() + angle*256);

		  robot_turn_to(fixup_angle_24_8(best_angle - 90*256));

		  ticks = mot_get_ticks();
		  mot_move(MOT_STEPS_PER_INCH * 6, robot_acc, robot_vel/2,
			   ticks+1);
		  wait_for_mot_stopped(10);

		  robot_turn_to(best_angle);
		  moving = 0;
		  continue;
		}

	      /* Move forward to the candle. */
	      if (!moving)
		{
		  TRACE_LOG0 (ROBOT, INIT_MOVE);
		  if (dist < (CANDLE_DIST + 1))
		    dist = CANDLE_DIST + 1;
		  ticks = mot_get_ticks();
		  mot_get_status(&m0);
		  m0pos_orig = m0.pos;
		  mot_move(MOT_STEPS_PER_INCH * (dist - CANDLE_DIST) / 10,
			   robot_acc, robot_vel/2, ticks+1);
		  moving = 1;
		  dist_to_move = dist - CANDLE_DIST;
		}
	      else
		{
		  uint32 dist_moved;

		  TRACE_LOG0 (ROBOT, ALREADY_MOVE);
		  mot_get_status(&m0);
		  if (m0.stopped) {
		    moving = 0;
		    TRACE_LOG0 (ROBOT, DETECT_STOP);
		  }

		  dist_moved = (m0.pos - m0pos_orig) * 10 / MOT_STEPS_PER_INCH;

		  if ((dist_to_move - dist_moved) > (dist - CANDLE_DIST))
		    {
		      /* Candle was closer than it appeared at first. */
		      m0pos_orig = m0.pos;
		      if (dist < (CANDLE_DIST+1))
			dist = CANDLE_DIST + 1;
		      ticks = mot_get_ticks();
		      mot_move(MOT_STEPS_PER_INCH *
			       (dist - CANDLE_DIST) / 10,
			       robot_acc, robot_vel/2, ticks+1);
		      moving = 1;
		      dist_to_move = dist - CANDLE_DIST;
		    }
		}
	    }
	}
    }
}

void set_front_dist(int desired_dist)
{
  int d, diff, moving = 0, moving_dist = 0, moving_dir = 1, dir;
  uint32 startpos0 = 0;
  mot_status_t m0;
  uint32 ticks;
  int stop_dist;

  TRACE_LOG1(ROBOT, SET_FRONT_DIST, desired_dist);

  while (1)
    {
      d = distance_read(DISTANCE_FRONT)-30;
      TRACE_LOG1(ROBOT, FRONT_DIST, d);

      mot_get_status(&m0);

      if (m0.stopped && (abs (desired_dist-d) <= DIST_CLOSE_ENOUGH))
	{
	  TRACE_LOG0(ROBOT, SFD_DONE);
	  return;
	}
      else
	{
	  diff = d - desired_dist;

	  if (d < 0)
	    {
	      dir = 1; /* forwards. */
	      diff = 180 - desired_dist;
	    }
	  else if (diff > 0)
	    {
	      dir = 1; /* forwards */
	    }
	  else
	    {
	      dir = -1; /* backwards. */
	    }
	  diff = abs(diff);

	  if (moving)
	    {
	      uint32 diff0;
	      uint32 dist_left;

	      /* Wait for any emergencies to clear. */
	      do {
		mot_get_status(&m0);
		if (m0.emergency) {
		  rtems_task_wake_after(ticks_per_sec/10);
		  moving = 0;
		}
	      } while (m0.emergency);
	      stop_dist = mot_calc_stop_dist(robot_acc, m0.velocity);
	      stop_dist = stop_dist * 10 / MOT_STEPS_PER_INCH;

	      if (m0.stopped)
		{
		  moving = 0;
		}
	      else if (moving_dir != dir)
		{
		  /* Hrm - we overshot.  Fix it. */
		  TRACE_LOG0(ROBOT, SFD_OVERSHOT);
		  stop_motors();
		  moving = 0;
		  continue;
		}
	      else
		{
		  if (d >= 0)
		    {
		      if (moving_dir < 0)
			{
			  /* Moving backwards. */
			  diff0 = (startpos0 - m0.pos) * 10 /
			    MOT_STEPS_PER_INCH;
			}
		      else
			{
			  /* Moving forwards. */
			  diff0 = (m0.pos - startpos0) * 10 /
			    MOT_STEPS_PER_INCH;
			}
		      dist_left = moving_dist - diff0 - stop_dist;
		      if ((abs (dist_left - d) > 2 * DIST_CLOSE_ENOUGH) &&
			  (abs (dist_left - d) > stop_dist))
			{
			  /* We're going to miss - adjust. */
			  TRACE_LOG5(ROBOT, SFD_ADJUSTING, dist_left,
				     stop_dist, m0.pos, startpos0, diff0);
			  moving = 0;
			}
		    }
		}
	    }

	  if (moving == 0)
	    {
	      moving = 1;
	      moving_dist = diff;
	      moving_dir = dir;
	      mot_get_status(&m0);
	      startpos0 = m0.pos;

	      TRACE_LOG2(ROBOT, SFD_MOVING, diff, dir);

	      ticks = mot_get_ticks();
	      mot_move(MAX (MOT_STEPS_PER_INCH * moving_dist / 10 -
			    MOT_STEPS_PER_INCH /* hack - we always seem to overcorrect. */,
			    MOT_STEPS_PER_INCH/4),
		       robot_acc, robot_vel/2 * moving_dir, ticks+1);
	    }
	}

      rtems_task_wake_after (ticks_per_sec/20);
    }
}

/* returns non-zero if at wall in front of robot. */
int check_for_front_wall(int front_dist, int *speedp, int *movingp,
			 uint32 *m0startposp, int *dist_leftp)
{
  mot_status_t m0;
  uint32 diff0, ticks;

  TRACE_LOG5(ROBOT, CFFW, front_dist, *speedp, *movingp, *m0startposp,
	     *dist_leftp);

  if (front_dist >= 0)
    {
      if (front_dist < 80) /* about to run into wall */
	{
	  TRACE_LOG0(ROBOT, CFFW_WALL);
	  set_front_dist(70);
	  return WALL_IN_FRONT;
	}
      else if (front_dist < 140)
	{
	  /* Wait for any emergencies to clear. */
	  do {
	    mot_get_status(&m0);
	    if (m0.emergency)
	      rtems_task_wake_after(ticks_per_sec/10);
	  } while (m0.emergency);

	  if (*speedp > robot_vel / 4)
	    {
	      *speedp = robot_vel / 4;
	      TRACE_LOG0(ROBOT, CFFW_SLOWING);

	      if (*dist_leftp == FOREVER)
		{
		  ticks = mot_get_ticks();
		  mot_set_vel(robot_acc, *speedp, ticks+1);
		}
	      else
		{
		  mot_get_status(&m0);
		  diff0 = m0.pos - *m0startposp + m0.velocity * 2;
		  *dist_leftp -= diff0 * 10 / MOT_STEPS_PER_INCH;
		  if (*dist_leftp <= 0)
		    {
		      *dist_leftp = 0;
		      mot_set_vel(robot_acc, 0, m0.tick+2);
		      TRACE_LOG0(ROBOT, CFFW_COMPLETE);
		      return DIST_COMPLETE;
		    }
		  else
		    {
		      mot_move((*dist_leftp * MOT_STEPS_PER_INCH) / 10,
			       robot_acc, *speedp, m0.tick+2);
		    }
		  *m0startposp = m0.pos + m0.velocity * 2;
		  *movingp = 1;
		}
	    }
	}
    }
  return NOT_STOPPED;
}

stop_condition_t drive_straight(int desired_dir,
				stop_condition_t stop_condition,
				int dist /* in deci-inches */,
				int start_speed)
{
  stop_condition_t retval;
  int delta, d, l, r;
  int speed = start_speed;
  int dist_left = dist;
  int moving = 0;
  mot_status_t m0;
  uint32 ticks;
  uint32 m0startpos;
  uint32 m0_lwa; /* motor positions at last wall avoidance. */
  int orig_dir = desired_dir;
  uint32 diff0;
  int keep_parallel = 0;

  TRACE_LOG5(ROBOT, DRIVE_STRAIGHT, desired_dir/256, (desired_dir%256)*100/256,
	     stop_condition, dist, start_speed);

  get_balance();

  mot_get_status(&m0);
  m0startpos = m0_lwa = m0.pos;

  while(1)
    {
      /* Wait for any emergencies to clear. */
      do {
	mot_get_status(&m0);
	if (m0.emergency) {
	  rtems_task_wake_after(ticks_per_sec/10);
	  moving = 0;
	}
      } while (m0.emergency);

      d = distance_read(DISTANCE_FRONT) - 30;
      l = distance_read(DISTANCE_LEFT) - 30;
      r = distance_read(DISTANCE_RIGHT) - 30;

      TRACE_LOG3(ROBOT, DRIVE_STRAIGHT_UPDATE, d, l, r);

      retval = check_for_front_wall(d, &speed, &moving, &m0startpos,
				    &dist_left);
      if (retval != NOT_STOPPED) {
	get_balance();
	return retval;
      }

      switch (stop_condition)
	{
	case WALL_IN_FRONT:
	  break;

	case WALL_ON_LEFT:
	  if ((l >= 0) && (l < 100))
	    {
	      get_balance();

	      mot_get_status(&m0);
	      if (dist_left != FOREVER)
		{
		  diff0 = m0.pos - m0startpos + m0.velocity * 2;
		  dist_left -= diff0 * 10 / MOT_STEPS_PER_INCH;
		}
	      moving = 0;

	      if (abs_dir_diff_24_8(desired_dir, orig_dir) > 5*256)
		{
		  /* If we're not parallel to the hallway, make us
                     parallel, then check to see if we still see the
                     wall. */
		  desired_dir = orig_dir;
		  robot_turn_to(desired_dir);
		  rtems_task_wake_after(ticks_per_sec/2);
		  l = distance_read(DISTANCE_LEFT) - 30;
		  r = distance_read(DISTANCE_RIGHT) - 30;
		  if ((l != -1) && (l < 100)) {
		    TRACE_LOG0(ROBOT, DS_WOL);
		    return WALL_ON_LEFT;
		  }
		}
	      else {
		TRACE_LOG0(ROBOT, DS_WOL);
		return WALL_ON_LEFT;
	      }

	    }
	  break;

	case WALL_ON_RIGHT:
	  if ((r >= 0) && (r < 100))
	    {
	      get_balance();

	      mot_get_status(&m0);
	      if (dist_left != FOREVER)
		{
		  diff0 = m0.pos - m0startpos + m0.velocity * 2;
		  dist_left -= diff0 * 10 / MOT_STEPS_PER_INCH;
		}
	      moving = 0;

	      if (abs_dir_diff_24_8(desired_dir, orig_dir) > 5*256)
		{
		  /* If we're not parallel to the hallway, make us
                     parallel, then check to see if we still see the
                     wall. */
		  desired_dir = orig_dir;
		  robot_turn_to(desired_dir);
		  rtems_task_wake_after(ticks_per_sec/2);
		  l = distance_read(DISTANCE_LEFT) - 30;
		  r = distance_read(DISTANCE_RIGHT) - 30;
		  if ((r >= 0) && (r < 100)) {
		    TRACE_LOG0(ROBOT, DS_WOR);
		    return WALL_ON_RIGHT;
		  }
		}
	      else {
		TRACE_LOG0(ROBOT, DS_WOR);
		return WALL_ON_RIGHT;
	      }
	    }
	  break;

	case NO_WALL_ON_LEFT:
	  if ((l < 0) || (l > 100))
	    {
	      get_balance();

	      mot_get_status(&m0);
	      if (dist_left != FOREVER)
		{
		  diff0 = m0.pos - m0startpos + m0.velocity * 2;
		  dist_left -= diff0 * 10 / MOT_STEPS_PER_INCH;
		}
	      moving = 0;

	      if (abs_dir_diff_24_8(desired_dir, orig_dir) > 5*256)
		{
		  /* If we're not parallel to the hallway, make us
                     parallel and then check to see if we still see
                     the wall.  If not, keep parallel and continue.. */
		  desired_dir = orig_dir;
		  robot_turn_to(desired_dir);
		  wait_for_mot_stopped(3);
		  rtems_task_wake_after(ticks_per_sec/2);
		  l = distance_read(DISTANCE_LEFT) - 30;
		  r = distance_read(DISTANCE_RIGHT) - 30;
		  if ((l < 0) || (l > 100)) {
		    TRACE_LOG0(ROBOT, DS_NWOL);
		    return NO_WALL_ON_LEFT;
		  } else {
		    keep_parallel = 1;
		  }
		}
	      else {
		TRACE_LOG0(ROBOT, DS_NWOL);
		return NO_WALL_ON_LEFT;
	      }
	    }
	  break;

	case NO_WALL_ON_RIGHT:
	  if ((r < 0) || (r > 100))
	    {
	      get_balance();

	      mot_get_status(&m0);
	      if (dist_left != FOREVER)
		{
		  diff0 = m0.pos - m0startpos + m0.velocity * 2;
		  dist_left -= diff0 * 10 / MOT_STEPS_PER_INCH;
		}
	      moving = 0;

	      if (abs_dir_diff_24_8(desired_dir, orig_dir) > 5*256)
		{
		  /* If we're not parallel to the hallway, make us
                     parallel, then check to see if we still see the
                     wall. */
		  desired_dir = orig_dir;
		  robot_turn_to(desired_dir);
		  wait_for_mot_stopped(3);
		  rtems_task_wake_after(ticks_per_sec/2);
		  l = distance_read(DISTANCE_LEFT) - 30;
		  r = distance_read(DISTANCE_RIGHT) - 30;
		  if ((r < 0) || (r > 100)) {
		    TRACE_LOG0(ROBOT, DS_NWOR);
		    return NO_WALL_ON_RIGHT;
		  } else {
		    keep_parallel = 1;
		  }
		}
	      else {
		TRACE_LOG0(ROBOT, DS_NWOR);
		return NO_WALL_ON_RIGHT;
	      }
	    }
	  break;

	case NO_WALL_ON_LEFT_OR_RIGHT:
	  if (((l < 0) || (l > 100)) &&
	      ((r < 0) || (r > 100)))
	    {
	      get_balance();

	      mot_get_status(&m0);
	      if (dist_left != FOREVER)
		{
		  diff0 = m0.pos - m0startpos + m0.velocity * 2;
		  dist_left -= diff0 * 10 / MOT_STEPS_PER_INCH;
		}
	      moving = 0;

	      if (abs_dir_diff_24_8(desired_dir, orig_dir) > 5*256)
		{
		  /* If we're not parallel to the hallway, make us
                     parallel, then check to see if we still see the
                     wall. */
		  desired_dir = orig_dir;
		  robot_turn_to(desired_dir);
		  wait_for_mot_stopped(3);
		  rtems_task_wake_after(ticks_per_sec/2);
		  l = distance_read(DISTANCE_LEFT) - 30;
		  r = distance_read(DISTANCE_RIGHT) - 30;
		  if (((l < 0) || (l > 100)) &&
		      ((r < 0) || (r > 100))) {
		    TRACE_LOG0(ROBOT, DS_NWOLR);
		    return NO_WALL_ON_LEFT_OR_RIGHT;
		  } else {
		    keep_parallel = 1;
		  }
		}
	      else {
		TRACE_LOG0(ROBOT, DS_NWOLR);
		return NO_WALL_ON_LEFT_OR_RIGHT;
	      }
	    }
	  break;

	default:
	  break;
	}

      mot_get_status(&m0);
      if (m0.emergency)
	continue; /* jump back to top to wait for emergency to
		     clear. */

      if (keep_parallel) {
	desired_dir = orig_dir;
      } else if ((l >= 0) && (l < 120) &&
	  (r >= 0) && (r < 120))
	{
	  /* Can see both walls - make sure we're close to the middle. */
	  desired_dir = fixup_angle_24_8(orig_dir + ((r - l)/2)*256);
	}
      else if ((l >= 0) && (l < 120))
	{
	  /* Can only see left wall. */
	  desired_dir = fixup_angle_24_8 (((50 - l)/2)*256 + orig_dir);
	}
      else if ((r >= 0) && (r < 120))
	{
	  /* Can only see right wall. */
	  desired_dir = fixup_angle_24_8 (((r - 50)/2)*256 + orig_dir);
	}

      /* Keep our heading to within DIR_MAX_CORRECT degrees of the
         original heading. */
      if (abs_dir_diff_24_8(desired_dir, orig_dir) > DIR_MAX_CORRECT*256)
	{
	  delta = desired_dir - orig_dir;
	  if (delta < 0)
	    delta += 360*256;

	  if (delta < 180)
	    desired_dir = fixup_angle_24_8(orig_dir + DIR_MAX_CORRECT*256);
	  else
	    desired_dir = fixup_angle_24_8(orig_dir - DIR_MAX_CORRECT*256);
	}

      TRACE_LOG2 (ROBOT, DS_DESIRED_DIR, desired_dir/256,
		  (desired_dir%256)*100/256);
      mot_set_heading(desired_dir, 45*256);

      mot_get_status(&m0);
      if (m0.emergency)
	continue; /* jump back to top to wait for emergency to
		     clear. */
      if (moving)
	{
	  if (m0.stopped) {
	    TRACE_LOG0(ROBOT, DS_DIST_COMPLETE);
	    get_balance();
	    return DIST_COMPLETE;
	  }
	}
      else
	{
	  m0startpos = m0.pos;
	  ticks = mot_get_ticks();
	  if (dist_left == FOREVER)
	    {
	      mot_set_vel(robot_acc, speed, ticks+1);
	    }
	  else
	    {
	      mot_move((dist_left * MOT_STEPS_PER_INCH) / 10,
		       robot_acc, speed, ticks+1);
	    }
	  moving = 1;
	}

      rtems_task_wake_after(ticks_per_sec/10);
    }
}

/* Routine to run through the maze, find the candle, put it out, and
   return back home. */
int
run_maze(void)
{
  int desired_dir;
  int speed = robot_vel;
  int i;

  /* First, setup current orientation. */
  desired_dir = 0; /* north */

  /* Drive until we encounter a wall */
  drive_straight(desired_dir, NO_WALL_ON_LEFT_OR_RIGHT, FOREVER, speed);
  set_front_dist(70);

  /* Turn left 90 degrees & check in the first room. */
  desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* west */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);
  desired_dir = fixup_angle_24_8 (desired_dir + 90*256); /* north */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, 100, speed);
  desired_dir = fixup_angle_24_8 (desired_dir + 45*256); /* northeat */
  robot_turn_to(desired_dir);
  desired_dir = fixup_angle_24_8 (desired_dir - 45*256); /* north */

  rtems_task_wake_after(ticks_per_sec/2);
  TRACE_LOG0(ROBOT, ROOM1);
  if (flame_read(0) > 0)
    {
      /* OK, this is _NOT_ a drill! */
      robot_turn_to(desired_dir); /* north */
      drive_straight(desired_dir, WALL_IN_FRONT, 100, speed);
      for (i=0; i<2; i++)
	{
	  if (put_out_fire() == 0)
	    {
	      /* Yay! We put out out!  Now go home. */
	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* west */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* south */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* east */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* south */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* west */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* north */
	      robot_turn_to(desired_dir);
	      return 0;
	    }
	}

      /* We failed to put it out.  Maybe it was a false alarm after
         all... */
    }

  desired_dir = fixup_angle_24_8 (desired_dir - 90*256); /* west */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  /* Turn around & drive out - go check the second room (which used to
     be the first room...). */
  desired_dir = fixup_angle_24_8 (desired_dir - 90*256); /* south */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8 (desired_dir - 90*256); /* east */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, NO_WALL_ON_RIGHT, FOREVER, speed/2);
  drive_straight(desired_dir, WALL_IN_FRONT, 100, speed);

  desired_dir = fixup_angle_24_8 (desired_dir + 90*256); /* south */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, 150, speed);
  desired_dir = fixup_angle_24_8 (desired_dir + 45*256); /* southwest */
  robot_turn_to(desired_dir);
  desired_dir = fixup_angle_24_8 (desired_dir - 45*256); /* south */

  rtems_task_wake_after(ticks_per_sec/2);
  TRACE_LOG0(ROBOT, ROOM2);
  if (flame_read(0) > 0)
    {
      /* OK, this is _NOT_ a drill! */
      robot_turn_to(desired_dir);
      drive_straight(desired_dir, WALL_IN_FRONT, 100, speed);
      for (i=0; i<2; i++)
	{
	  if (put_out_fire() == 0)
	    {
	      /* Yay! We put out out!  Now go home. */
	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* east */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* north */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* east */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* south */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* west */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* north */
	      robot_turn_to(desired_dir);
	      return 0;
	    }
	}

      /* We failed to put it out.  Maybe it was a false alarm after
         all... */
    }

  desired_dir = fixup_angle_24_8(desired_dir -90*256); /* east */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  /* Leave this room & try room 3. */
  desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* north */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* east */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, NO_WALL_ON_LEFT, FOREVER, speed);
  drive_straight(desired_dir, WALL_IN_FRONT, 60, speed);

  desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* north */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  /* Try the third room. */
  desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* east */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, 100, speed);

  rtems_task_wake_after(ticks_per_sec/2);
  TRACE_LOG0(ROBOT, ROOM3);
  if (flame_read(0) > 0)
    {
      /* OK, this is _NOT_ a drill! */
      robot_turn_to(desired_dir);
      drive_straight(desired_dir, WALL_IN_FRONT, 100, speed);
      for (i=0; i<2; i++)
	{
	  if (put_out_fire() == 0)
	    {
	      /* Yay! We put out out!  Now go home. */
	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* north */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* west */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* south */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);
	      
	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* east */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, NO_WALL_ON_LEFT, FOREVER, speed);
	      drive_straight(desired_dir, WALL_IN_FRONT, 80, speed);
	      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* south */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 180*256); /* north */
	      robot_turn_to(desired_dir);
	      return 0;
	    }
	}

      /* We failed to put it out.  Maybe it was a false alarm after
         all... */
      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* north */
      robot_turn_to(desired_dir);
      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

      /* set desired dir back to what will be expected. */
      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* east */
    }

  desired_dir = fixup_angle_24_8(desired_dir + 180*256); /* west */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* south */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* east */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* south */
  robot_turn_to(desired_dir);

  /* Try the fourth room: */
  drive_straight(desired_dir, WALL_ON_RIGHT, FOREVER, speed/2);
  drive_straight(desired_dir, NO_WALL_ON_RIGHT, FOREVER, speed/2);
  drive_straight(desired_dir, WALL_IN_FRONT, 80, speed);
  desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* west */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, 100, speed);
  rtems_task_wake_after(ticks_per_sec/2);
  TRACE_LOG0(ROBOT, ROOM4);
  if (flame_read(0) > 0)
    {
      /* OK, this is _NOT_ a drill! */
      desired_dir = fixup_angle_24_8(desired_dir - 45*256); /* southwest */
      robot_turn_to(desired_dir);
      drive_straight(desired_dir, WALL_IN_FRONT, 80, speed);
      desired_dir = fixup_angle_24_8(desired_dir + 45*256); /* west */
      for (i=0; i<2; i++)
	{
	  if (put_out_fire() == 0)
	    {
	      /* Yay! We put out out!  Now go home. */
	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* north */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* east */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* south */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);
	      
	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* west */
	      robot_turn_to(desired_dir);
	      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

	      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* north */
	      robot_turn_to(desired_dir);
	      return 0;
	    }
	}

      /* We failed to put it out.  Maybe it was a false alarm after
         all... */
      desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* north */
      robot_turn_to(desired_dir);
      drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

      /* set desired dir back to what will be expected. */
      desired_dir = fixup_angle_24_8(desired_dir - 90*256); /* west */
    }

  desired_dir = fixup_angle_24_8(desired_dir + 180*256); /* east */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* south */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* west */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir + 90*256); /* north */
  robot_turn_to(desired_dir);

  return 1;
}

int
doit(void)
{
  int desired_dir = 0;
  int speed = robot_vel;

  drive_straight(desired_dir, WALL_IN_FRONT, 120, robot_vel);

  desired_dir = fixup_angle_24_8(desired_dir + 180*256); /* south */
  robot_turn_to(desired_dir);
  drive_straight(desired_dir, WALL_IN_FRONT, FOREVER, speed);

  desired_dir = fixup_angle_24_8(desired_dir + 180*256); /* north */
  robot_turn_to(desired_dir);

  return 0;
}
