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
 * PID-based Motor Controller
 */

#include <bsp.h>

#include <stdio.h>
#include <stdlib.h>
#include "tpu.h"
#include "fqd.h"
#include <sim.h>
#include "mcpwm.h"
#include "motor.h"
#include "global.h"
#include "kalman.h"
#include "f16_16.h"
#include "robot_trace.h"

#define MOTOR_HZ		250
#define TILT_UPDATE_INTERVAL	25

#define HEADING_UPDATE_TICKS	MOTOR_HZ/4
#define HEADING_UPD_PCT		10 /* what percentage of a heading update to
				   apply. */

typedef struct mot_info
{
  uint32 curpos;		/* Current position of motor. */
} mot_info_t;

uint32 mot_curpos; /* position of center of platform. */
int32 mot_wheel_velocity;


uint32 mot_desired_pos;		/* Desired position of motor. */
int32 mot_desired_pos_frac;	/* fractional portion of desired pos,
				   as a 16.16 number. */
mot_velocity_t mot_v;		/* Current velocity of motor. */
mot_accel_t mot_a;		/* Acceleraion to apply. */
mot_velocity_t mot_desired_v;	/* Desired velocity - when this is reached,
				   'a' should be set to 0. */

int32 mot_preverr;		/* PID data - previous error. */
int32 mot_interr;			/* PID data - integrated error. */

int32 mot_preverr_bal;		/* PID data - previous error (balancing) */
int32 mot_interr_bal;		/* PID data - integrated error (balancing) */

int mot_stop_at_valid;		/* non-zero if a stopping point is defined. */
int32 mot_stop_at;

int mot_next_cmd_valid;		/* non-zero if there is a command set up. */
uint32 mot_next_cmd_time;	/* Time (in ticks) when next command should
				   be applied. */
mot_accel_t mot_next_a;		/* Acceleration of next command. */
mot_accel_t mot_next_desired_v;	/* Velocity of next command. */
int32 mot_next_s;		/* Number of steps to move for next command. */

int mot_stopped;		/* Has the motor completed the last motion
				   command and come to rest at the final
				   position? */

int mot_heading_stopped;	/* Have we completed the last heading
				   adjustment motion command and come
				   to rest at the desired_heading? */

/* Current tick count. */
uint32 mot_ticks;

f16_16 mot_left_factor = 68813; /* 1.05 */
f16_16 mot_right_factor = 65536; /* 1.0 */

/* Current heading of robot, in degrees as a 16.16 number. */
f16_16 mot_heading;
f16_16 mot_desired_heading;

/* tick at the last time we accepted a heading update.  Only accept a
   heading update every HEADING_UPDATE_TICKS ticks. */
uint32 mot_last_heading_update;

/* mot_heading_dest is the heading we are turning towards.  We dont
   just set mot_heading because the PID loop would freak out.  If we
   are in the process of turning to a specific heading, this will not
   be equal to mot_desired_heading until we get there. */
f16_16 mot_heading_dest;

/* The maximum amount we will change mot_desired_heading per tick. */
f16_16 mot_heading_steps = 15 * 65536 / MOTOR_HZ; /* 15 degrees per second. */

f16_16 mot_hd_preverr;
f16_16 mot_hd_interr;

f16_16 mot_hd_kp = 0xa0000;
f16_16 mot_hd_kd = 0;
f16_16 mot_hd_ki = 0;

/* Motor info for both motors. */
mot_info_t mot_info[2];

/* Position PID constants. */
int32 mot_kp = 0x180;
int32 mot_kd = 0xf00;
int32 mot_ki = 0x4;

/* Balancing PID constants. */
int32 mot_bal_kp = 0x4000;
int32 mot_bal_kd = 0xf000;
int32 mot_bal_ki = 0x333;

/* Whether balancing is on or off. */
int32 mot_bal_on = 0;

/* Previous position of balance switch - used to detect changes, and
   so that if the sw is started with the switch on, you have to switch
   it off & back on before balancing will be enabled. */
int32 mot_bal_switch;

/* Using balance to control position.  This involves a concept of
   'desired tilt' that can be other than 0, and a PID loop that
   controls the desired tilt based on the current position of the
   encoders. */
int32 mot_desired_tilt = 0;

int32 mot_max_tilt = 5 * 256;
int32 mot_min_tilt = -5 * 256;

#if 0
int32 mot_normal_max_tilt = 3 * 256;
int32 mot_normal_min_tilt = -3 * 256;

int32 mot_emergency_max_tilt = 6 * 256;
int32 mot_emergency_min_tilt = -6 * 256;
#endif

/* This is the maximum amount the position PID loop is allowed to
   change the tilt every update (which happens 10 times a second). */
int32 mot_max_tilt_delta = 1*256;

/* If mot_emergency is non-zero, the motor task detected a dangerous
   overbalance/position condition, stopped everything, and is trying
   to correct.  The higher level task needs to wait for the emergency
   condition to clear, and then try to resume where it left off. */
int mot_emergency;

int mot_pending_emergency_cnt;
int mot_emergency_cnt;

/* How many times the motor task failed to meet it's deadline. */
uint32 motor_pos_task_timeouts = 0;

/* PID trace elements. */
typedef struct pid_trace
{
  uint32 ticks;
  int32	desired_tilt;
  int32 measured_tilt;
  uint32 desired_pos;
  uint32 measured_pos;
  int32 pid_out;
  f16_16 mot_heading;
  f16_16 mot_desired_heading;
} pid_trace_t;

pid_trace_t mot_pid_trace[1000];
int mot_pid_trace_size = sizeof(mot_pid_trace) / sizeof(mot_pid_trace[0]);
int mot_pid_trace_idx = 0;
int mot_pid_trace_pause = 0;
int mot_pid_trace_wrapped = 0;

void
mot_log_pid_trace(int32 desired_tilt, int32 measured_tilt, uint32 desired_pos,
		  uint32 measured_pos, int32 pid_out)
{
  if (mot_pid_trace_pause)
    return;

  mot_pid_trace[mot_pid_trace_idx].ticks = mot_ticks;
  mot_pid_trace[mot_pid_trace_idx].desired_tilt = desired_tilt;
  mot_pid_trace[mot_pid_trace_idx].measured_tilt = measured_tilt;
  mot_pid_trace[mot_pid_trace_idx].desired_pos = desired_pos;
  mot_pid_trace[mot_pid_trace_idx].measured_pos = measured_pos;
  mot_pid_trace[mot_pid_trace_idx].pid_out = pid_out;
  mot_pid_trace[mot_pid_trace_idx].mot_heading = mot_heading;
  mot_pid_trace[mot_pid_trace_idx].mot_desired_heading = mot_desired_heading;

  mot_pid_trace_idx++;
  if (mot_pid_trace_idx >= mot_pid_trace_size) {
    mot_pid_trace_wrapped = 1;
    mot_pid_trace_idx = 0;
  }
}

void
print_24_8(int32 a)
{
  printf ("%c%d.%02d", a < 0 ? '-' : ' ',
	  abs(a) / 256,
	  abs((a % 256) * 100 / 256));
}

/* Dump out the PID trace to the console. */
void
mot_dump_pid_trace(void)
{
  int idx;
  int wrapped;
  int t;

  if ((!mot_pid_trace_wrapped) && (mot_pid_trace_idx == 0))
    return;

  /* Pause logging. */
  mot_pid_trace_pause = 1;

  if (mot_pid_trace_wrapped) {
    wrapped = 0;
    idx = mot_pid_trace_idx;
  } else {
    wrapped = 1;
    idx = 0;
  }

  printf ("\"time\",\"pid_out\",\"desired_tilt\",\"measured_tilt\",\"desired_pos\",\"measured_pos\",\"heading\",\"desired_heading\"\n");
  for (t = 0;
       ;
       t++) {
    printf ("%d,%d,", mot_pid_trace[idx].ticks,mot_pid_trace[idx].pid_out);
    print_24_8 (mot_pid_trace[idx].desired_tilt);
    printf (",");
    print_24_8 (mot_pid_trace[idx].measured_tilt);
    printf (",%ld,%ld,", (long)mot_pid_trace[idx].desired_pos,
	    (long)mot_pid_trace[idx].measured_pos);
    print_f16_16(mot_pid_trace[idx].mot_heading);
    printf (",");
    print_f16_16(mot_pid_trace[idx].mot_desired_heading);
    printf ("\n");

    idx = (idx + 1) % mot_pid_trace_size;
    if (idx == 0)
      wrapped = 1;
    if (wrapped && idx == mot_pid_trace_idx)
      break;
  }

  /* Resume logging. */
  mot_pid_trace_pause = 0;
}

/* Multiply two 24.8 fixed numbers.  Does the equivalent of long
   multiplication. */
int
mult_24_8 (int a, int b)
{
  int a_frac = a & 0x000000ff;
  int a_int  = (int) (a & 0xffffff00) / 256;
  int b_frac = b & 0x000000ff;
  int b_int  = (int) (b & 0xffffff00) / 256;

  return ((a_frac * b_frac) >> 8) +
    (a_frac * b_int) +
    (a_int * b_frac) +
    ((a_int * b_int) << 8);
}

int
mot_do_pid (int kalman_angle, int do_tilt_update)
{
  int32 error, output = 0, error_bal;
  int32 prev_desired_tilt;
  int32 min_tilt, max_tilt;

  if (mot_bal_on) {
    if (do_tilt_update) {
      error = mot_desired_pos - mot_curpos;

      prev_desired_tilt = mot_desired_tilt;
      min_tilt = MAX(mot_min_tilt, mot_desired_tilt - mot_max_tilt_delta);
      max_tilt = MIN(mot_max_tilt, mot_desired_tilt + mot_max_tilt_delta);

      mot_desired_tilt = (mult_24_8 (mot_kp, error) +
			  mult_24_8 (mot_kd, error - mot_preverr) +
			  mult_24_8 (mot_ki, mot_interr));

      if (mot_desired_tilt < min_tilt)
	mot_desired_tilt = min_tilt;
      else if (mot_desired_tilt > max_tilt)
	mot_desired_tilt = max_tilt;
      else
	mot_interr += error;

      mot_preverr = error;
    }

    error_bal = (kalman_angle / 256) - mot_desired_tilt;

    /* error_bal is a 24.8 number, too, so we can't just multiply the
       pid constants - must use mult_24_8(). */
    output += (mult_24_8 (mot_bal_kp, error_bal) +
	       mult_24_8 (mot_bal_kd, error_bal - mot_preverr_bal) +
	       mult_24_8 (mot_bal_ki, mot_interr_bal)) >> 8;

    mot_preverr_bal = error_bal;

    /* Accumulate integral error *OR* limit output.  Stop accumulating
       when output saturates.  Valid output values are 1 (max reverse)
       to 255 (max forward) with 128 being full stop. */
    if (output >= 127)
      output = 127;
    else if (output <= -127)
      output = -127;
    else {
      mot_interr_bal += error_bal;
    }

    mot_log_pid_trace(mot_desired_tilt, kalman_angle/256,
		      mot_desired_pos, mot_curpos, output);
  }

  return output;
}

int
mot_do_heading_pid(void)
{
  f16_16 error;
  int pid_out = 0;

  if (mot_bal_on) {
    /* Calculating the error is funky because these are headings, and
       wrap around at 360.  We want an error where a positive value
       means we need to turn that many degrees to the right, and a
       negative value means we need to turn that many degrees to the
       left. */
    error = mot_desired_heading - mot_heading;
    if (error < 0)
      error += 360 * 65536;

    if (error > (180 * 65536)) {
      error = error - (360 * 65536);
    }

    pid_out = (mult_f16_16 (mot_hd_kp, error) +
	       mult_f16_16 (mot_hd_kd, error - mot_hd_preverr) +
	       mult_f16_16 (mot_hd_ki, mot_hd_interr)) >> 16;

    mot_hd_preverr = error;

    /* Limit heading to only being able to affect up to 1/3 of the
       motor's pwm range. */
    if (pid_out >= 43)
      pid_out = 43;
    else if (pid_out <= -43)
      pid_out = -43;
    else
      mot_hd_interr += error;
  }

  /* NOTE: we do not add 128 to bring it into the 32-255 range like
     mot_do_pid, that is because this value is added to the output of
     mot_do_pid(). */
  return pid_out;
}

/* Returns the value of "1 + 2 + 3 + ... + n" */
int32
sum_1ton(int32 n)
{
  return ( (n+1) * (n/2) + (n%2 ? (n+1)/2 : 0) );
}

int
mot_calc_stop_dist(mot_accel_t a, mot_velocity_t v)
{
  int32 rampdown_t, rampdown_d;

  /* Calculate amount of time to decelerate from current v
     to 0 and distance traveled. */
  rampdown_t = (abs(v) - a + (a/2)) / a;
  rampdown_d = (sum_1ton(rampdown_t) * a + 128) / 256;

  return rampdown_d;
}

void
mot_do_motion(void)
{
  /* First, check if it's time to initiate another motion command. */
  if ((mot_next_cmd_valid) &&
      (mot_ticks >= mot_next_cmd_time))
    {
      mot_a = abs(mot_next_a);
      mot_desired_v = mot_next_desired_v;

      if (mot_next_s)
	{
	  mot_stop_at_valid = 1;
	  if (mot_desired_v < 0)
	    mot_stop_at = mot_desired_pos - mot_next_s;
	  else
	    mot_stop_at = mot_desired_pos + mot_next_s;
	}
      mot_next_cmd_valid = 0;
      mot_stopped = 0;
    }

  /* If a stopping point is set, check if we need to start decelerating. */
  if (mot_stop_at_valid == 1)
    {
      int32 rampdown_t, rampdown_d;

      /* Calculate amount of time to decelerate from current v
	 to 0 and distance traveled. */
      rampdown_t = (abs(mot_v) - mot_a + (mot_a/2)) / mot_a;
      rampdown_d = (sum_1ton(rampdown_t) * mot_a + 128) / 256;

      if (rampdown_d >= abs(mot_desired_pos - mot_stop_at))
	{
	  /* We need to start decelerating now so that we can
	   * stop in time.
	   */
	  TRACE_LOG6(ROBOT, START_DECEL, mot_v, mot_a, rampdown_t, rampdown_d,
		     mot_desired_pos, mot_stop_at);
	  mot_desired_v = 0;
	  mot_stop_at_valid = 2;
	}
    }
  else if (mot_stop_at_valid == 2)
    {
      /* this causes big issues if the stop_at distance is less than
	 we can reasonably stop in, so remove it & let higher level
	 code deal. */
#if 0
      int32 rampdown_t, rampdown_d, err;

      /* Calculate amount of time to decelerate from current v
	 to 0 and distance traveled. */
      rampdown_t = (abs(mot_v) - mot_a + (mot_a/2)) / mot_a;
      rampdown_d = (sum_1ton(rampdown_t) * mot_a + 128) / 256;

      /* As the motor is slowing down to hit a target step value,
	 adjust the desired position slightly so we come reasonably
	 close to hitting the target when our velocity is 0. */
      if (rampdown_t != 0)
	{
	  if (mot_v > 0)
	    err = mot_stop_at - (mot_desired_pos + rampdown_d);
	  else
	    err = mot_stop_at - (mot_desired_pos - rampdown_d);
	  mot_desired_pos += ((err + rampdown_t/2) / rampdown_t);
	}
#endif
    }

  /* Update the velocity. */
  if (mot_v < mot_desired_v)
    {
      mot_v += mot_a;
      if (mot_v > mot_desired_v)
	mot_v = mot_desired_v;
    }
  else if (mot_v > mot_desired_v)
    {
      mot_v -= mot_a;
      if (mot_v < mot_desired_v)
	mot_v = mot_desired_v;
    }

  if ((mot_v == 0) && (mot_stop_at_valid == 2))
    {
      TRACE_LOG3(ROBOT, DECEL_COMPLETE, 
		 mot_desired_pos, mot_stop_at,
		 abs(mot_desired_pos - mot_stop_at));
#if 0
      /* causes problems if given an unreasonable stopping distance. */
      mot_desired_pos = mot_stop_at;
#endif
      mot_stop_at_valid = 0;
    }
  else
    {
      mot_desired_pos_frac += mot_v;
      mot_desired_pos += mot_desired_pos_frac / 256;
      mot_desired_pos_frac %= 256;
    }
}

/* Update mot_desired_heading to eventually meet up with mot_heading_dest. */
void
mot_do_heading_motion(void)
{
  f16_16 delta_dir;

  if (abs_dir_diff_f16_16(mot_desired_heading, mot_heading_dest) >
      mot_heading_steps) {
    delta_dir = mot_heading_dest - mot_desired_heading;
    if (delta_dir < 0)
      delta_dir += 360*65536;
    
    if (delta_dir < 180*65536) {
      mot_desired_heading += mot_heading_steps;
      if (mot_desired_heading >= 360*65536)
	mot_desired_heading -= 360*65536;
    } else {
      mot_desired_heading -= mot_heading_steps;
      if (mot_desired_heading < 0)
	mot_desired_heading += 360*65536;
    }
  } else {
    /* If we're here, the desired_heading is within mot_heading_steps
       of mot_heading_dest.  Just make it match. */
    mot_desired_heading = mot_heading_dest;
  }
}

/* This is called just before doing motion & PID calculations for this
   tick.  If the motor has come to rest (all errors have cuaght up &
   motor is stopped close to the final position), then set the stopped
   flag to a 1. */
void
mot_check_stopped(void)
{
  if ((mot_bal_on == 0) ||
      ((mot_v == 0) &&
       (mot_desired_v == 0) &&
       (mot_next_cmd_valid == 0) &&
       (abs(mot_curpos - mot_desired_pos) < 100) &&
       (abs(mot_preverr) < 100) &&
       (mot_wheel_velocity == 0)))
    {
      mot_stopped = 1;
    }
  else
    {
      mot_stopped = 0;
    }

  /* Similarly, check if we're close to the right heading. */
  if ((mot_bal_on == 0) ||
      ((mot_desired_heading == mot_heading_dest) &&
       (abs_dir_diff_f16_16(mot_desired_heading, mot_heading) < (3 * 65536))))
    {
      mot_heading_stopped = 1;
    }
  else
    {
      mot_heading_stopped = 0;
    }
}

rtems_task
motor_pos_task (rtems_task_argument ignored)
{
  short prev_fqd0, prev_fqd1;
  short new_fqd0, new_fqd1;
  short diff;
  int diff0, diff1;
  rtems_name period_name;
  rtems_id period;
  rtems_status_code status;
  int kalman_out;
  int pwm0, pwm1;
  int pwm_l, pwm_r;
  int do_tilt_update_counter = 0, do_tilt_update;
  f16_16 heading_update;
  int toggle_rounding = 1;
  int bal_switch;

  period_name = rtems_build_name ('M', 'T', 'P', 'D');
  status = rtems_rate_monotonic_create (period_name, &period);
  if (status != RTEMS_SUCCESSFUL)
    {
      printf ("motor_pos_task: rate_monotonic_create failed with status %d\n",
	      status);
      exit (1);
    }

  prev_fqd0 = read_tpu_fqd0();
  prev_fqd1 = read_tpu_fqd1();
  while (1)
    {
      if (rtems_rate_monotonic_period (period, ticks_per_sec/MOTOR_HZ) ==
	  RTEMS_TIMEOUT)
	{
	  /* I'd like to do a printf here, but that would make us miss
	     our next timeout, until the end of time... */
	  motor_pos_task_timeouts++;
	}

      /* Check if we should turn balancing on or off: */
      bal_switch = (*PORTE0 & PORTE_BALANCE_ON) != 0;
      if (mot_bal_switch != bal_switch) {
	mot_bal_switch = bal_switch;
	mot_balance(mot_bal_switch);
      }

      /* emergency recovery does not seem to be helpful. */
#if 0
      /* Check if we need to do emergency recovery, or if we can turn
	 off emergency mode. */
      if (mot_bal_on) {
	if (mot_emergency == 0) {
	  if ((mot_desired_tilt >= mot_max_tilt) ||
	      (mot_desired_tilt <= mot_min_tilt) ||
	      (abs(mot_wheel_velocity) > 5)) {
	    mot_pending_emergency_cnt++;
	  } else {
	    mot_pending_emergency_cnt = 0;
	  }

	  if ((mot_preverr > (MOT_STEPS_PER_INCH)*6) ||
	      (mot_pending_emergency_cnt >= (MOTOR_HZ))) {
	    /* Emergency! */
	    TRACE_LOG2(ROBOT, EMERGENCY, mot_preverr,
		       mot_pending_emergency_cnt);
	    mot_pending_emergency_cnt = 0;
	    mot_emergency_cnt = 0;
	    mot_emergency = 1;

	    /* Grant the balance PID loop emergency powers. */
	    mot_max_tilt = mot_emergency_max_tilt;
	    mot_min_tilt = mot_emergency_min_tilt;

	    mot_desired_tilt = 0;
	    kalman_out = kalman_read();

	    /* Try to come to a rest quickly - if we're tilted forward,
	       set our desired position a little bit backward, otherwise
	       set it a little bit forward. */
	    mot_desired_pos = mot_curpos -
	      (kalman_out / 256 * MOT_STEPS_PER_INCH / mot_max_tilt );
	    mot_desired_pos_frac = 0;
	    mot_interr = 0;
	    mot_preverr = 0;

	    mot_next_cmd_valid = 0;
	    mot_stop_at_valid = 0;
	    mot_v = 0;
	    mot_desired_v = 0;

	    mot_desired_heading = mot_heading;
	  }
	} else {
	  /* In emergency mode, check if we're safe to exit. */
	  mot_emergency_cnt++;
	  if ((abs(mot_curpos - mot_desired_pos) < 100) &&
	      (abs(mot_preverr) < 100) &&
	      (abs(kalman_read() < 65536/2))) {
	    mot_pending_emergency_cnt++;

	    if (mot_emergency_cnt >= MOTOR_HZ*2) {
	      /* We're not recovering!  Set position again. */
	      mot_desired_tilt = 0;

	      mot_desired_pos = mot_curpos -
		(kalman_out / 256 * MOT_STEPS_PER_INCH / mot_max_tilt );

	      mot_desired_pos_frac = 0;
	      mot_interr = 0;
	      mot_preverr = 0;

	      mot_emergency_cnt = 0;
	    }
	  } else {
	    mot_pending_emergency_cnt = 0;
	  }

	  if (mot_pending_emergency_cnt >= MOTOR_HZ/2) {
	    TRACE_LOG0(ROBOT, EMERGENCY_CLEAR);
	    mot_emergency = 0;
	    mot_pending_emergency_cnt = 0;
	    mot_emergency_cnt = 0;

	    /* Revoke the balance PID loop's emergency powers. */
	    mot_max_tilt = mot_normal_max_tilt;
	    mot_min_tilt = mot_normal_min_tilt;
	  }
	}
      }
#endif

      if (do_tilt_update_counter++ == TILT_UPDATE_INTERVAL) {
	do_tilt_update = 1;
	do_tilt_update_counter = 0;
      } else
	do_tilt_update = 0;

      /* First, get current position of each motor. */
      new_fqd0 = read_tpu_fqd0();
      new_fqd1 = read_tpu_fqd1();

      diff = new_fqd0 - prev_fqd0;
      diff0 = diff;
      mot_info[0].curpos = mot_info[0].curpos + diff;
      prev_fqd0 = new_fqd0;

      diff = new_fqd1 - prev_fqd1;
      diff1 = diff;
      mot_info[1].curpos = mot_info[1].curpos + diff;
      prev_fqd1 = new_fqd1;

      /* Update heading. */
      heading_update = (diff0 - diff1) * 65536 / MOT_WHEEL_BASE; /* in radians. */
      heading_update = div_f16_16 (heading_update * 180, F16_16_PI);
      mot_heading += heading_update;
      if (mot_heading >= 360*65536)
	mot_heading -= 360*65536;
      if (mot_heading < 0)
	mot_heading += 360*65536;

      /* Update current position. */
      mot_wheel_velocity = (diff0 + diff1 + toggle_rounding)/2;
      mot_curpos += mot_wheel_velocity;
      toggle_rounding = -toggle_rounding;

      /* Get tilt of robot. */
      kalman_out = kalman_read();

      /* Do PID loops. */
      mot_check_stopped();
      mot_do_motion();
      pwm0 = mot_do_pid(kalman_out, do_tilt_update);

      mot_do_heading_motion();
      pwm1 = mot_do_heading_pid();

      pwm_l = (((pwm0+pwm1) * mot_left_factor + 32768)/65536) + 128;
      pwm_r = (((pwm0-pwm1) * mot_left_factor + 32768)/65536) + 128;

      set_tpu_pwm0(MIN(MAX(pwm_l, 16), 255));
      set_tpu_pwm1(MIN(MAX(pwm_r, 16), 255));

      /* Finally, update ticks */
      mot_ticks++;
    }
}

/* Initialize the motor controller.  Returns 0 on success, non-zero
   on error. */
int
mot_init(void)
{
  int i;
  rtems_status_code code;
  Objects_Id t1;

  printf ("Initializing fqd:\n");
  init_tpu_fqd();
  printf ("Done.\n\n");

  printf ("Initializing pwm:\n");
  init_tpu_pwm();
  printf ("Done.\n\n");

  printf ("Setting Port E bit 0 to output 1:\n");
  *PEPAR &= ~PORTE_MOTOR_ON;
  *DDRE |= PORTE_MOTOR_ON;
  *PORTE0 |= PORTE_MOTOR_ON;
  printf ("Done.\n\n");

  printf ("Reading balance switch:\n");
  *PEPAR &= ~PORTE_BALANCE_ON;
  *DDRE &= ~PORTE_BALANCE_ON;
  mot_bal_switch = (*PORTE0 & PORTE_BALANCE_ON) != 0;
  printf ("Done, switch is %d\n", mot_bal_switch);

  printf ("Initializing motor structures:\n");
  mot_ticks = 0;
  mot_heading = 0;
  for (i=0; i<2; i++)
    {
      mot_info[i].curpos = 0;
    }
  mot_curpos = 0;
  mot_wheel_velocity = 0;
  mot_desired_pos = 0;
  mot_desired_pos_frac = 0;
  mot_v = 0;
  mot_a = 0;
  mot_desired_v = 0;
  mot_stopped = 0;
  mot_preverr = 0;
  mot_interr = 0;
  mot_preverr_bal = 0;
  mot_interr_bal = 0;
  mot_stop_at_valid = 0;
  mot_next_cmd_valid = 0;

  printf ("Spawning motor position task:\n");
  code = rtems_task_create(rtems_build_name('M', 'O', 'T', 'R'),
			   5, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, motor_pos_task, 0);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);

  return 0;
}

/* All of the motor control is based on 10ms ticks.  This gets the
   current tick count. */
uint32
mot_get_ticks(void)
{
  return mot_ticks;
}

/* Get the PID loop constants.  Values are in 24.8 format. */
void
mot_get_pid(int32 *kp, int32 *kd, int32 *ki)
{
  rtems_mode prev_mode, dummy;

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  *kp = mot_kp;
  *kd = mot_kd;
  *ki = mot_ki;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
}

/* Set the PID loop constants.  Values are in 24.8 format. */
void
mot_set_pid(int32 kp, int32 kd, int32 ki)
{
  rtems_mode prev_mode, dummy;

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  mot_kp = kp;
  mot_kd = kd;
  mot_ki = ki;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
}

/* Get the motor status. */
void
mot_get_status(mot_status_t *mot0p)
{
  rtems_mode prev_mode, dummy;

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  mot0p->pos = mot_desired_pos /* mot_curpos */;
  mot0p->velocity = mot_v;
  mot0p->accel = mot_a;
  mot0p->tick = mot_ticks;
  mot0p->stopped = mot_stopped;
  mot0p->heading_stopped = mot_heading_stopped;
  mot0p->emergency = mot_emergency;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
}

/* Ramp a motor up (or down) to velocity 'v' using acceleration 'a'
   starting at time 'tick'. */
int
mot_set_vel(mot_accel_t a, mot_velocity_t v, uint32 tick)
{
  rtems_mode prev_mode, dummy;

  TRACE_LOG3(ROBOT, SET_VEL, a, v, tick);

  if (mot_emergency) {
    TRACE_LOG0(ROBOT, EMERGENCY_IGNORED);
    return 1;
  }

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  mot_stop_at_valid = 0;
  mot_next_cmd_valid = 1;
  mot_next_cmd_time = tick;
  mot_next_a = a;
  mot_next_desired_v = v;
  mot_next_s = 0;
  mot_stopped = 0;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);

  return 0;
}

/* Move a motor forward (or backward) 's' steps, using acceleration
   'a' and max velocity 'v', starting at time 'tick'. */
int
mot_move(int32 s, mot_accel_t a, mot_velocity_t v, uint32 tick)
{
  rtems_mode prev_mode, dummy;

  TRACE_LOG4(ROBOT, MOVE, s, a, v, tick);

  if (mot_emergency) {
    TRACE_LOG0(ROBOT, EMERGENCY_IGNORED);
    return 1;
  }

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

#if 0
  if (v == 0)
    s = 0; /* ignore steps if velocity is 0. */
#endif

  mot_stop_at_valid = 0;
  mot_next_cmd_valid = 1;
  mot_next_cmd_time = tick;
  mot_next_a = a;
  mot_next_desired_v = v;
  mot_next_s = s;
  mot_stopped = 0;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);

  return 0;
}

/* Turn on or off balancing - the motor task will monitor the tilt
   output and keep the bot on it's feet.  If 'on' is 0, then balancing
   will be turned off; otherwise it will be turned on. */
int mot_balance(int on)
{
  if (on) {
    mot_preverr_bal = 0;
    mot_interr_bal = 0;
    mot_preverr = 0;
    mot_interr = 0;
    mot_curpos = 0;
    mot_wheel_velocity = 0;
    mot_desired_pos = 0;
    mot_desired_pos_frac = 0;

    mot_desired_heading = mot_heading_dest = mot_heading = 0;

    mot_info[0].curpos = 0;
    mot_info[1].curpos = 0;

    mot_emergency = 0;
    mot_pending_emergency_cnt = 0;
  }
  mot_bal_on = on;

  return mot_bal_on;
}

/* Returns non-zero if the robot is balancing now. */
int
mot_balancing(void)
{
  return mot_bal_on;
}

/* Get the balancing PID loop constants.  Values are in 24.8 format. */
void mot_get_bal_pid(int32 *kp, int32 *kd, int32 *ki)
{
  rtems_mode prev_mode, dummy;

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  *kp = mot_bal_kp;
  *kd = mot_bal_kd;
  *ki = mot_bal_ki;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
}

/* Set the PID loop constants.  Values are in 24.8 format. */
void mot_set_bal_pid(int32 kp, int32 kd, int32 ki)
{
  rtems_mode prev_mode, dummy;

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  mot_bal_kp = kp;
  mot_bal_kd = kd;
  mot_bal_ki = ki;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
}

/* Get the current heading of the robot (24.8 number in degrees). */
int
mot_get_heading(void)
{
  return mot_heading / 256;
}

/* Set a heading to turn to (24.8 number in degrees and degrees/s). */
int
mot_set_heading(int new_heading, int heading_vel)
{
  rtems_mode prev_mode, dummy;

  TRACE_LOG4(ROBOT, SET_HD, new_heading/256, (new_heading%256)*100/256,
	     heading_vel/256, (heading_vel%256)*100/256);

  if (mot_emergency) {
    TRACE_LOG0(ROBOT, EMERGENCY_IGNORED);
    return 1;
  }

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  mot_heading_stopped = 0;
  mot_heading_steps = (heading_vel * 256) / MOTOR_HZ;
  mot_heading_dest = (new_heading * 256) % (360 * 65536);

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);

  return 0;
}

/* Receive a heading update from the distance task.  This gives us an
   estimate of our heading relative to one or both walls to our sides.
   0 degrees is lined up with the wall.  It will only give us an angle
   above 270 or below 90; we will only consider angles above 330 and
   below 30, as we should be lined up with a wall most of the time.
   We will then have to compare that to the quadrant we are in, and if
   we think this update is valid, we update our heading.  Note the
   angle is an integer from 0 to 359. */
void
mot_heading_update(int update_angle)
{
  int quadrant_offset;
  int mot_heading_int = (mot_heading + 32768) / 65536;
  int new_angle;
  f16_16 new_heading, heading_diff;
  rtems_mode prev_mode, dummy;

  if (((update_angle < 330) && (update_angle > 30)) ||
      ((mot_ticks - mot_last_heading_update) < HEADING_UPDATE_TICKS) ||
      mot_emergency) {
    /* throw out this update. */
    return;
  }

  /* Figure out which quadrant our current heading is in.  But using
     quadrants offset by 45 degrees.  So headings between 315 and 45
     would be in quadrant 0 (and have an offset of 0), headings
     between 45 and 135 in quadrant 1 (offset 90), etc. */
  
  if (mot_heading_int <= 135) {
    if (mot_heading_int <= 45)
      quadrant_offset = 0;
    else
      quadrant_offset = 90;
  } else {
    /* greater than 135 */
    if (mot_heading_int <= 225)
      quadrant_offset = 180;
    else if (mot_heading_int <= 315)
      quadrant_offset = 270;
    else
      quadrant_offset = 0; /* angle greater than 315 */
  }

  new_angle = fixup_angle (update_angle + quadrant_offset);
  if (abs_dir_diff (mot_heading_int, new_angle) <= 10) {
    rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

    heading_diff = fixup_angle_f16_16 ((new_angle * 65536) - mot_heading);
    if (heading_diff > (180*65536)) {
      /* Make a difference of >180 degrees be the equivalent negative
	 angle. */
      heading_diff -= 360*65536;
    }
    new_heading = fixup_angle_f16_16 (mot_heading +
				      (heading_diff * HEADING_UPD_PCT / 100));

    TRACE_LOG4 (ROBOT, ACCEPT_HEADING_UPDATE,
		mot_heading/65536, (mot_heading % 65536) * 100 / 65536,
		new_heading/65536, (new_heading % 65536) * 100 / 65536);

    mot_heading = new_heading;

    mot_last_heading_update = mot_ticks;

    rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
  }
}

/* Get the heading maintenance PID loop constants.  Values are in 16.16
   format. */
void
mot_get_hd_pid(int32 *kp, int32 *kd, int32 *ki)
{
  rtems_mode prev_mode, dummy;

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  *kp = mot_hd_kp;
  *kd = mot_hd_kd;
  *ki = mot_hd_ki;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
}

/* Set the heading maintenance PID loop constants.  Values are in
   16.16 format. */
void
mot_set_hd_pid(int32 kp, int32 kd, int32 ki)
{
  rtems_mode prev_mode, dummy;

  rtems_task_mode(RTEMS_NO_PREEMPT, RTEMS_PREEMPT_MASK, &prev_mode);

  mot_hd_kp = kp;
  mot_hd_kd = kd;
  mot_hd_ki = ki;

  rtems_task_mode(prev_mode, RTEMS_PREEMPT_MASK, &dummy);
}
