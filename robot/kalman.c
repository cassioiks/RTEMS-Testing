/*
 * Kalman filter
 *
 * A kalman filter to filter the output from a gyro sensor and
 * accelerometer to calculate the actual current tilt of the robot.
 *
 * (c) Aaron Kahn <Aaron.Kahn@itt.com>
 * (c) Trammell Hudson <hudson@rotomotion.com>
 * Copyright (c) 2003 by Matt Cross (matt@dragonflyhollow.org)
 *
 * Portions of this code originated in the file 'imu-1d.c' in the
 * outpilot package, which is covered by the following license.  The
 * original version of 'imu-1d.c' is available in the directory 'imu'.
 *
 *************
 *
 *  This file is part of the autopilot simulation package.
 *
 *  Autopilot is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <bsp.h>
#include <math.h>
#include <stdio.h>
#include "global.h"
#include "kalman.h"
#include "gyro.h"
#include "accel.h"
#include "f16_16.h"

/* Types. */

/* Constants */
#define KALMAN_HZ 10
#define UPDATE_HZ 250

const f16_16	dt		= (65536 + UPDATE_HZ/2) / UPDATE_HZ;
const f16_16	Q		= 655;		/* 0.01 as a 16.16 fixed
						   (Noise weighting matrix) */

/* Globals. */
f16_16	theta		= 0;		/*  (Our initial state estimate) */
f16_16	gyro_only_theta = 0;
f16_16	R		= 6554;	/* 0.1 as a 16.16 fixed
					   (Measurement error weight) */
f16_16	P		= 6553600;	/* 100 as a 16.16 fixed
					   (Covariance matrix) */
f16_16  last_theta_m;
int kalman_timeouts = 0;

/* Kalman filter routine. */

f16_16
kalman(f16_16 q, /* Pitching gyro reading */
       f16_16 theta_m, /* Measured angle from accelerometer. */
       int do_kalman) /* whether to run the filter or just update
			 based on gyro reading. */
{
  f16_16 Pdot; /* Derivative of P */
  f16_16 E; /* ? */
  f16_16 K; /* ? */

  /* A = 0 */
  Pdot = Q; /* Pdot = A*P + P*A' + Q */
  P = add_f16_16 (P, mult_f16_16 (Pdot, dt));

  /* Update our state estimate from the rate gyro */
  theta = add_f16_16 (theta, mult_f16_16 (q, dt));

  gyro_only_theta = add_f16_16 (gyro_only_theta, mult_f16_16 (q, dt));

  if (!do_kalman)
    return theta;

  E = add_f16_16 (P, R);				/* E = CPC' + R */
  K = div_f16_16 (P, E);				/* K = PC'inv(E) */

  /* Update the state */
  theta = add_f16_16 (theta, mult_f16_16 (K, sub_f16_16(theta_m, theta )));

  /* Covariance update */
  P = add_f16_16 (mult_f16_16 (P, mult_f16_16 (sub_f16_16 (1 * 65536, K),
					       sub_f16_16 (1 * 65536, K))),
		  mult_f16_16 (R, mult_f16_16 (K, K)));

  return theta;
}

/* Kalman filter task.  Reads the gyro & accelerometer, and runs the
   kalman filter. */
rtems_task
kalman_task(rtems_task_argument ignored)
{
  rtems_name period_name;
  rtems_id period;
  rtems_status_code status;
  f16_16 gyro_reading;
  f16_16 accel_reading;
  f16_16 theta_m;
  int kalman_cnt = 0, do_kalman;

  period_name = rtems_build_name ('K', 'L', 'P', 'D');
  status = rtems_rate_monotonic_create (period_name, &period);
  if (status != RTEMS_SUCCESSFUL)
    {
      printf ("kalman_task: rate_monotonic_create failed with status %d\n",
	      status);
      return;
    }

  while (1)
    {
      if (rtems_rate_monotonic_period (period, ticks_per_sec/UPDATE_HZ) ==
	  RTEMS_TIMEOUT)
	{
	  /* I'd like to do a printf here, but that would make us miss
	     our next timeout, until the end of time... */
	  kalman_timeouts++;
	}

      /* We update our angle at UPDATE_HZ, but only run the kalman
	 filter at KALMAN_HZ.  These should be a multiple of each
	 other, so that we run the kalman filter every
	 UPDATE_HZ/KALMAN_HZ updates. */
      do_kalman = (kalman_cnt++ == (UPDATE_HZ/KALMAN_HZ));
      if (do_kalman)
	kalman_cnt = 0;

      /* Read the gyro. */
      gyro_reading = gyro_read(GYRO_X);

      /* Only deal with the accelerometer reading if we're going to
	 run the filter. */
      if (do_kalman) {
	accel_reading = accel_read();

	if (accel_reading > 65536)
	  accel_reading = 65536;
	if (accel_reading < -65536)
	  accel_reading = -65536;

	/* Convert the accelerometer reading to an angle. */
	theta_m = f16_16_from_double(asin(double_from_f16_16(accel_reading)) *
				     180.0 / M_PI);
	last_theta_m = theta_m;
      }

      /* Run the filter. */
      theta = kalman(gyro_reading, theta_m, do_kalman);
    }
}

/* Initialize the kalman filter task. */
void
kalman_init(void)
{
  rtems_status_code code;
  Objects_Id t1;

  printf ("Spawning kalman filter task:\n");
  code = rtems_task_create(rtems_build_name('K', 'A', 'L', 'M'),
			   10, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, kalman_task, 0);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);
}

/* Get the current tilt of the platform.  Output is degrees as a 16.16
   number.  Note that a forward tilt is a positive number, and a
   backwards tilt is a negative number. */
int
kalman_read(void)
{
  return (int)theta;
}

/* Get the most recent 'theta_m' reading (which is the accelerometer
   reading converted to degrees). */
int
kalman_read_theta_m(void)
{
  return (int)last_theta_m;
}

/* Get the most recent gyro only reading (which is the integrated gyro
   readings). */
int
kalman_read_gyro_only(void)
{
  return (int)gyro_only_theta;
}
