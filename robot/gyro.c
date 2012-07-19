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

#include <bsp.h>
#include <qsm.h>
#include <sim.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "global.h"
#include "gyro.h"
#include "f16_16.h"

/**********************************************************************/
/* Constants */
/**********************************************************************/

/* Base address of analog-to-digital converter. */
#define ATOD_BASE	0xf00000

/* Specifies which channel of ATOD chip that the gyros are on. */
#define GYRO_X_ATOD	1
#define GYRO_Z_ATOD	0 /* TBD! */

#if 0
/* simple filter gyro board: */

/* The gyro itself generates a signal that is 1.1mv/deg/s +- 20%.
   There is an op-amp that multiplies this by 10, so that results in
   11mv/deg/s +- 20% (8.8 to 13.2).  With an 8-bit 5V A-D
   converter, there are 19.6mv/bit.  So each bit different means
   somewhere between 1.48 deg/s and 2.23.  A rough experiment shows
   the actual value for the X gyro to be about 18.7% high, or 2.1.
   Expressed as a 16.16 fixed-point number, this is 137626. */
#define GYRO_DEGS_PER_BIT	137626
#else
/* complex filter gyro board: */

/* The gyro itself generates a signal that is 1.1mv/deg/s +- 20%.
   There is an op-amp that multiplies this by 23, so that results in
   25.3mv/deg/s +- 20% (20.24 to 30.36).  With an 8-bit 5V A-D
   converter, there are 19.6mv/bit.  So each bit different means
   somewhere between 0.646 deg/s and 0.968.  A rough experiment shows
   the actual value for the X gyro to be about 18.7% high, or 0.920.
   Expressed as a 16.16 fixed-point number, this is 60265. */
#define GYRO_DEGS_PER_BIT	60265
#endif

int gyro_degs_per_bit = 92569/*GYRO_DEGS_PER_BIT*/;

/* Number of gyro readings taken per second.  NOTE: this should match
   UPDATE_HZ in kalman.c! */
#define GYRO_HZ	250

/**********************************************************************/
/* Globals */
/**********************************************************************/

/* Contains the neutral values of the gyros, as 16.16 fixed point
   numbers. */
volatile int gyro_x_neutral, gyro_z_neutral;

volatile int gyro_last_x = 0, gyro_last_z = 0;

#define MAX_GYRO_CALIBRATE_SECONDS 10

volatile int gyro_calibrating = 0;
volatile int gyro_cal_cnt = 0;
volatile int gyro_x_vals[GYRO_HZ*MAX_GYRO_CALIBRATE_SECONDS];
volatile int gyro_z_vals[GYRO_HZ*MAX_GYRO_CALIBRATE_SECONDS];

int gyro_calibrate_seconds = 5;

int gyro_timeouts = 0;

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Read an AtoD channel. */
int
read_atod(int channel)
{
  unsigned char in;
  volatile unsigned char *chanptr = (char *)(ATOD_BASE + channel);

  /* Read it three times - the first time to set the channel, the
     second time to trigger a conversion, and the the third time to
     get the result of the conversion. */
  in = *chanptr;
  in = *chanptr;
  in = *chanptr;

  return in;
}

rtems_task
gyro_task(rtems_task_argument ignored)
{
  rtems_name period_name;
  rtems_id period;
  rtems_status_code status;

  period_name = rtems_build_name ('G', 'Y', 'P', 'D');
  status = rtems_rate_monotonic_create (period_name, &period);
  if (status != RTEMS_SUCCESSFUL)
    {
      printf ("gyro_task: rate_monotonic_create failed with status %d\n",
	      status);
      return;
    }

  while (1)
    {
      if (rtems_rate_monotonic_period (period, ticks_per_sec/GYRO_HZ) ==
	  RTEMS_TIMEOUT)
	{
	  /* I'd like to do a printf here, but that would make us miss
	     our next timeout, until the end of time... */
	  gyro_timeouts++;
	}

      gyro_last_x = read_atod(GYRO_X_ATOD);
      gyro_last_z = read_atod(GYRO_Z_ATOD);

      if (gyro_calibrating) {
	gyro_x_vals[gyro_cal_cnt] = gyro_last_x;
	gyro_z_vals[gyro_cal_cnt] = gyro_last_z;
	if (++gyro_cal_cnt >= GYRO_HZ*gyro_calibrate_seconds)
	  gyro_calibrating = 0;
      }
    }
}

/* Initialize the gyro library.  Returns 0 on success, non-zero on
   error. */
int
gyro_init(void)
{
  rtems_status_code code;
  Objects_Id t1;

  gyro_x_neutral = GYRO_X_NEUTRAL_DEFAULT;
  gyro_z_neutral = GYRO_Z_NEUTRAL_DEFAULT;

  printf ("Spawning gyro task:\n");
  code = rtems_task_create(rtems_build_name('G', 'Y', 'R', 'O'),
			   10, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, gyro_task, 0);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);

  return 0;
}

/* Returns the current reading of the gyro.  Return value is a 16.16
   fixed point number in degrees per second.  The 'gyro' parameter
   should be either GYRO_X or GYRO_Z. */
int
gyro_read(int gyro)
{
  int reading;

  if (gyro == GYRO_X) {
    reading = gyro_last_x;
    reading = (reading * 65536) - gyro_x_neutral;
  } else if (gyro == GYRO_Z) {
    reading = gyro_last_z;
    reading = (reading * 65536) - gyro_z_neutral;
  } else {
    return -1;
  }

  reading = mult_f16_16 (reading, gyro_degs_per_bit);

  return reading;
}

/* Reads the current neutral settings of the gyros, as 16.16 fixed
   point numbers. */
void gyro_read_neutral(int *gyro_x_neutral_p, int *gyro_z_neutral_p)
{
  *gyro_x_neutral_p = gyro_x_neutral;
  *gyro_z_neutral_p = gyro_z_neutral;
}

/* Sets the current neutral settings of the gyros, as 16.16 fixed
   point numbers. */
void gyro_set_neutral(int gyro_x_neutral_new, int gyro_z_neutral_new)
{
  gyro_x_neutral = gyro_x_neutral_new;
  gyro_z_neutral = gyro_z_neutral_new;
}

int
compare_ints(const void *pav, const void *pbv)
{
  int *pai = (int *)pav;
  int *pbi = (int *)pbv;
  int a = *pai;
  int b = *pbi;

  if (a < b)
    return -1;
  else if (a > b)
    return 1;
  else
    return 0;
}

/* Calculate the current neutral settings of the gyro.  The gyro
   _must_ be kept still while this is going on.  It will take 10
   seconds.  NOTE: Only one thread is allowed to call this routine at
   a time! */
void gyro_calibrate(void)
{
  int gyro_x_sum, gyro_z_sum;
  int i;

  /* Kick off the caibration. */
  gyro_cal_cnt = 0;
  gyro_calibrating = 1;

  while (gyro_calibrating)
    rtems_task_wake_after(ticks_per_sec/10);

  /* Now sort the arrays. */
  qsort ((int *)gyro_x_vals, GYRO_HZ*gyro_calibrate_seconds, sizeof(int), compare_ints);
  qsort ((int *)gyro_z_vals, GYRO_HZ*gyro_calibrate_seconds, sizeof(int), compare_ints);

  /* Now drop the GYRO_HZ highest and GYRO_HZ lowest values, and
     average the rest. */
  gyro_x_sum = gyro_z_sum = 0;
  for (i = GYRO_HZ; i < GYRO_HZ*(gyro_calibrate_seconds-1); i++) {
    gyro_x_sum += gyro_x_vals[i];
    gyro_z_sum += gyro_z_vals[i];
  }

  /* Converting one of these sums straight to a 16.16
     number would overflow it, so do the fractional
     part first, then add in the integer part. */
  gyro_x_neutral = ((gyro_x_sum % (GYRO_HZ * (gyro_calibrate_seconds-2))) * 65536 /
		    (GYRO_HZ * (gyro_calibrate_seconds-2))) +
    ((gyro_x_sum / (GYRO_HZ * (gyro_calibrate_seconds-2))) * 65536);
  gyro_z_neutral = ((gyro_z_sum % (GYRO_HZ * 8)) * 65536 /
		    (GYRO_HZ * (gyro_calibrate_seconds-2))) +
    ((gyro_z_sum / (GYRO_HZ * (gyro_calibrate_seconds-2))) * 65536);
}
