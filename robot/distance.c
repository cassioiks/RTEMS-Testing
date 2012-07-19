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

#include <bsp.h>
#include <stdio.h>
#include <sim.h>
#include "distance.h"
#include "global.h"
#include "fastint.h"
#include "motor.h"
#include "robot_trace.h"

/**********************************************************************/
/* Globals */
/**********************************************************************/

unsigned long dist_raw[5];

/* Conversion tables - used to convert from sensor readings to
   deci-inches.  Note that these are calibrated to the actual sensors
   on the robot as of 4/6/2003.  See the code in utils/dc3.c to
   generate these tables. */
signed short dist_tbl0[255] =
{
  /*   0 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  10 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  20 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  30 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  40 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  50 */  -1,  -1, 244, 235, 227, 219, 212, 205, 199, 193,
  /*  60 */ 187, 182, 177, 172, 168, 163, 159, 156, 152, 148,
  /*  70 */ 145, 142, 139, 136, 133, 130, 127, 125, 122, 120,
  /*  80 */ 118, 116, 114, 112, 110, 108, 106, 104, 102, 101,
  /*  90 */  99,  97,  96,  94,  93,  92,  90,  89,  88,  86,
  /* 100 */  85,  84,  83,  82,  81,  80,  78,  77,  76,  75,
  /* 110 */  75,  74,  73,  72,  71,  70,  69,  68,  68,  67,
  /* 120 */  66,  65,  65,  64,  63,  62,  62,  61,  60,  60,
  /* 130 */  59,  59,  58,  57,  57,  56,  56,  55,  54,  54,
  /* 140 */  53,  53,  52,  52,  51,  51,  50,  50,  49,  49,
  /* 150 */  48,  48,  48,  47,  47,  46,  46,  45,  45,  45,
  /* 160 */  44,  44,  43,  43,  43,  42,  42,  42,  41,  41,
  /* 170 */  41,  40,  40,  40,  39,  39,  39,  38,  38,  38,
  /* 180 */  37,  37,  37,  36,  36,  36,  36,  35,  35,  35,
  /* 190 */  34,  34,  34,  34,  33,  33,  33,  33,  32,  32,
  /* 200 */  32,  32,  31,  31,  31,  31,  30,  30,  30,  30,
  /* 210 */  29,  29,  29,  29,  29,  28,  28,  28,  28,  28,
  /* 220 */  27,  27,  27,  27,  27,  26,  26,  26,  26,  26,
  /* 230 */  25,  25,  25,  25,  25,  -1,  -1,  -1,  -1,  -1,
  /* 240 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /* 250 */  -1,  -1,  -1,  -1,  -1, 
};

signed short dist_tbl1[255] =
{
  /*   0 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  10 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  20 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  30 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  40 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  50 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  60 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  70 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  80 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  90 */  -1,  -1, 244, 236, 228, 220, 214, 207, 201, 195,
  /* 100 */ 190, 184, 180, 175, 170, 166, 162, 158, 155, 151,
  /* 110 */ 148, 144, 141, 138, 135, 133, 130, 127, 125, 123,
  /* 120 */ 120, 118, 116, 114, 112, 110, 108, 106, 104, 103,
  /* 130 */ 101,  99,  98,  96,  95,  93,  92,  90,  89,  88,
  /* 140 */  86,  85,  84,  83,  82,  80,  79,  78,  77,  76,
  /* 150 */  75,  74,  73,  72,  71,  70,  69,  69,  68,  67,
  /* 160 */  66,  65,  64,  64,  63,  62,  61,  61,  60,  59,
  /* 170 */  59,  58,  57,  57,  56,  55,  55,  54,  53,  53,
  /* 180 */  52,  52,  51,  50,  50,  49,  49,  48,  48,  47,
  /* 190 */  47,  46,  46,  45,  45,  44,  44,  43,  43,  42,
  /* 200 */  42,  41,  41,  41,  40,  40,  39,  39,  39,  38,
  /* 210 */  38,  37,  37,  37,  36,  36,  35,  35,  35,  34,
  /* 220 */  34,  34,  33,  33,  32,  32,  32,  31,  31,  31,
  /* 230 */  30,  30,  30,  30,  29,  29,  29,  28,  28,  28,
  /* 240 */  27,  27,  27,  26,  26,  26,  26,  25,  25,  25,
  /* 250 */  -1,  -1,  -1,  -1,  -1, 
};

signed short dist_tbl2[255] =
{
  /*   0 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  10 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  20 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  30 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  40 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  50 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  60 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  70 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  80 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1, 244,
  /*  90 */ 235, 227, 219, 212, 205, 199, 193, 187, 182, 177,
  /* 100 */ 172, 168, 163, 159, 155, 152, 148, 145, 142, 138,
  /* 110 */ 135, 133, 130, 127, 125, 122, 120, 117, 115, 113,
  /* 120 */ 111, 109, 107, 105, 103, 102, 100,  98,  97,  95,
  /* 130 */  94,  92,  91,  89,  88,  87,  86,  84,  83,  82,
  /* 140 */  81,  80,  78,  77,  76,  75,  74,  73,  72,  71,
  /* 150 */  71,  70,  69,  68,  67,  66,  65,  65,  64,  63,
  /* 160 */  62,  62,  61,  60,  59,  59,  58,  57,  57,  56,
  /* 170 */  56,  55,  54,  54,  53,  53,  52,  51,  51,  50,
  /* 180 */  50,  49,  49,  48,  48,  47,  47,  46,  46,  45,
  /* 190 */  45,  44,  44,  44,  43,  43,  42,  42,  41,  41,
  /* 200 */  41,  40,  40,  39,  39,  39,  38,  38,  38,  37,
  /* 210 */  37,  36,  36,  36,  35,  35,  35,  34,  34,  34,
  /* 220 */  33,  33,  33,  32,  32,  32,  32,  31,  31,  31,
  /* 230 */  30,  30,  30,  30,  29,  29,  29,  28,  28,  28,
  /* 240 */  28,  27,  27,  27,  27,  26,  26,  26,  26,  25,
  /* 250 */  25,  25,  25,  -1,  -1, 
};

signed short dist_tbl3[255] =
{
  /*   0 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  10 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  20 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  30 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  40 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  50 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  60 */  -1, 247, 239, 231, 224, 217, 211, 205, 199, 194,
  /*  70 */ 189, 184, 179, 175, 171, 167, 163, 160, 156, 153,
  /*  80 */ 150, 146, 144, 141, 138, 135, 133, 130, 128, 126,
  /*  90 */ 123, 121, 119, 117, 115, 113, 112, 110, 108, 107,
  /* 100 */ 105, 103, 102, 100,  99,  97,  96,  95,  93,  92,
  /* 110 */  91,  90,  89,  87,  86,  85,  84,  83,  82,  81,
  /* 120 */  80,  79,  78,  77,  76,  75,  75,  74,  73,  72,
  /* 130 */  71,  71,  70,  69,  68,  68,  67,  66,  65,  65,
  /* 140 */  64,  63,  63,  62,  62,  61,  60,  60,  59,  59,
  /* 150 */  58,  57,  57,  56,  56,  55,  55,  54,  54,  53,
  /* 160 */  53,  52,  52,  51,  51,  51,  50,  50,  49,  49,
  /* 170 */  48,  48,  48,  47,  47,  46,  46,  46,  45,  45,
  /* 180 */  44,  44,  44,  43,  43,  43,  42,  42,  42,  41,
  /* 190 */  41,  41,  40,  40,  40,  39,  39,  39,  39,  38,
  /* 200 */  38,  38,  37,  37,  37,  36,  36,  36,  36,  35,
  /* 210 */  35,  35,  35,  34,  34,  34,  34,  33,  33,  33,
  /* 220 */  33,  32,  32,  32,  32,  31,  31,  31,  31,  30,
  /* 230 */  30,  30,  30,  30,  29,  29,  29,  29,  29,  28,
  /* 240 */  28,  28,  28,  28,  27,  27,  27,  27,  27,  26,
  /* 250 */  26,  26,  26,  26,  25, 
};

signed short dist_tbl4[255] =
{
  /*   0 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  10 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  20 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  30 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  40 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  50 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  60 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  70 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  80 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /*  90 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /* 100 */  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,  -1,
  /* 110 */  -1,  -1,  -1,  -1,  -1,  -1, 245, 238, 231, 225,
  /* 120 */ 219, 213, 207, 202, 197, 193, 188, 184, 180, 175,
  /* 130 */ 172, 168, 164, 161, 158, 154, 151, 148, 145, 143,
  /* 140 */ 140, 137, 135, 132, 130, 128, 125, 123, 121, 119,
  /* 150 */ 117, 115, 113, 111, 109, 108, 106, 104, 103, 101,
  /* 160 */  99,  98,  96,  95,  93,  92,  91,  89,  88,  87,
  /* 170 */  85,  84,  83,  82,  80,  79,  78,  77,  76,  75,
  /* 180 */  74,  73,  72,  71,  70,  69,  68,  67,  66,  65,
  /* 190 */  64,  63,  62,  61,  60,  59,  59,  58,  57,  56,
  /* 200 */  55,  55,  54,  53,  52,  52,  51,  50,  49,  49,
  /* 210 */  48,  47,  47,  46,  45,  44,  44,  43,  43,  42,
  /* 220 */  41,  41,  40,  39,  39,  38,  37,  37,  36,  36,
  /* 230 */  35,  35,  34,  33,  33,  32,  32,  31,  31,  30,
  /* 240 */  29,  29,  28,  28,  27,  27,  26,  26,  25,  25,
  /* 250 */  -1,  -1,  -1,  -1,  -1, 
};

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Calculates our angle relative to the wall, and (if we calculate
   something reasonable) calls mot_heading_update() to update the
   motor task with a heading reference. */
void
do_heading_update(void)
{
  int lf, lr, avgl;
  int rf, rr, avgr;
  int al, ar;
  int angle;

  if (!mot_balancing())
    return;

  lf = dist_tbl1[dist_raw[1]];
  lr = dist_tbl3[dist_raw[3]];
  rf = dist_tbl2[dist_raw[2]];
  rr = dist_tbl4[dist_raw[4]];

  if ((lf > 0) && (lr > 0))
    al = fixup_angle (fastatan2 (lf-lr, DIST_SENSOR_SEPARATION));
  else
    al = -1;

  if ((rf > 0) && (rr > 0))
    ar = fixup_angle (fastatan2 (rr-rf, DIST_SENSOR_SEPARATION));
  else
    ar = -1;

  if ((ar >= 0) && (al >= 0)) {
    /* We can see walls with all four sensors, so we have two possible
       headings.  If they are close (within 10 degrees of each other),
       then average them and call that the update.  Otherwise, pick
       the one where the wall is closest & use that.  This should
       avoid cases where we are going around a corner and see two
       different walls. */
    if (abs_dir_diff (ar,al) <= 10) {
      /* Note the funky calculation to average two angles.  This
	 handles the case where one angle is just above 0 & the other
	 is just below 360. */
      angle = fixup_angle (leftmost_angle(al, ar) + (abs_dir_diff(ar,al) / 2));
    } else {
      avgl = (lf+lr+1) / 2;
      avgr = (rf+rr+1) / 2;
      if (avgl < avgr)
	angle = al;
      else
	angle = ar;
    }
  } else if (al >= 0) {
    /* We can only get a heading from the left side - use that. */
    angle = al;
  } else if (ar >= 0) {
    /* We can only get a heading from the right side - use that. */
    angle = ar;
  } else {
    /* We can't get a heading update this time. */
    angle = -1;
  }

#if 0
  /* This fills up the log quick and should probably be commented out. */
  TRACE_LOG5 (ROBOT, HEADING_UPDATE, lf, lr, rf, rr, angle);
#endif

  if (angle > 0)
    mot_heading_update(angle);
}

/* The task that reads the sensors and updates the global
   variables. */
rtems_task
distance_task (rtems_task_argument ignored)
{
  unsigned long in_progress[5];
  unsigned char val;
  int i;
  volatile int v;
  rtems_interrupt_level level;

  /* First, set up port F pins.  Bit 4 is output - the clock to all
     the sensors, and bits 5-7 are inputs - the result from each
     sensor.

     Added 2 more sensors on bits 2 and 3 on 4/6/2003. */
  *PFPAR &= 0x03; /* clear the PAR bits */
  *PORTF0 |= 0x10; /* Output a 1 to clock. */
  *DDRF |= 0x10; /* Set bit 4 output. */
  *DDRF &= 0x13; /* Set bits 2-3 and 5-7 input. */

  /* Ensure that the sensors are reset - they reset if the clock is
     held high for 1.5ms. */
  rtems_task_wake_after(MAX(ticks_per_sec / 200 + 1, 2));

  while (1)
    {
      /* Output a low to the clock line, wait for all inputs to go
         low. */
      *PORTF0 &= 0xEF;
      while ((*PORTF0 & 0xEC) != 0)
	continue;

      /* Now wait for all inputs to go high. */
      while ((*PORTF0 & 0xEC) != 0xEC)
	rtems_task_wake_after(1);

      /* Clock out the data. */
      in_progress[0] = in_progress[1] = in_progress[2] =
	in_progress[3] = in_progress[4] = 0;
      for (i=7; i>=0; i--)
	{
	  rtems_interrupt_disable(level);
	  *PORTF0 |= 0x10;
	  for (v=0; v<20; v++)
	    continue;
	  *PORTF0 &= 0xEF;
	  rtems_interrupt_enable(level);
	  for (v=0; v<20; v++)
	    continue;
	  val = *PORTF0;
	  in_progress[0] |= ((val & 0x20) >> 5) << i;
	  in_progress[1] |= ((val & 0x40) >> 6) << i;
	  in_progress[2] |= ((val & 0x80) >> 7) << i;
	  in_progress[3] |= ((val & 0x04) >> 2) << i;
	  in_progress[4] |= ((val & 0x08) >> 3) << i;
	}

      /* Update global variables. */
      for (i=0; i<5; i++)
	dist_raw[i] = in_progress[i];

      /* Set clock high */
      *PORTF0 |= 0x10;

      /* Update the motor task with our current heading relative to
	 the walls we can see, if any. */
      do_heading_update();

      /* give the sensor a rest, then go again. */
      rtems_task_wake_after(MAX(ticks_per_sec/100, 1));
    }
}

/* Initialize the distance sensor task.  Returns 0 on success,
   non-zero on error. */
int
distance_init(void)
{
  Objects_Id t1;
  rtems_status_code code;

  printf ("Spawning Distance Sensor task:\n");
  code = rtems_task_create(rtems_build_name('D', 'I', 'S', 'T'),
			   18, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_NO_PREEMPT | RTEMS_NO_TIMESLICE | RTEMS_NO_ASR,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  if (code == 0)
    {
      code = rtems_task_start(t1, distance_task, 0);
      printf ("Done. (rtems_task_start returned %d)\n\n", code);
    }

  return code;
}

/* Read a distance sensor.  The return value is a 32-bit signed
   number specifying centi-inches (inches * 10). */
long
distance_read(int which_sensor)
{
  switch (which_sensor)
    {
    case DISTANCE_FRONT:
      return dist_tbl0[dist_raw[which_sensor]];

    case DISTANCE_LEFT:
      return dist_tbl1[dist_raw[which_sensor]];

    case DISTANCE_RIGHT:
      return dist_tbl2[dist_raw[which_sensor]];

    case DISTANCE_LEFT_REAR:
      return dist_tbl3[dist_raw[which_sensor]];

    case DISTANCE_RIGHT_REAR:
      return dist_tbl4[dist_raw[which_sensor]];

    default:
      return 0;
    }
}

/* Read the raw value from a distance sensor.  Returns a number in the
   range 0 to 255. */
unsigned long
distance_read_raw(int which_sensor)
{
  switch (which_sensor)
    {
    case DISTANCE_FRONT:
    case DISTANCE_LEFT:
    case DISTANCE_RIGHT:
    case DISTANCE_LEFT_REAR:
    case DISTANCE_RIGHT_REAR:
      return dist_raw[which_sensor];

    default:
      return 0;
    }
}
