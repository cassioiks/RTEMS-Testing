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
 *  Simple test program -- simplified version of sample test hello.
 */

#define TEST_INIT

#include <bsp.h>

#include <stdio.h>
#include <stdlib.h>
#include <sim.h>
#include <string.h>
#include <rtems/rtmonuse.h>
#include <sys/ioctl.h>
#include <sys/termios.h>

#include "global.h"
#include "tpu.h"
#include "lcd.h"
#include "motor.h"
#include "servo.h"
#include "distance.h"
#include "spi.h"
#include "robot.h"
#include "flame.h"
#include "mcp3208.h"
#include "gyro.h"
#include "accel.h"
#include "kalman.h"
#include "fastint.h"
#include "robot_trace.h"
#include "tone.h"

#include <qsm.h>

int
is_character_ready(void)
{
#if 1
  return 0;
#elif 0
  /* Use this code if using polled-mode tty driver. */
  if (*SCSR & RDRF)
    return 1;
  else
    return 0;
#else
  struct termios termios, otermios;
  char ch;
  int retval;
  static int printed_error = 0;

  if (ioctl(0, RTEMS_IO_GET_ATTRIBUTES, &termios)) {
    if (!printed_error) {
      printf ("is_character_ready: ioctl(0, RTEMS_IO_GET_ATTRIBUTES, &termios) failed!\n");
      printed_error = 1;
    }
    return 0;
  }
  memcpy (&otermios, &termios, sizeof(struct termios));
  termios.c_lflag &= ~ICANON;
  termios.c_cc[VTIME] = 0;
  termios.c_cc[VMIN] = 0;
  if (ioctl(0, RTEMS_IO_SET_ATTRIBUTES, &termios)) {
    if (!printed_error) {
      printf ("is_character_ready: ioctl(0, RTEMS_IO_SET_ATTRIBUTES, &termios) failed!\n");
      printed_error = 1;
    }
    return 0;
  }

  retval = read(0, &ch, 1);
  if (retval < 0)
    retval = 0;

  if (ioctl(0, RTEMS_IO_SET_ATTRIBUTES, &otermios)) {
    if (!printed_error) {
      printf ("is_character_ready: ioctl(0, RTEMS_IO_SET_ATTRIBUTES, &otermios) failed!\n");
      printed_error = 1;
    }
  }

  return retval;
#endif
}

rtems_interval ticks_per_sec = 0;

typedef enum lcd_mode {
  LCD_MIN = 0,
  LCD_MOT0 = 0,
  LCD_DIST = 1,
  LCD_HEADING = 2,
  LCD_FLAME = 3,
  LCD_ATOD = 4,
  LCD_CANDLE = 5,
  LCD_GYRO = 6,
  LCD_ACCEL = 7,
  LCD_KALMAN = 8,
  LCD_DIST_REAR = 9,
  LCD_TONE,
  LCD_MAX
} lcd_mode_t;

char *lcd_mode_names[] = { "MOT0", "DIST", "HEADING", "FLAME", "ATOD",
			   "CANDLE", "GYRO", "ACCEL", "KALMAN", "DIST_REAR",
			   "TONE" };
lcd_mode_t lcd_mode = LCD_KALMAN;

int ir_angles[8] = { -27, -20, -12, -4, 4, 12, 20, 27 };

#undef ACCEL_MODE_RAW_AVG

/* mode of 2nd line in kalman mode:
 * 0 -> theta_m
 * 1 -> gyro_only
 * 2 -> desired_tilt
 */
#define KALMAN_SECOND_LINE 2

rtems_task
lcd_task (rtems_task_argument ignored)
{
  char buf[80];
  char *spaces = "                    "; /* 20 space characters */
  mot_status_t m0;
  int i, j, sensor;
  unsigned long rd[3];
  long d[3];
  char dbuf[3][20];
  int chans[8];
  unsigned short results[8];
  unsigned sums[8];
  unsigned avg;
  int hi, hi2;
  int hi_idx, hi2_idx;
  int num_samples;
  int retval, dist, angle;
  int hi_l, hi_r;
  int heading;
  int gyro_x, gyro_z;
  int accel, accel_raw;
  unsigned int tone_raw;
  int tone;
#ifdef ACCEL_MODE_RAW_AVG
  int accel_raw_avg = 0;
#endif
  int kalman;
#if KALMAN_SECOND_LINE == 2
  extern int32 mot_desired_tilt;
#endif

  while (1)
    {
      switch (lcd_mode)
	{
	case LCD_MOT0:
	  mot_get_status(&m0);

	  sprintf(buf, "m0pos: 0x%08x", m0.pos);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  sprintf(buf, "v0x%04x, a0x%04x", (short)m0.velocity,
		  (short)m0.accel);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(1, buf);
	  break;

	case LCD_DIST:
	  for (i=0; i<3; i++)
	    {
	      rd[i] = distance_read_raw(i);
	      d[i] = distance_read(i);
	      if (d[i] >= 0)
		sprintf(dbuf[i], "%2ld.%ld", d[i]/10, d[i]%10);
	      else
		sprintf(dbuf[i], " inf");
	    }
	  sprintf(buf, "0:%s 1:%s 2:%s",
		  dbuf[0], dbuf[1], dbuf[2]);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  sprintf(buf, "0:%02lx   1:%02lx   2:%02lx",
		  rd[0], rd[1], rd[2]);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(1, buf);
	  break;

	case LCD_HEADING:
	  heading = mot_get_heading();
	  sprintf(buf, "Mot Heading: %3d.%02d    ", heading / 256,
		  (heading % 256) * 100 / 256);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  lcd_string(1, spaces);
	  break;

	case LCD_FLAME:
	  sprintf(buf, "flame: %d    ", flame_read(0));
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  lcd_string(1, spaces);
	  break;

	case LCD_ATOD:
	  for (sensor=0; sensor<2; sensor++)
	    {
	      /* Init. */
	      for (i=0; i<8; i++)
		{
		  sums[i] = 0;
		  chans[i] = i;
		}

	      /* Poll all IR phototransistors 4 times. */
	      num_samples = 8;
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
		  sums[i] /= num_samples;

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
	      avg = avg/8;

	      if ((hi2_idx >= 0) &&
		  (abs(hi_idx - hi2_idx) == 1))
		{
		  angle = (ir_angles[hi_idx] * hi +
			   ir_angles[hi2_idx] * hi2) / (hi + hi2);
		}
	      else
		{
		  angle = ir_angles[hi_idx];
		}

	      if ((hi > (avg * 3 / 2)) &&
		  (hi > 0x20))
		{
		  sprintf(buf, "%d: %3d %03x %03x", sensor, angle,
			  avg, hi);
		}
	      else
		{
		  sprintf(buf, "%d: --- %03x %03x %3d", sensor, avg, hi,
			  angle);
		}
	      if (strlen(buf) < 20)
		strncat(buf, spaces, 20 - strlen(buf));
	      lcd_string(sensor, buf);
	    }
	  break;

	case LCD_CANDLE:
	  retval = find_candle(&dist, &angle, &hi_l, &hi_r);
	  if (retval == 0)
	    sprintf(buf, "%3d %3d %03x %03x", dist, angle, hi_l, hi_r);
	  else if (retval == 1)
	    sprintf(buf, "far left %03x %03x", hi_l, hi_r);
	  else if (retval == 2)
	    sprintf(buf, "far right %03x %03x", hi_l, hi_r);
	  else
	    sprintf(buf, "???? %03x %03x", hi_l, hi_r);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  lcd_string(1, spaces);
	  break;

	case LCD_GYRO:
	  gyro_x = gyro_read(GYRO_X);
	  gyro_z = gyro_read(GYRO_Z);
	  sprintf(buf, "x: %c%d.%02d", gyro_x < 0 ? '-' : ' ',
		  abs(gyro_x) / 65536,
		  abs((gyro_x % 65536) * 100 / 65536));
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  sprintf(buf, "z: %c%d.%02d", gyro_z < 0 ? '-' : ' ',
		  abs(gyro_z) / 65536,
		  abs((gyro_z % 65536) * 100 / 65536));
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(1, buf);
	  break;

	case LCD_ACCEL:
	  accel_raw = accel_read_raw();
	  sprintf(buf, "raw: %6u", accel_raw);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
#ifndef ACCEL_MODE_RAW_AVG
	  accel = accel_read();
	  sprintf(buf, "real: %c%d.%02d",
		  accel < 0 ? '-' : ' ',
		  abs(accel) / 65536,
		  abs((accel % 65536) * 100 / 65536));
#else
	  if (accel_raw_avg == 0)
	    accel_raw_avg = accel_raw;
	  else {
	    accel_raw_avg = ((accel_raw_avg * 99) + accel_raw) / 100;
	  }
	  sprintf(buf, "avg: %6u", accel_raw_avg);
#endif
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(1, buf);
	  break;

	case LCD_KALMAN:
	  kalman = kalman_read();
	  sprintf(buf, "kalman: %c%d.%02d", kalman < 0 ? '-' : ' ',
		  abs(kalman) / 65536,
		  abs((kalman % 65536) * 100 / 65536));
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
#if KALMAN_SECOND_LINE == 0
	  kalman = kalman_read_theta_m();
	  sprintf(buf, "theta_m: %c%d.%02d", kalman < 0 ? '-' : ' ',
		  abs(kalman) / 65536,
		  abs((kalman % 65536) * 100 / 65536));
#elif KALMAN_SECOND_LINE == 1
	  kalman = kalman_read_gyro_only();
	  sprintf(buf, "gyro_only: %c%d.%02d", kalman < 0 ? '-' : ' ',
		  abs(kalman) / 65536,
		  abs((kalman % 65536) * 100 / 65536));
#elif KALMAN_SECOND_LINE == 2
	  sprintf(buf, "des_tilt: %c%d.%02d",
		  mot_desired_tilt < 0 ? '-' : ' ',
		  abs(mot_desired_tilt) / 256,
		  abs((mot_desired_tilt % 256) * 100 / 256));
#else
	  strcpy (buf, "problem in init.c");
#endif
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(1, buf);
	  break;

	case LCD_DIST_REAR:
	  for (i=0; i<2; i++)
	    {
	      rd[i] = distance_read_raw(i+3);
	      d[i] = distance_read(i+3);
	      if (d[i] >= 0)
		sprintf(dbuf[i], "%2ld.%ld", d[i]/10, d[i]%10);
	      else
		sprintf(dbuf[i], " inf");
	    }
	  sprintf(buf, "3:%s 4:%s",
		  dbuf[0], dbuf[1]);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  sprintf(buf, "3:%02lx   4:%02lx",
		  rd[0], rd[1]);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(1, buf);
	  break;

	case LCD_TONE:
	  tone_raw = tone_read_raw();
	  sprintf(buf, "tone raw: %u    ", tone_raw);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(0, buf);
	  tone = tone_read();
	  sprintf(buf, "tone good? %d    ", tone);
	  if (strlen(buf) < 20)
	    strncat(buf, spaces, 20 - strlen(buf));
	  lcd_string(1, buf);
	  break;

	default:
	  lcd_string(0, spaces);
	  lcd_string(1, spaces);
	  break;
	}

      rtems_task_wake_after(ticks_per_sec/10);
    }
}

rtems_task
demo_task(rtems_task_argument which_demo)
{
  uint32 ticks;
  mot_status_t m0;
  int count = 0;
  int heading;
  int i;

  while (1) {
    if (which_demo == 0)
    {
      rtems_task_wake_after(ticks_per_sec * 10);

      mot_balance(1);

      rtems_task_wake_after(ticks_per_sec); /* give it a second to get
					       it's balance */

      heading = mot_get_heading();
      for (i=0; i<8; i++)
	{
	  printf ("Moving forward 6 inches.\n");
	  ticks = mot_get_ticks();
	  mot_move(MOT_STEPS_PER_INCH*6, 1, 1, ticks+2);

	  while(1)
	    {
	      if (is_character_ready())
		goto demo_done;
	      rtems_task_wake_after(ticks_per_sec/10);
	      mot_get_status(&m0);
	      if (m0.stopped)
		break;
	    }

	  printf ("Turning left 90 degrees.\n");
	  heading -= 90 * 256;
	  while (heading < 0)
	    heading += 360 * 256;
	  if (heading > 360 * 256)
	    heading %= 360 * 256;
	  printf ("  to heading %d.%02d\n", heading/256,
		  (heading % 256) * 1000 / 256);
	  robot_turn_to(heading);

	  printf ("Done turning.\n");

	  while(1)
	    {
	      if (is_character_ready())
		goto demo_done;
	      rtems_task_wake_after(ticks_per_sec/10);
	      mot_get_status(&m0);
	      if (m0.stopped)
		break;
	    }

	  if (((++count) % 4) == 0)
	    {
	      printf ("Blasting extinguisher!");
	      servo_set(FAN_SERVO,FAN_SERVO_RUNNING);
	      rtems_task_wake_after(ticks_per_sec/2);
	      servo_set(FAN_SERVO,FAN_SERVO_STOPPED);
	    }
	}
    }
    else if (which_demo == 1)
    {
      /* demo 1 removed, it only tested the compass. */
    }
    else if (which_demo == 2)
    {
      *PORTC |= 0x30;
      *CSPAR1 &= ~0x3c;
      while (1)
	{
	  if (flame_read(0) > 2)
	    {
	      *PORTC &= ~0x20;
	      rtems_task_wake_after(ticks_per_sec*5);
	      *PORTC |= 0x20;
	      mot_balance(1);
	      if (put_out_fire() == 0)
		{
		  for (i=0; i<5; i++)
		    {
		      *PORTC &= ~0x10;
		      rtems_task_wake_after(ticks_per_sec/2);
		      *PORTC |= 0x10;
		      rtems_task_wake_after(ticks_per_sec/2);
		    }
		}
	    }
	  else
	    {
	      rtems_task_wake_after(ticks_per_sec/2);
	      
	      if (is_character_ready()) /* HACK! Calling console driver directly! */
		goto demo_done;

	    }
	}
    }
    else if (which_demo == 3)
    {
      while (1)
	{
	  printf ("Hit buzzer to start demo 3.\n");
	  while (!tone_read())
	    {
	      rtems_task_wake_after(ticks_per_sec/10);
	      if (is_character_ready()) /* HACK! Calling console driver directly! */
		goto demo_done;
	    }
	  run_maze();
	}
    }
    else if (which_demo == 4)
    {
      while (1)
	{
	  printf ("Hit buzzer to start demo 4\n");
	  while (!tone_read())
	    {
	      rtems_task_wake_after(ticks_per_sec/10);
	      if (is_character_ready()) /* HACK! Calling console driver directly! */
		goto demo_done;
	    }
	  doit();
	}
    }

  demo_done:
    mot_balance(0);
    printf ("Demo finished.\n");
  }
}

int demo_task_started = 0;

void
demo(int which_demo)
{
  Objects_Id t1;
  rtems_status_code code;

  if (demo_task_started) {
    printf ("Demo task already started - reboot to run it again.\n");
    return;
  }

  printf ("Spawning demo task:\n");
  code = rtems_task_create(rtems_build_name('D', 'E', 'M', 'O'),
			   20, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, demo_task, which_demo);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);

  demo_task_started = 1;
}

void
ui_help(void)
{
  printf ("\n");
  printf ("Valid commands:\n");
  printf ("\n");
  printf ("pid - print PID constants\n");
  printf ("pidt - dump PID trace\n");
  printf ("kp - set kp (position) constant of PID loop\n");
  printf ("kd - set kd (derivative) constant of PID loop\n");
  printf ("ki - set ki (integral) constant of PID loop\n");
  printf ("bal - turn balancing on/off\n");
  printf ("maxt - set min/max tilt.\n");
  printf ("kpb - set kp (position) constant of PID loop for balancing\n");
  printf ("kdb - set kd (derivative) constant of PID loop for balancing\n");
  printf ("kib - set ki (integral) constant of PID loop for balancing\n");
  printf ("kph - set kp (position) constant of PID loop for eading\n");
  printf ("kdh - set kd (derivative) constant of PID loop for heading\n");
  printf ("kih - set ki (integral) constant of PID loop for heading\n");
  printf ("\n");
  printf ("acc - set acceleration (steps/tick/tick)\n");
  printf ("vel - set velocity (steps/tick)\n");
  printf ("steps - set # of steps to move\n");
  printf ("go - tell motors to move\n");
  printf ("hvel - set heading velocity (degrees/second)\n");
  printf ("head - set heading (in degrees)\n");
  printf ("demo - run a little demo\n");
  printf ("lcd - set lcd mode\n");
  printf ("ad - read a-to-d converter (specify channel 0-15)\n");
  printf ("\n");
  printf ("grn - read gyro neutral values.\n");
  printf ("gsnx - set gyro X neutral value.\n");
  printf ("gsnz - set gyro Z neutral value.\n");
  printf ("gc - calibrate gyros.\n");
  printf ("rt - dump robot trace buffer.\n");
  printf ("There are about %d steps to an inch, 100 ticks per second\n",
	  MOT_STEPS_PER_INCH);
  printf ("\n");
}

rtems_task
ui_task (rtems_task_argument ignored)
{
  int32 kp = 0, kd = 0, ki = 0;
  int32 kp_bal = 0, kd_bal = 0, ki_bal = 0;
  int32 kp_hd = 0, kd_hd = 0, ki_hd = 0;
  int32 acc = robot_acc, vel = robot_vel, steps = 0;
  int heading_vel = 15;
  int32 val;
  int i;
  char buf[80];
  char *cmd, *sval;
  int chan, status;
  unsigned short result;
  extern int mot_max_tilt, mot_min_tilt;

  demo(2); /* 2 - just fire test, 3 - whole shebang. */

  servo_set(FLAME_CAP_SERVO, FLAME_CAP_UP);

  mot_get_pid(&kp, &kd, &ki);
  mot_get_bal_pid(&kp_bal, &kd_bal, &ki_bal);
  mot_get_hd_pid(&kp_hd, &kd_hd, &ki_hd);
  while (1)
    {
      printf ("cmd: ");
      fgets(buf, 80, stdin);

      cmd = strtok(buf, " \t\n");
      if (cmd == NULL)
	{
	  continue;
	}
      sval = strtok(NULL, " \t\n");
      if (sval == NULL)
	{
	  val = 0;
	}
      else
	{
	  val = strtol(sval, NULL, 0);
	}

      if (strcmp(cmd, "pid") == 0)
	{
	  printf ("kp 0x%08x kd 0x%08x ki 0x%08x\n",
		  kp, kd, ki);
	  printf ("kp_bal 0x%08x kd_bal 0x%08x ki_bal 0x%08x\n",
		  kp_bal, kd_bal, ki_bal);
	  printf ("kp_hd 0x%08x kd_hd 0x%08x ki_hd 0x%08x\n",
		  kp_hd, kd_hd, ki_hd);
	}
      else if (strcmp(cmd, "pidt") == 0)
	{
	  mot_dump_pid_trace();
	}
      else if (strcmp(cmd, "rt") == 0)
	{
	  TRACE_DUMP (ROBOT, 1);
	}
      else if (strcmp(cmd, "kp") == 0)
	{
	  kp = val;
	  printf ("Set kp to 0x%08x\n", kp);
	  mot_set_pid(kp, kd, ki);
	  printf ("kp 0x%08x kd 0x%08x ki 0x%08x\n",
		  kp, kd, ki);
	}
      else if (strcmp(cmd, "kd") == 0)
	{
	  kd = val;
	  printf ("Set kd to 0x%08x\n", kd);
	  mot_set_pid(kp, kd, ki);
	  printf ("kp 0x%08x kd 0x%08x ki 0x%08x\n",
		  kp, kd, ki);
	}
      else if (strcmp(cmd, "ki") == 0)
	{
	  ki = val;
	  printf ("Set ki to 0x%08x\n", ki);
	  mot_set_pid(kp, kd, ki);
	  printf ("kp 0x%08x kd 0x%08x ki 0x%08x\n",
		  kp, kd, ki);
	}
      else if (strcmp(cmd, "kpb") == 0)
	{
	  kp_bal = val;
	  printf ("Set kp_bal to 0x%08x\n", kp_bal);
	  mot_set_bal_pid(kp_bal, kd_bal, ki_bal);
	  printf ("kp_bal 0x%08x kd_bal 0x%08x ki_bal 0x%08x\n",
		  kp_bal, kd_bal, ki_bal);
	}
      else if (strcmp(cmd, "kdb") == 0)
	{
	  kd_bal = val;
	  printf ("Set kd_bal to 0x%08x\n", kd_bal);
	  mot_set_bal_pid(kp_bal, kd_bal, ki_bal);
	  printf ("kp_bal 0x%08x kd_bal 0x%08x ki_bal 0x%08x\n",
		  kp_bal, kd_bal, ki_bal);
	}
      else if (strcmp(cmd, "kib") == 0)
	{
	  ki_bal = val;
	  printf ("Set ki_bal to 0x%08x\n", ki_bal);
	  mot_set_bal_pid(kp_bal, kd_bal, ki_bal);
	  printf ("kp_bal 0x%08x kd_bal 0x%08x ki_bal 0x%08x\n",
		  kp_bal, kd_bal, ki_bal);
	}
      else if (strcmp(cmd, "kph") == 0)
	{
	  kp_hd = val;
	  printf ("Set kp_hd to 0x%08x\n", kp_hd);
	  mot_set_hd_pid(kp_hd, kd_hd, ki_hd);
	  printf ("kp_hd 0x%08x kd_hd 0x%08x ki_hd 0x%08x\n",
		  kp_hd, kd_hd, ki_hd);
	}
      else if (strcmp(cmd, "kdh") == 0)
	{
	  kd_hd = val;
	  printf ("Set kd_hd to 0x%08x\n", kd_hd);
	  mot_set_hd_pid(kp_hd, kd_hd, ki_hd);
	  printf ("kp_hd 0x%08x kd_hd 0x%08x ki_hd 0x%08x\n",
		  kp_hd, kd_hd, ki_hd);
	}
      else if (strcmp(cmd, "kih") == 0)
	{
	  ki_hd = val;
	  printf ("Set ki_hd to 0x%08x\n", ki_hd);
	  mot_set_hd_pid(kp_hd, kd_hd, ki_hd);
	  printf ("kp_hd 0x%08x kd_hd 0x%08x ki_hd 0x%08x\n",
		  kp_hd, kd_hd, ki_hd);
	}
      else if (strcmp(cmd, "head") == 0)
	{
	  printf ("Turning to heading %d\n", val % 360);
	  mot_set_heading((val % 360) * 256, heading_vel*256);
	}
      else if (strcmp(cmd, "bal") == 0)
	{
	  if (!val)
	    printf ("Turning balancing off\n");
	  else
	    printf ("Turning balancing on\n");
	  mot_balance(val);
	}
      else if (strcmp(cmd, "maxt") == 0)
	{
	  if (!val)
	    printf ("max tilt is 0x%x\n", mot_max_tilt);
	  else {
	    mot_max_tilt = val;
	    mot_min_tilt = -val;
	    printf ("set max tilt to 0x%x\n", mot_max_tilt);
	  }
	}
      else if (strcmp(cmd, "acc") == 0)
	{
	  if (val == 0)
	    {
	      printf ("acc is %ld\n", robot_acc);
	    }
	  else
	    {
	      robot_acc = acc = val;
	      printf ("Set acc to %d\n", acc);
	    }
	}
      else if (strcmp(cmd, "vel") == 0)
	{
	  if (val == 0)
	    {
	      printf ("vel is %ld\n", robot_vel);
	    }
	  else
	    {
	      robot_vel = vel = val;
	      printf ("Set vel to %d\n", vel);
	    }
	}
      else if (strcmp(cmd, "hvel") == 0)
	{
	  if (val == 0)
	    {
	      printf ("hvel is %d\n", heading_vel);
	    }
	  else
	    {
	      heading_vel = val;
	      printf ("Set hvel to %d\n", heading_vel);
	    }
	}
      else if (strcmp(cmd, "go") == 0)
	{
	  if (steps == 0)
	    {
	      printf ("Calling mot_set_vel(0x%x, 0x%x, mot_get_ticks()+2)\n",
		      acc, vel);
	      mot_set_vel(acc, vel, mot_get_ticks()+2);
	    }
	  else
	    {
	      printf ("Calling mot_move(0x%x, 0x%x, 0x%x, mot_get_ticks()+2)\n",
		      steps, acc, vel);
	      mot_move(steps, acc, vel, mot_get_ticks()+2);
	    }
	}
      else if (strcmp(cmd, "steps") == 0)
	{
	  steps = val;
	  printf ("Set steps to 0x%08x\n", steps);
	}
      else if (strcmp(cmd, "demo") == 0)
	{
	  demo(val);
	}
      else if (strcmp(cmd, "s15") == 0)
	{
	  printf ("Moving servo (on tpu 15) to angle %d\n", val);
	  servo_set(15, val);
	}
      else if (strcmp(cmd, "s14") == 0)
	{
	  printf ("Moving servo (on tpu 14) to angle %d\n", val);
	  servo_set(14, val);
	}
      else if (strcmp(cmd, "lcd") == 0)
	{
	  if ((val >= LCD_MIN) &&
	      (val < LCD_MAX))
	    {
	      printf ("Setting LCD mode to %d (%s)\n", val,
		      lcd_mode_names[val]);
	      lcd_mode = val;
	    }
	  else
	    {
	      printf ("Invalid LCD mode - valid modes are:\n");
	      for (i=LCD_MIN; i<LCD_MAX; i++)
		{
		  printf ("  %d: %s\n", i, lcd_mode_names[i]);
		}
	    }
	}
      else if (strcmp(cmd, "ad") == 0)
	{
	  chan = val & 0x7;
	  status = mcp3208_read(val / 8, 1, &chan, &result);
	  printf ("mcp3208_read returned %d, result: 0x%03x\n",
		  status, result);
	}
      else if (strcmp(cmd, "grn") == 0)
	{
	  int x, z;
	  gyro_read_neutral(&x, &z);
	  printf ("gyro X neutral: %d.%02d (%d)\n", x / 65536,
		  abs(((x % 65536) * 100) / 65536), x);
	  printf ("gyro Z neutral: %d.%02d (%d)\n", z / 65536,
		  abs(((z % 65536) * 100) / 65536), z);
	}
      else if (strcmp(cmd, "gsnx") == 0)
	{
	  int x, z;
	  gyro_read_neutral(&x, &z);
	  x = val;
	  gyro_set_neutral(x, z);
	  printf ("gyro X neutral: %d.%02d (%d)\n", x / 65536,
		  abs(((x % 65536) * 100) / 65536), x);
	  printf ("gyro Z neutral: %d.%02d (%d)\n", z / 65536,
		  abs(((z % 65536) * 100) / 65536), z);
	}
      else if (strcmp(cmd, "gsnz") == 0)
	{
	  int x, z;
	  gyro_read_neutral(&x, &z);
	  z = val;
	  gyro_set_neutral(x, z);
	  printf ("gyro X neutral: %d.%02d (%d)\n", x / 65536,
		  abs(((x % 65536) * 100) / 65536), x);
	  printf ("gyro Z neutral: %d.%02d (%d)\n", z / 65536,
		  abs(((z % 65536) * 100) / 65536), z);
	}
      else if (strcmp(cmd, "gc") == 0)
	{
	  int x, z;
	  gyro_calibrate();
	  gyro_read_neutral(&x, &z);
	  printf ("gyro X neutral: %d.%02d (%d)\n", x / 65536,
		  abs(((x % 65536) * 100) / 65536), x);
	  printf ("gyro Z neutral: %d.%02d (%d)\n", z / 65536,
		  abs(((z % 65536) * 100) / 65536), z);
	}
      else if (strcmp(cmd, "h") == 0)
	{
	  ui_help();
	}
      else
	{
	  printf ("Invalid command '%s' - type h for help.\n", cmd);
	}
    }
}

rtems_task Init(
  rtems_task_argument ignored
)
{
  Objects_Id t1;
  rtems_status_code code;
  struct termios termios;

  printf ("going to 115200 baud.\n");
  if (ioctl(0, RTEMS_IO_GET_ATTRIBUTES, &termios)) {
    printf ("Init: ioctl(0, RTEMS_IO_GET_ATTRIBUTES, &termios) failed!\n");
  } else {
    termios.c_cflag = (termios.c_cflag & ~CBAUD) | B115200;
    if (ioctl(0, RTEMS_IO_SET_ATTRIBUTES, &termios)) {
      printf ("Init: ioctl(0, RTEMS_IO_SET_ATTRIBUTES, &termios) failed!\n");
    } else {
      printf ("Now at 115200 baud.\n");
    }
  }

  Period_usage_Initialize();

  /* Configure port E pins 2 & 3 to be inputs. */
  *PEPAR &= ~0x0c;
  *DDRE &= ~0x0c;

  TRACE_INIT(ROBOT);

  rtems_clock_get(RTEMS_CLOCK_GET_TICKS_PER_SECOND, &ticks_per_sec);
  printf ("Init: %d ticks per second\n", ticks_per_sec);

  printf ("Initializing fast trig functions:\n");
  init_trig();
  printf ("Done.\n\n");

  printf ("Initializing tpu:\n");
  init_tpu();
  printf ("Done.\n\n");

  printf ("Initializing servo on tpu15:\n");
  servo_init(FAN_SERVO, FAN_SERVO_STOPPED);
  printf ("Done.\n\n");

  printf ("Initializing servo on tpu14:\n");
  servo_init(FLAME_CAP_SERVO, FLAME_CAP_DOWN);
  printf ("Done.\n\n");

#if 1
  printf ("Calling mot_init():\n");
  mot_init();
  printf ("mot_init() done.\n\n");
#endif

  printf ("Calling lcd_init():\n");
  lcd_init(2,16);
  printf ("Done.\n\n");

#if 1
  printf ("Calling distance_init():\n");
  distance_init();
  printf ("Done.\n\n");
#endif

  printf ("Calling spi_init():\n");
  spi_init();
  printf ("Done.\n\n");

  printf ("Calling flame_init():\n");
  flame_init();
  printf ("Done.\n\n");

  printf ("Calling gyro_init():\n");
  gyro_init();
  printf ("Done.\n\n");

  printf ("Calling accel_init():\n");
  accel_init();
  printf ("Done.\n\n");

  printf ("Calling tone_init():\n");
  tone_init();
  printf ("Done.\n\n");

  printf ("Calling kalman_init():\n");
  kalman_init();
  printf ("Done.\n\n");

  printf ("Calibrating gyro:\n");
  gyro_calibrate();
  printf ("Done.\n\n");

  printf ("Spawning UI task:\n");
  code = rtems_task_create(rtems_build_name('U', 'I', ' ', ' '),
			   20, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, ui_task, 0);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);

  printf ("Spawning LCD task:\n");
  code = rtems_task_create(rtems_build_name('L', 'C', 'D', ' '),
			   19, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, lcd_task, 0);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);

  rtems_task_delete(RTEMS_SELF);
}

/* configuration information */

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_MAXIMUM_TASKS 15
#define CONFIGURE_MAXIMUM_PERIODS 15

#define CONFIGURE_EXTRA_TASK_STACKS         (4 * RTEMS_MINIMUM_STACK_SIZE)

#define CONFIGURE_INIT

#define CONFIGURE_MICROSECONDS_PER_TICK RTEMS_MILLISECONDS_TO_MICROSECONDS(1)

#include <confdefs.h>

/* end of file */
