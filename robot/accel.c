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
 * Accelerometer API
 */

#include "accel.h"
#include "pta.h"

#define ACCEL_AVG 10

unsigned int last_accel_reading;
unsigned int accel_sum = 0;
unsigned int accel_cnt = 0;
unsigned int accel_pta_cb_bad_tpu_pin = 0;
int accel_0g = ACCEL_0G;

void
accel_pta_cb (int tpu_pin, unsigned int reading)
{
  if (tpu_pin == ACCEL_TPU_PIN) {
    accel_sum += reading;
    accel_cnt++;
    if (accel_cnt == ACCEL_AVG) {
      last_accel_reading = (accel_sum + ACCEL_AVG/2) / ACCEL_AVG;
      accel_sum = 0;
      accel_cnt = 0;
    }
  } else
    accel_pta_cb_bad_tpu_pin++;
}

/* Initialize the accelerometer. */
void
accel_init (void)
{
  pta_init(ACCEL_TPU_PIN, accel_pta_cb, 0, 1);
}

/* Read the most recent raw reading from the accelerometer. */
unsigned int
accel_read_raw (void)
{
  return last_accel_reading;
}

/* Read the most recent accelerometer reading.  Output is G's as a
   16.16 fixed point number.  Note that it may be negative - a
   positive reading indicates a forward tilt of the platform. */
int
accel_read (void)
{
  return (((int)last_accel_reading - accel_0g) * 65536) / ACCEL_1G;
}
