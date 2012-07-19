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
 * Tone Detector API
 */

#include <bsp.h>
#include <stdio.h>
#include "tone.h"
#include "pta.h"
#include "global.h"

unsigned int tone_good = 0;
unsigned int tone_last_raw = 0;
unsigned int tone_good_cnt = 0;
unsigned int tone_bad_cnt = 0;
unsigned int tone_pta_cb_bad_tpu_pin = 0;
unsigned int tone_task_timeouts = 0;

void
tone_pta_cb (int tpu_pin, unsigned int reading)
{
  if (tpu_pin == TONE_TPU_PIN) {
    tone_last_raw = reading;
    if ((reading >= TONE_MIN) && (reading <= TONE_MAX)) {
      tone_bad_cnt = 0;
      tone_good_cnt++;
      if (tone_good_cnt > 5)
	tone_good = 1;
    } else {
      tone_bad_cnt++;
      tone_good_cnt = 0;
      if (tone_bad_cnt > 3)
	tone_good = 0;
    }
  } else
    tone_pta_cb_bad_tpu_pin++;
}

rtems_task
tone_task(rtems_task_argument ignored)
{
  rtems_name period_name;
  rtems_id period;
  rtems_status_code status;
  int last_good_cnt, last_bad_cnt;

  period_name = rtems_build_name ('T', 'N', 'P', 'D');
  status = rtems_rate_monotonic_create (period_name, &period);
  if (status != RTEMS_SUCCESSFUL)
    {
      printf ("tone_task: rate_monotonic_create failed with status %d\n",
	      status);
      return;
    }

  last_good_cnt = tone_good_cnt;
  last_bad_cnt = tone_bad_cnt;

  while (1)
    {
      if (rtems_rate_monotonic_period (period, ticks_per_sec/50) ==
	  RTEMS_TIMEOUT)
	{
	  /* I'd like to do a printf here, but that would make us miss
	     our next timeout, until the end of time... */
	  tone_task_timeouts++;
	}

      if ((last_good_cnt == tone_good_cnt) &&
	  (last_bad_cnt == tone_bad_cnt)) {
	/* Haven't heard anything for 20ms - no tone. */
	tone_good = 0;
	tone_good_cnt = 0;
	tone_bad_cnt = 0;
      }

      last_good_cnt = tone_good_cnt;
      last_bad_cnt = tone_bad_cnt;
    }
}

/* Initialize the tone detector. */
void
tone_init (void)
{
  Objects_Id t1;
  rtems_status_code code;

  *PEPAR &= ~PORTE_BUTTON0;
  *DDRE &= ~PORTE_BUTTON0;

  pta_init(TONE_TPU_PIN, tone_pta_cb, 1, 10);

  printf ("Spawning tone task:\n");
  code = rtems_task_create(rtems_build_name('T', 'O', 'N', 'E'),
			   20, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, tone_task, 0);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);
}

/* Read the most recent raw reading from the tone detector. */
unsigned int
tone_read_raw (void)
{
  return tone_last_raw;
}

/* Read whether the tone is active or not. */
int
tone_read (void)
{
  return ((tone_good != 0) ||
	  ((*PORTE0 & PORTE_BUTTON0) != 0));
}
