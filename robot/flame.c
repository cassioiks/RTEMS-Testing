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
 * Flame sensor API
 *
 * Uses the TPU's 'PTA' function to read the output from a Hamamtsu
 * UVTRON flame sensor.  Returns the number of counts in the last
 * second.
 */

#include <bsp.h>
#include <stdio.h>
#include "flame.h"
#include "tpu.h"
#include "global.h"

/**********************************************************************/
/* Globals */
/**********************************************************************/

/* Contains the last value read from the flame sensor. */
int flame_last = 0;

/* Contains the last 10 partial values read from the flame sensor. */
int flame_last_partial[10];

/* Set to one whenever a new reading is placed in flame_last */
int flame_new = 0;

/* Also, this semaphore is flushed whenever a new reading is placed in
   flame_last */
rtems_id flame_sem;

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Flame sensor polling task. */
rtems_task
flame_task (rtems_task_argument ignored)
{
  struct tpu_Def *Tpu;
  struct tpu_pta_ram *Tpu_pram;
  int count = 0, i, sum;
  int tpu_pin = FLAME_TPU_CHAN;

  Tpu = (struct tpu_Def *)TPU_BASE;
  Tpu_pram = (struct tpu_pta_ram *)(TPU_RAM + (tpu_pin << 4));

  printf ("flame_task: ticks_per_second = %d\n", ticks_per_sec);

  while (1)
    {
      /* First, disable the channel. */
      if (tpu_pin < 8)
	Tpu->CPR1 &= ~(0x3 << (tpu_pin * 2));
      else
	Tpu->CPR0 &= ~(0x3 << ((tpu_pin - 8) * 2));

      /* Read out the previous count (except on the first time). */
      if (count != 0)
	{
	  flame_last_partial[count] = Tpu_pram->period_count;

	  if (count == 9)
	    {
	      sum = 0;
	      for (i=0; i<10; i++)
		{
		  sum += flame_last_partial[i];
		  if (i != 9)
		    flame_last_partial[i] = flame_last_partial[i+1];
		}
	      count--;

	      flame_last = sum;
	      flame_new = 1;
	      rtems_semaphore_flush(flame_sem);
	    }
	}
      count++;

      /* Set the proper function number. */
      if (tpu_pin < 4)
	{
	  Tpu->CFSR3 &= (short int) ~(0xF << (tpu_pin * 4));
	  Tpu->CFSR3 |= (short int) (PTA_FUNCT << (tpu_pin * 4));
	}
      else if (tpu_pin < 8)
	{
	  Tpu->CFSR2 &= (short int) ~(0xF << ((tpu_pin - 4) * 4));
	  Tpu->CFSR2 |= (short int) (PTA_FUNCT << ((tpu_pin - 4) * 4));
	}
      else if (tpu_pin < 12)
	{
	  Tpu->CFSR1 &= (short int) ~(0xF << ((tpu_pin - 8) * 4));
	  Tpu->CFSR1 |= (short int) (PTA_FUNCT << ((tpu_pin - 8) * 4));
	}
      else
	{
	  Tpu->CFSR0 &= (short int) ~(0xF << ((tpu_pin - 12) * 4));
	  Tpu->CFSR0 |= (short int) (PTA_FUNCT << ((tpu_pin - 12) * 4));
	}

      /* Initialize parameter ram. */
      Tpu_pram->chan_ctl = 0x000F;
      Tpu_pram->max_count = 255;
      Tpu_pram->period_count = 0;
      Tpu_pram->last_time = 0;
      Tpu_pram->accum = 0;
      Tpu_pram->hw = 0;
      Tpu_pram->lw = 0;

      /* Set host sequence bits (set mode to 'High time accumulate'). */
      if (tpu_pin < 8)
	{
	  Tpu->HSQR1 &= (short int) ~(0x3 << (tpu_pin*2));
	  Tpu->HSQR1 |= (short int) (PTA_HSQ_HTA << (tpu_pin*2));
	}
      else
	{
	  Tpu->HSQR0 &= (short int) ~(0x3 << ((tpu_pin - 8) * 2));
	  Tpu->HSQR0 |=
	    (short int) (PTA_HSQ_HTA << ((tpu_pin - 8) * 2));
	}

      /* Set host service request bits to init channel. */
      if (tpu_pin < 8)
	{
	  Tpu->HSRR1 &= (short int) ~(0x3 << (tpu_pin*2));
	  Tpu->HSRR1 |= (short int) (PTA_HSR_INIT << (tpu_pin*2));
	}
      else
	{
	  Tpu->HSRR0 &= (short int) ~(0x3 << ((tpu_pin - 8) * 2));
	  Tpu->HSRR0 |=
	    (short int) (PTA_HSR_INIT << ((tpu_pin - 8) * 2));
	}

      /* Enable the channel by setting the priority bits. */
      if (tpu_pin < 8)
	{
	  Tpu->CPR1 |= (short int) (0x3 << (tpu_pin*2));
	}
      else
	{
	  Tpu->CPR0 |= (short int) (0x3 << ((tpu_pin - 8) * 2));
	}


      if (tpu_pin < 8)
	{
	  while((short int)0x0000 !=
		(short int)(Tpu->HSRR1 &
			    (short int)(0x3 << (tpu_pin*2))))
	    {
	      /* pause here and do nothing until after the tpu function is
		 serviced. */
	    }
	}
      else
	{
	  while((short int)0x0000 !=
		(short int)(Tpu->HSRR0
			    & (short int)(0x3 << ((tpu_pin - 8) * 2))))
	    {
	      /* pause here and do nothing until after the tpu function is
		 serviced. */
	    }
	}

      rtems_task_wake_after(ticks_per_sec / 10);
    }
}

/* Initialize the flame sensor subsystem.  Returns 0 on success,
   non-zero on error. */
int
flame_init(void)
{
  Objects_Id t1;
  rtems_status_code code;

  /* Create the flame semaphore. */
  printf ("flame_init: creating flame_sem...\n");
  code = rtems_semaphore_create(rtems_build_name('U', 'V', 'T', 'S'),
				0,
				RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE |
				RTEMS_INHERIT_PRIORITY |
				RTEMS_NO_PRIORITY_CEILING | RTEMS_LOCAL,
				255, &flame_sem);
  printf ("Done.  code %d, flame_sem = 0x%08x\n\n", code, flame_sem);

  printf ("Spawning flame task:\n");
  code = rtems_task_create(rtems_build_name('U', 'V', 'T', 'C'),
			   19, RTEMS_MINIMUM_STACK_SIZE * 2,
			   RTEMS_DEFAULT_MODES,
			   RTEMS_DEFAULT_ATTRIBUTES,
			   &t1);
  printf ("  rtems_task_create returned %d; t1 = 0x%08x\n", code, t1);
  code = rtems_task_start(t1, flame_task, 0);
  printf ("Done. (rtems_task_start returned %d)\n\n", code);

  return code;
}

/* Return the most recent reading from the flame sensor.  If
   'wait_for_new' is non-zero, it will sleep until there's a new
   reading, and then return the new reading. */
int
flame_read(int wait_for_new)
{
  if (wait_for_new)
    {
      flame_new = 0;
      while (flame_new == 0)
	rtems_semaphore_obtain(flame_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
    }
  return flame_last;
}
