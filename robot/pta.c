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
 * API for configuring & using a TPU channel in PTA mode.
 */

#include <bsp.h>
#include "pta.h"
#include "mrm332.h"

/**********************************************************************/
/* Globals */
/**********************************************************************/

pta_callback_t *pta_callbacks[16];

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* PTA ISR handler.  Extracts the reading, prepares the channel for
   the next reading, and then calls the callback. */
rtems_isr pta_isr (rtems_vector_number vector)
{
  struct tpu_Def *Tpu;
  struct tpu_pta_ram *Tpu_pram;
  int channel;
  unsigned int reading;

  Tpu = (struct tpu_Def *)TPU_BASE;

  channel = vector & 0xf;
  Tpu_pram = (struct tpu_pta_ram *)(TPU_RAM + (channel << 4));

  /* Clear interrupt pending bit. */
  Tpu->CISR &= ~(1 << channel);

  reading = *((unsigned int *)&Tpu_pram->hw);
  Tpu_pram->hw = 0;

  if (pta_callbacks[channel] != 0)
    pta_callbacks[channel] (channel, reading);
}

/* Initialize a TPU channel to perform the PTA function.  This
   function programs the PTA to measure the high time of a pule using
   TCR1 as a timebase.  The callback 'pta_callback' will be called
   everytime a measurement is taken.  NOTE: the callback will be
   called inside an ISR! */
void
pta_init(int tpu_pin, pta_callback_t *pta_callback, int period, int count)
{
  struct tpu_Def *Tpu;
  struct tpu_pta_ram *Tpu_pram;
  int vector;
  rtems_isr_entry old_vector;

  Tpu = (struct tpu_Def *)TPU_BASE;
  Tpu_pram = (struct tpu_pta_ram *)(TPU_RAM + (tpu_pin << 4));

  /* Disable interrupts from this channel. */
  Tpu->CIER &= ~(1 << tpu_pin);

  /* Clear outstandinf interrupt (if any). */
  Tpu->CISR &= ~(1 << tpu_pin);

  /* Set up to catch the interrupt. */
  pta_callbacks[tpu_pin] = pta_callback;
  vector = (Tpu->TICR & 0x00F0) | tpu_pin;
  rtems_interrupt_catch (pta_isr, vector, &old_vector);

  while((short int)0x0000 != (short int)(Tpu->HSRR1))
    {
    }

  /* step one: disable the channel by clearing the two channel priority bits */
  if (tpu_pin < 8)
    {
      Tpu->CPR1 &= (short int) ~(0x3 << (tpu_pin*2));
    }
  else
    {
      Tpu->CPR0 &= (short int) ~(0x3 << ((tpu_pin - 8) * 2));
    }

  /* Enale interrupts from this channel. */
  Tpu->CIER |= (1 << tpu_pin);

  /* step two: select the PTA function on the channels by writing the
     PTA function number to the function select bits. */
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

  /* step three: initialize parameter ram. */
  Tpu_pram->max_count_period_count = ((unsigned)count & 0xff) << 8;
  Tpu_pram->last_time = 0;
  Tpu_pram->accum = 0;
  Tpu_pram->hw = 0;
  Tpu_pram->lw = 0;
  if (period)
    Tpu_pram->channel_control = PTA_CCTRL_PERIOD_RISING_TCR1;
  else
    Tpu_pram->channel_control = PTA_CCTRL_PULSE_TCR1;

  /* step four: configure HSQ bits. */
  if (tpu_pin < 8)
    {
      Tpu->HSQR1 &= (short int) ~(0x3 << (tpu_pin*2));
      if (period)
	Tpu->HSQR1 |= (short int) (PTA_HSQ_PERIOD_RISING << (tpu_pin*2));
      else
	Tpu->HSQR1 |= (short int) (PTA_HSQ_HIGH_WIDTH << (tpu_pin*2));
    }
  else
    {
      Tpu->HSQR0 &= (short int) ~(0x3 << ((tpu_pin - 8) * 2));
      if (period)
	Tpu->HSQR0 |= (short int) (PTA_HSQ_PERIOD_RISING << ((tpu_pin - 8) * 2));
      else
	Tpu->HSQR0 |= (short int) (PTA_HSQ_HIGH_WIDTH << ((tpu_pin - 8) * 2));
    }

  /* step five: Issue an HSR to the channel to initialize. */
  if (tpu_pin < 8)
    {
      Tpu->HSRR1 &= (short int) ~(0x3 << (tpu_pin*2));
      Tpu->HSRR1 |= (short int) (PTA_HSR_INIT << (tpu_pin*2));
    }
  else
    {
      Tpu->HSRR0 &= (short int) ~(0x3 << ((tpu_pin - 8) * 2));
      Tpu->HSRR0 |= (short int) (PTA_HSR_INIT << ((tpu_pin - 8) * 2));
    }

  /* step six: Enable servicing by assigning the H, M, or L priority
     to the channel priority bits */
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
	    (short int)(Tpu->HSRR1 & (short int)(0x3 << (tpu_pin*2))))
	{
	  /* pause here and do nothing until after the tpu function is
             serviced. */
	}
    }
  else
    {
      while((short int)0x0000 !=
	    (short int)(Tpu->HSRR0 & (short int)(0x3 << ((tpu_pin - 8) * 2))))
	{
	  /* pause here and do nothing until after the tpu function is
             serviced. */
	}
    }

  return;
}
