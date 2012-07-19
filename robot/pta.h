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

#ifndef _PTA_H
#define _PTA_H

#include "tpu.h"

/**********************************************************************/
/* Constants */
/**********************************************************************/

#define PTA_FUNCT 0xF

#define PTA_HSQ_HIGH_WIDTH	0x0
#define PTA_HSQ_LOW_WIDTH	0x1
#define PTA_HSQ_PERIOD_RISING	0x2
#define PTA_HSQ_PERIOD_FALLING	0x3

#define PTA_HSR_NONE		0x0
#define PTA_HSR_INIT		0x3

#define PTA_CCTRL_PULSE_TCR1		0x000F
#define PTA_CCTRL_PERIOD_RISING_TCR1	0x0007
#define PTA_CCTRL_PERIOD_FALLING_TCR1	0x000B
#define PTA_CCTRL_PULSE_TCR2		0x004F
#define PTA_CCTRL_PERIOD_RISING_TCR2	0x0047
#define PTA_CCTRL_PERIOD_FALLING_TCR2	0x004B
#define PTA_CCTRL_NO_CHANGE		0x01FF

/**********************************************************************/
/* Types */
/**********************************************************************/

struct tpu_pta_ram
{
  volatile unsigned short channel_control;
  volatile unsigned short max_count_period_count;
  volatile unsigned short last_time;
  volatile unsigned short accum;
  volatile unsigned short hw;
  volatile unsigned short lw;
};

/* Type of callback that will be called when a reading is made.  NOTE:
   callback will be called with an ISR! */
typedef void pta_callback_t (int tpu_pin, unsigned int reading);

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize a TPU channel to perform the PTA function.  This
   function programs the PTA to measure the high time of a pule using
   TCR1 as a timebase.  The callback 'pta_callback' will be called
   everytime a measurement is taken.  NOTE: the callback will be
   called inside an ISR!

   If 'period' is non-zero, then this will count the whole period of
   the signal, not just the high time.  'count' specifies how many
   cycles to count before interrupting with a reading. */
void pta_init(int tpu_pin, pta_callback_t *pta_callback, int period,
	      int count);

#endif /* _PTA_H */
