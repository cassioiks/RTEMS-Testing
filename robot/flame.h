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

#ifndef _FLAME_H
#define _FLAME_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

/* TPU channel to use. */
#define FLAME_TPU_CHAN	7

#define PTA_FUNCT	0xF

#define PTA_HSQ_HTA	0x0 /* High time accumulate */
#define PTA_HSQ_LTA	0x1 /* Low time accumumulate */
#define PTA_HSQ_PAR	0x2 /* Period accumulate - rising */
#define PTA_HSQ_PAF	0x3 /* Period accumulate - falling */

#define PTA_HSR_NONE	0x0
#define PTA_HSR_NA1	0x1
#define PTA_HSR_NA2	0x2
#define PTA_HSR_INIT	0x3

/**********************************************************************/
/* Types */
/**********************************************************************/

struct tpu_pta_ram
{
  volatile unsigned short chan_ctl;
  volatile unsigned char max_count;
  volatile unsigned char period_count;
  volatile unsigned short last_time;
  volatile unsigned short accum;
  volatile unsigned short hw;
  volatile unsigned short lw;
};

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the flame sensor subsystem.  Returns 0 on success,
   non-zero on error. */
extern int flame_init(void);

/* Return the most recent reading from the flame sensor.  If
   'wait_for_new' is non-zero, it will sleep until there's a new
   reading, and then return the new reading. */
extern int flame_read(int wait_for_new);

#endif /* _FLAME_H */
