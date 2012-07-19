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

#ifndef _FQD_H
#define _FQD_H

void init_tpu_fqd(void);		/* maintain function prototypes */
unsigned short read_tpu_fqd0(void);
unsigned short read_tpu_fqd1(void);

#define FQD_FUNCT 6

struct tpu_Primary_fqd_ram		/* this structure for TPU RAM overlay developed directly from Motorola docs */
{
	volatile unsigned short int edge_time;
	volatile unsigned short int position_count;
	volatile unsigned short int tcr1_value;
	volatile unsigned short int chan_pinstate;
	volatile unsigned short int corr_pinstate_addr;
	volatile unsigned short int edge_time_lsb_addr;
};

struct tpu_Secondary_fqd_ram		/* this structure for TPU RAM overlay developed directly from Motorola docs */
{
	volatile unsigned short int unused_1;
	volatile unsigned short int unused_2;
	volatile unsigned short int tcr1_value;
	volatile unsigned short int chan_pinstate;
	volatile unsigned short int corr_pinstate_addr;
	volatile unsigned short int edge_time_lsb_addr;
};

#endif /* _FQD_H */
