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

#ifndef _TPU_H
#define _TPU_H

#define TPU_BASE 0xFFFFFE00
#define TPU_RAM 0xFFFFFF00

struct tpu_Def
{
	volatile unsigned short int TPUMCR;				/* The following defines are derived directly from the TPURM provide above */
	volatile unsigned short int TCR;
	volatile unsigned short int DSCR;
	volatile unsigned short int DSSR;
	volatile unsigned short int TICR;
	volatile unsigned short int CIER;
	volatile unsigned short int CFSR0;
	volatile unsigned short int CFSR1;
	volatile unsigned short int CFSR2;
	volatile unsigned short int CFSR3;
	volatile unsigned short int HSQR0;
	volatile unsigned short int HSQR1;
	volatile unsigned short int HSRR0;
	volatile unsigned short int HSRR1;
	volatile unsigned short int CPR0;
	volatile unsigned short int CPR1;
	volatile unsigned short int CISR;
	volatile unsigned short int LR;
	volatile unsigned short int SGLR;
	volatile unsigned short int DCNR;
};

/* Init the TPU to a known state. */
extern void init_tpu(void);

#endif /* _TPU_H */
