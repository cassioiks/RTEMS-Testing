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

#include "tpu.h"

/* Init the TPU to a known state. */
void init_tpu(void)
{
	struct tpu_Def * Tpu;

	Tpu = (struct tpu_Def *)TPU_BASE;

	Tpu->CIER = 0x0000; /* disable interrupts. */
	Tpu->TICR = 0x0550; /* int. lvl 5, base vector 0x50. */
	Tpu->TPUMCR = 0x00c2; /* supv mode, tcr1 = sysclk/4, iarb = 2 */
	Tpu->CPR0 = (short int) 0; /* disable all channels. */
	Tpu->CPR1 = (short int) 0;
	Tpu->CFSR0 = (short int) 0x0000; /* disable channels. */
	Tpu->CFSR1 = (short int) 0x0000;
	Tpu->CFSR2 = (short int) 0x0000;
	Tpu->CFSR3 = (short int) 0x0000;
 	Tpu->HSQR0 = (short int) 0x0000; /* clear em all for now */
	Tpu->HSQR1 = (short int) 0x0000; /* clear em all for now */
}
