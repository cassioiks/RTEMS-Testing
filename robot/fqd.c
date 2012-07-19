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
#include "fqd.h"

/* Configure TPU channels 0&1 and 2&3 to do fast quadrature decode. */

void init_tpu_fqd(void)
{
  struct tpu_Def *Tpu;
  struct tpu_Primary_fqd_ram *primary_Tpu_ram0, *primary_Tpu_ram2;
  struct tpu_Secondary_fqd_ram *secondary_Tpu_ram1, *secondary_Tpu_ram3;

  Tpu = (struct tpu_Def *)TPU_BASE;
  primary_Tpu_ram0 = (struct tpu_Primary_fqd_ram *)(TPU_RAM + (0 << 4));
  primary_Tpu_ram2 = (struct tpu_Primary_fqd_ram *)(TPU_RAM + (2 << 4));
  secondary_Tpu_ram1 = (struct tpu_Secondary_fqd_ram *)(TPU_RAM + (1 << 4));
  secondary_Tpu_ram3 = (struct tpu_Secondary_fqd_ram *)(TPU_RAM + (3 << 4));

  while((short int)0x0000 != (short int)(Tpu->HSRR1))
    {
    }
  /* step one: disable the channel by clearing the two channel priority bits */
  Tpu->CPR1 &= (short int) 0xFF00;	/* chanels 0, 1, 2 & 3  */

  /* step two: select the FQD function on both channels by writing the FQD function number to their function select bits. */
  Tpu->CFSR3 |= (short int) ((FQD_FUNCT << 12) | (FQD_FUNCT << 8) | (FQD_FUNCT << 4) | (FQD_FUNCT));

  /* step three: initialize corr_pinsate_addr and edge_time_lsb_addr in parameter RAM of both channels.*/
  primary_Tpu_ram0->corr_pinstate_addr = (short int)((1 << 4) + 6);	/* each one points to the other pinstate */
  secondary_Tpu_ram1->corr_pinstate_addr = (short int)((0 << 4) + 6);	/* each one points to the other pinstate */
  primary_Tpu_ram0->edge_time_lsb_addr = (short int)((0 << 4) + 1);	/* both chanels point to the same one */
  secondary_Tpu_ram1->edge_time_lsb_addr = (short int)((0 << 4) + 1);	/* both chanels point to the same one */

  primary_Tpu_ram2->corr_pinstate_addr = (short int)((3 << 4) + 6);	/* each one points to the other pinstate */
  secondary_Tpu_ram3->corr_pinstate_addr = (short int)((2 << 4) + 6);	/* each one points to the other pinstate */
  primary_Tpu_ram2->edge_time_lsb_addr = (short int)((2 << 4) + 1);	/* both chanels point to the same one */
  secondary_Tpu_ram3->edge_time_lsb_addr = (short int)((2 << 4) + 1);	/* both chanels point to the same one */

  /* step four: initializes position_count to the desired start value */
  primary_Tpu_ram0->position_count = (short int)0x0800;	/* arbitrary choice for test purposes only */
  primary_Tpu_ram2->position_count = (short int)0x0800;	/* arbitrary choice for test purposes only */

  /* step five: select one channel as the primary channel and the other as the secondary channel vias hsq0 */
  Tpu->HSQR1 |= (short int)0x0044;	/* these are channel specific */

  /* step six: select normal mode of operation by ensuring that hsq1 of the primary channel is cleared */
  Tpu->HSQR1 &= (short int)0xFF44;

  /* step seven: Issue an HSR type %11 to each channel to initialize the function */
  Tpu->HSRR1 = (short int)0x00FF;		/* this line is channel specific */

  /* step eight: Enable servicing by assigning the H, M, or L priority to the channel priority bits */
  Tpu->CPR1 |= (short int)0x00AA;		/* this line is channel specific */

  while((short int)0x0000 != (short int)(Tpu->HSRR1 & (short int)0x00FF))
    {
      /* pause here and do nothing until after the tpu function is serviced. */
    }
  return;
}

unsigned short
read_tpu_fqd0(void)
{
  struct tpu_Primary_fqd_ram *primary_Tpu_ram0;

  primary_Tpu_ram0 = (struct tpu_Primary_fqd_ram *)(TPU_RAM + (0 << 4));
  return primary_Tpu_ram0->position_count;
}

unsigned short
read_tpu_fqd1(void)
{
  struct tpu_Primary_fqd_ram *primary_Tpu_ram2;

  primary_Tpu_ram2 = (struct tpu_Primary_fqd_ram *)(TPU_RAM + (2 << 4));
  return primary_Tpu_ram2->position_count;
}
