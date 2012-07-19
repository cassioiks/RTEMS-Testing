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
#include "mcpwm.h"

void init_tpu_pwm(void)
{
  struct tpu_Def * Tpu;
  struct tpu_Master_mcpwm_ram *mtpu_Ram;
  struct tpu_Slave_mcpwm_ram *stpu1_Ram, *stpu2_Ram;

  Tpu = (struct tpu_Def *)TPU_BASE;
  mtpu_Ram = (struct tpu_Master_mcpwm_ram *)(TPU_RAM + (MASTER_CHAN << 4));
  stpu1_Ram = (struct tpu_Slave_mcpwm_ram *)(TPU_RAM +(SLAVE_1_CHAN << 4));
  stpu2_Ram = (struct tpu_Slave_mcpwm_ram *)(TPU_RAM +(SLAVE_2_CHAN << 4));

  /* step one set channel priority bits */
  Tpu->CPR1 &= (short int) ~0x3F00;	/* force priority for channels 6, 5,
					   and 4 to 00 = disabled, re-enabled
					   before function exit*/

  /* Step two set channel function selection registers */
  Tpu->CFSR2 &= (short int) ~0x0FFF;
  Tpu->CFSR2 |= (short int) ((MCPWM_FUNC << 8) | (MCPWM_FUNC << 4) |
			     (MCPWM_FUNC << 0));

  /* Step three Initialize master period, irq_rate, period_count,
     rise_time_ptr and fall_time_ptr */
  mtpu_Ram->period = (short int) MAX_PWM_WIDTH;
  mtpu_Ram->irq_rate = (unsigned char) 0x00;	/* no rate defined as we'll
						   poll, no interrupts */
  mtpu_Ram->period_count = (unsigned char) 0xFF;	/* when not used
							   defined to 0xFF
							   as it is
							   incremented by
							   1 during HSR */
  mtpu_Ram->rise_time_ptr = (unsigned int)((MASTER_CHAN << 4) +
					   RISE_TIME_OFFSET);
  mtpu_Ram->fall_time_ptr = (unsigned int)((MASTER_CHAN << 4) +
					   FALL_TIME_OFFSET);

  /* Step four initialize slave period, high_time_ptr, rise_time_ptr,
     and fall_time_ptr */
  stpu1_Ram->period = (short int) MAX_PWM_WIDTH;	/* this must match the
							   period value
							   assigned to the
							   master above */
  stpu1_Ram->high_time_ptr = (short int) ((SLAVE_1_CHAN << 4) +
					 HIGH_TIME_OFFSET);
  stpu1_Ram->rise_time_ptr = (short int) ((MASTER_CHAN << 4) +
					 RISE_TIME_OFFSET);
  stpu1_Ram->fall_time_ptr = (short int) ((MASTER_CHAN << 4) +
					 FALL_TIME_OFFSET);

  stpu2_Ram->period = (short int) MAX_PWM_WIDTH;	/* this must match the
							   period value
							   assigned to the
							   master above */
  stpu2_Ram->high_time_ptr = (short int) ((SLAVE_2_CHAN << 4) +
					 HIGH_TIME_OFFSET);
  stpu2_Ram->rise_time_ptr = (short int) ((MASTER_CHAN << 4) +
					 RISE_TIME_OFFSET);
  stpu2_Ram->fall_time_ptr = (short int) ((MASTER_CHAN << 4) +
					 FALL_TIME_OFFSET);

  /* Step five select edge aligned mode for slave channels */
  Tpu->HSQR1 &= (short int) ~0x3F00;	/* clear em all for now */

  /* Step six set the interrupt for the master channel if desired The
     interrupt for all slave channels should be cleared */
  Tpu->CIER &= (short int) ~0x0070;	/* disable interupts */

  /* Step seven initialize high_time parameters */
  stpu1_Ram->high_time = (short int) 0x7F;

  stpu2_Ram->high_time = (short int) 0x7F;

  /* Step eight issue a HSR %11 to the master channel and an HSR%10 to each
     slave channel */
  Tpu->HSRR1 &= (short int) ~0x3F00;
  Tpu->HSRR1 |= (short int) 0x3A00;

  /* Step nine turn on non-zero priorities to TPU channels */
  Tpu->CPR1 &= (short int) ~0x3F00;
  Tpu->CPR1 |= (short int) 0x2A00;	/* assign medium to master and
					   slaves */

  return;
}

void
set_tpu_pwm0(int val)
{
  struct tpu_Slave_mcpwm_ram *stpu1_Ram;

  stpu1_Ram = (struct tpu_Slave_mcpwm_ram *)(TPU_RAM +(SLAVE_1_CHAN << 4));

  if (val < 0)
    val = 0;
  else if (val > MAX_PWM_WIDTH)
    val = MAX_PWM_WIDTH;

  stpu1_Ram->high_time = val;
}

void
set_tpu_pwm1(int val)
{
  struct tpu_Slave_mcpwm_ram *stpu2_Ram;

  stpu2_Ram = (struct tpu_Slave_mcpwm_ram *)(TPU_RAM +(SLAVE_2_CHAN << 4));

  if (val < 0)
    val = 0;
  else if (val > MAX_PWM_WIDTH)
    val = MAX_PWM_WIDTH;

  stpu2_Ram->high_time = val;
}
