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
 *  Autopilot is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Autopilot; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Servo Controller API
 */

#include "servo.h"
#include "mrm332.h"

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the servo output on a TPU pin.  'angle' is in degrees - 0
   is the center position of the CPU, -45 is ninety degrees to the left,
   45 is ninety degrees to the right. */
void
servo_init(int tpu_pin, int angle)
{
  struct tpu_Def *Tpu;
  struct tpu_qom_ram *Tpu_pram;
  int tcr1_ms; /* number of counts of TCR1 clock to equal 1ms. */

  angle = -angle;
  tcr1_ms = SYS_CLOCK / 4000; /* Assumes TCR1 runs in divide-by-4 mode. */

  Tpu = (struct tpu_Def *)TPU_BASE;
  Tpu_pram = (struct tpu_qom_ram *)(TPU_RAM + (tpu_pin << 4));

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

  /* step two: select the QOM function on the channels by writing the
     QOM function number to the function select bits. */
  if (tpu_pin < 4)
    {
      Tpu->CFSR3 &= (short int) ~(0xF << (tpu_pin * 4));
      Tpu->CFSR3 |= (short int) (QOM_FUNCT << (tpu_pin * 4));
    }
  else if (tpu_pin < 8)
    {
      Tpu->CFSR2 &= (short int) ~(0xF << ((tpu_pin - 4) * 4));
      Tpu->CFSR2 |= (short int) (QOM_FUNCT << ((tpu_pin - 4) * 4));
    }
  else if (tpu_pin < 12)
    {
      Tpu->CFSR1 &= (short int) ~(0xF << ((tpu_pin - 8) * 4));
      Tpu->CFSR1 |= (short int) (QOM_FUNCT << ((tpu_pin - 8) * 4));
    }
  else
    {
      Tpu->CFSR0 &= (short int) ~(0xF << ((tpu_pin - 12) * 4));
      Tpu->CFSR0 |= (short int) (QOM_FUNCT << ((tpu_pin - 12) * 4));
    }

  /* step three: initialize parameter ram. */
  Tpu_pram->ref_addr_b = 0;
  Tpu_pram->last_off_addr_a = (tpu_pin << 4) | 0xa; /* 0xa is ptr to last
						       offset */
  Tpu_pram->off_ptr_c = 0;
  Tpu_pram->offset_1 = (tcr1_ms * 5) << 1 | 0;
  Tpu_pram->offset_2 = (tcr1_ms * 5) << 1 | 0;
  Tpu_pram->offset_3 = (tcr1_ms * 5) << 1 | 1;
  Tpu_pram->offset_4 = ((tcr1_ms * (angle + 45) / 90) + tcr1_ms) << 1 | 0;

  /* step four: set to continuous mode via HSQ bits. */
  if (tpu_pin < 8)
    {
      Tpu->HSQR1 &= (short int) ~(0x3 << (tpu_pin*2));
      Tpu->HSQR1 |= (short int) (QOM_HSQ_CONTINUOUS << (tpu_pin*2));
    }
  else
    {
      Tpu->HSQR0 &= (short int) ~(0x3 << ((tpu_pin - 8) * 2));
      Tpu->HSQR0 |= (short int) (QOM_HSQ_CONTINUOUS << ((tpu_pin - 8) * 2));
    }

  /* step five: Issue an HSR to the channel to initialize the pin low. */
  if (tpu_pin < 8)
    {
      Tpu->HSRR1 &= (short int) ~(0x3 << (tpu_pin*2));
      Tpu->HSRR1 |= (short int) (QOM_HSR_INIT_PINLOW << (tpu_pin*2));
    }
  else
    {
      Tpu->HSRR0 &= (short int) ~(0x3 << ((tpu_pin - 8) * 2));
      Tpu->HSRR0 |= (short int) (QOM_HSR_INIT_PINLOW << ((tpu_pin - 8) * 2));
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

/* Set the servo to a specific angle. */
void
servo_set(int tpu_pin, int angle)
{
  struct tpu_qom_ram *Tpu_pram;
  int tcr1_ms; /* number of counts of TCR1 clock to equal 1ms. */

  tcr1_ms = SYS_CLOCK / 4000; /* Assumes TCR1 runs in divide-by-4 mode. */
  Tpu_pram = (struct tpu_qom_ram *)(TPU_RAM + (tpu_pin << 4));

  angle = -angle;
  Tpu_pram->offset_4 = ((tcr1_ms * (angle + 45) / 90) + tcr1_ms) << 1 | 0;
}
