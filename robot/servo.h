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
 * Servo Controller API
 */

#ifndef _SERVO_H
#define _SERVO_H

#include "tpu.h"

/**********************************************************************/
/* Constants */
/**********************************************************************/

#define QOM_FUNCT 0xE

#define QOM_HSQ_SINGLESHOT	0x0
#define QOM_HSQ_LOOP		0x1
#define QOM_HSQ_CONTINUOUS	0x2
#define QOM_HSQ_CONTINUOUS2	0x3

#define QOM_HSR_NONE		0x0
#define QOM_HSR_INIT_NOCHG	0x1
#define QOM_HSR_INIT_PINLOW	0x2
#define QOM_HSR_INIT_PINHIGH	0x3

/**********************************************************************/
/* Types */
/**********************************************************************/

struct tpu_qom_ram
{
  volatile unsigned char ref_addr_b;
  volatile unsigned char last_off_addr_a;
  volatile unsigned char loop_cnt;
  volatile unsigned char off_ptr_c;
  volatile unsigned short offset_1;
  volatile unsigned short offset_2;
  volatile unsigned short offset_3;
  volatile unsigned short offset_4;
};

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the servo output on a TPU pin.  'angle' is in degrees - 0
   is the center position of the CPU, -90 is ninety degrees to the left,
   90 is ninety degrees to the right. */
void servo_init(int tpu_pin, int angle);

/* Set the servo to a specific angle. */
void servo_set(int tpu_pin, int angle);

#endif /* _SERVO_H */
