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

#ifndef _MCPWM_H
#define _MCPWM_H

void init_tpu_pwm(void);
void set_tpu_pwm0(int val);
void set_tpu_pwm1(int val);

#define MASTER_CHAN 6
#define SLAVE_1_CHAN 5
#define SLAVE_2_CHAN 4
#define MCPWM_FUNC 7
#define RISE_TIME_OFFSET 4
#define FALL_TIME_OFFSET 6
#define HIGH_TIME_OFFSET 2

#define MIN_PWM_WIDTH 0x0020
#define MAX_PWM_WIDTH 0x00FF

struct tpu_Master_mcpwm_ram
{
	volatile unsigned short int	period;
	volatile unsigned char irq_rate;
	volatile unsigned char period_count;
	volatile unsigned short int last_rise_time;
	volatile unsigned short int last_fall_time;
	volatile unsigned short int rise_time_ptr;
	volatile unsigned short int fall_time_ptr;	
};

struct tpu_Slave_mcpwm_ram
{
	volatile unsigned short int period;
	volatile unsigned short int high_time;
	volatile unsigned short int undefined;
	volatile unsigned short int high_time_ptr;
	volatile unsigned short int rise_time_ptr;
	volatile unsigned short int fall_time_ptr;
};

#endif /* _MCPWM_H */
