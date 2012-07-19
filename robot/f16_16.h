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
 * Routines for doing math with 16.16 fixed point numbers.
 */

#ifndef _F16_16_H
#define _F16_16_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

#define		F16_16_PI	  205887	/* 3.14159 as a 16.16 fixed */

/**********************************************************************/
/* Types */
/**********************************************************************/

typedef long f16_16;

/**********************************************************************/
/* Macros */
/**********************************************************************/

#define add_f16_16(a,b) ((a) + (b))
#define sub_f16_16(a,b) ((a) - (b))

#define round_f16_16(a) (((a) + 32768) >> 16)

/**********************************************************************/
/* Functions */
/**********************************************************************/

f16_16 mult_f16_16(f16_16 a, f16_16 b);

/* Loses some precision in the fractional part of the divide, but should
   be good enough. */
f16_16 div_f16_16(f16_16 a, f16_16 b);

void print_f16_16(f16_16 a);

double double_from_f16_16(f16_16 a);
f16_16 f16_16_from_double(double a);

#endif /* _F16_16_H */
