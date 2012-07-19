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

/* Include file for fast trig routines. */

/* These arrays are indexed by the angle in degrees, and return the
   sin & cos value in a fixed 18.14 format. */
extern int isinof[360],icosof[360];

/* Must be called at boot time to fill in trig tables. */
extern void init_trig(void);

/* Returns angle in degrees. */
extern int fastatan2(long y, long x);

/* Expects sin value in 18.14 format, returns angle in degrees. */
extern int fastasin(int x);

/* Expects cos value in 18.14 format, returns angle in degrees. */
extern int fastacos(int x);

/* Returns the sqrt of x. */
extern unsigned sqrti(unsigned long x);
