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
 * Tone detector API
 */

#ifndef _TONE_H
#define _TONE_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

#define TONE_TPU_PIN	11

#define TONE_MIN 11000
#define TONE_MAX 11500

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the tone detector. */
void tone_init (void);

/* Read the most recent raw reading from the tone detector. */
unsigned int tone_read_raw (void);

/* Read whether the tone is active or not. */
int tone_read (void);

#endif
