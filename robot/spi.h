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
 * SPI API
 *
 * API to abstract usage of the SPI subsystem in the 68332
 * Motorola microprocessor (and similar processors as well).
 * The init routine is not thread-safe, and should only be
 * called once at boot time.  All other routines are thread-
 * safe, and can be called from any thread at any time - the
 * API locks internally to prevent one access from overwriting
 * another one.
 */

#ifndef _SPI_H
#define _SPI_H

/**********************************************************************/
/* Constants */
/**********************************************************************/

/* To simplify the interface (and because all the devices I used when
   I wrote this worked in the same mode), some SPI settings are
   hardcoded constants: */

#define SPI_CLK		25000L

#define SPI_CPOL	1
#define SPI_CPHA	1

#define SPI_DSCKL	0 /* meaning 128 clocks */
#define SPI_DTL		0 /* meaning 256 clocks */

/* Pass in this for chip select to cause the SPI module to not assert
   a chip select during the transaction. */
#define SPI_CSNONE	4

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the SPI interface.  Returns 0 on success, non-zero on
   error. */
int spi_init(void);

/* Perform an SPI transfer.  Returns 0 on success, non-zero on
   error. */
int spi_xfer(int csno, /* which chip select to assert */
	     int nbytes, /* number of bytes to transfer */
	     unsigned char *outbytes, /* data to transmit */
	     unsigned char *inbytes, /* received data will be placed here */
	     unsigned char *releasecs); /* non-zero if necessary to release
					   chip select after specified byte. */

/* Lock out other tasks from doing SPI transfers - needed for devices
   that need extremely long pre- or post-transfer chip selects. */
int spi_lock(void);

/* Let other tasks do SPI transfers again. */
int spi_unlock(void);

#endif /* _SPI_H */
