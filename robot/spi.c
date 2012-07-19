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

#include <bsp.h>
#include <qsm.h>
#include <stdio.h>
#include "spi.h"
#include "global.h"

/**********************************************************************/
/* Globals */
/**********************************************************************/

rtems_id spi_sem; /* Semaphore that controls access to the SPI hardware. */

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Initialize the SPI interface.  Returns 0 on success, non-zero on
   error. */
int
spi_init(void)
{
  rtems_status_code code;

  /* Set up QSPI pins. */
  *PQSPAR = 0x7B;
  *DDRQS = (*DDRQS | 0x7E) & 0xFE;
  *PORTQS |= 0x7E;

  /* Set up some basic control registers. */
  *SPCR0 = MSTR |
    (8 << 10) |
    (SPI_CPOL ? CPOL : 0) |
    (SPI_CPHA ? CPHA : 0) |
    (((unsigned)SYS_CLOCK) / (2 * SPI_CLK));
  *SPCR1 = (SPI_DSCKL << 8) | SPI_DTL;

  printf ("spi_init: creating spi_sem...\n");
  code = rtems_semaphore_create(rtems_build_name('S', 'P', 'I', 'S'),
				1,
				RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE |
				RTEMS_INHERIT_PRIORITY |
				RTEMS_NO_PRIORITY_CEILING | RTEMS_LOCAL,
				255, &spi_sem);
  printf ("Done.  code %d, spi_sem = 0x%08x\n\n", code, spi_sem);

  return code;
}

/* Perform an SPI transfer.  Returns 0 on success, non-zero on
   error. */
int
spi_xfer(int csno, /* which chip select to assert */
	 int nbytes, /* number of bytes to transfer */
	 unsigned char *outbytes, /* data to transmit */
	 unsigned char *inbytes, /* received data will be placed here */
	 unsigned char *releasecs) /* non-zero if necessary to release
				      chip select after specified byte. */
{
  int retval = 0;
  int i;
  volatile unsigned char *qspirr = QSPIRR,
    *qspitr = QSPITR, *qspicr = QSPIcR;
  int begin_xfer, end_xfer;

  if (((csno < 0) ||
       (csno > 3)) &&
      (csno != SPI_CSNONE))
    return 1; /* Invalid chip select number. */

  if ((nbytes < 1) ||
      (nbytes > 16))
    return 2; /* Invalid # of bytes to xfer. */

  /* Only one task allowed in here at a time, so take the semaphore: */
  rtems_semaphore_obtain (spi_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);

  /* Set up control registers, but leave QSPI disabled until
     everything is set up. */
  *SPCR2 = ((nbytes - 1) << 8);
  *SPCR3 = 0;

  /* Load tx data into transmit ram & set up control words. */
  for (i=0; i<nbytes; i++)
    {
      if ((i == 0) ||
	  ((releasecs != NULL) &&
	   (releasecs[i-1] != 0)))
	begin_xfer = 1;
      else
	begin_xfer = 0;

      if ((i == nbytes-1) ||
	  ((releasecs != NULL) &&
	   (releasecs[i] != 0)))
	end_xfer = 1;
      else
	end_xfer = 0;

      qspitr[2*i+1] = outbytes[i];
      qspicr[i] = ((end_xfer) ? 0x20 : 0x80) | /* set CONT on all xfers
						  but last, set DT on
						  last xfer. */
	((begin_xfer) ? 0x10 : 0x00) | /* set DSCK on first xfer. */
	0x40 | /* set BITSE on all xfers */
	(0xf & ~((csno == SPI_CSNONE) ? 0 : (1 << csno))); /* Set
							      correct
							      chip
							      select
							      field */
    }

  /* Read & clear status register. */
  *SPSR &= 0;

  /* Enable QSPI, then wait for it to complete. */
  *SPCR1 |= SPE;

  while ((*SPSR & SPIF) == 0)
    rtems_task_wake_after(1);

  if ((*SPCR1) & SPE)
    printf ("SPE still on after SPI xfer!\n");

  /* Copy the received data into inbytes */
  for (i=0; i<nbytes; i++)
    {
      inbytes[i] = qspirr[2*i+1];
    }

  /* Let other tasks into here. */
  rtems_semaphore_release (spi_sem);
  return retval;
}

/* Lock out other tasks from doing SPI transfers - needed for devices
   that need extremely long pre- or post-transfer chip selects. */
int
spi_lock(void)
{
  rtems_semaphore_obtain (spi_sem, RTEMS_WAIT, RTEMS_NO_TIMEOUT);
  return 0;
}

/* Let other tasks do SPI transfers again. */
int
spi_unlock(void)
{
  rtems_semaphore_release (spi_sem);
  return 0;
}

