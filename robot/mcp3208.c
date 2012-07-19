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
 * MCP3208 Analog-to-Digital converter API
 *
 * Simple API to read 1-4 channel(s) of an MCP3208 A-to-D converter
 * connected to one of the SPI ports of the MRM.
 *
 * Uses the SPI API, so that must be initialized prior to calling
 * anything in this API.
 */

#include <bsp.h>
#include <stdio.h>
#include "spi.h"
#include "mcp3208.h"
#include "global.h"

/* Un-comment this to turn on debug printf's */
/* #define MCP3208_DEBUG_PRINT */

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Read 1-4 channels of an MCP3208.  Returns 0 on success, non-zero on
   error. */
int
mcp3208_read(int spi_port, int nchans, int *chans, unsigned short *results)
{
  unsigned char outbytes[12];
  unsigned char inbytes[12];
  unsigned char releasecs[12];
  int i, status = 0;

  /* Sanity check arguments. */
  if ((spi_port < 0) || (spi_port > 3) || (nchans < 1) || (nchans > 4))
    return -1;

  /* Set up the output (control) bytes. */
  for (i=0; i<nchans; i++)
    {
      /* See the section 6.1 of the data sheet for the MCP3208 (Using
         the MCP3204/3208 with Microcontroller SPI Ports) for an
         explanation of these values. */
      outbytes[i*3+0] = 0x6 | ((chans[i] >> 2) & 0x01);
      outbytes[i*3+1] = 0xC0 & (chans[i] << 6);
      outbytes[i*3+2] = 0;
#ifdef MCP3208_DEBUG_PRINT
      printf ("mcp3208: Writing %02x %02x %02x to access channel %d\n",
	      outbytes[i*3+0], outbytes[i*3+1], outbytes[i*3+2], chans[i]);
#endif

      releasecs[i*3+0] = 0;
      releasecs[i*3+1] = 0;
      releasecs[i*3+2] = 1;
    }

  /* Do the spi transfer. */
  status = spi_xfer(spi_port, 3 * nchans, outbytes, inbytes, releasecs);
#ifdef MCP3208_DEBUG_PRINT
  printf ("mcp3208: spi_xfer returned %d\n", status);
#endif

  for (i=0; (i<nchans) && (status == 0); i++)
    {
      results[i] = (inbytes[i*3+1] & 0x0F) << 8 | inbytes[i*3+2];
#ifdef MCP3208_DEBUG_PRINT
      printf ("mcp3208: channel %d, got %02x %02x %02x, for a result %d\n",
	      chans[i], inbytes[i*3+0], inbytes[i*3+1], inbytes[i*3+2],
	      results[i]);
#endif
    }

  return status;
}
