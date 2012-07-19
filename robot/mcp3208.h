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

#ifndef _MCP3208_H
#define _MCP3208_H

/**********************************************************************/
/* Functions */
/**********************************************************************/

/* Read 1-4 channels of an MCP3208.  Returns 0 on success, non-zero on
   error. */
extern int mcp3208_read(int spi_port, int nchans, int *chans,
			unsigned short *results);

#endif /* _MCP3208_H */
