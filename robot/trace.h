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

/* Utilities for setting up trace ringbuffers. */

#ifndef _TRACE_H
#define _TRACE_H

#include <bsp.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "motor.h" /* for mot_get_ticks() */

/* Size of trace print buffer, NOT number of entries in ring buffer! */
#define TRACE_BUFSIZ	200

typedef struct trace_ctl {
  int cur;
  int wrapped;
  char buf[TRACE_BUFSIZ];
} trace_ctl_t;

typedef struct trace_entry {
  unsigned time;
  unsigned index;
  int _0;
  int _1;
  int _2;
  int _3;
  int _4;
  int _5;
  int _6;
  int _7;
} trace_entry_t;

#ifdef TRACE_ALLOC

#define TRACE_DEFINE(_c,_s)		\
	trace_entry_t _c##_entries[_s];	\
	int _c##_num_entries = _s;	\
	trace_ctl_t _c##_ctl;

#define TRACE_ENTRIES_BEGIN(_c)	\
	char *_c##_formats[] = {

#define TRACE_ENTRY(_c, _n, _fmt)	\
	"%8u ( %8u ) " _fmt,

#define TRACE_ENTRIES_END(_c) \
	};

#else

#define TRACE_DEFINE(_c,_s)			\
	extern trace_entry_t _c##_entries[];	\
	extern int _c##_num_entries;		\
	extern trace_ctl_t _c##_ctl;		\
	extern char *_c##_formats[]

#define TRACE_ENTRIES_BEGIN(_c)		\
	enum _c##_indices {

#define TRACE_ENTRY(_c, _n, _fmt)	\
	_c##_##_n,

#define TRACE_ENTRIES_END(_c)		\
	_c##_NUM_TRACES			\
	};

#endif

#define TRACE_INIT(_c)	\
	do { _c##_ctl.cur = 0; _c##_ctl.wrapped = 0; } while (0)

#define TRACE_LOG8(_c, _n, a0, a1, a2, a3, a4, a5, a6, a7) \
	do { \
		register int idx = _c##_ctl.cur++; \
		register trace_entry_t *entp = &_c##_entries[idx]; \
		if (_c##_ctl.cur >= _c##_num_entries) { \
			_c##_ctl.cur = 0; _c##_ctl.wrapped = 1; \
		} \
		entp->time = mot_get_ticks(); entp->index = _c##_##_n; \
		entp->_0 = a0; entp->_1 = a1; entp->_2 = a2; entp->_3 = a3; \
		entp->_4 = a4; entp->_5 = a5; entp->_6 = a6; entp->_7 = a7; \
	} while(0)

#define TRACE_LOG7(_c, _n, _0, _1, _2, _3, _4, _5, _6) \
	TRACE_LOG8(_c, _n, _0, _1, _2, _3, _4, _5, _6,  0)
#define TRACE_LOG6(_c, _n, _0, _1, _2, _3, _4, _5) \
	TRACE_LOG8(_c, _n, _0, _1, _2, _3, _4, _5,  0,  0)
#define TRACE_LOG5(_c, _n, _0, _1, _2, _3, _4) \
	TRACE_LOG8(_c, _n, _0, _1, _2, _3, _4,  0,  0,  0)
#define TRACE_LOG4(_c, _n, _0, _1, _2, _3) \
	TRACE_LOG8(_c, _n, _0, _1, _2, _3,  0,  0,  0,  0)
#define TRACE_LOG3(_c, _n, _0, _1, _2) \
	TRACE_LOG8(_c, _n, _0, _1, _2,  0,  0,  0,  0,  0)
#define TRACE_LOG2(_c, _n, _0, _1) \
	TRACE_LOG8(_c, _n, _0, _1,  0,  0,  0,  0,  0,  0)
#define TRACE_LOG1(_c, _n, _0) \
	TRACE_LOG8(_c, _n, _0,  0,  0,  0,  0,  0,  0,  0)
#define TRACE_LOG0(_c, _n) \
	TRACE_LOG8(_c, _n,  0,  0,  0,  0,  0,  0,  0,  0)

#define TRACE_DUMP(_c, _fd) \
	do { \
		int idx, wrapped; \
		unsigned prev_time=0; \
		trace_entry_t *entp; \
		char *fmt; \
		wrapped = !_c##_ctl.wrapped; \
		if (_c##_ctl.wrapped) \
			idx = _c##_ctl.cur; \
		else \
			idx = 0; \
		while(!wrapped || (idx != _c##_ctl.cur)) { \
			entp = &_c##_entries[idx]; \
			if (entp->index >= _c##_NUM_TRACES) \
				fmt = "%8d ( %8d ) unknown trace! 0x%x 0x%x " \
					"0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n"; \
			else \
				fmt = _c##_formats[entp->index]; \
			snprintf(_c##_ctl.buf, TRACE_BUFSIZ, fmt, \
				 entp->time, entp->time - prev_time, \
				 entp->_0, entp->_1, entp->_2, entp->_3, \
				 entp->_4, entp->_5, entp->_6, entp->_7); \
			_c##_ctl.buf[TRACE_BUFSIZ-1] = '\0'; \
			write(_fd, _c##_ctl.buf, strlen(_c##_ctl.buf)); \
			prev_time = entp->time; \
			if (++idx >= _c##_num_entries) { \
				wrapped = 1; idx = 0; \
			} \
		} \
	} while (0)

#endif /* _TRACE_H */
