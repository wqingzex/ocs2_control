/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_fifo.h - all about fifo buffer management
 *
 * Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd <www.kunhong-elec.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Contact:      <support@cdkhdz.com>
 * Maintainer:   Kunhong-Elec driver team <gene@cdkhdz.com>
 */
#ifndef __KCAN_FIFO_H__
#define __KCAN_FIFO_H__

#include "src/kcan_common.h"

#define FIFO_CIRCULAR		0x00000001

typedef struct {
	u16	wStepSize;	/* size of bytes to step to next entry */
	u16	wCopySize;	/* size of bytes to copy */
	u32	flags;
	void *	bufferBegin;	/* points to first element */
	void *	bufferEnd;	/* points to the last element */
	u32	nCount;		/* max count of elements in fifo */
	u32	nStored;	/* count of currently rcvd stored msgs */
	u32	dwTotal;	/* received messages */
	void *	r;		/* next Msg to read from the read buffer */
	void *	w;		/* next Msg to write into read buffer */
	kcan_lock_t lock;	/* mutual exclusion lock (if not NULL) */
} FIFO_MANAGER;

typedef int (*FIFO_CALLBACK)(FIFO_MANAGER *anchor, void *pvPutData,
			     void *arg, int err);

int kcan_fifo_items_count(FIFO_MANAGER *anchor);

static inline int kcan_fifo_items_max(FIFO_MANAGER *anchor)
{
	return anchor->nCount;
}

static inline int kcan_fifo_empty(FIFO_MANAGER *anchor)
{
	return !kcan_fifo_items_count(anchor);
}

static inline int kcan_fifo_full(FIFO_MANAGER *anchor)
{
	return kcan_fifo_items_count(anchor) >= kcan_fifo_items_max(anchor);
}

int kcan_fifo_reset(FIFO_MANAGER *anchor);
int kcan_fifo_init(FIFO_MANAGER *anchor, void *bufferBegin,
		   void *bufferEnd, int nCount, u16 wCopySize);

int kcan_fifo_put_nolock(FIFO_MANAGER *anchor, void *pvPutData);
int kcan_fifo_put_ex(FIFO_MANAGER *anchor, void *pvPutData,
		     FIFO_CALLBACK put_completion_func, void *arg);
static inline int kcan_fifo_put(FIFO_MANAGER *anchor, void *pvPutData)
{
	return kcan_fifo_put_ex(anchor, pvPutData, NULL, NULL);
}

int kcan_fifo_get_ex(FIFO_MANAGER *anchor, void *pvPutData,
		     FIFO_CALLBACK get_completion_func, void *arg);
static inline int kcan_fifo_get(FIFO_MANAGER *anchor, void *pvPutData)
{
	return kcan_fifo_get_ex(anchor, pvPutData, NULL, NULL);
}

int kcan_fifo_peek(FIFO_MANAGER *anchor, void *pvGetData);

int kcan_fifo_foreach_back(FIFO_MANAGER *anchor,
			   int (*pf)(void *item, void *arg), void *arg);
u32 kcan_fifo_ratio(FIFO_MANAGER *anchor);
#endif
