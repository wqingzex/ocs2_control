/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_fifo.c - manages the ring buffers for read and write data
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
#include "src/kcan_common.h"

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/sched.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 4, 0)
#include <asm/system.h>     /* cli(), save_flags(), restore_flags() */
#endif

#include "src/kcan_fifo.h"

static int __kcan_fifo_reset(FIFO_MANAGER *anchor)
{
	kcan_lock_irqsave_ctxt lck_ctx;

	kcan_lock_get_irqsave(&anchor->lock, lck_ctx);

	anchor->dwTotal = 0;
	anchor->nStored = 0;
	anchor->r = anchor->w = anchor->bufferBegin;

	kcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return 0;
}

int kcan_fifo_reset(FIFO_MANAGER *anchor)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s() %d %p %p\n",
		__func__, anchor->nStored, anchor->r, anchor->w);
#endif
	return __kcan_fifo_reset(anchor);
}

int kcan_fifo_init(FIFO_MANAGER *anchor, void *bufferBegin,
		   void *bufferEnd, int nCount, u16 wCopySize)
{
	if ((bufferBegin > bufferEnd) || (nCount <= 1) || !wCopySize)
		return -EINVAL;

	anchor->wStepSize = (bufferBegin == bufferEnd) ? 0 : \
			    ((bufferEnd - bufferBegin) / (nCount - 1));

	/* check for fatal program errors */
	if (anchor->wStepSize < wCopySize)
		return -EINVAL;

	anchor->wCopySize = wCopySize;
	anchor->nCount = nCount;

	anchor->bufferBegin = bufferBegin;
	anchor->bufferEnd = bufferEnd;

	anchor->flags = 0;

	kcan_lock_init(&anchor->lock);

	return __kcan_fifo_reset(anchor);
}

/*
 * Push an item into a locked fifo.
 *
 * @return:
 *
 *	> 0	if item successfully stored (=fifo current number of items).
 *	< 0	error (errno)
 */
int kcan_fifo_put_nolock(FIFO_MANAGER *anchor, void *pvPutData)
{
	int err;

#ifndef FIFO_CIRCULAR
	if (anchor->nStored < anchor->nCount) {
		memcpy(anchor->w, pvPutData, anchor->wCopySize);

		err = ++anchor->nStored;
		anchor->dwTotal++;

		if (anchor->w < anchor->bufferEnd)
			anchor->w += anchor->wStepSize;
		else
			anchor->w = anchor->bufferBegin;
	} else {
		err = -ENOSPC;
	}

#else /* FIFO_CIRCULAR */

	if (anchor->nStored >= anchor->nCount) {

		/* if the fifo is a circular buffer, then remove oldest one */
		if (!(anchor->flags & FIFO_CIRCULAR)) {
			err = -ENOSPC;
			goto lbl_exit;
		}

		/* move reader pointer forward */
		if (anchor->r < anchor->bufferEnd)
			anchor->r += anchor->wStepSize;
		else
			anchor->r = anchor->bufferBegin;

		anchor->nStored--;
	}

	memcpy(anchor->w, pvPutData, anchor->wCopySize);

	err = ++anchor->nStored;
	anchor->dwTotal++;

	if (anchor->w < anchor->bufferEnd)
		anchor->w += anchor->wStepSize;
	else
		anchor->w = anchor->bufferBegin;

lbl_exit:
#endif
	return err;
}

/*
 * Push an item into a fifo with completion function processing.
 *
 * @return:
 *
 *	== 0	provided completion forbids to store
 *	> 0	if item successfully stored (=fifo current number of items).
 *	< 0	error (errno)
 */
int kcan_fifo_put_ex(FIFO_MANAGER *anchor, void *pvPutData,
		     FIFO_CALLBACK put_completion_func, void *arg)
{
	kcan_lock_irqsave_ctxt lck_ctx;
	int err;

	kcan_lock_get_irqsave(&anchor->lock, lck_ctx);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s() %d %p %p\n",
		__func__, anchor->nStored, anchor->r, anchor->w);
#endif
#ifdef FIFO_PRE_ROUTINE
	/* call pre routine (err = 0) */
	if (put_completion_func) {
		err = put_completion_func(anchor, pvPutData, arg, 0);
		if (err)
			return 0;
	}
#endif
	err = kcan_fifo_put_nolock(anchor, pvPutData);

	/* call post routine (err != 0) */
	if (put_completion_func)
		err = put_completion_func(anchor, pvPutData, arg, err);

	kcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

/*
 * Read an item from a fifo
 *
 * @return:
 *
 *	>= 0	if item successfully copied (number of items still in the fifo)
 *	< 0	if not (errno)
 */
int kcan_fifo_get_ex(FIFO_MANAGER *anchor, void *pvGetData,
		     FIFO_CALLBACK get_completion_func, void *arg)
{
	int err;
	kcan_lock_irqsave_ctxt lck_ctx;

	kcan_lock_get_irqsave(&anchor->lock, lck_ctx);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s() %d %p %p\n",
		__func__, anchor->nStored, anchor->r, anchor->w);
#endif

	if (anchor->nStored > 0) {
		if (pvGetData)
			memcpy(pvGetData, anchor->r, anchor->wCopySize);

		if (anchor->r < anchor->bufferEnd)
			anchor->r += anchor->wStepSize;
		else
			anchor->r = anchor->bufferBegin;

		err = --anchor->nStored;
	} else
		err = -ENODATA;

	if (get_completion_func)
		err = get_completion_func(anchor, pvGetData, arg, err);

	kcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

int kcan_fifo_foreach_back(FIFO_MANAGER *anchor,
			   int (*pf)(void *item, void *arg), void *arg)
{
	u32 i;
	void *p;
	int err = 0;
	kcan_lock_irqsave_ctxt lck_ctx;

	kcan_lock_get_irqsave(&anchor->lock, lck_ctx);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s() %d %p %p\n",
		__func__, anchor->nStored, anchor->r, anchor->w);
#endif

	p = anchor->w;
	for (i = 0; i < anchor->nStored; i++) {

		if (p == anchor->bufferBegin)
			p = anchor->bufferEnd;
		else
			p -= anchor->wStepSize;

		err = pf(p, arg);
		if (err)
			break;
	}

	kcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

/*
 * Read an item from a fifo without changing any pointers.
 *
 * @return:
 *
 *	> 0	if item successfully copied (current items still in fifo)
 *	< 0	if not (errno)
 */
int kcan_fifo_peek(FIFO_MANAGER *anchor, void *pvGetData)
{
	int err;
	kcan_lock_irqsave_ctxt lck_ctx;

	kcan_lock_get_irqsave(&anchor->lock, lck_ctx);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s() %d %p %p\n",
		__func__, anchor->nStored, anchor->r, anchor->w);
#endif

	if (anchor->nStored > 0) {
		memcpy(pvGetData, anchor->r, anchor->wCopySize);
		err = anchor->nStored;
	} else {
		err = -ENODATA;
	}

	kcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return err;
}

u32 kcan_fifo_ratio(FIFO_MANAGER *anchor)
{
	u32 ratio;

	kcan_lock_irqsave_ctxt lck_ctx;

	kcan_lock_get_irqsave(&anchor->lock, lck_ctx);
	ratio = anchor->nCount ? (anchor->nStored * 10000) / anchor->nCount : 0;
	kcan_lock_put_irqrestore(&anchor->lock, lck_ctx);

	return ratio;
}

int kcan_fifo_items_count(FIFO_MANAGER *anchor)
{
	return anchor->nStored;
}
