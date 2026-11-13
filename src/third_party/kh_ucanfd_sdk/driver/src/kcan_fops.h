/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_fops.h - header for struct fops only
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
#ifndef __KCAN_FOPS_H__
#define __KCAN_FOPS_H__

#include <linux/kernel.h>   /* printk() */
#include <linux/file.h>

#ifdef NO_RT
extern struct file_operations kcan_fops;

#elif defined(XENOMAI3)
#include <rtdm/driver.h>

extern struct rtdm_driver kcandrv_rt;
#else
#include <rtdm/rtdm_driver.h>

extern struct rtdm_device kcandev_rt;
#endif

int kcan_alloc_tx_msgs(struct kcandev *dev);

#ifndef NETDEV_SUPPORT
int kcan_alloc_rx_msgs(struct kcandev *dev);
#endif

int kcan_open_path(struct kcandev *dev, struct kcanusr *ctx);
void kcan_release_path(struct kcandev *dev, struct kcanusr *ctx);

int kcan_ioctl_extended_status_common(struct kcandev *dev,
					TPEXTENDEDSTATUS *pes);
int kcan_ioctl_status_common(struct kcandev *dev, TPSTATUS *ps);
int kcan_ioctl_diag_common(struct kcandev *dev, TPDIAG *pd);

#endif
