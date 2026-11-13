/* SPDX-License-Identifier: GPL-2.0 */
/*
 * CAN-FD extension to Kunhong-Elec CAN products.
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
 * Author:       Kunhong-Elec driver team <gene@cdkhdz.com>
 */
#ifndef __kcanfd_core_h__
#define __kcanfd_core_h__

#include "src/kcan_common.h"
#include "src/kcan_main.h"
#include "src/kcan_fops.h"

/* hidden flag used to check init settings
 * (see lib/libkcanfd.h:#define OFD_KCANFD_MASK              (~0xff000000)
 */
#define KCANFD_INIT_USER		0x80000000
#define KCANFD_INIT_BTR_NOK		0x40000000

int kcan_bittiming_normalize(struct kcan_bittiming *pbt,
			u32 clock_Hz, const struct kcanfd_bittiming_range *caps);
struct kcan_bittiming *kcan_btr0btr1_to_bittiming(struct kcan_bittiming *pbt,
						  u16 btr0btr1);
struct kcanfd_init *kcan_init_to_fd(struct kcandev *dev,
				    struct kcanfd_init *pfdi,
				    const TKCANInit *pi);
static inline void kcanfd_copy_init(struct kcanfd_init *pd,
				    const struct kcanfd_init *ps)
{
	*pd = *ps;
}

/* CAN-FD new API */
void __kcanfd_dev_reset(struct kcandev *dev);

#define kcanfd_dev_reset(d)		__kcanfd_dev_reset(d)

void kcanfd_dev_open_init(struct kcandev *dev);
int kcanfd_dev_open(struct kcandev *dev, struct kcanfd_init *pfdi);

int kcanfd_ioctl_set_init(struct kcandev *dev, struct kcanfd_init *pfdi);
int kcanfd_ioctl_get_init(struct kcandev *dev, struct kcanfd_init *pfdi);
int kcanfd_ioctl_reset(struct kcandev *dev, unsigned long flags);
int kcanfd_ioctl_get_state(struct kcandev *dev, struct kcanfd_state *pfds);
int kcanfd_ioctl_add_filter(struct kcandev *dev, struct kcanfd_msg_filter *pf);
int kcanfd_ioctl_add_filters(struct kcandev *dev,
						struct kcanfd_msg_filters *pfl);
int kcanfd_ioctl_get_filters(struct kcandev *dev,
						struct kcanfd_msg_filters *pfl);
int kcanfd_ioctl_send_msg(struct kcandev *dev, struct kcanfd_txmsg *pmsgfd,
						struct kcanusr *usr);
int kcanfd_ioctl_send_msgs(struct kcandev *dev, struct kcanfd_txmsgs *pl,
						struct kcanusr *usr);
int kcanfd_ioctl_recv_msg(struct kcandev *dev, struct kcanfd_rxmsg *pmsgfd,
						struct kcanusr *usr);
int kcanfd_ioctl_recv_msgs(struct kcandev *dev, struct kcanfd_rxmsgs *pl,
						struct kcanusr *usr);
#endif
