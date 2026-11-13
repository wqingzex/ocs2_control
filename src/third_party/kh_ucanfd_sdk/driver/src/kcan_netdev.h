/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_netdev.h - CAN network device support defines / prototypes
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
#ifndef KCAN_NETDEV_H
#define KCAN_NETDEV_H

#include <linux/netdevice.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,31)
#include <linux/can/dev.h>
#endif

#ifndef LINUX_26
#define netdev_priv(dev) ((dev)->priv)
#endif

struct can_bittiming *kcan_netdev_get_bittiming(struct kcandev *dev);
struct can_bittiming *kcan_netdev_get_dbittiming(struct kcandev *dev);

struct net_device_stats *kcan_netdev_get_stats(struct net_device *dev);
int kcan_netdev_register(struct kcandev *dev);
int kcan_netdev_unregister(struct kcandev *dev);
int kcan_netdev_rx(struct kcandev *dev, struct kcanfd_rxmsg *pf);

#endif /* KCAN_NETDEV_H */
