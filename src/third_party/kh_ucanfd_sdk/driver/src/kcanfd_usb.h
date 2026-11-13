/* SPDX-License-Identifier: GPL-2.0 */
/*
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
#ifndef __KCAN_USBFD_H__
#define __KCAN_USBFD_H__

/*
 * INCLUDES
 */
#include <linux/types.h>
#include <linux/usb.h>

#include "src/kcan_main.h"
#include "src/kcanfd_ucan.h"

/*
 * DEFINES
 */
#define KCAN_USBPROFD_PRODUCT_ID	0x0011
#define KCAN_USBFD_PRODUCT_ID		0x0012
#define KCAN_USBCHIP_PRODUCT_ID		0x0013
#define KCAN_USBX6_PRODUCT_ID		0x0014
#define KCAN_USBX4_PRODUCT_ID		0x0020

#ifdef LINUX_26
#define __usb_submit_urb(x)	usb_submit_urb(x, GFP_ATOMIC)
#define __usb_alloc_urb(x)	usb_alloc_urb(x, GFP_ATOMIC)
#else
#define __usb_submit_urb(x)	usb_submit_urb(x)
#define __usb_alloc_urb(x)	usb_alloc_urb(x)
#endif

/*
 * External API
 */
#ifdef __cplusplus__
extern "C" {
#endif

int kcan_usbfd_init(struct kcan_usb_interface *);
int kcan_usbfd_init_ep(struct kcan_usb_interface *usb_if);

/* KCAN-Chip specific options */
int kcan_usb_chip_get_dig_cfg(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c);
int kcan_usb_chip_set_dig_cfg(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c);
int kcan_usb_chip_get_dig_val(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c);
int kcan_usb_chip_set_dig_val(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c);
int kcan_usb_chip_set_dig_bit(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c);
int kcan_usb_chip_clr_dig_bit(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c);
int kcan_usb_chip_get_ana_val(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c);

#ifdef __cplusplus__
};
#endif

#endif
