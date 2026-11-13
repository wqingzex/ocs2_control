/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_usbpro.h - the inner usb parts header for kcan-usb pro support
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
#ifndef __KCAN_USBPRO_H__
#define __KCAN_USBPRO_H__

#include <linux/types.h>
#include <linux/usb.h>

#include <src/kcan_main.h>

extern const struct kcanfd_bittiming_range sja2010_caps;
extern const kcanfd_mono_clock_device sja2010_clocks;

#ifdef LINUX_26
#define __usb_submit_urb(x) usb_submit_urb(x, GFP_ATOMIC)
#define __usb_alloc_urb(x)  usb_alloc_urb(x, GFP_ATOMIC)
#define FILL_BULK_URB(a, b, c, d, e, f, g) usb_fill_bulk_urb(a, b, c, d, e, (usb_complete_t)f, (void *)g)
#else
#define __usb_submit_urb(x) usb_submit_urb(x)
#define __usb_alloc_urb(x)  usb_alloc_urb(x)
#endif

#ifdef __cplusplus__
extern "C" {
#endif

int kcan_usbpro_init(struct kcan_usb_interface *);

#ifdef __cplusplus__
};
#endif

#endif /* __KCAN_USBPRO_H__ */
