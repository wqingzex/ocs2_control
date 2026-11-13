/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_timing.h - timing/baudrate conversion facilities
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
#ifndef __kcan_timing_h__
#define __kcan_timing_h__

#include <linux/types.h>
#include <kcanfd.h>

/* sample_point unit:
 * Sample Point = (sample_point / KCAN_SAMPT_SCALE)%
 *
 * Warning: linux-can layer uses KCAN_SAMPT_SCALE=1000
 * #define KCAN_SAMPT_SCALE		1000
 */
#define KCAN_SAMPT_SCALE		10000

/* Hardware-independent description of a baud rate */
struct kcan_bittiming_abs {
	u32   prescaler;  /* the below values are for this prescaler */

	u32   sync_seg_ns;	/* Duration of sync segment */
	u32   tseg1_ns;		/* Duration sync to sample point */
	u32   tseg2_ns;		/* Duration sample point to end-of-bit */
	u32   sjw_ns;		/* tollerance for timing:
	                         * either tseg is extended by max. sjw,
	                         * or tseg2 is reduced by max sjw. */
	/* Info only */
	u32   bitrate;
	u32   delta_bitrate;	/* calcualted from sjw */
	u8    sample3;		/* similar to SAM: 1=3 samples. 0=1 sample */
};

/* helper function */
void kcan_btr0btr1_to_abstract(struct kcan_bittiming_abs *pa, u16 btr0btr1);

int kcan_abstract_to_bittiming(struct kcan_bittiming *bitrate,
				const struct kcan_bittiming_abs *pa,
				const struct kcanfd_bittiming_range *pcap,
				u32 sysclock_Hz);

int kcan_convert_abstract(struct kcan_bittiming_abs *pba,
				const struct kcan_bittiming_abs *pbac,
				const struct kcanfd_bittiming_range *caps,
				u32 sysclock_Hz);

int kcan_bitrate_to_bittiming(struct kcan_bittiming *pbt,
			const struct kcanfd_bittiming_range *caps,
			u32 sysclock_Hz);

static inline int kcan_is_bittiming_valid(struct kcan_bittiming *pbt)
{
	return pbt->brp || pbt->bitrate;
}

#endif
