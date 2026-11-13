/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_filter.h - all about CAN message filtering - interface
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
#ifndef __KCAN_FILTER_H__
#define __KCAN_FILTER_H__

#include "src/kcan_common.h"
#include "src/kcan_main.h"

void *kcan_create_filter_chain(void);
int kcan_add_filter(void *handle, u32 FromID, u32 ToID, u32 flags);
void kcan_delete_filter_all(void *handle);
int kcan_do_filter(void *handle, struct kcanfd_rxmsg *pe);
void *kcan_delete_filter_chain(void *handle);

int kcan_get_filters_count(void *handle);
int kcan_add_filters(void *handle, struct kcanfd_msg_filter *pf, int count);
int kcan_get_filters(void *handle, struct kcanfd_msg_filter *pf, int count);

#endif
