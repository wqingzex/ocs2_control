/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_parse.h - header for read input parser and write output formatter
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
#ifndef __KCAN_PARSE_H__
#define __KCAN_PARSE_H__

int kcan_make_output(char *buffer, struct kcanfd_msg *pf);
int kcan_parse_input_idle(char *buffer);
int kcan_parse_input_message(char *buffer, struct kcanfd_msg *pf);
int kcan_parse_input_init(char *buffer, struct kcanfd_init *pi);

#endif
