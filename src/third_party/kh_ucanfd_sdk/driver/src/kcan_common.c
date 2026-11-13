/* SPDX-License-Identifier: GPL-2.0 */
/*
 * global defines to include in all files this module is made of
 * common compatibilty sources
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
#include <linux/errno.h>
#include <linux/ioport.h>

int ___request_region(unsigned long from, unsigned long length,
			const char *name)
{
	if (!request_region(from, length, name))
		return -EBUSY;
  
	return 0;
}
