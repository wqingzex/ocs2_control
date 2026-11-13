/* SPDX-License-Identifier: GPL-2.0 */
/*
 * all parts to handle device interface specific for KCAN-PCIExpressCard
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
#ifndef __KCAN_PCIEC_H__
#define __KCAN_PCIEC_H__

#include <src/kcan_main.h>

KCAN_PCIEC_CARD *kcan_pciec_create_card(struct pci_dev *pciDev,
					struct kcandev *dev);
KCAN_PCIEC_CARD *kcan_pciec_locate_card(struct pci_dev *pciDev,
					struct kcandev *dev);
void kcan_pciec_delete_card(struct kcandev *dev);
void kcan_setVCCEN(KCAN_PCIEC_CARD *card, int On);

#endif /* __KCAN_PCIEC_H__ */
