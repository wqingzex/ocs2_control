/* SPDX-License-Identifier: GPL-2.0 */
/*
 * all parts to handle the SPI flash of the PCIe devices.
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
#ifndef __KCAN_PCI_SPI_H__
#define __KCAN_PCI_SPI_H__

#define KCAN_PCI_FLASHED_DEVID		16

/* If defined, the driver handles virtual device numbers for each channel:
 * canX device number = device_number[0] + X
 * If not defined, then:
 * canX device number = device_number[X]
 *
 * KCAN_PCI_SINGLE_DEVID was defined for Beta____20200122_n-8.10.0 only.
 */
//#define KCAN_PCI_SINGLE_DEVID

/* this could be used as a signature value */
#define KCAN_PCI_FLASH_SIZEOF		0x254

struct __packed kcan_pci_flash_data {
	__le32 recordsize;	/* sizeof(this) or 0xffffffff */
	__le32 serial_number;
	__le32 windows_dnr;
	__le32 device_number[KCAN_PCI_FLASHED_DEVID];
	char user_location[256];
	__le64 oem_code;
	char user_data[256];
};

struct kcan_pci_adapter;	/* need kunhong_pci.h */

int kcan_pci_spi_flash_read(struct kcan_pci_adapter *kcan_pci, u32 addr,
			    void *data, int l);
int kcan_pci_spi_flash_write(struct kcan_pci_adapter *kcan_pci, u32 addr,
			     void *data, int l);

#endif
