/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_main_linux.c - the starting point of the driver,
 *                     init and cleanup and proc interface
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
#define DEV_REGISTER()		dev_register()
#define DEV_UNREGISTER()	dev_unregister()
#define REMOVE_DEV_LIST		remove_dev_list

static int kcan_is_hotplug(struct kcandev *dev)
{
	switch (dev->wType) {
	case HW_PCI:
	case HW_PCIE_FD:
	case HW_ISA:
		/* udev events were not generated for ISA.
		 * Thx David Leonard
		 */
	case HW_ISA_SJA:
	case HW_DONGLE_SJA:
	case HW_DONGLE_SJA_EPP:
		return 0;
	default:
		break;
	}

	/* hotplug devices have to register/unregister by themselves */
	return 1;
}

/* contrary to former implementation this function only registers devices and
 * do register a driver nor request a major number in case of dynamic major
 * number allocation
 */
static int dev_register(void)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	/* Note: since this function is called at module _init() then
	 * there is no need to lock access to the devices global list:
	 * no interrupt handler has been requested yet
	 */
#ifdef NETDEV_SUPPORT
	{
		struct list_head *ptr;
		struct kcandev *dev;

		/* create all netdevice entries except those for hotplug-devices
		 * USB   : is done by kcan_usb_plugin().
		 * PCCARD: is done by kcan_pccard_register_devices() at driver
		 *         init time (here & now! - see above) or at plugin
		 *         time.
		 */
		list_for_each(ptr, &kcan_drv.devices) {
			dev = list_entry(ptr, struct kcandev, list_dev);
			if (!kcan_is_hotplug(dev))
				kcan_netdev_register(dev);
		}
	}
#endif

#ifdef UDEV_SUPPORT
	{
		struct list_head *ptr;
		struct kcandev *dev;

		list_for_each(ptr, &kcan_drv.devices) {
			dev = list_entry(ptr, struct kcandev, list_dev);
			if (!kcan_is_hotplug(dev))
				kcan_sysfs_dev_node_create(dev);
		}
	}
#endif

	/* for compatibility to former implementation it is returned */
	return kcan_drv.nMajor;
}

/* contrary to former implementation this function only unregisters
 * devices
 */
static void dev_unregister(void)
{
#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	/* Note: since this function is called at module _exit() then
	 * there is no need to lock access to the devices global list:
	 * all interrupt handlers have been released.
	 */
#ifdef UDEV_SUPPORT
	{
		struct list_head *ptr;
		struct kcandev *dev;

		list_for_each(ptr, &kcan_drv.devices) {
			dev = list_entry(ptr, struct kcandev, list_dev);
			if (!kcan_is_hotplug(dev))
				kcan_sysfs_dev_node_destroy(dev);
		}
	}
#endif

#ifdef NETDEV_SUPPORT
	{
		struct list_head *ptr;
		struct kcandev *dev;

		/* remove all netdevice registrations except those for
		 * USB-devices which is done by kcan_usb_plugout().
		 */
		list_for_each(ptr, &kcan_drv.devices) {
			dev = list_entry(ptr, struct kcandev, list_dev);
			if (!kcan_is_hotplug(dev))
				kcan_netdev_unregister(dev);
		}
	}
#endif
}
