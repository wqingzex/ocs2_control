/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_main.c - the starting point of the driver,
 *               init and cleanup and proc interface
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
/* #define DEBUG */
/* #undef DEBUG */

#include "src/kcan_common.h"	/* must always be the 1st include */

/* #define KBUILD_MODNAME kcan */

#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <linux/capability.h>
#include <linux/param.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <asm/system.h>
#endif
#include <asm/uaccess.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(3,9,0)
/* if defined, create_proc_entry() is not used to create /proc/kcan */
#define CREATE_PROC_ENTRY_DEPRECATED
#endif

#ifdef DEBUG
#define DEBUG_PATCH
#define DEBUG_BUS_STATE
#define DEBUG_BUS_LOAD
#define DEBUG_RX_QUEUE
#define DEBUG_TS
#define DEBUG_ALLOC_DEV
#define DEBUG_OLD_API
#define DEBUG_INVALID_TS
#define DEBUG_CLEANUP
#define DEBUG_SYSFS
#define DEBUG_TRACE_DEFERRED
#define DEBUG_INVALID_BUS_STATE
#define DEBUG_TRACE
#else
//#define DEBUG_PATCH
//#define DEBUG_RX_QUEUE
//#define DEBUG_TS
//#define DEBUG_TS_HWTYPE	HW_USB_PRO_FD
//#define DEBUG_TS_HWTYPE	HW_USB_FD
//#define DEBUG_TS_HWTYPE	HW_PCIE_FD
//#define DEBUG_TS_HWTYPE	HW_USB
//#define DEBUG_TS_HWTYPE		HW_USB_PRO
//#define DEBUG_BUS_STATE
//#define DEBUG_BUS_LOAD
//#define DEBUG_ALLOC_DEV
//#define DEBUG_OLD_API
//#define DEBUG_INVALID_TS
//#define DEBUG_CLEANUP
//#define DEBUG_SYSFS
//#define DEBUG_TRACE_DEFERRED
//#define DEBUG_INVALID_BUS_STATE
//#define DEBUG_TRACE
#endif

#ifdef PCI_SUPPORT
#include "src/kcan_pci.h"	/* get support for KCAN-PCI */
#endif
#ifdef ISA_SUPPORT
#include "src/kcan_isa.h"	/* get support for KCAN-ISA and KCAN-104 */
#endif
#ifdef DONGLE_SUPPORT
#include "src/kcan_dongle.h"	/* get support for KCAN-Dongle */
#endif
#ifdef USB_SUPPORT
#include "src/kcan_usb_core.h"	/* get support for KCAN-USB */
#endif
#ifdef PCCARD_SUPPORT
#include "src/kcan_pccard.h"
#endif
#ifdef NETDEV_SUPPORT
#include "src/kcan_netdev.h"
#endif

#include "src/kcanfd_core.h"
#include "src/kcan_fifo.h"
#include "src/kcan_filter.h"
#include "src/kcan_sja1000.h"
#include "src/kcanfd_ucan.h"

/* if defined, create /sys/class/kcan tree */
#define SYSFS_SUPPORT

#ifndef NETDEV_SUPPORT

/* if defined, then sysfs attr "ts_mode" is writable: user can select timestamp
 * mode for a specific device via sysfs.
 * if not defined, ts_mode cannot be changed via sysfs. ts_mode can only be
 * set with deftsmode or the corresponding option.
 */
#define KCAN_SYSFS_TSMODE_RW

/* if defined, hw timestamps given to application are checked whether they
 * are valid, that is:
 * 1/ tv_usec must belong to [0..999999]
 * 2/ whole timestamp must not come from the future.
 */
#define KCAN_FIX_INVALID_TS
#endif

/* Variables needed for both chardev and netdev modes */
#if KCAN_ENABLE_TIME_SYNC
#define KCANFD_OPT_HWTIMESTAMP_DEF	KCANFD_OPT_HWTIMESTAMP_MAX
#else
#define KCANFD_OPT_HWTIMESTAMP_DEF	KCANFD_OPT_HWTIMESTAMP_RAW
#endif
#define KCANFD_OPT_HWTIMESTAMP_LOW	KCANFD_OPT_HWTIMESTAMP_OFF
#define KCANFD_OPT_HWTIMESTAMP_HIGH	KCANFD_OPT_HWTIMESTAMP_MAX-1

static ushort deftsmode = KCANFD_OPT_HWTIMESTAMP_DEF;	/* use def ts_mode */
module_param(deftsmode, ushort, 0644);
MODULE_PARM_DESC(deftsmode, " default ts mode");

static int _kcan_set_ts_mode(struct kcandev *dev, u32 ts_mode, bool fallback);
void kcan_sync_init(struct kcandev *dev);

#define KCANFD_OPT_BUSLOADINDFREQ_DEF	500

static ushort defblperiod = KCANFD_OPT_BUSLOADINDFREQ_DEF;
module_param(defblperiod, ushort, 0644);
MODULE_PARM_DESC(defblperiod, " default bus load msg period (def="
		__stringify(KCANFD_OPT_BUSLOADINDFREQ_DEF) " ms.)");

/* note: using 10^6 scale generates useless divisions: cooked count of us does
 * not change in fine, while 10^4 creates a few dozens of us. difference.
 * note: when using _SHIFT value, then the driver scales the clock drift with
 * 2^_SHIFT so that it saves the multiplication.
 * note: the _SHIFT value should not be too large: having a too large _SCALE
 * value will generate lots of divisions in kcan_sync_decode() for a very amount
 * of µs of clock shift.
 *
 * NETDEV mode may build without KCAN_HANDLE_CLOCK_DRIFT; keep sane defaults.
 */
#ifndef KCAN_CLOCK_DRIFT_SCALE_SHIFT
#define KCAN_CLOCK_DRIFT_SCALE_SHIFT	17
#endif
#ifndef KCAN_CLOCK_DRIFT_SCALE
#define KCAN_CLOCK_DRIFT_SCALE		(1<<(KCAN_CLOCK_DRIFT_SCALE_SHIFT))
#endif

/* if defined, then time sync system is reset every KCAN_RESET_TIME_SYNC_PERIOD
 * calibration messages when no activity has been detected on the channel.
 * This allows to fix time_lag drift.
 */
#define KCAN_RESET_TIME_SYNC_PERIOD	10

#define DEFAULT_BTR0BTR1	CAN_BAUD_500K	/* defaults to 500 kbit/sec */
#define DEFAULT_DBITRATE	2000000		/* default data bitrate = 2M */

/* filled by module initialisation */
char *type[8] = {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL};
u32  io[8]    = {0, 0, 0, 0, 0, 0, 0, 0};
u8   irq[8]   = {0, 0, 0, 0, 0, 0, 0, 0};
u16  btr0btr1  = DEFAULT_BTR0BTR1;
char *bitrate = NULL;
char *dbitrate = NULL;

/* the global driver object, create it */
struct kcan_driver kcan_drv = {};

static u32 kcan_def_bitrate = 0;
static u32 kcan_def_dbitrate = DEFAULT_DBITRATE;

#ifdef SYSFS_SUPPORT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
/* some stuff to support SysFS coming with kernel 2.6 */
#include <linux/device.h>
#endif
#endif

/* build current driver config string for output in kernel log and procfs */
const char current_config[] = " "
#ifdef DEBUG
"[dbg] "
#endif
#ifdef MODVERSIONS
"[mod] "
#endif
#ifdef ISA_SUPPORT
"[isa] "
#endif
#ifdef PCI_SUPPORT
"[pci] "
#endif
#ifdef PCIEC_SUPPORT
"[pec] "
#endif
#ifdef DONGLE_SUPPORT
"[dng] "
#endif
#ifdef PARPORT_SUBSYSTEM
"[par] "
#endif
#ifdef USB_SUPPORT
"[usb] "
#endif
#ifdef PCCARD_SUPPORT
"[pcc] "
#endif
#ifdef NETDEV_SUPPORT
"[net] "
#endif
#ifndef NO_RT
"[rt] "
#endif
;

#ifndef NETDEV_SUPPORT

#define KCAN_DEV_RXQSIZE_MIN	2

/* 4194304 / 152 ~ 27594 with x86 */
#define KCAN_DEV_RXQSIZE_MAX	(KMALLOC_MAX_SIZE / sizeof(struct kcanfd_rxmsg))

extern ushort rxqsize;
extern unsigned char rxqprealloc;
#endif

#define KCANFD_DRIVER_CLOCK_DEF		CLOCK_REALTIME

ushort drvclkref = KCANFD_DRIVER_CLOCK_DEF;
module_param(drvclkref, ushort, 0644);
MODULE_PARM_DESC(drvclkref, " driver clock reference (def="
			__stringify(KCANFD_DRIVER_CLOCK_DEF) ")");

#define KCAN_DEV_TXQSIZE_MIN	2

/* 4194304 / 136 ~ 30840 with x86 */
#define KCAN_DEV_TXQSIZE_MAX	(KMALLOC_MAX_SIZE / sizeof(struct kcanfd_txmsg))

extern ushort txqsize;
extern unsigned char txqprealloc;

#define KCAN_DEV_DMA_MASK_DEF	64
#define KCAN_DEV_DMA_MASK_LOW	24
#define KCAN_DEV_DMA_MASK_HIGH	64

ushort dmamask = KCAN_DEV_DMA_MASK_DEF;

module_param(dmamask, ushort, 0644);
MODULE_PARM_DESC(dmamask, " ["
			__stringify(KCAN_DEV_DMA_MASK_LOW) ".."
			__stringify(KCAN_DEV_DMA_MASK_HIGH) "] (def="
			__stringify(KCAN_DEV_DMA_MASK_DEF) ")");

#define KCANFD_TX_HIWAT_MIN		5000
#define KCANFD_TX_HIWAT_MAX		10000
#ifdef NETDEV_SUPPORT
#define KCANFD_TX_HIWAT_DEF		8000
#else
#define KCANFD_TX_HIWAT_DEF		KCANFD_TX_HIWAT_MAX
#endif
ushort txqhiwat = KCANFD_TX_HIWAT_DEF;
module_param(txqhiwat, ushort, 0644);
MODULE_PARM_DESC(txqhiwat,
		 " Tx fifo hi-water ratio ["
		 __stringify(KCANFD_TX_HIWAT_MIN) ".."
		 __stringify(KCANFD_TX_HIWAT_MAX) "] def="
		 __stringify(KCANFD_TX_HIWAT_DEF) ")");

/* for procfs output the current_config is copied into this centered string */
char config[] = "*----------------------------------------------------------------------------";

#ifdef SYSFS_SUPPORT

/* linux < 2.6.27 use device_create_drvdata() */
#ifndef device_create_drvdata
#define	device_create_drvdata	device_create
#endif

static ssize_t show_u32(char *buf, u32 v)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", v);
}

static ssize_t show_int(char *buf, int v)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", v);
}

static ssize_t show_str(char *buf, char *str)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", str);
}

static ssize_t show_kcan_devid(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->device_alt_num);
}

static ssize_t store_kcan_devid(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct kcandev *pdev = to_kcandev(dev);

	if (pdev->device_params) {
		char *endptr;
		int err;
		TPEXTRAPARAMS exp = {
			.nSubFunction = SF_SET_HCDEVICENO,
		};

		exp.func.dwSerialNumber = simple_strtoul(buf, &endptr, 0);
		if (*endptr != '\n')
			return -EINVAL;

		err = pdev->device_params(pdev, &exp);
		if (err)
			return err;
	}

	return count;
}

static ssize_t show_kcan_sn(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	TPEXTRAPARAMS exp = {
		.nSubFunction = SF_GET_SERIALNUMBER,
	};
	int err;

	if (!pdev->device_params)
		return -EOPNOTSUPP;

	err = pdev->device_params(pdev, &exp);
	if (err)
		return err;

	return show_u32(buf, exp.func.dwSerialNumber);
}

static ssize_t show_device_guid(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	int l = 0;

	if (pdev && pdev->adapter) {
		int i;

		/* RFC-4122 format: 8-4-4-4-12 */
		for (i = 0; i < sizeof(pdev->adapter->guid); i++) {
			switch (i) {
			case 4:
			case 6:
			case 8:
			case 10:
				if (l < PAGE_SIZE)
					buf[l++] = '-';
				fallthrough;
			default:
				l += snprintf(buf+l, PAGE_SIZE, "%02x",
					      pdev->adapter->guid[i]);
			}
		}
	}

	if (l < PAGE_SIZE)
		buf[l++] = '\n';

	return l;
}

static ssize_t show_kcan_mass_storage_mode(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	u32 v;
	struct kcanfd_option opt = {
		.size = sizeof(v),
		.name = KCANFD_OPT_MAX,	/* trick to avoid copy_to_user() */
		.value = &v,
	};
	int err;

	if (!pdev->option[KCANFD_OPT_MASS_STORAGE_MODE].get) {
		pr_warn(DEVICE_NAME ": %s(L=%u): ABNORMAL NULL pointer\n",
			__func__, __LINE__);
		return -EOPNOTSUPP;
	}

	err = pdev->option[KCANFD_OPT_MASS_STORAGE_MODE].get(pdev, &opt, NULL);
	if (err)
		return err;

	return show_u32(buf, v);
}

static ssize_t store_kcan_mass_storage_mode(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t count)
{
	struct kcandev *pdev = to_kcandev(dev);
	u32 v;
	struct kcanfd_option opt = {
		.size = sizeof(v),
		.name = KCANFD_OPT_MAX,	/* trick to avoid copy_from_user() */
		.value = &v,
	};
	char *endptr;
	int err;

	if (!pdev->option[KCANFD_OPT_MASS_STORAGE_MODE].set) {
		pr_warn(DEVICE_NAME ": %s(L=%u): ABNORMAL NULL pointer\n",
			__func__, __LINE__);
		return -EOPNOTSUPP;
	}

	v = simple_strtoul(buf, &endptr, 0);
	if (*endptr != '\n')
		return -EINVAL;

	err = pdev->option[KCANFD_OPT_MASS_STORAGE_MODE].set(pdev, &opt, NULL);
	if (err)
		return err;

	return count;
}

static ssize_t show_kcan_identify(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	u32 v;
	struct kcanfd_option opt = {
		.size = sizeof(v),
		.name = KCANFD_OPT_MAX,	/* trick to avoid copy_to_user() */
		.value = &v,
	};
	int err;

	if (!pdev->option[KCANFD_OPT_FLASH_LED].get) {
		pr_warn(DEVICE_NAME ": %s(L=%u): ABNORMAL NULL pointer\n",
			__func__, __LINE__);
		return -EOPNOTSUPP;
	}

	err = pdev->option[KCANFD_OPT_FLASH_LED].get(pdev, &opt, NULL);
	if (err)
		return err;

	return show_u32(buf, v);
}

static ssize_t store_kcan_identify(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct kcandev *pdev = to_kcandev(dev);
	u32 v;
	struct kcanfd_option opt = {
		.size = sizeof(v),
		.name = KCANFD_OPT_MAX,	/* trick to avoid copy_from_user() */
		.value = &v,
	};
	char *endptr;
	int err;

	if (!pdev->option[KCANFD_OPT_FLASH_LED].set) {
		pr_warn(DEVICE_NAME ": %s(L=%u): ABNORMAL NULL pointer\n",
			__func__, __LINE__);
		return -EOPNOTSUPP;
	}

	v = simple_strtoul(buf, &endptr, 0);
	if (*endptr != '\n')
		return -EINVAL;

	err = pdev->option[KCANFD_OPT_FLASH_LED].set(pdev, &opt, NULL);
	if (err)
		return err;

	return count;
}

static ssize_t show_kcan_hwtype(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->wType);
}

static ssize_t show_kcan_minor(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return show_int(buf, to_kcandev(dev)->nMinor);
}

static ssize_t show_kcan_ts_mode(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	u32 tmp = pdev->ts_mode;

	if (pdev->flags & KCAN_DEV_TS_SOF)
		tmp |= KCANFD_OPT_HWTIMESTAMP_SOF;

	return show_u32(buf, tmp);
}

#ifdef KCAN_SYSFS_TSMODE_RW
static ssize_t store_kcan_ts_mode(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct kcandev *pdev = to_kcandev(dev);
	char *endptr;
	u32 v;
	int err;

	v = simple_strtoul(buf, &endptr, 0);
	if (*endptr != '\n')
		return -EINVAL;

	err = _kcan_set_ts_mode(pdev, v, false);
	if (err)
		return err;

	return count;
}
#endif

static ssize_t show_kcan_dev_name(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);

#ifdef NO_RT
	return snprintf(buf, PAGE_SIZE, "/dev/kcan%s%u\n",
			pdev->type, pdev->nMinor);
#else
	return snprintf(buf, PAGE_SIZE, "kcan%u\n",
			pdev->nMinor);
#endif
}

static ssize_t show_kcan_ctrlr_number(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);

	return show_int(buf, kcan_idx(pdev));
}

static ssize_t show_kcan_bitrate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.nominal.bitrate);
}

static ssize_t show_kcan_nsp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf,
			to_kcandev(dev)->init_settings.nominal.sample_point);
}

static ssize_t show_kcan_nom_tq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.nominal.tq);
}

static ssize_t show_kcan_nom_brp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.nominal.brp);
}

static ssize_t show_kcan_nom_tseg1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.nominal.tseg1);
}

static ssize_t show_kcan_nom_tseg2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.nominal.tseg2);
}

static ssize_t show_kcan_nom_sjw(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.nominal.sjw);
}

static ssize_t show_kcan_init_flags(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%08x\n",
					to_kcandev(dev)->init_settings.flags);
}

static ssize_t show_kcan_clock(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.clock_Hz);
}

static ssize_t show_kcan_bus_state(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->bus_state);
}

static ssize_t show_kcan_rx_err_cnt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->rx_error_counter);
}

static ssize_t show_kcan_tx_err_cnt(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->tx_error_counter);
}

static ssize_t show_kcan_bus_load(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	return snprintf(buf, PAGE_SIZE, "%u.%02u\n",
			pdev->bus_load / 100, pdev->bus_load % 100);
}

/* only when dev->adapter is not NULL! */
static ssize_t show_kcan_adapter_number(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	return show_int(buf, pdev && pdev->adapter ? pdev->adapter->index : -1);
}

static ssize_t show_kcan_adapter_name(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	int l = 0;

	if (pdev && pdev->adapter)
		l += snprintf(buf+l, PAGE_SIZE, "%s", pdev->adapter->name);
	else
		l += snprintf(buf+l, PAGE_SIZE, "pdev=%p adapter=%p",
			pdev, (pdev) ? pdev->adapter : NULL);

	buf[l++] = '\n';

	return l;
}

static ssize_t show_kcan_adapter_partnum(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	int l = 0;

	if (pdev->adapter && pdev->adapter->part_num)
		l += snprintf(buf+l, PAGE_SIZE, "%s", pdev->adapter->part_num);

	buf[l++] = '\n';

	return l;
}

static ssize_t show_kcan_adapter_version(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	int l = 0;

	if (pdev->hw_ver && pdev->hw_ver->major >= 0) {
		l += snprintf(buf+l, PAGE_SIZE, "%u", pdev->hw_ver->major);
		if (pdev->hw_ver->minor >= 0) {
			l += snprintf(buf+l, PAGE_SIZE, ".%u",
					pdev->hw_ver->minor);

			if (pdev->hw_ver->subminor >= 0)
				l += snprintf(buf+l, PAGE_SIZE, ".%u",
						pdev->hw_ver->subminor);
		}

		if (l >= PAGE_SIZE)
			l = PAGE_SIZE - 2;
	}

	buf[l++] = '\n';

	return l;
}

/* /proc/kcan redundant */
static ssize_t show_kcan_type(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	return show_str(buf, to_kcandev(dev)->type);
}

#ifdef NETDEV_SUPPORT
static ssize_t show_kcan_ndev(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	return show_str(buf, pdev->netdev ? pdev->netdev->name : "can?");
}
#else
static ssize_t show_rx_fifo_ratio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	u32 fifo_ratio = kcan_fifo_ratio(&pdev->rx_fifo);
	return snprintf(buf, PAGE_SIZE, "%u.%02u\n",
			fifo_ratio / 100, fifo_ratio % 100);
}
#endif

static ssize_t show_kcan_base(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%x\n", to_kcandev(dev)->dwPort);
}

static ssize_t show_tx_fifo_ratio(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	u32 fifo_ratio = kcan_fifo_ratio(&pdev->tx_fifo);
	return snprintf(buf, PAGE_SIZE, "%u.%02u\n",
			fifo_ratio / 100, fifo_ratio % 100);
}

static ssize_t show_clk_drift(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->time_sync.clock_drift);
}

#ifdef KCAN_FIX_INVALID_TS
static ssize_t show_ts_fixed(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->time_sync.ts_fixed);
}

static ssize_t show_time_lag(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->time_sync.evt_time_lag);
}
#endif

static ssize_t show_kcan_irq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->wIrq);
}

static ssize_t show_kcan_btr0btr1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	u16 dev_btr0btr1 = (pdev->sysclock_Hz == 8*MHz) ?
		kcan_bittiming_to_btr0btr1(&pdev->init_settings.nominal) :
		sja1000_bitrate(pdev->init_settings.nominal.bitrate,
				pdev->init_settings.nominal.sample_point,
				pdev->init_settings.nominal.sjw);

	return snprintf(buf, PAGE_SIZE, "0x%04x\n", dev_btr0btr1);
}

static ssize_t show_kcan_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
#ifdef NETDEV_SUPPORT
	struct net_device_stats *stats = (pdev->netdev) ?
				kcan_netdev_get_stats(pdev->netdev) : NULL;
	u32 dev_read = (stats) ? stats->rx_packets : 0;

#else
	u32 dev_read = pdev->rx_fifo.dwTotal;
#endif
	return show_u32(buf, dev_read);
}

static ssize_t show_kcan_write(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
#ifdef NETDEV_SUPPORT
	struct net_device_stats *stats = (pdev->netdev) ?
				kcan_netdev_get_stats(pdev->netdev) : NULL;
	u32 dev_write = (stats) ? stats->tx_packets : 0;

#else
	u32 dev_write = pdev->tx_fifo.dwTotal;
#endif
	return show_u32(buf, dev_write);
}

static ssize_t show_kcan_rx_frames_counter(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->rx_frames_counter);
}

static ssize_t show_kcan_tx_frames_counter(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->tx_frames_counter);
}

static ssize_t show_kcan_irqs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct kcandev *pdev = to_kcandev(dev);
	return show_u32(buf, pdev->rx_irq_counter +
			     pdev->tx_irq_counter);
}

static ssize_t show_kcan_rx_irqs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->rx_irq_counter);
}

static ssize_t show_kcan_tx_irqs(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->tx_irq_counter);
}

static ssize_t show_kcan_errors(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->dwErrorCounter);
}

static ssize_t show_kcan_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "0x%04x\n",
						to_kcandev(dev)->wCANStatus);
}

/* Note: 2nd arg is sysfs entry name */
static KCAN_DEVICE_ATTR_RW(devid, devid, show_kcan_devid, store_kcan_devid);
static KCAN_DEVICE_ATTR_RW(mass_storage_mode, mass_storage_mode,\
			   show_kcan_mass_storage_mode,\
			   store_kcan_mass_storage_mode);
static KCAN_DEVICE_ATTR_RW(led, led, show_kcan_identify, store_kcan_identify);
static KCAN_DEVICE_ATTR(hwtype, hwtype, show_kcan_hwtype);
#ifdef KCAN_SYSFS_TSMODE_RW
static KCAN_DEVICE_ATTR_RW(ts_mode, ts_mode, show_kcan_ts_mode, \
			   store_kcan_ts_mode);
#else
static KCAN_DEVICE_ATTR(ts_mode, ts_mode, show_kcan_ts_mode);
#endif
static KCAN_DEVICE_ATTR(serialno, serialno, show_kcan_sn);
static KCAN_DEVICE_ATTR(minor, minor, show_kcan_minor);
static KCAN_DEVICE_ATTR(dev_name, dev_name, show_kcan_dev_name);
static KCAN_DEVICE_ATTR(ctrlr_number, ctrlr_number, show_kcan_ctrlr_number);
static KCAN_DEVICE_ATTR(bitrate, nom_bitrate, show_kcan_bitrate);
static KCAN_DEVICE_ATTR(sample_point, nom_sample_point, show_kcan_nsp);
static KCAN_DEVICE_ATTR(nom_tq, nom_tq, show_kcan_nom_tq);
static KCAN_DEVICE_ATTR(nom_brp, nom_brp, show_kcan_nom_brp);
static KCAN_DEVICE_ATTR(nom_tseg1, nom_tseg1, show_kcan_nom_tseg1);
static KCAN_DEVICE_ATTR(nom_tseg2, nom_tseg2, show_kcan_nom_tseg2);
static KCAN_DEVICE_ATTR(nom_sjw, nom_sjw, show_kcan_nom_sjw);
static KCAN_DEVICE_ATTR(init_flags, init_flags, show_kcan_init_flags);
static KCAN_DEVICE_ATTR(clock, clock, show_kcan_clock);
static KCAN_DEVICE_ATTR(bus_state, bus_state, show_kcan_bus_state);
/* /proc/kcan redundant */
static KCAN_DEVICE_ATTR(type, type, show_kcan_type);
#ifdef NETDEV_SUPPORT
static KCAN_DEVICE_ATTR(ndev, ndev, show_kcan_ndev);
#else
static KCAN_DEVICE_ATTR(rx_fifo_ratio, rx_fifo_ratio, show_rx_fifo_ratio);
#endif
static KCAN_DEVICE_ATTR(base, base, show_kcan_base);
static KCAN_DEVICE_ATTR(irq, irq, show_kcan_irq);
static KCAN_DEVICE_ATTR(btr0btr1, btr0btr1, show_kcan_btr0btr1);
static KCAN_DEVICE_ATTR(read, read, show_kcan_read);
static KCAN_DEVICE_ATTR(write, write, show_kcan_write);
static KCAN_DEVICE_ATTR(errors, errors, show_kcan_errors);
static KCAN_DEVICE_ATTR(rx_frames_counter, rx_frames_counter, show_kcan_rx_frames_counter);
static KCAN_DEVICE_ATTR(tx_frames_counter, tx_frames_counter, show_kcan_tx_frames_counter);
static KCAN_DEVICE_ATTR(irqs, irqs, show_kcan_irqs);
static KCAN_DEVICE_ATTR(rx_irqs, rx_irqs, show_kcan_rx_irqs);
static KCAN_DEVICE_ATTR(tx_irqs, tx_irqs, show_kcan_tx_irqs);
static KCAN_DEVICE_ATTR(status, status, show_kcan_status);
static KCAN_DEVICE_ATTR(tx_fifo_ratio, tx_fifo_ratio, show_tx_fifo_ratio);
static KCAN_DEVICE_ATTR(clk_drift, clk_drift, show_clk_drift);
#ifdef KCAN_FIX_INVALID_TS
static KCAN_DEVICE_ATTR(ts_fixed, ts_fixed, show_ts_fixed);
static KCAN_DEVICE_ATTR(time_lag, time_lag, show_time_lag);
#endif
static struct attribute *kcan_dev_sysfs_attrs[] = {
	&kcan_dev_attr_hwtype.attr,
	&kcan_dev_attr_ts_mode.attr,
	&kcan_dev_attr_serialno.attr,
	&kcan_dev_attr_minor.attr,
	&kcan_dev_attr_dev_name.attr,
	&kcan_dev_attr_ctrlr_number.attr,
	&kcan_dev_attr_bitrate.attr,
	&kcan_dev_attr_sample_point.attr,
	&kcan_dev_attr_nom_tq.attr,
	&kcan_dev_attr_nom_brp.attr,
	&kcan_dev_attr_nom_tseg1.attr,
	&kcan_dev_attr_nom_tseg2.attr,
	&kcan_dev_attr_nom_sjw.attr,
	&kcan_dev_attr_init_flags.attr,
	&kcan_dev_attr_clock.attr,
	&kcan_dev_attr_bus_state.attr,
	/* /proc/kcan redundant */
	&kcan_dev_attr_type.attr,
#ifdef NETDEV_SUPPORT
	&kcan_dev_attr_ndev.attr,
#else
	&kcan_dev_attr_rx_fifo_ratio.attr,
#endif
	&kcan_dev_attr_base.attr,
	&kcan_dev_attr_irq.attr,
	&kcan_dev_attr_btr0btr1.attr,
	&kcan_dev_attr_read.attr,
	&kcan_dev_attr_write.attr,
	&kcan_dev_attr_rx_frames_counter.attr,
	&kcan_dev_attr_tx_frames_counter.attr,
	&kcan_dev_attr_errors.attr,
	&kcan_dev_attr_irqs.attr,
	&kcan_dev_attr_rx_irqs.attr,
	&kcan_dev_attr_tx_irqs.attr,
	&kcan_dev_attr_status.attr,
	&kcan_dev_attr_tx_fifo_ratio.attr,
	&kcan_dev_attr_clk_drift.attr,
#ifdef KCAN_FIX_INVALID_TS
	&kcan_dev_attr_ts_fixed.attr,
	&kcan_dev_attr_time_lag.attr,
#endif
	NULL
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 11, 0)
static struct attribute_group kcan_dev_attrs_group = {
	/* NULL ".name" => attrs will be created under kcanxxx node 
	 * .name = "kcan-dev", 
	 */
	.attrs = kcan_dev_sysfs_attrs,
};
static const struct attribute_group *kcan_dev_attrs_groups[] = {
	&kcan_dev_attrs_group,
	NULL,
};
#endif

static ssize_t show_kcan_dbitrate(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.data.bitrate);
}

static ssize_t show_kcan_dsp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.data.sample_point);
}

static ssize_t show_kcan_data_tq(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.data.tq);
}

static ssize_t show_kcan_data_brp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.data.brp);
}

static ssize_t show_kcan_data_tseg1(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.data.tseg1);
}

static ssize_t show_kcan_data_tseg2(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.data.tseg2);
}

static ssize_t show_kcan_data_sjw(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return show_u32(buf, to_kcandev(dev)->init_settings.data.sjw);
}

static KCAN_DEVICE_ATTR(guid, device_guid, show_device_guid);

static KCAN_DEVICE_ATTR(dbitrate, data_bitrate, show_kcan_dbitrate);
static KCAN_DEVICE_ATTR(dsample_point, data_sample_point, show_kcan_dsp);
static KCAN_DEVICE_ATTR(data_tq, data_tq, show_kcan_data_tq);
static KCAN_DEVICE_ATTR(data_brp, data_brp, show_kcan_data_brp);
static KCAN_DEVICE_ATTR(data_tseg1, data_tseg1, show_kcan_data_tseg1);
static KCAN_DEVICE_ATTR(data_tseg2, data_tseg2, show_kcan_data_tseg2);
static KCAN_DEVICE_ATTR(data_sjw, data_sjw, show_kcan_data_sjw);

static struct attribute *kcan_dev_sysfs_fd_attrs[] = {
	&kcan_dev_attr_dbitrate.attr,
	&kcan_dev_attr_dsample_point.attr,
	&kcan_dev_attr_data_tq.attr,
	&kcan_dev_attr_data_brp.attr,
	&kcan_dev_attr_data_tseg1.attr,
	&kcan_dev_attr_data_tseg2.attr,
	&kcan_dev_attr_data_sjw.attr,

	NULL
};

static KCAN_DEVICE_ATTR(bus_load, bus_load, show_kcan_bus_load);
static KCAN_DEVICE_ATTR(rx_err_cnt, rx_error_counter, show_kcan_rx_err_cnt);
static KCAN_DEVICE_ATTR(tx_err_cnt, tx_error_counter, show_kcan_tx_err_cnt);

static struct attribute *kcan_dev_sysfs_err_cnt_attrs[] = {
	&kcan_dev_attr_rx_err_cnt.attr,
	&kcan_dev_attr_tx_err_cnt.attr,

	NULL
};

static KCAN_DEVICE_ATTR(adapter_number, adapter_number,\
			show_kcan_adapter_number);
static KCAN_DEVICE_ATTR(adapter_name, adapter_name, show_kcan_adapter_name);
static KCAN_DEVICE_ATTR(adapter_partnum, adapter_partnum, show_kcan_adapter_partnum);
static KCAN_DEVICE_ATTR(adapter_version, adapter_version,\
			show_kcan_adapter_version);

static struct attribute *kcan_dev_sysfs_adapter_attrs[] = {
	&kcan_dev_attr_adapter_number.attr,
	&kcan_dev_attr_adapter_name.attr,
	&kcan_dev_attr_adapter_partnum.attr,
	&kcan_dev_attr_adapter_version.attr,

	NULL,
};

/* Create sysfs entries that need the whole devices list of an adapter to be
 * probed/created first.
 */
static void kcan_sysfs_dev_node_add_adapter(struct kcan_adapter *adapter)
{
	struct list_head *ptr;

#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	kcan_lock_get_irqsave(&kcan_drv.devices_lock, flags);
#endif

	list_for_each(ptr, &kcan_drv.devices) {
		struct kcandev *dev = list_entry(ptr, struct kcandev, list_dev);

		if (adapter != dev->adapter)
			continue;

		if (!dev->sysfs_dev)
			continue;

		/* KCAN-USB X6/X4: some interfaces can run newer FW than others.
		 * This means that GUID might be flashed only in the last
		 * interface. But it should be visible for all interfaces.
		 */
		if (dev->adapter->flags & KCAN_ADAPTER_GUID)
			kcan_sysfs_add_attr(dev->sysfs_dev,
					    &kcan_dev_attr_guid.attr);
	}

#ifdef HANDLE_HOTPLUG
	kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);
#endif
}

/* create a device node to export devices properties into /sys/class/kcan tree.
 */
void kcan_sysfs_dev_node_create_ex(struct kcandev *dev, struct device *parent)
{
	char tmp[32];

	/* tinker my device node name, eg. "kcanpci%d" */
	snprintf(tmp, sizeof(tmp), DEVICE_NAME "%s%s", dev->type, "%u");

#if defined(DEBUG_TRACE) || defined(DEBUG_SYSFS)
	pr_info(DEVICE_NAME ": %s(%p=%s, %d, %d)\n",
		__func__, dev, tmp, dev->nMajor, dev->nMinor);
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
	/* Should use WAIT_FOR key in Udev rules... */
	dev->sysfs_dev = device_create_drvdata(kcan_drv.class, parent,
				MKDEV(dev->nMajor, dev->nMinor),
				dev, tmp, dev->nMinor);

	if (!IS_ERR(dev->sysfs_dev)) {
		kcan_sysfs_add_attrs(dev->sysfs_dev, kcan_dev_sysfs_attrs);
#else
	/* since 3.11, it is possible to add attrs when creating the device
	 * node, that is *BEFORE* the UEVENT is being sent to userspace!
	 * Doing this, Udev rules does not need of WAIT_FOR key anymore!
	 */
	dev->sysfs_dev = device_create_with_groups(kcan_drv.class, parent,
				MKDEV(dev->nMajor, dev->nMinor),
				dev, kcan_dev_attrs_groups, tmp, dev->nMinor);

	if (!IS_ERR(dev->sysfs_dev)) {
#endif /* KERNEL_VERSION(3, 11, 0) */

#ifdef DEBUG_SYSFS
		pr_info(DEVICE_NAME ": %s(%p=\"%s\")\n", __func__, dev,
			dev->sysfs_dev->kobj.name);
#endif

		if (dev->features & KCAN_DEV_FD_RDY)
			kcan_sysfs_add_attrs(dev->sysfs_dev,
						kcan_dev_sysfs_fd_attrs);

		if (dev->features & KCAN_DEV_BUSLOAD_RDY)
			kcan_sysfs_add_attr(dev->sysfs_dev,
						&kcan_dev_attr_bus_load.attr);

		if (dev->option[KCANFD_OPT_DEVICE_ID].get)
			kcan_sysfs_add_attr(dev->sysfs_dev,
						&kcan_dev_attr_devid.attr);

		if (dev->features & KCAN_DEV_ERRCNT_RDY)
			kcan_sysfs_add_attrs(dev->sysfs_dev,
						kcan_dev_sysfs_err_cnt_attrs);

		if (dev->features & KCAN_DEV_MSD_RDY)
			kcan_sysfs_add_attr(dev->sysfs_dev,
					&kcan_dev_attr_mass_storage_mode.attr);

		if (dev->device_identify)
			kcan_sysfs_add_attr(dev->sysfs_dev,
					&kcan_dev_attr_led.attr);

		if (dev->sysfs_attrs)
			kcan_sysfs_add_attrs(dev->sysfs_dev, dev->sysfs_attrs);

		/* these attrs are are not used with Udev rules... */
		kcan_sysfs_add_attrs(dev->sysfs_dev,
				     kcan_dev_sysfs_adapter_attrs);

		/* create/update siblings with adapter specific entries if
		 * we are creating the last device of the adapter
		 */
		if (kcan_idx(dev) == (dev->adapter->can_count - 1))
			kcan_sysfs_dev_node_add_adapter(dev->adapter);
	} else {
#ifdef DEBUG_SYSFS
		pr_info(DEVICE_NAME ": %s(): device_create() failure err=%ld\n",
			__func__, (long )IS_ERR(dev->sysfs_dev));
#endif
		 dev->sysfs_dev = NULL;
	}
}

/* destroy a UDEV allocated device node */
void kcan_sysfs_dev_node_destroy(struct kcandev *dev)
{
#if defined(DEBUG_TRACE) || defined(DEBUG_SYSFS)
	pr_info(DEVICE_NAME ": %s(%p=\"%s\")\n", __func__, dev,
		(dev->sysfs_dev) ? dev->sysfs_dev->kobj.name : "NULL");
#endif
	if (!dev->sysfs_dev)
		return;

	if (dev->sysfs_attrs)
		kcan_sysfs_del_attrs(dev->sysfs_dev, dev->sysfs_attrs);

	if (dev->device_identify)
		kcan_sysfs_del_attr(dev->sysfs_dev, &kcan_dev_attr_led.attr);

	if (dev->features & KCAN_DEV_MSD_RDY)
		kcan_sysfs_del_attr(dev->sysfs_dev,
					&kcan_dev_attr_mass_storage_mode.attr);

	if (dev->features & KCAN_DEV_ERRCNT_RDY)
		kcan_sysfs_del_attrs(dev->sysfs_dev,
					kcan_dev_sysfs_err_cnt_attrs);

	if (dev->option[KCANFD_OPT_DEVICE_ID].get)
		kcan_sysfs_del_attr(dev->sysfs_dev,
					&kcan_dev_attr_devid.attr);

	if (dev->features & KCAN_DEV_BUSLOAD_RDY)
		kcan_sysfs_del_attr(dev->sysfs_dev,
					&kcan_dev_attr_bus_load.attr);

	if (dev->features & KCAN_DEV_FD_RDY)
		kcan_sysfs_del_attrs(dev->sysfs_dev,
					kcan_dev_sysfs_fd_attrs);

	if (dev->adapter->flags & KCAN_ADAPTER_GUID)
		kcan_sysfs_del_attr(dev->sysfs_dev,
					&kcan_dev_attr_guid.attr);

	kcan_sysfs_del_attrs(dev->sysfs_dev, kcan_dev_sysfs_adapter_attrs);

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 11, 0)
	kcan_sysfs_del_attrs(dev->sysfs_dev, kcan_dev_sysfs_attrs);
#endif
	device_destroy(kcan_drv.class, MKDEV(dev->nMajor, dev->nMinor));

	dev->sysfs_dev = NULL;
}

#else /* SYSFS_SUPPORT */

void kcan_sysfs_dev_node_create_ex(struct kcandev *dev, struct device *parent)
{
	dev->sysfs_dev = NULL;
}

void kcan_sysfs_dev_node_destroy(struct kcandev *dev)
{
}

#endif /* SYSFS_SUPPORT */

#include "kcan_main_linux.c"

#define DUMP_WIDTH	16
#ifdef DEBUG
#define DUMP_OUT	KERN_DEBUG
#else
#define DUMP_OUT	KERN_INFO
#endif

/*
 * void dump_mem(const char *prompt, const void *p, int l)
 */
void dump_mem(const char *prompt, const void *p, int l)
{
	printk(DUMP_OUT "%s: dumping %s (%d bytes):\n",
		DEVICE_NAME, prompt ? prompt : "memory", l);
	print_hex_dump(DUMP_OUT, DEVICE_NAME ": ", DUMP_PREFIX_NONE,
		       DUMP_WIDTH, 1, p, l, false);
}

void kcan_sync_init(struct kcandev *dev)
{
	memset(&dev->time_sync, '\0', sizeof(dev->time_sync));
}

/*
 * fill the pqm->msg.timestamp field from pqm->hwtv to give to user.
 *
 * This function is called from userland. Therefore, it can change the saved
 * timestamsp according to CURRENT dev->ts_mode (in case it has been changed
 * between the moment the event has been saved and the moment it is read).
 */
struct kcanfd_msg *kcan_sync_timestamps(struct kcandev *dev,
					struct kcanfd_rxmsg *pqm)
{
	struct timeval now;
	int fix_ts = 0;
	struct kcanfd_msg *pm = &pqm->msg;
	int s = 1;
	u64 dus;
	signed long dtv;

	/* fix hw timestamp */
	pm->flags |= KCANFD_TIMESTAMP|KCANFD_HWTIMESTAMP;

#if !KCAN_ENABLE_TIME_SYNC
	pm->timestamp.tv_usec = do_div(pqm->hwtv.ts_us, USEC_PER_SEC);
	pm->timestamp.tv_sec = (__kernel_time_t )pqm->hwtv.ts_us;
	return pm;
#endif

	/* 		pqm->ts_mode	dev->ts_mode
	 * -----------------------------------------
	 * _OFF		(2)		x
	 * _ON		x		x
	 * _COOKED	x		x
	 * _RAW		(2)		(1)
	 */
	switch (dev->ts_mode) {
	case KCANFD_OPT_HWTIMESTAMP_RAW:

		/* (1): if dev->ts_mode is RAW, we can give the raw hw
		 * timestamps, whatever pqm->hwtv.ts_mode was at push time:
		 */
		pm->timestamp.tv_usec = do_div(pqm->hwtv.ts_us, USEC_PER_SEC);
		pm->timestamp.tv_sec = (__kernel_time_t )pqm->hwtv.ts_us;

#ifdef DEBUG_TS
#ifdef DEBUG_TS_HWTYPE
		if (dev->wType == DEBUG_TS_HWTYPE)
#endif
			pr_info(DEVICE_NAME
				": RAW CAN%u %llu %ld.%06ld\n",
				kcan_idx(dev)+1, pqm->hwtv.ts_us,
				pm->timestamp.tv_sec, pm->timestamp.tv_usec);
#endif
		return pm;

	default:
		break;
	}

	/* Init timestamp with last host (sync) time */
	pm->timestamp = pqm->hwtv.tv;

	switch (pqm->hwtv.ts_mode) {
	case KCANFD_OPT_HWTIMESTAMP_RAW:
	case KCANFD_OPT_HWTIMESTAMP_OFF:

		/* (2): we only have event host time =>
		 * don't set KCANFD_HWTIMESTAMP so user will know that it's
		 * host time only.
		 */
		pm->flags &= ~KCANFD_HWTIMESTAMP;

#ifdef DEBUG_TS
#ifdef DEBUG_TS_HWTYPE
		if (dev->wType == DEBUG_TS_HWTYPE)
#endif
			pr_info(DEVICE_NAME
				": OFF CAN%u %llu %ld.%06ld\n",
				kcan_idx(dev)+1, pqm->hwtv.ts_us,
				pm->timestamp.tv_sec, pm->timestamp.tv_usec);
#endif

		return pm;

	default:
		break;
	}

	/* KCANFD_HWTIMESTAMP is always set now: whever dev->ts_mode is, user
	 * will now that read timestamp is made of hw timestamps.
	 *
	 * ts_us = hw timestamp of the event
	 * tv_us = hw timestamp of last sync
	 * => dus = count of hw µs since last sync
	 *
	 * note: PCIe card sync ts is always >= event ts while
	 * USB CAN sync ts is always <= event ts
	 *
	 * note: because the sync might contain a greater timestamp than
	 * then event timestamp, should check the values before diff:
	 */
	if (unlikely(pqm->hwtv.ts_us < pqm->hwtv.tv_us)) {
		dus = pqm->hwtv.tv_us - pqm->hwtv.ts_us;
		s = -1;
	} else {
		dus = pqm->hwtv.ts_us - pqm->hwtv.tv_us;
	}

	/* Note: dus == 0 means that this event is the first resync'ed */
	if (!dus) {
		dev->time_sync.evt_time_lag = 0;

#ifdef KCAN_HANDLE_CLOCK_DRIFT
	//} else if (pqm->hwtv.ts_mode == KCANFD_OPT_HWTIMESTAMP_COOKED &&
	} else if (dev->ts_mode == KCANFD_OPT_HWTIMESTAMP_COOKED &&
	    pqm->hwtv.clock_drift &&
	    pqm->hwtv.clock_drift != KCAN_CLOCK_DRIFT_SCALE) {

		dus <<= KCAN_CLOCK_DRIFT_SCALE_SHIFT;

		/* - The smaller clock_drift is, the larger dus goes:
		 *   For example, a 60 s hardware dus becomes:
		 *
		 *   clock_drift	dus (host time)
		 *   131069		60.001.373 us
		 *   131070		60.000.915 us
		 *   131071		60.000.457 us
		 *
		 *   This means that roughly 1/2 ms per mn can be lost (or win)
		 *   if the clock drift is not accurate...
		 */
		do_div(dus, pqm->hwtv.clock_drift);
#endif
	}

	/* add number of us since last sync as well as measured time lag
	 * (the larger evt_time_lag is, the further back in time we go)
	 */
	dtv = s * dus - dev->time_sync.evt_time_lag;

	/* timeval_add_us() returns valid tv_sec and tv_usec */
	timeval_add_us(&pm->timestamp, dtv);

#ifdef DEBUG_TS
#ifdef DEBUG_TS_HWTYPE
	if (dev->wType == DEBUG_TS_HWTYPE)
#endif
		pr_info(DEVICE_NAME
			": COOK CAN%u %llu %ld.%06ld\n",
			kcan_idx(dev)+1, pqm->hwtv.ts_us,
			pm->timestamp.tv_sec, pm->timestamp.tv_usec);
#endif

	kcan_gettimeofday(&now);
	dtv = timeval_diff(&pm->timestamp, &now);
	if (dtv > 0) {

		fix_ts++;

		/* If the timestamp is more than 100 µs from the future, then
		 * it is fixed to now and the KCANFD_HWTIMESTAMP flag is
		 * removed. Otherwise, it is not.
		 */
		if (dtv >= 100) {
#ifdef DEBUG_INVALID_TS
			pr_info(DEVICE_NAME ": WARNING: ts too far from now "
				"(%ld us)\n", dtv);
#endif
			fix_ts++;
		}

		dev->time_sync.evt_time_lag += dtv;

	/* not safe to keep a too large time_lag. This version decrements it
	 * each time a timestamp is ok.
	 */
	} else if (dev->time_sync.evt_time_lag >= 10) {
		dev->time_sync.evt_time_lag -= 10;
	}

	if (fix_ts) {

		if (fix_ts > 1) {
#ifdef DEBUG_INVALID_TS
			pr_err(DEVICE_NAME ": WARNING: %s ts fixed(%d) from: "
			       "%ld.%06ld to: %ld.%06ld (now)\n",
				dev->adapter->name, fix_ts,
				pm->timestamp.tv_sec,
				pm->timestamp.tv_usec,
				now.tv_sec, now.tv_usec);

			pr_info(DEVICE_NAME ": ts[base=%llu off=%llu cd=%lu]\n",
				pqm->hwtv.tv_us, pqm->hwtv.ts_us,
				pqm->hwtv.clock_drift);
#endif
			pm->flags &= ~KCANFD_HWTIMESTAMP;
			dev->time_sync.ts_fixed++;
		}

		pm->timestamp = now;

#ifdef DEBUG_TS
#ifdef DEBUG_TS_HWTYPE
		if (dev->wType == DEBUG_TS_HWTYPE)
#endif
			pr_info(DEVICE_NAME ": FIX CAN%u %llu %ld.%06ld %u\n",
				kcan_idx(dev)+1, pqm->hwtv.ts_us,
				pm->timestamp.tv_sec,
				pm->timestamp.tv_usec,
				fix_ts);
#endif /* DEBUG_TS */
	}

	return pm;
}

int kcan_sync_decode(struct kcandev *dev, u32 ts_low, u32 ts_high,
						struct kcan_timeval *hwtv)
{
	hwtv->ts_us = ((u64 )ts_high << 32) | ts_low;

#if !KCAN_ENABLE_TIME_SYNC
	hwtv->ts_mode = KCANFD_OPT_HWTIMESTAMP_RAW;
	kcan_gettimeofday(&hwtv->tv);
	return 1;
#endif

	hwtv->ts_mode = dev->ts_mode;

#ifdef DEBUG_TS
#ifdef DEBUG_TS_HWTYPE
	if (dev->wType == DEBUG_TS_HWTYPE)
#endif
		pr_info(DEVICE_NAME
			": DEC CAN%u %llu %llu %u\n",
			kcan_idx(dev)+1,
			hwtv->ts_us, dev->time_sync.evt_base_us,
			hwtv->ts_mode);
#endif

	switch (hwtv->ts_mode) {
	default:
#ifdef KCAN_RESET_TIME_SYNC_PERIOD
		/* this is a proof of activity */
		dev->time_sync.cm_inactivity_count = 0;
#endif
		hwtv->tv_us = dev->time_sync.evt_base_us;

		/* sync is not started yet: use host time instead */
		if (!hwtv->tv_us) {

			hwtv->tv_us = dev->time_sync.evt_base_us = hwtv->ts_us;

			kcan_gettimeofday(&dev->time_sync.evt_base_tv);
		}

		/* otherwise, save all the necessary stuff for user to compute
		 * host (cooked) timestamp later...
		 */
		hwtv->tv = dev->time_sync.evt_base_tv;
		hwtv->clock_drift = dev->time_sync.clock_drift;

		return 1;

	case KCANFD_OPT_HWTIMESTAMP_RAW:
		/* keep a trace of the (host) time for further use in case
		 * dev->ts_mode changed meanwhile...
		 */
		kcan_gettimeofday(&hwtv->tv);
		return 1;

	case KCANFD_OPT_HWTIMESTAMP_OFF:
		kcan_gettimeofday(&hwtv->tv);
		break;
	}

	hwtv->ts_mode = KCANFD_OPT_HWTIMESTAMP_OFF;

	return 0;
}

/* called to synchronize host time and hardware time
 * Note:
 * - USB-FD devices sync every 1s.
 * - PCIe-FD devices sync every 1,2 ms
 */
int kcan_sync_times_host(struct kcandev *dev, u32 ts_low, u32 ts_high,
			 int tv_off, const struct timeval *host_tv,
			 u64 host_tv_ns)
{
#if !KCAN_ENABLE_TIME_SYNC
	(void)dev;
	(void)ts_low;
	(void)ts_high;
	(void)tv_off;
	(void)host_tv;
	(void)host_tv_ns;
	return 0;
#else
	int err = 0;
	long dts_us = 0, dtv_us = 0;
	long sync_err_us = 0;
	bool first_sync = false;
	struct kcan_time_sync now = {
		.ts_us = ((u64 )ts_high << 32) + ts_low,
		.clock_drift = dev->time_sync.clock_drift,

		.evt_base_us = dev->time_sync.evt_base_us,
		.evt_base_tv = dev->time_sync.evt_base_tv,
		.evt_time_lag = dev->time_sync.evt_time_lag,
		.cm_inactivity_count = dev->time_sync.cm_inactivity_count + 1,
		.ts_fixed = dev->time_sync.ts_fixed,
	};

	if (host_tv) {
		now.tv = *host_tv;
		now.tv_ns = host_tv_ns;
	} else {
		kcan_gettimeofday_ex(&now.tv, &now.tv_ns);
	}

	if (!dev->time_sync.ts_us) {
		first_sync = true;

		/* get host time between substract any host time offset */
		timeval_add_us(&now.tv, tv_off);

		now.clock_drift = KCAN_CLOCK_DRIFT_SCALE;
		goto lbl_reset;
	}

	/* WARNING: CLOCK_REALTIME may generate negative dtv_us!!! */
	dtv_us = timeval_to_us(&now.tv) - timeval_to_us(&dev->time_sync.tv);
	if (dtv_us <= 0) {

		/* in that case, do ignore that sync tick so that next one
		 * will hopefully get a greater now.tv value...
		 */
		err = 1;
		goto lbl_exit;
	}

	/* do clock_drift computation as soon as possible */
	/* dts_us = count of µs in hardware time between 2x calls */
	/* be sure of this (and protect from several calls with
	 * same ts_high.ts_low that would change value of dev->time_sync.tv)
	 */
	dts_us = now.ts_us - dev->time_sync.ts_us;
	if (dts_us < 0) {
#ifdef DEBUG
		/* silently discard that call when calibration messages are
		 * ignored (<=> now.ts_us = 0). In this case, sync is done on
		 * events timestamps only. clock_drift is always computed.
		 */
		if (now.ts_us)
			pr_warn(DEVICE_NAME ": %s CAN%u: abnormal sync times: "
				"ts=%llu < ts=%llu\n",
				dev->adapter->name, kcan_idx(dev)+1,
				now.ts_us, dev->time_sync.ts_us);
#endif
		err = 2;
		goto lbl_exit;
	}

	/* get host time between each sync and substract any host time offset */
	if (unlikely(tv_off < 0)) {
		timeval_add_us(&now.tv, tv_off);
		dtv_us += tv_off;
	}

	sync_err_us = dtv_us - dts_us;

	if (dtv_us <= 0) {

		if (dtv_us < 0)
			pr_warn(DEVICE_NAME
				": %s CAN%u: abnormal sync times: "
				"tv=%ld.%06ld < tv=%ld.%06ld dtv_us=%ld "
				"tv_off=%d. Did host time change?\n",
				dev->adapter->name, kcan_idx(dev)+1,
				now.tv.tv_sec, now.tv.tv_usec,
				dev->time_sync.tv.tv_sec,
				dev->time_sync.tv.tv_usec,
				dtv_us, tv_off);

		err = 3;
		goto lbl_exit;
	}

#ifdef KCAN_HANDLE_CLOCK_DRIFT
	/* clock_drift computation is critial:
	 *
	 * A 60 s hardware difference time can change up to ~1 ms if the clock
	 * drift differs from 131072 to 131070:
	 *
	 *   clock_drift	dus (host time)
	 *   131069		60.001.373 us
	 *   131070		60.000.915 us
	 *   131071		60.000.457 us
	 *   131072		60.000.000 us
	 *
	 * Therefore, should reset counters used to compute clock drift
	 * periodically:
	 */
	if (dev->time_sync.ttv_us > 120*USEC_PER_SEC) {
		dev->time_sync.ttv_us = 0;
		dev->time_sync.tts_us = 0;
	}

	/* Now, calculate clock drift by comparing count of µs in both
	 * times:
	 */

	/* ttv_us = count of host µs since start of sync */
	now.ttv_us = dev->time_sync.ttv_us + dtv_us;

	/* tts_us = count of hw µs since start of sync */
	now.tts_us = dev->time_sync.tts_us + dts_us;

	/* prevent from div by 0 */
	if (now.ttv_us) {

		/* this test is useless if ttv_us and tts_us are periodically
		 * reset to 0
		 */
		u64 d = now.tts_us << KCAN_CLOCK_DRIFT_SCALE_SHIFT;
		int diff;

		/* DIV_ROUND_DOWN_ULL gives 131069 => time_lag grows up
		 * DIV_ROUND_UP_SECTOR_T gives 131070 => time_lag does
		 * not change but may be too large...
		 */
		now.clock_drift = DIV_ROUND_UP_SECTOR_T(d, now.ttv_us);
		//now.clock_drift = DIV_ROUND_DOWN_ULL(d, now.ttv_us);

		/* protect from crazy values: accept clock drift changes
		 * in the range [-8 ; +8] only.
		 */
		diff = abs(now.clock_drift - dev->time_sync.clock_drift);
		if (diff > 16) {
#ifdef DEBUG_TS
			pr_info(DEVICE_NAME
				": %s CAN%u: ABNORMAL clock drift value %ld "
				"regarding current one %ld\n",
				dev->adapter->name, kcan_idx(dev)+1,
				now.clock_drift,
				dev->time_sync.clock_drift);
#endif
			diff = 16;
		}

		/* otherwise, smoothly change to the new value */
		if (diff > 4) {
			now.clock_drift += dev->time_sync.clock_drift;
			now.clock_drift /= 2;
		}
	}

#endif /* KCAN_HANDLE_CLOCK_DRIFT */

lbl_reset:
	/* if clock drift changed, then reset event time sync to compute
	 * new time_lag
	 */
	if (now.clock_drift != dev->time_sync.clock_drift) {
		now.evt_base_us = 0;
	}

	dev->time_sync = now;

	pr_info(DEVICE_NAME
		": sync-detail CAN%u first=%d ts_us=%llu host_dt_us=%ld dev_dt_us=%ld diff_us=%ld tv_off=%d drift=%lu lag=%u\n",
		kcan_idx(dev)+1, first_sync ? 1 : 0,
		(unsigned long long)dev->time_sync.ts_us,
		dtv_us, dts_us, sync_err_us, tv_off,
		dev->time_sync.clock_drift,
		dev->time_sync.evt_time_lag);

#ifdef KCAN_RESET_TIME_SYNC_PERIOD
	if (dev->time_sync.cm_inactivity_count > KCAN_RESET_TIME_SYNC_PERIOD) {
		dev->time_sync.cm_inactivity_count = 0;

		/* inactivity period: reset event time sync */
		dev->time_sync.evt_base_us = 0;
	}
#endif

lbl_exit:
#ifdef DEBUG_TS
#ifdef DEBUG_TS_HWTYPE
	if (dev->wType == DEBUG_TS_HWTYPE)
#endif
	{
		if (!err)
			pr_info(DEVICE_NAME
				": SYNC CAN%u %llu %ld.%06ld %lu %lu %ld\n",
				kcan_idx(dev)+1,
				(unsigned long long)dev->time_sync.ts_us,
				dev->time_sync.tv.tv_sec,
				dev->time_sync.tv.tv_usec,
				dtv_us, dts_us,
				dev->time_sync.clock_drift);
		else
			pr_info(DEVICE_NAME
				": SYNC CAN%u %d\n",
				kcan_idx(dev)+1,  err);
	}
#endif

	return !err;
#endif
}

/* Add rxerr/txerr to posted msg *ONLY* if they are consistant with current
 * bus state
 */
static int kcan_fix_msg_errcnt(struct kcandev *dev, struct kcanfd_rxmsg *rx)
{
	/* default is: no counters given to userspace */
	rx->msg.flags &= ~KCANFD_ERRCNT;

	if (!(dev->features & KCAN_DEV_ERRCNT_RDY))
		return -EACCES;

	/* don't post if rx/tx counters don't match current bus state.
	 * The uCAN STATUS event should arrive next...
	 */
	switch (dev->bus_state) {
	case KCANFD_ERROR_ACTIVE:
		if ((dev->rx_error_counter >= 96) ||
					(dev->tx_error_counter >= 96))
			return -EINVAL;

		break;
	case KCANFD_ERROR_WARNING:
		if ((dev->rx_error_counter >= 128) ||
					(dev->tx_error_counter >= 128))
			return -EINVAL;

		if ((dev->rx_error_counter < 96) &&
					(dev->tx_error_counter < 96))
			return -EINVAL;

		break;
	case KCANFD_ERROR_PASSIVE:
		if ((dev->rx_error_counter < 128) &&
					(dev->tx_error_counter < 128))
			return -EINVAL;
	default:
		break;
	}

	rx->msg.ctrlr_data[KCANFD_RXERRCNT] = dev->rx_error_counter;
	rx->msg.ctrlr_data[KCANFD_TXERRCNT] = dev->tx_error_counter;

	rx->msg.flags |= KCANFD_ERRCNT;

	return 0;
}



/* x = (x >= y) ? x - y : 0; */
static int subtract_timeval(struct timeval *x, struct timeval *y)
{
	if (x->tv_usec >= y->tv_usec)
		x->tv_usec -= y->tv_usec;
	else {
		if (x->tv_sec) {
			x->tv_sec--;
			x->tv_usec += (USEC_PER_SEC - y->tv_usec);
		} else
			goto fail;
	}

	if (x->tv_sec >= y->tv_sec) {
		x->tv_sec -= y->tv_sec;
		return 1;
	}

fail:
	return 0;
}

/* get relative time to start of driver */
static void __maybe_unused to_drv_rel_time(struct timeval *tv)
{
	if (!subtract_timeval(tv, &kcan_drv.sInitTime)) {

#ifdef DEBUG
		pr_info(DEVICE_NAME
			": WARNING: \"%u.%06u s.\" < drv \"%u.%06u s.\"\n",
			(u32 )tv->tv_sec, (u32 )tv->tv_usec,
			(u32 )kcan_drv.sInitTime.tv_sec,
			(u32 )kcan_drv.sInitTime.tv_usec);
#endif
		tv->tv_sec = tv->tv_usec = 0;
	}
}

/* get relative time to start of device */
static void __maybe_unused to_dev_rel_time(struct kcandev *dev, struct timeval *tv)
{
	if (!subtract_timeval(tv, &dev->init_timestamp)) {

#ifdef DEBUG
		pr_info(DEVICE_NAME
			": WARNING: \"%u.%06u s.\" < dev \"%u.%06u s.\"\n",
			(u32 )tv->tv_sec, (u32 )tv->tv_usec,
			(u32 )dev->init_timestamp.tv_sec,
			(u32 )dev->init_timestamp.tv_usec);
#endif
		tv->tv_sec = tv->tv_usec = 0;
	}
}

#ifndef NETDEV_SUPPORT

/* this function is used to prevent from flooding rx fifo with STATUS msgs 
 * given by the hardware, especially when this hw is able to give rx and tx
 * error counters: when bus goes into warning or passive states, there can be 
 * a lot a STATUS msgs pushed in the rx fifo. If no task is reading this fifo
 * fast enough, it can be quickly full, preventing from CAN frames to be pushed.
 */
static int kcan_do_patch_rxmsg(void *item, void *arg)
{
	struct kcanfd_rxmsg *fifo = (struct kcanfd_rxmsg *)item;
	struct kcanfd_rxmsg *rx = (struct kcanfd_rxmsg *)arg;

	/* if last msg is also the same status msg, excepting counters value,
	 * it can be overwritten
	 */
	if ((fifo->msg.type == rx->msg.type) && /* same STATUS msg */
	    (fifo->msg.id == rx->msg.id) &&		 /* same id. */
	    (fifo->msg.flags == rx->msg.flags) && /* same flags */
	    (fifo->msg.data_len == rx->msg.data_len) /* same content len */
	   ) {
		/* patch fifo item with the new message, that is:
		 * - new timestamp
		 * - new data bytes
		 */
		*fifo = *rx;

#ifdef DEBUG_PATCH
		pr_info(DEVICE_NAME ": %s(): STATUS[%u] patched\n",
				__func__, fifo->msg.id);
#endif
		return -EEXIST;
	}

	/* MUST return != 0 to only process last item */
	return -ENOENT;
}

static int kcan_status_rx(struct kcandev *dev, struct kcanfd_rxmsg *rx)
{
	int rxerr = dev->posted.rxerr;
	int txerr = dev->posted.txerr;

#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_STATE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u, rx[id=%u]) "
			"bus=%u rx=%u/%u tx=%u/%u\n",
			__func__, dev->adapter->name, kcan_idx(dev)+1,
			rx->msg.id, dev->posted.bus_state,
			dev->rx_error_counter, dev->posted.rxerr,
			dev->tx_error_counter, dev->posted.txerr);
#endif

#ifdef KCAN_RUN_AS_OLD_API_DID
	/* consider notifying ACTIVE only if ioctl(SET_INIT) has been called */
	if ((rx->msg.id == KCANFD_ERROR_ACTIVE) &&
		(!(dev->init_settings.flags & KCANFD_INIT_USER))) {
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": %s CAN%u: "
			"!KCANFD_INIT_USER => STATUS[ACTIVE] discarded\n",
			dev->adapter->name, kcan_idx(dev)+1);
#endif
		return -EEXIST;
	}
#endif

	/* donot post any STATUS msg if user already knows it */
	if (rx->msg.id != dev->posted.bus_state) {
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME
			": %s CAN%u: msg.id %d != %d: post-it!\n",
			dev->adapter->name, kcan_idx(dev)+1,
			rx->msg.id, dev->posted.bus_state);
#endif
		goto lbl_post_it;
	}

	if (rx->msg.flags & KCANFD_ERRCNT) {
		rxerr = rx->msg.ctrlr_data[KCANFD_RXERRCNT];
		txerr = rx->msg.ctrlr_data[KCANFD_TXERRCNT];

		if (rxerr != dev->posted.rxerr) {
#ifdef DEBUG_BUS_STATE
			pr_info(DEVICE_NAME
				": %s CAN%u: rxerr %d != %d: post-it!\n",
				dev->adapter->name, kcan_idx(dev)+1,
				rxerr, dev->posted.rxerr);
#endif
			goto lbl_post_it;
		}

		if (txerr != dev->posted.txerr) {
#ifdef DEBUG_BUS_STATE
			pr_info(DEVICE_NAME
				": %s CAN%u: txerr %d != %d: post-it!\n",
				dev->adapter->name, kcan_idx(dev)+1,
				txerr, dev->posted.txerr);
#endif
			goto lbl_post_it;
		}
	}

	return -EEXIST;

lbl_post_it:

	/* keep a copy of last STATUS posted */
	dev->posted.bus_state = rx->msg.id;
	dev->posted.rxerr = rxerr;
	dev->posted.txerr = txerr;

	/* complete STATUS message before posting it */
	memset(rx->msg.data, '\0', sizeof(rx->msg.data));
	rx->msg.data_len = 0;

	return 0;
}

static int kcan_error_rx(struct kcandev *dev, struct kcanfd_rxmsg *rx)
{
	int rxerr = dev->posted.rxerr;
	int txerr = dev->posted.txerr;

	/* donot post any ERROR msg twice */
	if (rx->msg.id != dev->posted.bus_error)
		goto lbl_post_it;

	if (rx->msg.flags & KCANFD_ERRCNT) {
		rxerr = rx->msg.ctrlr_data[KCANFD_RXERRCNT];
		txerr = rx->msg.ctrlr_data[KCANFD_TXERRCNT];

		if (rxerr != dev->posted.rxerr)
			goto lbl_post_it;

		if (txerr != dev->posted.txerr)
			goto lbl_post_it;
	}

	return -EEXIST;

lbl_post_it:

	/* keep a copy of last ERROR posted */
	dev->posted.bus_error = rx->msg.id;
	dev->posted.rxerr = rxerr;
	dev->posted.txerr = txerr;

	return 0;
}

static void kcan_status_normalize_rx(struct kcandev *dev,
				     struct kcanfd_rxmsg *rx)
{
	u8 rxcnt, txcnt;

	if (!(rx->msg.flags & KCANFD_ERRCNT))
		return;

	rxcnt = rx->msg.ctrlr_data[KCANFD_RXERRCNT];
	txcnt = rx->msg.ctrlr_data[KCANFD_TXERRCNT];

	switch (rx->msg.id) {

	case KCANFD_ERROR_ACTIVE:
		if ((rxcnt < 96) && (txcnt < 96))
			break;

		rx->msg.id = KCANFD_ERROR_WARNING;

		/* fall through */
		fallthrough;
	case KCANFD_ERROR_WARNING:
		if ((rxcnt < 128) && (txcnt < 128))
			break;

		rx->msg.id = KCANFD_ERROR_PASSIVE;

		/* fall through */
		fallthrough;
	case KCANFD_ERROR_PASSIVE:
	default:
		break;
	}
}

#ifdef FIFO_PRE_ROUTINE
/*
 * int msgcmp(struct kcanfd_msg *rx1, struct kcanfd_msg *rx2)
 *
 * Compare two messages.
 *
 * @RETURN:
 *
 *	0	message are equal
 */
static int msgcmp(struct kcanfd_msg *msg1, struct kcanfd_msg *msg2)
{
	if ((msg1->type != msg2->type) ||
	    (msg1->id != msg2->id) ||
	    (msg1->flags != msg2->flags))
		return 1;

	if (msg1->flags & KCANFD_ERRCNT) {

		if (msg1->ctrlr_data[KCANFD_RXERRCNT] !=
		    msg2->ctrlr_data[KCANFD_RXERRCNT])
			return 2;

		if (msg1->ctrlr_data[KCANFD_TXERRCNT] !=
		    msg2->ctrlr_data[KCANFD_TXERRCNT])
			return 2;
	}

	if (msg1->flags & KCANFD_BUSLOAD) {

		if (msg1->ctrlr_data[KCANFD_BUSLOAD_UNIT] !=
		    msg2->ctrlr_data[KCANFD_BUSLOAD_UNIT])
			return 3;

		if (msg1->ctrlr_data[KCANFD_BUSLOAD_DEC] !=
		    msg2->ctrlr_data[KCANFD_BUSLOAD_DEC])
			return 3;
	}

	if (msg1->flags & KCANFD_OVRCNT) {

		if (msg1->ctrlr_data[KCANFD_RXERRCNT] !=
		    msg2->ctrlr_data[KCANFD_RXERRCNT])
			return 4;
	}

	return 0;
}
#endif

/*
 * This function patches the last msg pushed into the fifo with
 * STATUS[RX_OVERFLOW] (and increase the count of lost msgs in), in case msg
 * item cannot be put into the device rx fifo.
 *
 * WARNING:
 * - this function is called while the fifo is locked.
 * - its return value is returned by kcan_fifo_put_ex().
 * - if err == 0, the function is called BEFORE pushing the item. In that case
 *   any return value != 0 prevents from the item to be pushed.
 */
static int kcan_rxfifo_put_completion(FIFO_MANAGER *anchor,
				      void *item,
				      void *arg,
				      int err)
{
	struct kcanfd_rxmsg *rx = (struct kcanfd_rxmsg *)item;
	struct kcandev *dev = (struct kcandev *)arg;
	struct kcanfd_rxmsg *fifo;

#ifdef FIFO_PRE_ROUTINE
	/* pre routine: return != 0 to not push the item */
	if (!err) {

		/* check whether pushed msg is the same than the last read */
		switch (rx->msg.type) {

		case KCANFD_TYPE_NOP:
			/* don't post NOP */
			err = sizeof(*rx);
			break;

		case KCANFD_TYPE_STATUS:
			err = !msgcmp(&rx->msg, &dev->posted.status.msg);
			break;

		case KCANFD_TYPE_ERROR_MSG:
			err = !msgcmp(&rx->msg, &dev->posted.error.msg);
			break;
		}

		return err;
	}
#endif
#ifdef DEBUG_BUS_STATE
	if (rx->msg.type == KCANFD_TYPE_STATUS) {
		pr_info(DEVICE_NAME ": %s CAN%u: STATUS[id=%d] pushed\n",
			dev->adapter->name, kcan_idx(dev)+1, rx->msg.id);
	}
#endif

	/* simply ignore calls to that callback if fifo is not full */
	if (err != -ENOSPC)
		return err;

	/* To avoid filing kernel log buffer, display a msg *ONLY* if
	 * CAN_ERR_OVERRUN is not set
	 */
	if (!(dev->wCANStatus & CAN_ERR_OVERRUN)) {
		pr_err(DEVICE_NAME ": %s CAN%u: msg[type=%d id=%xh l=%u] lost: "
		       "err %d rxqsize=%u\n",
		       dev->adapter->name, kcan_idx(dev)+1,
		       rx->msg.type, rx->msg.id, rx->msg.data_len,
		       err, rxqsize);
	}

	/* rewind to last msg in fifo */
	fifo = (anchor->w == anchor->bufferBegin)
			? anchor->bufferEnd
			: anchor->w - anchor->wStepSize;

	/* if last msg is not STATUS[RX_OVERFLOW] then patch it and initialize
	 * counter of lost msg to 0 (or 1)
	 */
	if ((fifo->msg.type != KCANFD_TYPE_STATUS) ||
	    (fifo->msg.id != KCANFD_RX_OVERFLOW) ||
	    !(fifo->msg.flags & KCANFD_ERROR_INTERNAL)) {

		__u16 fifo_type = fifo->msg.type;

		/* Note: this set device status OVERRUN bit while in a critical
		 * section
		 */
		kcan_handle_error_internal(dev, fifo, KCANFD_RX_OVERFLOW);

		fifo->msg.flags |= KCANFD_OVRCNT;
		fifo->msg.ctrlr_data[KCANFD_RXERRCNT] = 0;

		fifo->hwtv.ts_mode = KCANFD_OPT_HWTIMESTAMP_OFF;
		kcan_gettimeofday(&fifo->hwtv.tv);

		/* if msg being patched was a CAN frame, then this msg is lost
		 * too
		 */
		switch (fifo_type) {
		case KCANFD_TYPE_CAN20_MSG:
		case KCANFD_TYPE_CANFD_MSG:
			fifo->msg.ctrlr_data[KCANFD_RXERRCNT]++;
			break;
		}
	}

	/* if message lost was a CAN frame, then incr count of lost CAN msg */
	switch (rx->msg.type) {
	case KCANFD_TYPE_CAN20_MSG:
	case KCANFD_TYPE_CANFD_MSG:
		fifo->msg.ctrlr_data[KCANFD_RXERRCNT]++;
		break;
	}

	/* task won't need to be signaled next because it has already been when
	 * (last) fifo msg was pushed
	 */
	return 0;
}

/*
 * Fix timestamp in msg before pushing it into Rx fifo.
 */
static void kcan_fix_msg_timestamp(struct kcandev *dev, struct kcanfd_rxmsg *rx)
{
	/* if no timestamp in this message, put current time in it */
	if (!(rx->msg.flags & KCANFD_TIMESTAMP)) {
		/* this hw does not provide any hw timestamp: use time of day */
		rx->hwtv.ts_mode = KCANFD_OPT_HWTIMESTAMP_OFF;
		kcan_gettimeofday(&rx->hwtv.tv);
	}

	/* default kcan gives timestamp relative to
	 * the time the driver has been loaded.
	 * Note: raw timestamsp cannot be changed according to any time base
	 */
	if (rx->hwtv.ts_mode != KCANFD_OPT_HWTIMESTAMP_RAW) {

		switch (dev->init_settings.flags & KCANFD_INIT_TS_FMT_MASK) {

		case KCANFD_INIT_TS_DRV_REL:
			to_drv_rel_time(&rx->hwtv.tv);
			break;

		case KCANFD_INIT_TS_DEV_REL: 
			to_dev_rel_time(dev, &rx->hwtv.tv);
			break;

		case KCANFD_INIT_TS_HOST_REL:
			break;
		}
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s(): tv=%u.%06u s\n", __func__,
		(u32 )rx->hwtv.tv.tv_sec,
		(u32 )rx->hwtv.tv.tv_usec);
#endif
}

/*
 * Fix ctrlr_data[] field in msg given to application.
 */
static void kcan_fix_msg_ctrlr_data(struct kcandev *dev,
				    struct kcanfd_rxmsg *rx)
{
	rx->msg.flags &= ~(KCANFD_BUSLOAD|KCANFD_ERRCNT);

	{
		/* since KCANFD_ECHOID == KCANFD_RXERRCNT, then don't
		 * provide any error counter in CAN msg when KCANFD_MSG_ECHO
		 * is set.
		 * Up to now, this client id. is one byte length but,
		 * thinking to the future, do consider that all 4 bytes of
		 * ctrlr_data[] will be able to be used to save a 32 bit client
		 * id. Therefore, don't provide current bus load in that case
		 * too.
		 */
		switch (rx->msg.type) {
		case KCANFD_TYPE_CAN20_MSG:
		case KCANFD_TYPE_CANFD_MSG:
			if (rx->msg.flags & KCANFD_MSG_ECHO)
				break;

			/* fall through */
			fallthrough;
		default:
			if (dev->features & KCAN_DEV_BUSLOAD_RDY) {
				rx->msg.ctrlr_data[KCANFD_BUSLOAD_UNIT] =
					dev->bus_load / 100;
				rx->msg.ctrlr_data[KCANFD_BUSLOAD_DEC] =
					dev->bus_load % 100;
				rx->msg.flags |= KCANFD_BUSLOAD;
			}

			kcan_fix_msg_errcnt(dev, rx);
			break;
		}
	}
}

/*
 * Initializes a message to be ready to be pushed into Rx fifo.
 */
int kcan_init_rxmsg(struct kcandev *dev, struct kcanfd_rxmsg *rx,
		    __u16 type, __u32 id, __u32 flags)
{
	memset(rx, '\0', sizeof(*rx));

	rx->msg.type = type;
	rx->msg.id = id;
	rx->msg.flags = flags;

	kcan_fix_msg_timestamp(dev, rx);
	kcan_fix_msg_ctrlr_data(dev, rx);

	return sizeof(*rx);
}

/*
 * put received CAN frame into chardev receive FIFO
 * maybe this goes to a new file kcan_chardev.c some day.
 *
 * WARNING: this function returns >0 when frame HAS been enqueued,
 *                                0  when frame HAS NOT been enqueued,
 *                                <0 in case of error
 */
int kcan_chardev_rx(struct kcandev *dev, struct kcanfd_rxmsg *rx)
{
	int err;

#ifdef DEBUG_RX_QUEUE
	//if (dev->wType == HW_USB_PRO_FD)
		switch (rx->msg.type) {
		case KCANFD_TYPE_NOP:
			break;

		case KCANFD_TYPE_STATUS:
			if (rx->msg.id == KCANFD_BUS_LOAD)
				break;

			/* fall through */
			fallthrough;
		default:
			pr_info(DEVICE_NAME
				": %s(%s CAN%u > [type=%u id=%u flg=%xh len=%u "
				"%02x %02x %02x %02x])\n",
				__func__, dev->adapter->name,
				kcan_idx(dev)+1,
				rx->msg.type, rx->msg.id, rx->msg.flags,
				rx->msg.data_len, rx->msg.data[0],
				rx->msg.data[1], rx->msg.data[2],
				rx->msg.data[3]);
			break;
		}
#endif

	switch (rx->msg.type) {
	case KCANFD_TYPE_NOP:
		return 0;

	case KCANFD_TYPE_STATUS:

		/* process this case immediately to save cpu */
		if (rx->msg.id == KCANFD_BUS_LOAD) {

			dev->bus_load_total += dev->bus_load;
			dev->bus_load_count++;

			return 0;
		}
		break;

	case KCANFD_TYPE_CAN20_MSG:
	case KCANFD_TYPE_CANFD_MSG:

		/* inc rx frame counter, even if it is not posted! */
		dev->rx_frames_counter++;

		if (!(dev->allowed_msgs & KCANFD_ALLOWED_MSG_CAN)) {
#ifdef DEBUG_RX_QUEUE
			pr_info(DEVICE_NAME
				": %s(): rx [type=%d id=%d] "
				"discarded (not allowed)\n",
				__func__, rx->msg.type, rx->msg.id);
#endif

			return 0;
		}

		if (rx->msg.flags & MSGTYPE_EXTENDED) {
			if (!(dev->allowed_msgs & KCANFD_ALLOWED_MSG_EXT)) {
#ifdef DEBUG_RX_QUEUE
				pr_info(DEVICE_NAME
					": %s(): rx [type=%d id=%d] "
					"discarded (EXT fmt)\n",
					__func__, rx->msg.type, rx->msg.id);
#endif
				return 0;
			}

			/* then, check acceptance code/mask:
			 * mask:
			 * bit=1 = don't care;
			 * bit=0 = cmp this bit with coresponding bit in code
			 */
			if ((rx->msg.id & ~dev->acc_29b.mask) !=
							dev->acc_29b.code) {
#ifdef DEBUG_RX_QUEUE
				pr_info(DEVICE_NAME
					": %s(): rx [type=%d id=%0xh] "
					"discarded "
					"(acc_29b[code=%xh mask=%xh])\n",
					__func__, rx->msg.type, rx->msg.id,
					dev->acc_29b.code, dev->acc_29b.mask);
#endif
				return 0;
			}

		} else {

			/* first, check acceptance code/mask */
			if ((rx->msg.id & ~dev->acc_11b.mask) !=
							dev->acc_11b.code) {
#ifdef DEBUG_RX_QUEUE
				pr_info(DEVICE_NAME
					": %s(): rx [type=%d id=%0xh] "
					"discarded "
					"(acc_11b[code=%xh mask=%xh])\n",
					__func__, rx->msg.type, rx->msg.id,
					dev->acc_11b.code, dev->acc_11b.mask);
#endif
				return 0;
			}
		}

		if (rx->msg.flags & KCANFD_MSG_RTR)
			if (!(dev->allowed_msgs & KCANFD_ALLOWED_MSG_RTR)) {
#ifdef DEBUG_RX_QUEUE
			pr_info(DEVICE_NAME ": %s(): rx [type=%d id=%d] "
				"discarded (RTR frame)\n",
				__func__, rx->msg.type, rx->msg.id);
#endif
				return 0;
			}

		if (kcan_do_filter(dev->filter, rx)) {
#ifdef DEBUG_RX_QUEUE
			pr_debug(DEVICE_NAME ": %s(): rx [type=%d id=%d] "
				"discarded (filtered)\n",
				__func__, rx->msg.type, rx->msg.id);
#endif
			return 0;
		}
		break;
	}

	/* MUST check here if any path has been opened before posting any
	 * messages
	 */
	if (dev->nOpenPaths <= 0) {
#ifdef DEBUG_RX_QUEUE
		pr_debug(DEVICE_NAME ": %s(): rx [type=%d id=%d] "
			"discarded (nOpenPath=%d)\n",
			__func__, rx->msg.type, rx->msg.id, dev->nOpenPaths);
#endif
		return 0;
	}

	/* fix msg timestamp */
	kcan_fix_msg_timestamp(dev, rx);

	/* fix msg ctrlr_data[] */
	kcan_fix_msg_ctrlr_data(dev, rx);

	switch (rx->msg.type) {

	case KCANFD_TYPE_STATUS:
		/* other cases: message must be pushed into fifo: */

		/* donot post any STATUS message if it isn't allowed
		 * Note: this can be done here, because dev->bus_state
		 *       is already updated.
		 */
		if (!(dev->allowed_msgs & KCANFD_ALLOWED_MSG_STATUS))
			return 0;

		/* pre-process these msgs before posting them to user, to
		 * prevent to flood its rx queue with lost of same msgs
		 */
		switch (rx->msg.id) {

		case KCANFD_ERROR_ACTIVE:
		case KCANFD_ERROR_WARNING:
		case KCANFD_ERROR_PASSIVE:
			kcan_status_normalize_rx(dev, rx);

			/* fall through */
			fallthrough;
		case KCANFD_ERROR_BUSOFF:

			err = kcan_status_rx(dev, rx);
			if (err) {

				/* err == -EEXIST:
				 *
				 * no need to push this STATUS because the same
				 * status has already been posted before.
				 * Return 0 to inform that nothing has been
				 * posted.
				 */
#ifdef DEBUG_INVALID_BUS_STATE
				if (err == -EINVAL)
					pr_info(DEVICE_NAME ": %s CAN%u: "
						"bus_state=%d incompatibe with "
						"err counters rx=%u tx=%u\n",
						dev->adapter->name,
						kcan_idx(dev)+1,
						dev->bus_state,
						dev->rx_error_counter,
						dev->tx_error_counter);
#endif
				return 0;
			}
			break;

		case KCANFD_RX_EMPTY:
		case KCANFD_RX_OVERFLOW:
		case KCANFD_TX_EMPTY:
		case KCANFD_TX_OVERFLOW:
			break;

		default:
			break;
		}

		/* try to patch last posted msg if it was the same msg */
		err = kcan_fifo_foreach_back(&dev->rx_fifo,
						kcan_do_patch_rxmsg, rx);
		if (err == -EEXIST) {
			/* msg patched. don't post the msg */
			return 0;
		}

		break;

	case KCANFD_TYPE_ERROR_MSG:

		/* donot post any ERROR message if it isn't allowed */
		if (!(dev->allowed_msgs & KCANFD_ALLOWED_MSG_ERROR))
			return 0;

		/* kcan_error() should be called only when this message is
		 * allowed, to avoid to update driver rxerr/txerr cached
		 * values.
		 */
		err = kcan_error_rx(dev, rx);
		if (err == -EEXIST)
			/* no need to push this ERROR because the same
			 * error has already been posted before.
			 * Return 0 to inform that nothing has been
			 * posted.
			 */
			return 0;

		break;
	}

	/* push (or patch) frame into Rx fifo.
	 * don't return any error to not stop frames processing when they're
	 * received by packets.
	 */
	return kcan_fifo_put_ex(&dev->rx_fifo, rx,
				kcan_rxfifo_put_completion, dev);
}

/*
 * int kcan_post_bus_state(struct kcandev *dev)
 *
 *	Unconditionnaly post a STATUS[dev->bus_state] msg.
 *
 * @RETURN:
 *
 *	-EINVAL		bus state is not valid
 *	0		if msg has been pushed
 *	< 0		other errors
 */
int kcan_post_bus_state(struct kcandev *dev)
{
	int err = -EINVAL;

	if (dev->bus_state != KCANFD_UNKNOWN) {
		struct kcanfd_rxmsg rx = {
			.msg.type = KCANFD_TYPE_STATUS,
			.msg.id = dev->bus_state,
			.msg.flags = KCANFD_ERROR_BUS,
		};

		err = kcan_chardev_rx(dev, &rx);
		if (err > 0)
			kcan_event_signal(&dev->in_event);
#ifdef DEBUG_BUS_STATE
	} else {
		pr_err(DEVICE_NAME
		       ": %s(%s CAN%u): posting UNKNOWN bus_state\n",
		       __func__, dev->adapter->name, kcan_idx(dev)+1);
#endif
	}

	return err >= 0 ? 0 : err;
}
#endif /* !NETDEV_SUPPORT */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 15, 0)
static void kcan_push_bus_load_ind(unsigned long arg)
{
	struct kcandev *dev = (struct kcandev *)arg;
#else
static void kcan_push_bus_load_ind(struct timer_list *t)
{
#if defined(from_timer)
	struct kcandev *dev = from_timer(dev, t, bus_load_timer);
#elif defined(timer_container_of)
	struct kcandev *dev = timer_container_of(dev, t, bus_load_timer);
#else
	struct kcandev *dev = container_of(t, struct kcandev, bus_load_timer);
#endif
#endif
	int bl, bl_diff;

#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_LOAD)
	pr_info(DEVICE_NAME
		": %s CAN%u: bus_state=%d bl_count=%d bl_total=%d "
		"bl=%d prev_bl=%d\n",
		dev->adapter->name, kcan_idx(dev)+1,
		dev->bus_state, dev->bus_load_count,
		dev->bus_load_total, dev->bus_load, dev->posted.bus_load);
#endif
	/* in case the bus state is unknown or init settings had changed,
	 * don't post nor rearm the timer.
	 */
	if ((dev->bus_state == KCANFD_UNKNOWN) ||
	    !(dev->init_settings.flags & KCANFD_INIT_BUS_LOAD_INFO))
		return;

	bl = (dev->bus_load_count) ? 
			dev->bus_load_total / dev->bus_load_count : 
			dev->bus_load;

	bl_diff = bl - dev->posted.bus_load;

	/* post bus load change if it's greater than 0.5%
	 * or if bus load reaches 0 and previous one wasn't
	 */
	if ((bl_diff >= 50) || (bl_diff <= -50) ||
					(!bl && dev->posted.bus_load)) {

		struct kcanfd_rxmsg rx = {
			.msg = {
				.type = KCANFD_TYPE_STATUS,
				.id = KCANFD_BUS_LOAD,
				.flags = KCANFD_BUSLOAD,
			},
			.hwtv = {
				.ts_mode = KCANFD_OPT_HWTIMESTAMP_OFF,
			}
		};

		kcan_gettimeofday(&rx.hwtv.tv);

		dev->bus_load_total = 0;
		dev->bus_load_count = 0;

		rx.msg.ctrlr_data[KCANFD_BUSLOAD_UNIT] = bl / 100;
		rx.msg.ctrlr_data[KCANFD_BUSLOAD_DEC] = bl % 100;

		kcan_fix_msg_errcnt(dev, &rx);

#ifndef NETDEV_SUPPORT
		/* do update posted.bus_load only if we're certain that the
		 * BUSLOAD msg is really copied in the Rx fifo. If Rx fifo was
		 * full, then next timer will (try to) put it again.
		 */
		if (kcan_fifo_put(&dev->rx_fifo, &rx) > 0) {
			dev->posted.bus_load = bl;
			kcan_event_signal(&dev->in_event);
		}
#else
		/* in netdev mode, just update the posted.bus_load value */
		dev->posted.bus_load = bl;
#endif
	}

	mod_timer(&dev->bus_load_timer, jiffies + dev->bus_load_ind_period);
}

/* 
 * _kcan_set_ts_mode - simple implementation for netdev mode
 */
static int _kcan_set_ts_mode(struct kcandev *dev, u32 ts_mode, bool fallback)
{
	/* For netdev mode, simply set the timestamp mode without complex fallback logic */
	dev->ts_mode = ts_mode;
	return 0;
}

/*
 * get CAN frame from the device Tx fifo.
 */
int kcan_txfifo_get(struct kcandev *dev, struct kcanfd_txmsg *tx)
{
	return kcan_fifo_get_ex(&dev->tx_fifo, tx, NULL, NULL);
}

static void kcan_init_session_counters(struct kcandev *dev)
{
#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_STATE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u): bus=%u rx=%u/tx=%u\n",
		__func__, (dev->adapter) ? dev->adapter->name : "NULL",
		kcan_idx(dev)+1, dev->bus_state,
		dev->rx_error_counter, dev->tx_error_counter);
#endif

#ifdef KCAN_USES_O_ACCMODE_HACK
	/* init posted with values that are never given to user so that first
	 * message wont't be filtered
	 */
	dev->posted.bus_state = KCANFD_UNKNOWN;
	dev->posted.bus_error = KCANFD_ERRMSG_COUNT;
	dev->posted.bus_load = 0;
	dev->posted.rxerr = 0;
	dev->posted.txerr = 0;
#endif

	/* don't post useless 1st STATUS[BUS_LOAD=0%] */
	dev->bus_load = 0;
	dev->bus_error = KCANFD_ERRMSG_COUNT;
	dev->rx_error_counter = 0;
	dev->tx_error_counter = 0;

	dev->wCANStatus &= ~(CAN_ERR_BUSOFF|CAN_ERR_BUSHEAVY|\
							CAN_ERR_BUSLIGHT);

	/* controller errors should also be cleared */
	dev->wCANStatus &= ~(CAN_ERR_QOVERRUN|CAN_ERR_QRCVEMPTY|\
							CAN_ERR_QXMTFULL);
}

int kcan_set_bus_state(struct kcandev *dev, enum kcanfd_status bus_state)
{
	if (bus_state == dev->bus_state)
		return 0;

	switch (bus_state) {
	case KCANFD_ERROR_ACTIVE:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": kcan%u enters ERROR_ACTIVE\n",
			dev->nMinor);
#endif
// #ifndef NETDEV_SUPPORT
		/* start STATUS[KCANFD_BUS_LOAD] timer */
		if (dev->init_settings.flags & KCANFD_INIT_BUS_LOAD_INFO) {
			mod_timer(&dev->bus_load_timer,
					jiffies + dev->bus_load_ind_period);
		}
// #endif
		break;
	case KCANFD_ERROR_WARNING:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": kcan%u enters ERROR_WARNING\n",
			dev->nMinor);
#endif
		kcan_set_status_bit(dev, CAN_ERR_BUSLIGHT|CAN_ERR_BUSHEAVY);
		break;
	case KCANFD_ERROR_PASSIVE:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": kcan%u enters ERROR_PASSIVE\n",
			dev->nMinor);
#endif
		break;
	case KCANFD_ERROR_BUSOFF:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": kcan%u enters BUSOFF\n",
			dev->nMinor);
#endif
		if (kcan_set_status_bit(dev, CAN_ERR_BUSOFF))
			dev->dwErrorCounter++;
		break;
	case KCANFD_UNKNOWN:
#ifdef DEBUG_BUS_STATE
		pr_info(DEVICE_NAME ": kcan%u back to UNKNOWN\n",
			dev->nMinor);
#endif
		kcan_init_session_counters(dev);
		break;
	default:
		pr_err(DEVICE_NAME
			": trying to set unknown bus state %d to kcan%u\n",
			bus_state, dev->nMinor);
		return 0;
	}

	dev->bus_state = bus_state;

	return 1;
}

int kcan_handle_busoff(struct kcandev *dev, struct kcanfd_rxmsg *rx)
{
	if (dev->bus_state == KCANFD_ERROR_BUSOFF) {
		rx->msg.type = KCANFD_TYPE_NOP;
		return 0;
	}

	pr_info(DEVICE_NAME ": %s CAN%u (kcan%u): BUS OFF\n",
		dev->adapter->name, kcan_idx(dev)+1, dev->nMinor);

	rx->msg.type = KCANFD_TYPE_STATUS;
	rx->msg.flags = KCANFD_ERROR_BUS;
	rx->msg.id = KCANFD_ERROR_BUSOFF;

	kcan_set_bus_state(dev, rx->msg.id);

	/* In BUSOFF state only, wake up any task waiting for room in the
	 * device tx queue, because there is no chance for the BUS to go back to
	 * any active state without re-initializing it.
	 * Note that this "rx" message is to be put into the device rx queue,
	 * thus any task waiting for incoming messages will also be
	 * woken up.
	 */
	kcan_event_signal(&dev->out_event);

	return 1;
}

void kcan_handle_error_active(struct kcandev *dev, struct kcanfd_rxmsg *rx)
{
	const enum kcanfd_status bus_state = dev->bus_state;

#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_STATE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u): bus=%u rx=%u/tx=%u (@rx=%p)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, bus_state,
		dev->rx_error_counter, dev->tx_error_counter, rx);
#endif

	if (rx) {

		if (bus_state != KCANFD_ERROR_ACTIVE) {

			/* inform only if controller was is in bad state */
			if (bus_state == KCANFD_ERROR_BUSOFF)
				pr_info(DEVICE_NAME ": %s CAN%u: ACTIVE\n",
					dev->adapter->name, kcan_idx(dev)+1);

			/* prepare a STATUS message to put into Rx fifo */
			rx->msg.type = KCANFD_TYPE_STATUS;
			rx->msg.id = KCANFD_ERROR_ACTIVE;
			rx->msg.flags = KCANFD_ERROR_BUS;

			/* data is reset by kcan_status_rx() next */

		/* be sure to post nothing if already in ERROR_ACTIVE */
		} else {
			rx->msg.type = KCANFD_TYPE_NOP;
		}
	}

	kcan_set_bus_state(dev, KCANFD_ERROR_ACTIVE);

	/* (re)start writing AFTER having changed the bus state
	 * see also __kcan_dev_start_writing() in src/kcanfd_core.c
	 */
	if (bus_state == KCANFD_UNKNOWN) {

		 __kcan_dev_start_writing(dev, NULL);
	}
}

void kcan_soft_error_active(struct kcandev *dev)
{
	struct kcanfd_rxmsg rx;

#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_STATE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u)\n", __func__,
		dev->adapter->name, kcan_idx(dev)+1);
#endif

	kcan_handle_error_active(dev, &rx);

#ifdef NETDEV_SUPPORT
	kcan_netdev_rx(dev, &rx);
#else
	if (kcan_chardev_rx(dev, &rx) > 0)
		kcan_event_signal(&dev->in_event);
#endif
}

/*
 * Return 0 if bus should go to ERROR_ACTIVE next
 * (see kcan_handle_error_active()).
 *
 * Return 1 if not.
 */
int kcan_handle_error_status(struct kcandev *dev, struct kcanfd_rxmsg *rx,
			     int err_warning, int err_passive)
{
#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_STATE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u, EW=%u, EP=%u): "
		"bus=%u rxerr=%u txerr=%u\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1,
		err_warning, err_passive, dev->bus_state,
		dev->rx_error_counter, dev->tx_error_counter);
#endif

	rx->msg.type = KCANFD_TYPE_STATUS;
	rx->msg.flags = KCANFD_ERROR_BUS;

	if (err_passive) {
		rx->msg.id = KCANFD_ERROR_PASSIVE;

	} else if (err_warning) {
		rx->msg.id = KCANFD_ERROR_WARNING;

		/* fix error counters (if they aren't) */
		if (dev->rx_error_counter >= 128)
			dev->rx_error_counter = 127;
		if (dev->tx_error_counter >= 128)
			dev->tx_error_counter = 127;
	} else {

		/* fix error counters (if they aren't) */
		if (dev->rx_error_counter >= 96)
			dev->rx_error_counter = 95;
		if (dev->tx_error_counter >= 96)
			dev->tx_error_counter = 95;

		/* in fact, no error bit set: back to ERROR_ACTIVE */
		if (dev->bus_state != KCANFD_ERROR_ACTIVE)
			return 0;

		/* just to jump to next 'else' part (rx->msg.id=NOP) */
		rx->msg.id = KCANFD_ERROR_ACTIVE;
	}

	/* if bus state really changed, count as error */
	if (kcan_set_bus_state(dev, rx->msg.id))
		dev->dwErrorCounter++;

	/* otherwise, donot post the same bus state */
	else
		rx->msg.type = KCANFD_TYPE_NOP;

	/* say that error state has been handled (not in ACTIVE state). */
	return 1;
}

int kcan_handle_error_ctrl(struct kcandev *dev, struct kcanfd_rxmsg *rx,
				int err_ctrlr)
{
#if defined(DEBUG_TRACE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u, err_ctrlr=%d)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, err_ctrlr);
#endif
	dev->dwErrorCounter++;

	if (rx)
		rx->msg.type = KCANFD_TYPE_NOP;

	switch (err_ctrlr) {
	case KCANFD_RX_OVERFLOW:
		if (!kcan_set_status_bit(dev, CAN_ERR_QOVERRUN))
			return 0;

		if (net_ratelimit())
			pr_err(DEVICE_NAME
			       ": %s CAN%u: controller internal Rx queue "
			       "overflow: possible loss of msgs(s)\n",
			       dev->adapter->name, kcan_idx(dev)+1);

		break;
	case KCANFD_RX_EMPTY:
		if (!kcan_set_status_bit(dev, CAN_ERR_QRCVEMPTY))
			return 0;

		break;
	case KCANFD_TX_OVERFLOW:
		if (!kcan_set_status_bit(dev, CAN_ERR_QXMTFULL))
			return 0;

		pr_err(DEVICE_NAME ": %s CAN%u: controller internal Tx queue "
		       "overflow: possible loss of msgs(s)\n",
		       dev->adapter->name, kcan_idx(dev)+1);

		break;
	default:
		kcan_set_status_bit(dev, CAN_ERR_RESOURCE);
		pr_err(DEVICE_NAME ": %s CAN%u: controller internal error %d\n",
		       dev->adapter->name, kcan_idx(dev)+1, err_ctrlr);
		break;
	}

	if (rx) {
		rx->msg.type = KCANFD_TYPE_STATUS;
		rx->msg.flags = KCANFD_ERROR_CTRLR;
		rx->msg.id = err_ctrlr;
		rx->msg.data_len = 0;
	}

	return 1;
}

void kcan_handle_error_msg(struct kcandev *dev, struct kcanfd_rxmsg *rx,
				int err_type, u8 err_code,
				int err_rx, int err_gen)
{
#if defined(DEBUG_TRACE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u, err_type=%d)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, err_type);
#endif

	if (rx) {
		rx->msg.type = KCANFD_TYPE_ERROR_MSG;
		rx->msg.id = err_type;
		if (err_rx)
			rx->msg.flags |= KCANFD_ERRMSG_RX;
		if (err_gen)
			rx->msg.flags |= KCANFD_ERRMSG_GEN;

		rx->msg.data_len = 1;
		rx->msg.data[0] = err_code;
	}

	dev->dwErrorCounter++;
}

void kcan_handle_error_internal(struct kcandev *dev,
				struct kcanfd_rxmsg *rx,
				int err_internal)
{
#if defined(DEBUG_TRACE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u, err_internal=%d)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, err_internal);
#endif

	dev->dwErrorCounter++;

	/* Note: don't test those bits since they are set/unset from task too */
	switch (err_internal) {
	case KCANFD_RX_OVERFLOW:
		kcan_set_status_bit(dev, CAN_ERR_OVERRUN);
		break;
	case KCANFD_TX_OVERFLOW:
		kcan_set_status_bit(dev, CAN_ERR_XMTFULL);
		break;
	default:
		pr_err(DEVICE_NAME ": kcan%u: internal error %d\n",
				dev->nMinor, err_internal);
		kcan_set_status_bit(dev, CAN_ERR_RESOURCE);
		break;
	}

	if (rx) {
		rx->msg.type = KCANFD_TYPE_STATUS;
		rx->msg.flags = KCANFD_ERROR_INTERNAL;
		rx->msg.id = err_internal;
		rx->msg.data_len = 0;
	}
}

void kcan_handle_error_protocol(struct kcandev *dev,
				struct kcanfd_rxmsg *rx,
				int err_protocol)
{
#if defined(DEBUG_TRACE)
	pr_info(DEVICE_NAME ": %s(%s CAN%u, err_protocol=%d)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, err_protocol);
#endif

	dev->dwErrorCounter++;

	if (rx)
		rx->msg.type = KCANFD_TYPE_NOP;

	switch (err_protocol) {
	case KCANFD_RX_OVERFLOW:
		/* as old driver did */
		if (!kcan_set_status_bit(dev, CAN_ERR_QOVERRUN))
			return;

		break;
	case KCANFD_TX_OVERFLOW:
		if (!kcan_set_status_bit(dev, CAN_ERR_QXMTFULL))
			return;

		break;
	default:
		pr_err(DEVICE_NAME ": kcan%u: protocol error %d\n",
			dev->nMinor, err_protocol);
		kcan_set_status_bit(dev, CAN_ERR_RESOURCE);
		break;
	}

	if (rx) {
		rx->msg.type = KCANFD_TYPE_STATUS;
		rx->msg.flags = KCANFD_ERROR_PROTOCOL;
		rx->msg.id = err_protocol;
		rx->msg.data_len = 0;
	}
}

/*
 * Set a status bit only if it isn't yet. Return 0 if it was.
 */
int kcan_set_status_bit(struct kcandev *dev, u16 bits)
{
	if (dev->wCANStatus & bits)
		return 0;

	dev->wCANStatus |= bits;
	return bits;
}

/*
 * Clear any wCANStatus bit.
 *
 * @return:
 *
 * 0	if no bit were set
 * 1	if any was set.
 */
int kcan_clear_status_bit(struct kcandev *dev, u16 bits)
{
	if (!(dev->wCANStatus & bits))
		return 0;

	/* clear corresponding bits and generate a STATUS message to update
	 * the state in the application context
	 */
	dev->wCANStatus &= ~bits;

	return 1;
}

/* safety call the device "dev->cleanup()" specific callback and manage
 * to be called only once. The "dev->cleanup()" callback should not be
 * directlty called.
 */
void kcan_cleanup_dev(struct kcandev *dev)
{
#if defined(DEBUG_TRACE) || defined(DEBUG_CLEANUP)
	pr_info(DEVICE_NAME ": %s(dev=%p flags=%08Xh): cleaning %s CAN%d\n",
		 __func__, dev,
		 (dev) ? dev->flags : 0,
		 (dev && dev->adapter) ? dev->adapter->name : "Unknown adapter",
		 (dev) ? kcan_idx(dev)+1 : -1);
#endif
	if (dev) {
		if (!(dev->flags & (KCAN_DEV_CLEANED|KCAN_DEV_CLEANING)))
			if (dev->cleanup) {
				/* protect from reentrance */
				dev->flags |= KCAN_DEV_CLEANING;
				dev->cleanup(dev);
			}

		dev->flags &= ~(KCAN_DEV_OPENED|KCAN_DEV_CLEANING);
		dev->flags |= KCAN_DEV_CLEANED;
	}
}

/* request time in msec, fast */
u32 get_mtime(void)
{
	/* return (jiffies / HZ) * 1000; */
	return jiffies_to_msecs(jiffies);
}

/*
 * Safe add a new device to the driver registered devices list
 */
void kcan_add_dev_in_list_ex(struct kcandev *dev, u32 dev_flags)
{
#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	kcan_lock_get_irqsave(&kcan_drv.devices_lock, flags);
#endif
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%p)\n", __func__, dev);
#endif

	list_add_tail(&dev->list_dev, &kcan_drv.devices);
	dev->flags |= KCAN_DEV_LINKED | dev_flags;
	kcan_drv.wDeviceCount++;

#ifdef HANDLE_HOTPLUG
	kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);
#endif
}

/*
 * Safe remove a device from the driver registered devices list
 */
struct kcandev *kcan_remove_dev_from_list(struct kcandev *dev)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%p)\n", __func__, dev);
#endif

	if (dev->flags & KCAN_DEV_LINKED) {

#ifdef HANDLE_HOTPLUG
		unsigned long flags;

		/* kcandev is being destroyed by remove_dev_list(), so let
		 * remove_dev_list() unlink the kcandev...
		 * if (!kcan_mutex_trylock(&kcan_drv.devices_lock)) {
		 */
		if (!kcan_lock_try_irqsave(&kcan_drv.devices_lock, flags)) {
			pr_warn(DEVICE_NAME
				": can't unlink dev %p (global lock in use)\n",
				dev);
			return NULL;
		}
#endif
		list_del(&dev->list_dev);
		dev->flags &= ~KCAN_DEV_LINKED;

		if (kcan_drv.wDeviceCount)
			kcan_drv.wDeviceCount--;

#ifdef HANDLE_HOTPLUG
		kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);
#endif
		return dev;
	}

	return NULL;
}

/*
 * Safe check whether a device is linked in the kcan driver devices list.
 */
int kcan_is_device_in_list(struct kcandev *dev)
{
	struct list_head *ptr;
	int found = 0;

#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	kcan_lock_get_irqsave(&kcan_drv.devices_lock, flags);
#endif

	list_for_each(ptr, &kcan_drv.devices) {
		struct kcandev *d = list_entry(ptr, struct kcandev, list_dev);
		if (dev == d) {
			found = 1;
			break;
		}
	}

#ifdef HANDLE_HOTPLUG
	kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);
#endif
	return found;
}

#ifdef HANDLE_HOTPLUG
static struct kcandev *kcan_is_minor_used(struct kcandev *pdev, int minor)
{
	struct list_head *ptr;

	list_for_each(ptr, &kcan_drv.devices) {
		struct kcandev *dev = list_entry(ptr, struct kcandev, list_dev);

		/* Note: if not NULL, pdev is a linked device we don't want
		 * to test its nMinor.
		 */
		if (dev != pdev)
			if (dev->nMinor == minor)
				return dev;
	}

	return NULL;
}

/*
 * Look for enough minors numbers in a given range.
 * Each call returns 1st device number of a range that is enough to store all
 * the minors an adapter is able to handle.
 * Caller MUST update the return value according to the real channel.
 * Called by hotplug devices only.
 */
int kcan_find_free_minors(struct kcandev *pdev, int from, int until)
{
	int minor = (from >= 0) ? from : 0;
	unsigned long flags;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(from=%d, until=%d)\n", __func__, from, until);
#endif

	kcan_lock_get_irqsave(&kcan_drv.devices_lock, flags);

	while (1) {
		struct kcandev *dev = NULL;
		int i;

		/* check whether there are enough free minors for the adapter */
		for (i = 0; i < pdev->adapter->can_count; i++) {
			dev = kcan_is_minor_used(pdev, minor + i);
			if (dev) {
				/* a minor of the range is used:
				 * - if it is used by a channel of the same
				 *   adapter: range is already reserved,
				 *   nothing to do.
				 * - if that channel is of another
				 *   adapter then skip all its
				 *   channels and restart.
				 */
				break;
			}
		}

		/* dev == NULL: range [minor..] found
		 * dev adapters are equal: range [minor..] found too
		 */
		if (!dev || (dev->adapter == pdev->adapter))
			break;

		/* minor defines 1st channel of some other adapter.
		 * Skip all the others and restart
		 */
		minor += dev->adapter->can_count;

		/* if searching is limited, check whether minor is always
		 * into these limits...
		 */
		if (until > from)
			if (minor > until) {
				minor = -ENODEV;
				break;
			}
	}

	kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);

	return minor;
}
#endif /* HANDLE_HOTPLUG */

/* called when 'cat /proc/kcan' invoked */
#ifdef CREATE_PROC_ENTRY_DEPRECATED
static int kcan_read_procmem(struct seq_file *m, void *v)
{
#else
static int kcan_read_procmem(char *page, char **start, off_t offset, int count,
							int *eof, void *data)
{
	int len = 0;
#endif
	struct kcandev *dev;
	struct list_head *ptr;
#ifdef HANDLE_HOTPLUG
	unsigned long flags;
#endif

#ifdef CREATE_PROC_ENTRY_DEPRECATED
	seq_printf(m, "\n");
	seq_printf(m,
		"*------------- Kunhong-Elec CAN interfaces (www.kunhong-elec.com) -------------\n");
	seq_printf(m,
		"*------------- %s (%s) %s %s --------------\n",
		kcan_drv.szVersionString, CURRENT_VERSIONSTRING,
		__DATE__, __TIME__);
	seq_printf(m, "%s\n", config);
	seq_printf(m, "*--------------------- %d interfaces @ major %03d found -----------------------\n",
		kcan_drv.wDeviceCount, kcan_drv.nMajor);
	seq_printf(m,
		"*n -type- -ndev- --base-- irq --btr- --read-- --write- --irqs-- -errors- status\n");
#else
	len += sprintf(page + len, "\n");
	len += sprintf(page + len,
		"*------------- Kunhong-Elec CAN interfaces (www.kunhong-elec.com) -------------\n");
	len += sprintf(page + len,
		"*------------- %s (%s) %s %s --------------\n",
		kcan_drv.szVersionString, CURRENT_VERSIONSTRING,
		__DATE__, __TIME__);
	len += sprintf(page + len, "%s\n", config);
	len += sprintf(page + len,
		"*--------------------- %d interfaces @ major %03d found -----------------------\n",
		kcan_drv.wDeviceCount, kcan_drv.nMajor);
	len += sprintf(page + len,
		"*n -type- -ndev- --base-- irq --btr- --read-- --write- --irqs-- -errors- status\n");
#endif
#ifdef HANDLE_HOTPLUG
	/* enter critical section (get mutex) */
	kcan_lock_get_irqsave(&kcan_drv.devices_lock, flags);
#endif
	/* loop trough my devices */
	for (ptr = kcan_drv.devices.next; ptr != &kcan_drv.devices;
							ptr = ptr->next) {
		u32 dwPort = 0;
		u16 wIrq = 0, dev_btr0btr1;
		int minor;
#ifdef NETDEV_SUPPORT
		struct net_device_stats *stats; /* rx/tx stats */
#endif

		dev = (struct kcandev *)ptr;
		minor = dev->nMinor;

		dev_btr0btr1 = (dev->sysclock_Hz == 8*MHz) ?
			kcan_bittiming_to_btr0btr1(&dev->init_settings.nominal):
			sja1000_bitrate(dev->init_settings.nominal.bitrate,
				dev->init_settings.nominal.sample_point,
				dev->init_settings.nominal.sjw);

		switch (dev->wType) {
		case HW_USB:
		case HW_USB_FD:
		case HW_USB_PRO:
		case HW_USB_PRO_FD:
		case HW_USB_X6:
		case HW_USB_X4:
#ifdef USB_SUPPORT
			/* get serial number of device */
			if (dev->is_plugged) {
				dwPort = kcan_usb_get_if(dev)->dwSerialNumber;
				wIrq = dev->port.usb.ucHardcodedDevNr;
			} else {
				dwPort = 0x00dead00;  /* it is dead */
				wIrq = 0;
			}
#endif
			break;
		default:
			dwPort = dev->dwPort;
			wIrq = dev->wIrq;
			break;
		}

#ifdef NETDEV_SUPPORT
		stats = (dev->netdev) ?
				kcan_netdev_get_stats(dev->netdev) : NULL;
#endif
#ifdef CREATE_PROC_ENTRY_DEPRECATED
		seq_printf(m,
#else
		len += sprintf(page + len,
#endif
		"%2d %6s %6s %8x %03d 0x%04x %08lx %08lx %08x %08x 0x%04x\n",
			minor,
			dev->type,
#ifdef NETDEV_SUPPORT
			(dev->netdev) ? (dev->netdev->name) : "can?",
#else
			"-NA-",
#endif
			dwPort,
			wIrq,
			dev_btr0btr1,
#ifdef NETDEV_SUPPORT
			(stats) ? stats->rx_packets : 0,
			dev->tx_fifo.dwTotal +
					((stats) ? stats->tx_packets : 0),
#else
			(unsigned long)dev->rx_fifo.dwTotal,
			(unsigned long)dev->tx_fifo.dwTotal,
#endif
			dev->rx_irq_counter + dev->tx_irq_counter,
			dev->dwErrorCounter,
			dev->wCANStatus);
	}

#ifdef HANDLE_HOTPLUG
	/* release mutex */
	kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);
#endif
#ifdef CREATE_PROC_ENTRY_DEPRECATED
	return 0;
#else
	len += sprintf(page + len, "\n");

	*eof = 1;
	return len;
#endif
}

#ifdef CONFIG_SYSFS
int kcan_sysfs_add_attr(struct device *dev, struct attribute *attr)
{
	return sysfs_add_file_to_group(&dev->kobj, attr, NULL);
}

int kcan_sysfs_add_attrs(struct device *dev, struct attribute **attrs)
{
	int err = 0;
	struct attribute **ppa;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%p=\"%s\")\n", __func__, dev, dev->kobj.name);
#endif
	for (ppa = attrs; *ppa; ppa++) {
		err = kcan_sysfs_add_attr(dev, *ppa);
		if (err) {
			pr_err(DEVICE_NAME
				": failed to add \"%s\" to \"%s\" (err %d)\n",
				(*ppa)->name, dev->kobj.name, err);
			break;
		}
	}

	return err;
}

int kcan_sync_times(struct kcandev *dev, u32 ts_low, u32 ts_high, int tv_off)
{
	return kcan_sync_times_host(dev, ts_low, ts_high, tv_off, NULL, 0);
}

void kcan_sysfs_del_attr(struct device *dev, struct attribute *attr)
{
	sysfs_remove_file_from_group(&dev->kobj, attr, NULL);
}

void kcan_sysfs_del_attrs(struct device *dev, struct attribute **attrs)
{
	struct attribute **ppa;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%p=\"%s\")\n", __func__, dev, dev->kobj.name);
#endif
	for (ppa = attrs; *ppa; ppa++)
		kcan_sysfs_del_attr(dev, *ppa);
}
#else
int kcan_sysfs_add_attr(struct device *dev, struct attribute *attr)
{
	return 0;
}

int kcan_sysfs_add_attrs(struct device *dev, struct attribute **attrs)
{
	return 0;
}

void kcan_sysfs_del_attr(struct device *dev, struct attribute *attr) {}
void kcan_sysfs_del_attrs(struct device *dev, struct attribute **attrs) {}
#endif

#ifdef CREATE_PROC_ENTRY_DEPRECATED
static int open_callback(struct inode *inode, struct file *file)
{
	return single_open(file, kcan_read_procmem, NULL);
}

static struct proc_dir_entry *proc_file_entry;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0)
static const struct proc_ops proc_file_fops = {
	.proc_open  = open_callback,
	.proc_read  = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
};
#else
static const struct file_operations proc_file_fops = {
	.owner = THIS_MODULE,
	.open  = open_callback,
	.read  = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

#else
#define proc_file_entry		NULL
#endif

void remove_dev_list(void)
{
	struct list_head *pos, *n;
#ifdef DEBUG_ALLOC_DEV
	int n_free = 0, n_del = 0;
#endif
#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	kcan_lock_get_irqsave(&kcan_drv.devices_lock, flags);
#endif

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif
	list_for_each_prev_safe(pos, n, &kcan_drv.devices) {
		struct kcandev *dev = list_entry(pos, struct kcandev, list_dev);

		kcan_destroy_dev(dev);

		list_del(pos);

		/* free device object allocated memory */
#ifdef DEBUG_ALLOC_DEV
		if (!kcan_free_dev(dev))
			n_free++;
		n_del++;
#else
		kcan_free_dev(dev);
#endif
	}

	kcan_drv.wDeviceCount = 0;

#ifdef HANDLE_HOTPLUG
	kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);
#endif

#ifdef DEBUG_ALLOC_DEV
	pr_info(DEVICE_NAME ": %d device(s) removed (%u freed)\n",
		n_del, n_free);
#endif
}

#ifdef SYSFS_SUPPORT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
#define KCAN_ATTR_RO_CONST	const
#else
#define KCAN_ATTR_RO_CONST
#endif

static ssize_t version_show(KCAN_ATTR_RO_CONST struct class *cls,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
			    KCAN_ATTR_RO_CONST struct class_attribute *attr,
#endif
			    char *buf)
{
	return sprintf(buf, "%s\n", CURRENT_VERSIONSTRING);
}

static ssize_t clk_ref_show(KCAN_ATTR_RO_CONST struct class *cls,
			    KCAN_ATTR_RO_CONST struct class_attribute *attr,
			    char *buf)
{
	return sprintf(buf, "%u\n", drvclkref);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
static CLASS_ATTR_RO(version);
static CLASS_ATTR_RO(clk_ref);

static struct attribute *kcan_class_attrs[] = {
	&class_attr_version.attr,
	&class_attr_clk_ref.attr,
	NULL,
};

ATTRIBUTE_GROUPS(kcan_class);
#else
static struct class_attribute kcan_class_attrs[] = {
	__ATTR_RO(version),
	__ATTR_RO(clk_ref),
	__ATTR_NULL
};
#endif

static struct class kcan_class = {
	.name = DEVICE_NAME,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4, 10, 0)
	.class_groups = kcan_class_groups,
#else
	.class_attrs = kcan_class_attrs,
#endif
};
#endif

/* called when the device is removed 'rmmod kcan' */
static void kcan_cleanup_module(void)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(init_step=%u)\n",
					__func__, kcan_drv.wInitStep);
#endif
	switch (kcan_drv.wInitStep) {

	case 4:
		remove_proc_entry(DEVICE_NAME, NULL);

		/* fall through */
		fallthrough;
	case 3:
		DEV_UNREGISTER();

		/* fall through */
		fallthrough;
	case 2:
#ifdef USB_SUPPORT
		kcan_usb_deinit();
#endif

#ifdef PCCARD_SUPPORT
		kcan_pccard_deinit();
#endif

#ifdef KCAN_PCI_EVENT_DRIVEN
		kcan_pci_deinit();
#endif
		/* fall through */
		fallthrough;
	case 1:
#ifdef SYSFS_SUPPORT
		class_unregister(&kcan_class);
#endif

#ifdef NO_RT
		unregister_chrdev(kcan_drv.nMajor, DEVICE_NAME);
#endif

		REMOVE_DEV_LIST();

#ifdef HANDLE_HOTPLUG
		/* destroy mutex used to access kcan devices list */
		kcan_lock_destroy(&kcan_drv.devices_lock);
#endif
		/* fall through */
		fallthrough;
	case 0:
		kcan_drv.wInitStep = 0;
	}

	pr_info(DEVICE_NAME ": removed.\n");
}

void kcan_init_version(struct kcan_version *ver)
{
	ver->major = -1;
	ver->minor = -1;
	ver->subminor = -1;
}

void kcan_init_adapter(struct kcan_adapter *pa, const char *name,
		       const char *part_num, int index, int can_count)
{
	pa->flags = 0;
	pa->name = name;
	pa->part_num = part_num;
	pa->index = index;
	pa->can_count = can_count;
	pa->opened_count = 0;
	kcan_init_version(&pa->hw_ver);
	memset(pa->guid, 0xff, sizeof(pa->guid));
}

void *__kcan_alloc_adapter_ex(const char *name, const char *part_num, int index,
			      int can_count, int extra_size)
{
	struct kcan_adapter *pa;
	int l = sizeof(*pa);

	if (l < extra_size)
		l = extra_size;

	l += can_count * sizeof(struct kcandev *);

	pa = kcan_malloc(l, GFP_KERNEL);
	if (pa) {
		memset(pa, '\0', l);
		kcan_init_adapter(pa, name, part_num, index, can_count);
	}

	return pa;
}

struct kcan_adapter *__kcan_free_adapter(struct kcan_adapter *pa)
{
	kcan_free(pa);
	return NULL;
}

/* This function can be called next to kcan_alloc_dev().  */
void kcan_set_dev_adapter(struct kcandev *dev, struct kcan_adapter *pa)
{
	dev->adapter = pa;

	/* DON't do any doubly linkage here like:
	 *
	 * pa->devs[kcan_idx(dev)] = dev;
	 *
	 * because "struct kcan_adapter" is used as a strict base type and the
	 * "devs[]" array is in fact really defined at the end of the derived
	 * struct types (see "struct kcan_pci_adapter" and
	 * "struct pcifd_adapter" for ex)
	 */

	/* device hw version is the one read from the adapter */
	dev->hw_ver = &pa->hw_ver;
}

struct kcandev *__kcan_alloc_dev(char *type_str, u16 type, int index)
{
	const int sizeof_dev = sizeof(struct kcandev);
	struct kcandev *dev = kcan_malloc(sizeof_dev, GFP_KERNEL);

	if (!dev) {
		pr_err(DEVICE_NAME ": FATAL: failed to alloc dev %s CAN%u\n",
		       type_str, index+1);
		return NULL;
	}

	memset(dev, '\0', sizeof_dev);

	dev->type = type_str;
	dev->wType = type;
	dev->can_idx = index;

	return dev;
}

struct kcandev *__kcan_free_dev(struct kcandev *dev)
{
#ifndef NETDEV_SUPPORT
	if (rxqprealloc)
		dev->rx_msgs = kcan_free(dev->rx_msgs);
#endif

	if (txqprealloc)
		dev->tx_msgs = kcan_free(dev->tx_msgs);

	if (dev->flags & KCAN_DEV_STATIC)
		return dev;

	kcan_free(dev);
	return NULL;
}

/* called just before kcan_free_dev(), that is the device can't be used next.
 * the goal of this is to keep the "dev" pointer alive only.
 */
void __kcan_destroy_dev(struct kcandev *dev)
{
#ifdef NETDEV_SUPPORT
	kcan_netdev_unregister(dev);
#endif

	kcan_sysfs_dev_node_destroy(dev);

	/* Thank you Brad Johnson and his team from Astronics! */
	/* delete STATUS[KCANFD_BUS_LOAD] timer */
	if (dev->features & KCAN_DEV_BUSLOAD_RDY) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 16, 0)
		timer_delete_sync(&dev->bus_load_timer);
#else
		del_timer_sync(&dev->bus_load_timer);
#endif
	}

	/* prevent from calling more than once the cleanup proc */
	kcan_cleanup_dev(dev);

	dev->wInitStep = 0xff;
	dev->filter = kcan_delete_filter_chain(dev->filter);

	if (dev->nOpenPaths >= 0) {
		dev->nOpenPaths = 0;
		kcan_release_path(dev, NULL);
	}

	kcan_mutex_destroy(&dev->mutex);
}

static int kcan_get_opt_u32(struct kcandev *dev,
			struct kcanfd_option *opt, void *c, u32 tmp)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(tmp));
#endif

	opt->size = sizeof(tmp);
	if (kcan_copy_to_user(opt->value, &tmp, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

int kcan_get_dev_features(struct kcandev *dev, u32 *features)
{
	*features = 0;

	if (dev->features & KCAN_DEV_FD_RDY)
		*features |= KCANFD_FEATURE_FD;
	if (dev->features & KCAN_DEV_TXPAUSE_RDY)
		*features |= KCANFD_FEATURE_IFRAME_DELAYUS;
	if (dev->features & KCAN_DEV_BUSLOAD_RDY)
		*features |= KCANFD_FEATURE_BUSLOAD;
	if (dev->features & KCAN_DEV_HWTS_RDY)
		*features |= KCANFD_FEATURE_HWTIMESTAMP;
	if (dev->features & KCAN_DEV_SLF_RDY)
		*features |= KCANFD_FEATURE_SELFRECEIVE;
	if (dev->features & KCAN_DEV_ECHO_RDY)
		*features |= KCANFD_FEATURE_ECHO;
	if (dev->features & KCAN_DEV_MSD_RDY)
		*features |= KCANFD_FEATURE_MSD;
	if (dev->features & KCAN_DEV_TS_SOF_RDY)
		*features |= KCANFD_FEATURE_TS_SOF;
	if (dev->features & KCAN_DEV_SELF_ACK_RDY)
		*features |= KCANFD_FEATURE_SELF_ACK;
	if (dev->features & KCAN_DEV_BRS_IGN_RDY)
		*features |= KCANFD_FEATURE_BRS_IGN;

	if (dev->option[KCANFD_OPT_DEVICE_ID].get)
		*features |= KCANFD_FEATURE_DEVICEID;

	if (dev->features & KCAN_DEV_DEVDATA)
		*features |= KCANFD_FEATURE_DEVDATA;
	if (dev->features & KCAN_DEV_NEW_FW_AV)
		*features |= KCANFD_FEATURE_NEW_FW_AV;

	return 0;
}

static int kcan_get_channel_features(struct kcandev *dev,
				     struct kcanfd_option *opt, void *c)
{
	u32 tmp32;

	kcan_get_dev_features(dev, &tmp32);
	return kcan_get_opt_u32(dev, opt, c, tmp32);
}

static int kcan_get_avclocks(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	int lk;
	u32 ck;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size,
		(uint )sizeof(struct kcanfd_available_clocks));
#endif

	/* Copy only the nb of clock corresponding to the user buffer */
	ck = dev->clocks_list->count;
	lk = sizeof(struct kcanfd_available_clocks_0) +
			ck * sizeof(struct kcanfd_available_clock);

	if (opt->size >= lk)
		opt->size = lk;
	else {
		pr_err(DEVICE_NAME
			": %d bytes buffer needed to copy %s clocks\n",
			lk, dev->adapter->name);
		opt->size = lk;
		return -ENOSPC;
	}

	if (kcan_copy_to_user(opt->value, dev->clocks_list, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(1): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	/* update count of clocks given to user (if needed) */
	if (ck < dev->clocks_list->count)
		if (kcan_copy_to_user(opt->value, &ck, sizeof(ck), c)) {
			pr_err(DEVICE_NAME ": %s(2): copy_to_user() failure\n",
				__func__);
			return -EFAULT;
		}

	return 0;
}

static int kcan_get_bittiming_range(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(*dev->bittiming_caps));
#endif

	opt->size = sizeof(*dev->bittiming_caps);
	if (kcan_copy_to_user(opt->value, dev->bittiming_caps, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int kcan_get_dbittiming_range(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(*dev->dbittiming_caps));
#endif

	if (!dev->dbittiming_caps)
		return -EOPNOTSUPP;

	opt->size = sizeof(*dev->dbittiming_caps);
	if (kcan_copy_to_user(opt->value, dev->dbittiming_caps, opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int kcan_get_allowed_msgs(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->allowed_msgs));
#endif
	return kcan_get_opt_u32(dev, opt, c, dev->allowed_msgs);
}

static int kcan_set_allowed_msgs(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->allowed_msgs));
#endif

	if (kcan_copy_from_user(&dev->allowed_msgs, opt->value,
					sizeof(dev->allowed_msgs), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int kcan_get_acc_filter_29b(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->acc_29b.value64));
#endif

	opt->size = sizeof(dev->acc_29b.value64);
	if (kcan_copy_to_user(opt->value, &dev->acc_29b.value64,
				opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int kcan_set_acc_filter_29b(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->acc_29b.value64));
#endif

	if (kcan_copy_from_user(&dev->acc_29b.value64, opt->value,
					sizeof(dev->acc_29b.value64), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	dev->acc_29b.mask &= CAN_MAX_EXTENDED_ID;
	dev->acc_29b.code &= ~dev->acc_29b.mask;

	return 0;
}

static int kcan_get_acc_filter_11b(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->acc_11b.value64));
#endif

	opt->size = sizeof(dev->acc_11b.value64);
	if (kcan_copy_to_user(opt->value, &dev->acc_11b.value64,
					opt->size, c)) {
		pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int kcan_set_acc_filter_11b(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->acc_11b.value64));
#endif

	if (kcan_copy_from_user(&dev->acc_11b.value64, opt->value,
					sizeof(dev->acc_11b.value64), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	dev->acc_11b.mask &= CAN_MAX_STANDARD_ID;
	dev->acc_11b.code &= ~dev->acc_11b.mask;

	return 0;
}

static int kcan_get_ifrm_delay_us(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	return (dev->features & KCAN_DEV_TXPAUSE_RDY) ?
		kcan_get_opt_u32(dev, opt, c, dev->tx_iframe_delay_us) :
		-EOPNOTSUPP;
}

static int kcan_set_ifrm_delay_us(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	if (!(dev->features & KCAN_DEV_TXPAUSE_RDY))
		return -EOPNOTSUPP;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u): delay=%u\n",
		__func__, opt->size, (uint )sizeof(dev->tx_iframe_delay_us),
		dev->tx_iframe_delay_us);
#endif

	if (kcan_copy_from_user(&dev->tx_iframe_delay_us, opt->value,
					sizeof(dev->tx_iframe_delay_us), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return 0;
}

static int kcan_get_self_ack(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	if (!(dev->features & KCAN_DEV_SELF_ACK_RDY))
		return -EOPNOTSUPP;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d): slf_ack=%u\n",
		__func__, opt->size, !!(dev->flags & KCAN_DEV_SELF_ACK));
#endif

	return kcan_get_opt_u32(dev, opt, c,
				!!(dev->flags & KCAN_DEV_SELF_ACK));
}

static int kcan_set_self_ack(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	u32 tmp;
	int err;

#ifdef DEBUG_TRACE
	if (kcan_copy_from_user(&tmp, opt->value, sizeof(tmp), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	pr_info(DEVICE_NAME ": %s(tmp=%u)\n", __func__, tmp);
#endif

	if (!(dev->features & KCAN_DEV_SELF_ACK_RDY))
		return -EOPNOTSUPP;

#ifndef DEBUG_TRACE
	if (kcan_copy_from_user(&tmp, opt->value, sizeof(tmp), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}
#endif
	if (tmp) {
		dev->flags &= ~KCAN_DEV_SELF_ACK;
		err = ucan_set_options(dev, UCAN_OPTION_SELF_ACK, 0);
		if (!err)
			dev->flags |= KCAN_DEV_SELF_ACK;
		else
			pr_err(DEVICE_NAME
				": %s CAN%u: failed to set slf-ack (err %d)\n",
				dev->adapter->name, kcan_idx(dev)+1, err);
	} else {
		dev->flags |= KCAN_DEV_SELF_ACK;
		err = ucan_clr_options(dev, UCAN_OPTION_SELF_ACK, 0);
		if (!err)
			dev->flags &= ~KCAN_DEV_SELF_ACK;
		else
			pr_err(DEVICE_NAME
			       ": %s CAN%u: failed to clear slf-ack (err %d)\n",
			       dev->adapter->name, kcan_idx(dev)+1, err);
	}

	return err;
}

static int kcan_get_brs_ign(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	if (!(dev->features & KCAN_DEV_BRS_IGN_RDY))
		return -EOPNOTSUPP;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d): brs_ign=%u\n",
		__func__, opt->size, !!(dev->flags & KCAN_DEV_BRS_IGN));
#endif
	return kcan_get_opt_u32(dev, opt, c,
				!!(dev->flags & KCAN_DEV_BRS_IGN));
}

static int kcan_set_brs_ign(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	u32 tmp;
	int err;

#ifdef DEBUG_TRACE
	if (kcan_copy_from_user(&tmp, opt->value, sizeof(tmp), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	pr_info(DEVICE_NAME ": %s(tmp=%u)\n", __func__, tmp);
#endif

	if (!(dev->features & KCAN_DEV_BRS_IGN_RDY))
		return -EOPNOTSUPP;

#ifndef DEBUG_TRACE
	if (kcan_copy_from_user(&tmp, opt->value, sizeof(tmp), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}
#endif

	if (tmp) {
		dev->flags &= ~KCAN_DEV_BRS_IGN;
		err = ucan_set_options(dev, UCAN_OPTION_BRS_IGN, 0);
		if (!err)
			dev->flags |= KCAN_DEV_BRS_IGN;
		else
			pr_err(DEVICE_NAME
				": %s CAN%u: failed to set brs-ign (err %d)\n",
				dev->adapter->name, kcan_idx(dev)+1, err);
	} else {
		dev->flags |= KCAN_DEV_BRS_IGN;
		err = ucan_clr_options(dev, UCAN_OPTION_BRS_IGN, 0);
		if (!err)
			dev->flags &= ~KCAN_DEV_BRS_IGN;
		else
			pr_err(DEVICE_NAME
			       ": %s CAN%u: failed to clear brs-ign (err %d)\n",
			       dev->adapter->name, kcan_idx(dev)+1, err);
	}

	return err;
}

#ifndef NETDEV_SUPPORT
static int kcan_get_ts_mode(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	u32 tmp = dev->ts_mode;

	if (dev->flags & KCAN_DEV_TS_SOF)
		tmp |= KCANFD_OPT_HWTIMESTAMP_SOF;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u): ts_mode=%u\n",
		__func__, opt->size, (uint )sizeof(dev->ts_mode), tmp);
#endif

	return kcan_get_opt_u32(dev, opt, c, tmp);
}

static int _kcan_set_ts_mode(struct kcandev *dev, u32 ts_mode, bool fallback)
{
	u32 sof_flag = (ts_mode & KCANFD_OPT_HWTIMESTAMP_SOF);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(ts_mode=%u, fallback=%d)\n",
		__func__, ts_mode, fallback);
#endif

	ts_mode &= ~KCANFD_OPT_HWTIMESTAMP_SOF;

	switch (ts_mode) {
	case KCANFD_OPT_HWTIMESTAMP_DEF:

		ts_mode = KCANFD_OPT_HWTIMESTAMP_COOKED;

		/* fall through */
		fallthrough;
	case KCANFD_OPT_HWTIMESTAMP_ON:
	case KCANFD_OPT_HWTIMESTAMP_COOKED:
		if (dev->features & KCAN_DEV_HWTSC_RDY)
			break;

		if (!fallback)
			return -ENOTSUPP;

		ts_mode = KCANFD_OPT_HWTIMESTAMP_RAW;

		/* fall through */
		fallthrough;
	case KCANFD_OPT_HWTIMESTAMP_RAW:
		if (dev->features & KCAN_DEV_HWTS_RDY)
			break;

		if (!fallback)
			return -ENOTSUPP;

		ts_mode = KCANFD_OPT_HWTIMESTAMP_OFF;

		/* fall through */
		fallthrough;
	case KCANFD_OPT_HWTIMESTAMP_OFF:
		break;

		/* fall through */
		fallthrough;
	default:
		return -EINVAL;
	}

	/* set or clear the option according to SOF bit.
	 * Note: up to now, the TS_SOF feature is avalable only on uCAN FW
	 */
	if (dev->features & KCAN_DEV_TS_SOF_RDY) {
		if (sof_flag) {
			if (!ucan_set_options(dev, UCAN_OPTION_TS_SOF, 0))
				dev->flags |= KCAN_DEV_TS_SOF;

		} else {
			if (!ucan_clr_options(dev, UCAN_OPTION_TS_SOF, 0))
				dev->flags &= ~KCAN_DEV_TS_SOF;
		}
	} else if (sof_flag && !fallback) {
		return -ENOTSUPP;
	}

	dev->ts_mode = ts_mode;

	return 0;
}

static int kcan_set_ts_mode(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	u32 tmp;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->ts_mode));
#endif

	if (kcan_copy_from_user(&tmp, opt->value, sizeof(tmp), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	return _kcan_set_ts_mode(dev, tmp, false);
}
#endif

static int kcan_get_drv_version(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	const u32 tmp32 = KCAN_MAKE_VERSION(KCAN_VERSION_MAJOR,
						KCAN_VERSION_MINOR,
						KCAN_VERSION_SUBMINOR);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d)\n", __func__, opt->size);
#endif

	return kcan_get_opt_u32(dev, opt, c, tmp32);
}

static int kcan_get_drv_clk_ref(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d)\n", __func__, opt->size);
#endif

	return kcan_get_opt_u32(dev, opt, c, drvclkref);
}

static int kcan_get_linger(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->linger_opt_value));
#endif

	/* int are always 4 bytes long */
	return kcan_get_opt_u32(dev, opt, c, (u32 )dev->linger_opt_value);
}

static int kcan_set_linger(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	int tmp;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(dev->linger_opt_value));
#endif

	if (kcan_copy_from_user(&tmp, opt->value, sizeof(tmp), c)) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	dev->linger_opt_value = tmp;

	return 0;
}

static int kcan_get_fw_version(struct kcandev *dev,
					struct kcanfd_option *opt, void *c)
{
	const u32 tmp32 = KCAN_MAKE_VERSION(dev->hw_ver->major,
					    dev->hw_ver->minor,
					    dev->hw_ver->subminor);
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(u_size=%d k_size=%u)\n",
		__func__, opt->size, (uint )sizeof(tmp32));
#endif

	return kcan_get_opt_u32(dev, opt, c, tmp32);
}

static struct kcanfd_options kcan_def_opts[KCANFD_OPT_MAX] = 
{
	[KCANFD_OPT_CHANNEL_FEATURES] = {
		.req_size = sizeof(u32),
		.get = kcan_get_channel_features,
	},
	[KCANFD_OPT_AVAILABLE_CLOCKS] = {
		.req_size = sizeof(struct kcanfd_available_clocks_1),
		.get = kcan_get_avclocks,
	},
	[KCANFD_OPT_BITTIMING_RANGES] = {
		.req_size = sizeof(struct kcanfd_bittiming_range),
		.get = kcan_get_bittiming_range,
	},
	[KCANFD_OPT_DBITTIMING_RANGES] = {
		.req_size = sizeof(struct kcanfd_bittiming_range),
		.get = kcan_get_dbittiming_range,
	},
	[KCANFD_OPT_ALLOWED_MSGS] = {
		.req_size = sizeof(u32),
		.get = kcan_get_allowed_msgs,
		.set = kcan_set_allowed_msgs,
	},
	[KCANFD_OPT_ACC_FILTER_11B] = {
		.req_size = sizeof(u64),
		.get = kcan_get_acc_filter_11b,
		.set = kcan_set_acc_filter_11b,
	},
	[KCANFD_OPT_ACC_FILTER_29B] = {
		.req_size = sizeof(u64),
		.get = kcan_get_acc_filter_29b,
		.set = kcan_set_acc_filter_29b,
	},
	[KCANFD_OPT_IFRAME_DELAYUS] = {
		.req_size = sizeof(u32),
		.get = kcan_get_ifrm_delay_us,
		.set = kcan_set_ifrm_delay_us,
	},
#ifndef NETDEV_SUPPORT
	[KCANFD_OPT_HWTIMESTAMP_MODE] = {
		.req_size = sizeof(u32),
		.get = kcan_get_ts_mode,
		.set = kcan_set_ts_mode,
	},
#endif
	[KCANFD_OPT_DRV_VERSION] = {
		.req_size = sizeof(u32),
		.get = kcan_get_drv_version,
	},
	[KCANFD_OPT_FW_VERSION] = {
		.req_size = sizeof(u32),
		.get = kcan_get_fw_version,
	},
	[KCANFD_OPT_DRV_CLK_REF] = {
		.req_size = sizeof(u32),
		.get = kcan_get_drv_clk_ref,
	},
	[KCANFD_OPT_LINGER] = {
		.req_size = sizeof(int),	/* always 4 bytes long */
		.get = kcan_get_linger,
		.set = kcan_set_linger
	},
	[KCANFD_OPT_SELF_ACK] = {
		.req_size = sizeof(u32),
		.get = kcan_get_self_ack,
		.set = kcan_set_self_ack,
	},
	[KCANFD_OPT_BRS_IGNORE] = {
		.req_size = sizeof(u32),
		.get = kcan_get_brs_ign,
		.set = kcan_set_brs_ign,
	},
};

const struct kcanfd_options *kcan_inherit_options_from(
				struct kcanfd_options *child_opts,
				const struct kcanfd_options *parent_opts)
{
	int i;

	if (!parent_opts)
		parent_opts = kcan_def_opts;

	/* copy parent option only if child's isn't NULL */
	for (i = 0; i < KCANFD_OPT_MAX; i++) {
		if (!child_opts[i].req_size)
			child_opts[i].req_size = parent_opts[i].req_size;
		if (!child_opts[i].get)
			child_opts[i].get = parent_opts[i].get;
		if (!child_opts[i].set)
			child_opts[i].set = parent_opts[i].set;
	}

	return child_opts;
}

/* CAN 2.0 old API entry point (btr0btr1 with 8*MHz clock)
 * This function does the correct conversion of 8*MHz wBTR0BTR1 according to 
 * the default clock, then opens a CAN 2.0 channel with calling the
 * (new API) device_open_fd() entry point.
 */
static int kcan_device_open_fd_wrapper(struct kcandev *dev, u16 btr0btr1,
					u8 ext, u8 listen_only)
{
	struct kcanfd_init fdi = {};
	TKCANInit init = {
		.wBTR0BTR1 = btr0btr1,
		.ucCANMsgType = ext ? MSGTYPE_EXTENDED : MSGTYPE_STANDARD,
		.ucListenOnly = listen_only,
	};

#if defined(DEBUG_TRACE) || defined(DEBUG_OLD_API)
	pr_info(DEVICE_NAME
		": %s(CAN%u): %s(btr0btr1=%04xh, ext=%u, listen_only=%u)\n",
		dev->adapter->name, kcan_idx(dev)+1,
		__func__, btr0btr1, ext, listen_only);
#endif
	return dev->device_open_fd(dev, kcan_init_to_fd(dev, &fdi, &init));
}

/*
 * default locking mechanism to protect user task access to resources shared
 * with ISR.
 */
static void kcan_device_lock_irqsave(struct kcandev *dev,
				     kcan_lock_irqsave_ctxt *pflags)
{
	kcan_lock_get_irqsave(&dev->isr_lock, *pflags);
}

static void kcan_device_unlock_irqrestore(struct kcandev *dev,
					  kcan_lock_irqsave_ctxt *pflags)
{
	kcan_lock_put_irqrestore(&dev->isr_lock, *pflags);
}

/* init some equal parts of dev */
void kcan_soft_init_ex(struct kcandev *dev,
			const struct kcanfd_available_clocks *clocks,
			const struct kcanfd_bittiming_range *pc,
			u32 features)
{
	const u32 sysclock_Hz = clocks->list[0].clock_Hz;

	switch (dev->wType) {
	case HW_ISA:
	case HW_DONGLE_SJA:
	case HW_DONGLE_SJA_EPP:
	case HW_DONGLE_PRO:
	case HW_DONGLE_PRO_EPP:
	case HW_ISA_SJA:
	case HW_PCI:
	case HW_PCCARD:
		/* all of these old devices are SJA1000 based devices */
		features |= KCAN_DEV_ERRCNT_RDY;
		break;

	default:
		/* all of these devices have hw timestamps that can be cooked */
		features |= KCAN_DEV_HWTS_RDY|KCAN_DEV_HWTSC_RDY;
		break;
	}
	dev->features = features;
	dev->flags = 0;

	dev->bus_load_ind_period = msecs_to_jiffies(defblperiod);
	dev->bus_load_count = 0;
	dev->bus_load_total = 0;

	_kcan_set_ts_mode(dev, deftsmode, true);

	if (dev->features & KCAN_DEV_BUSLOAD_RDY) {
		kcan_setup_timer(&dev->bus_load_timer, kcan_push_bus_load_ind,
				 (unsigned long )dev);
	}

	kcan_sync_init(dev);

	/* fake to force kcan_set_bus_state(UNKNOWN) to work */
	dev->bus_state = KCANFD_ERROR_BUSOFF;
	kcanfd_dev_open_init(dev);

	dev->option = kcan_def_opts;

	dev->sysfs_dev = NULL;

	dev->nOpenPaths = 0;
	dev->nLastError = 0;
	dev->bus_error = KCANFD_ERRMSG_COUNT;
	dev->adapter = NULL;
	dev->wCANStatus = 0;
	dev->filter = NULL;
	dev->sysfs_attrs = NULL;

	dev->linger_opt_value = MAX_WAIT_UNTIL_CLOSE;

	dev->device_alt_num = 0xffffffff;

	dev->bittiming_caps = pc;
	dev->clocks_list = clocks;

	dev->sysclock_Hz = sysclock_Hz;

	/* default CAN device running fw version is the version of the
	 * real hardware equipment. KCAN-USB X6/X4 devices are quite
	 * different since they embed multiple USB interfaces that might run
	 * different FW...
	 */
	dev->hw_ver = NULL;

	memset(&dev->def_init_settings, '\0', sizeof(dev->def_init_settings));
	dev->def_init_settings.clock_Hz = sysclock_Hz;

	if (kcan_def_bitrate) {
		dev->def_init_settings.nominal.bitrate = kcan_def_bitrate;

		/* normalize default bit-timings specs */
		kcan_bittiming_normalize(&dev->def_init_settings.nominal,
					dev->sysclock_Hz,
					dev->bittiming_caps);
	} else {

		/* first, compute nominal bitrate from BTR0BTR1 */
		kcan_btr0btr1_to_bittiming(&dev->def_init_settings.nominal,
					   btr0btr1);

		/* if default clock is not 8*MHz, rebuild BTR0BTR1
		 * accordingly...
		 */
		if (dev->sysclock_Hz != 8*MHz) {

			/* compute real bittimings with real clock value */
			dev->def_init_settings.nominal.brp = 0;

			kcan_bittiming_normalize(
					&dev->def_init_settings.nominal,
					dev->sysclock_Hz,
					dev->bittiming_caps);
		}
	}

#ifdef DEBUG
	pr_info(DEVICE_NAME ": %s() btr0btr1=%04xh => nominal bitrate=%u bps\n",
		__func__, btr0btr1, dev->def_init_settings.nominal.bitrate);
#endif

	/* do the same for dbitrate */
	dev->def_init_settings.data.bitrate = kcan_def_dbitrate;
	dev->dbittiming_caps = NULL;

	kcanfd_copy_init(&dev->init_settings, &dev->def_init_settings);

	memset(&dev->props, 0, sizeof(dev->props));

	/* set default access functions: 
	 * init device_open() callback with wrapper to new device_open_fd()
	 * callback. Old controlers will then overload it with their own. 
	 * Newer (CAN FD) will then just have to provide their own
	 * device_open_fd() only.
	 */
	dev->device_open = kcan_device_open_fd_wrapper;

	/* set default locking mechanism protecting access to resources
	 * shared by ISR
	 */
	dev->lock_irq = kcan_device_lock_irqsave;
	dev->unlock_irq = kcan_device_unlock_irqrestore;

	dev->device_open_fd = NULL;
	dev->device_release = NULL;
	dev->device_write  = NULL;
	dev->cleanup = NULL;

	dev->device_params = NULL;    /* the default */

	dev->is_plugged = 1;  /* assume the device IS installed */
	dev->ucActivityState = ACTIVITY_NONE;

	/* suppose the device ready to write frames */
	kcan_lock_init(&dev->isr_lock);
	kcan_set_tx_engine(dev, TX_ENGINE_CLOSED);

	if (txqprealloc) {
		if (kcan_alloc_tx_msgs(dev))
			pr_warn(DEVICE_NAME
				"Tx fifo buffer pre-allocation failed. Will retry when device is opened\n");
	} else {
		dev->tx_msgs = NULL;
	}

	memset(&dev->tx_fifo, '\0', sizeof(dev->tx_fifo));

#ifdef NETDEV_SUPPORT
	dev->netdev = NULL;
#else

	if (rxqprealloc) {
		if (kcan_alloc_rx_msgs(dev))
			pr_warn(DEVICE_NAME
				"Rx fifo buffer pre-allocation failed. Will retry when device is opened\n");
	} else {
		dev->rx_msgs = NULL;
	}

	memset(&dev->rx_fifo, '\0', sizeof(dev->rx_fifo));

#endif
	kcan_lock_init(&dev->wlock);
	kcan_mutex_init(&dev->mutex);
}

/* create all declared Kunhong legacy devices */
static int make_legacy_devices(void)
{
	int result = 0;
	int i;

#if defined(DEBUG_TRACE)
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	for (i = 0; ((i < 8) && (type[i] != NULL)); i++) {
#ifdef DEBUG
		pr_info(DEVICE_NAME ": %s(): create devices for type=\"%s\"\n",
			__func__, type[i]);
#endif
#ifdef ISA_SUPPORT
		if (!strncmp(type[i], "isa", 4))
			result = kcan_create_isa_devices(type[i],
							io[i], irq[i]);
#endif

#ifdef DONGLE_SUPPORT
		if (!strncmp(type[i], "sp", 4) ||
					!strncmp(type[i], "epp", 4))
			result = kcan_create_dongle_devices(type[i],
							io[i], irq[i]);
#endif

		if (result)
			break;
	}

#if defined(ISA_SUPPORT) && defined(KCAN_HANDLE_IRQ_SHARING)
	/* create lists of devices with the same irqs */
	kcan_create_isa_shared_irq_lists();
#endif

	return result;
}

static int parmtoul(char *parm, u32 *pv)
{
	if (parm[0] == '0' && (parm[1] == 'x' || parm[1] == 'X')) {
		char *endptr = parm;
		u32 v = simple_strtoul(parm, &endptr, 16);
		if (*endptr)
			return -EINVAL;

		if (pv)
			*pv = v;
		return 'x';
	}

	return (strtounit(parm, pv, "kM") < 0) ? -EINVAL : 'd';
}

/* called when the device is installed 'insmod kcan.o' or 'insmod kcan.ko' */
static int __init kcan_init_module(void)
{
	int result = 0;

	memset(&kcan_drv, 0, sizeof(kcan_drv));
	kcan_drv.wInitStep = 0;

	/* in this version, "bitrate" parameter (and new "dbitrate" parameter)
	 * is a string parameter.
	 * Rule (for compatibility purpose):
	 * - if the string starts with "0x" and if the value is < 0xffff,
	 *   then it is considered as a BTR0BTR1 value
	 * - otherwise, the parameter should be a numeric value, optionaly
	 *   followed by "M" (mega) or "k" (kilo), to specify a new default
	 *   value for the nominal bitrate of the CAN channels.
	 */
	if (bitrate &&
	    (parmtoul(bitrate, &kcan_def_bitrate) == 'x') &&
	    (kcan_def_bitrate <= 0xffff)) {
		btr0btr1 = (u16 )kcan_def_bitrate;
		kcan_def_bitrate = 0;
	}

	if (dbitrate)
		strtounit(dbitrate, &kcan_def_dbitrate, "kM");

#ifdef KCAN_DEV_RXQSIZE_MIN
	/* check whether Rx/Tx queue default sizes are ok */
	if (rxqsize < KCAN_DEV_RXQSIZE_MIN)
		rxqsize = (ushort )KCAN_DEV_RXQSIZE_MIN;
#endif
#ifdef KCAN_DEV_RXQSIZE_MAX
	if (rxqsize > KCAN_DEV_RXQSIZE_MAX)
		rxqsize = (ushort )KCAN_DEV_RXQSIZE_MAX;
#endif
#ifdef KCAN_DEV_TXQSIZE_MIN
	if (txqsize < KCAN_DEV_TXQSIZE_MIN)
		txqsize = (ushort )KCAN_DEV_TXQSIZE_MIN;
#endif
#ifdef KCAN_DEV_TXQSIZE_MAX
	if (txqsize > KCAN_DEV_TXQSIZE_MAX)
		txqsize = (ushort )KCAN_DEV_TXQSIZE_MAX;
#endif
#ifdef KCANFD_TX_HIWAT_MIN
	if (txqhiwat < KCANFD_TX_HIWAT_MIN)
		txqhiwat = (ushort )KCANFD_TX_HIWAT_MIN;
#endif
#ifdef KCANFD_TX_HIWAT_MAX
	if (txqhiwat > KCANFD_TX_HIWAT_MAX)
		txqhiwat = (ushort )KCANFD_TX_HIWAT_MAX;
#endif
	if ((dmamask < KCAN_DEV_DMA_MASK_LOW) ||
			(dmamask > KCAN_DEV_DMA_MASK_HIGH))
		dmamask = KCAN_DEV_DMA_MASK_DEF;

#ifndef NETDEV_SUPPORT
	if (deftsmode > KCANFD_OPT_HWTIMESTAMP_MAX)
		deftsmode = KCANFD_OPT_HWTIMESTAMP_MAX;
#endif

	/* check driver clock reference */
	switch (drvclkref) {
	case CLOCK_REALTIME:		/* 0 */
	case CLOCK_MONOTONIC:		/* 1 */
	case CLOCK_MONOTONIC_RAW:	/* 4 */
	case CLOCK_BOOTTIME:		/* 7 */
		break;

	default:
		drvclkref = KCANFD_DRIVER_CLOCK_DEF;
		break;
	}

#ifdef RTAI
	/* this should be done at least once */
	start_rt_timer(0);
#endif
	/* store time for timestamp relation, increments in usec */
	kcan_gettimeofday(&kcan_drv.sInitTime);

	/* get the release name global */
	kcan_drv.szVersionString = CURRENT_RELEASE;
	kcan_drv.nMajor = KCAN_MAJOR;

	pr_info(DEVICE_NAME ": %s (%s)\n", kcan_drv.szVersionString,
#if defined(__BIG_ENDIAN)
		"be"
#else
		"le"
#endif
		);

	pr_info(DEVICE_NAME ": driver config%s\n", current_config);
	if (!kcan_drv.sInitTime.tv_sec && !kcan_drv.sInitTime.tv_usec)
		pr_warn(DEVICE_NAME ": WARNING: got abnormal NULL time\n");

#ifdef DEBUG
	pr_info(DEVICE_NAME ": driver start time=%u.%06u s.\n",
		(u32 )kcan_drv.sInitTime.tv_sec,
		(u32 )kcan_drv.sInitTime.tv_usec);
	pr_info(DEVICE_NAME ": DEBUG is switched on\n");

	/* give a quick descr of some module parameters values */
#ifndef NETDEV_SUPPORT
	pr_info(DEVICE_NAME ": rxqsize=%u\n", rxqsize);
	pr_info(DEVICE_NAME ": rxqprealloc=%u\n", rxqprealloc);
#endif
	pr_info(DEVICE_NAME ": txqsize=%u\n", txqsize);
	pr_info(DEVICE_NAME ": txqprealloc=%u\n", txqprealloc);
	pr_info(DEVICE_NAME ": txqhiwat=%u\n", txqhiwat);
#endif

	/* Copy the centered string only once and use sizeof() for
	 * compiletime value calculation and optimisation. Also ensure
	 * to have a valid current_config and that it fits into config[]
	 */
	if ((sizeof(current_config) > 3) &&
				(sizeof(config) > sizeof(current_config)))
		memcpy(config + (sizeof(config)-sizeof(current_config))/2,
			current_config,
			strlen(current_config));

	INIT_LIST_HEAD(&kcan_drv.devices);
	kcan_drv.wDeviceCount = 0;

#ifdef HANDLE_HOTPLUG
	/* initialize mutex used to access kcan devices list */
	kcan_lock_init(&kcan_drv.devices_lock);
#endif

	/* register the driver by the OS */
#ifdef NO_RT
	result = register_chrdev(kcan_drv.nMajor, DEVICE_NAME, &kcan_fops);
	if (result < 0) {
#ifdef HANDLE_HOTPLUG
		kcan_lock_destroy(&kcan_drv.devices_lock);
#endif
		goto fail;
	}
	if (!kcan_drv.nMajor)
		kcan_drv.nMajor = result;
#else
	INIT_LIST_HEAD(&kcan_rtdm_dev_list);
#endif

#ifdef SYSFS_SUPPORT
	/* register /sys/class/kcan */
	if (class_register(&kcan_class))
		goto fail;

	kcan_drv.class = &kcan_class;
#endif

	kcan_drv.wInitStep = 1;

#ifdef PCI_SUPPORT
#ifdef KCAN_PCI_EVENT_DRIVEN
	kcan_pci_init();
#else
	kcan_search_and_create_pci_devices();
#endif
	/* search pci devices */
#endif

	/* create isa and dongle devices */
	make_legacy_devices();

#ifdef USB_SUPPORT
	/* register usb devices only */
	kcan_usb_register_devices();
#endif

#ifdef PCCARD_SUPPORT
	kcan_pccard_register_devices();
#endif

#if !defined USB_SUPPORT && !defined PCCARD_SUPPORT
	/* no device found, stop all */
	if (!kcan_drv.wDeviceCount)
		goto fail;
#endif

	kcan_drv.wInitStep = 2;

	result = DEV_REGISTER();
	if (result < 0)
		goto fail;

	if (!kcan_drv.nMajor)
		kcan_drv.nMajor = result;

	kcan_drv.wInitStep = 3;

#ifdef CREATE_PROC_ENTRY_DEPRECATED
	proc_file_entry = proc_create(DEVICE_NAME, 0, NULL, &proc_file_fops);
	if (!proc_file_entry) {
		result = -ENOMEM;
		goto fail;
	}
#else
	/* create the proc entry */
	if (create_proc_read_entry(DEVICE_NAME, 0, NULL,
					kcan_read_procmem, NULL) == NULL) {
		/* maybe wrong if there is no proc filesystem configured */
		result = -ENODEV;
		goto fail;
	}
#endif
	kcan_drv.wInitStep = 4;

	pr_info(DEVICE_NAME ": major %d.\n", kcan_drv.nMajor);

	/* succeed */
	return 0;

fail:
	kcan_cleanup_module();
	return result;
}

module_init(kcan_init_module);
module_exit(kcan_cleanup_module);
