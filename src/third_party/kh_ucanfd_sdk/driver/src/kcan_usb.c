/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_usb.c - the inner parts for kcan-usb support
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
/*#define DEBUG*/
/*#undef DEBUG*/

#include "src/kcan_common.h"

#ifdef USB_SUPPORT

#include <linux/sched.h>
#include <linux/net.h>
#include "src/kcan_main.h"
#include "src/kcan_fifo.h"
#include "src/kcan_usb.h"
#include <asm/byteorder.h>

#ifdef NETDEV_SUPPORT
#include "src/kcan_netdev.h"     /* for hotplug kcan_netdev_register() */
#endif

#include "src/kcan_usbpro_fw.h"

#ifdef DEBUG_USB_LITE
#define DEBUG_TRACE
#define DEBUG_BUS_MODE
#ifdef DEBUG_USB
#define DEBUG_ENCODE
#define DEBUG_DECODE
#define DEBUG_TIMESTAMP
#endif
#endif

/* if defined, then bus_state is set to ACTIVE when the driver receives the
 * very first timestamp sync from the KCAN-USB. This enables to flush the tx
 * fifo quicker.
 * if not defined, then bus_state is set to ACTIVE when the driver receives the
 * first STATUS message. This one can be received up to 1 s after!
 */
#define BUS_STATE_ACTIVE_ON_1ST_TS

/* USB control cmds get/set */
#define KCAN_USB_GET		0x01
#define KCAN_USB_SET		0x02

/* KCAN-USB commands */
#define KCAN_USB_CMD_BITRATE	1
#define KCAN_USB_CMD_CLOCK	2
#define KCAN_USB_CMD_BUS	3
#define KCAN_USB_CMD_DEVID	4
#define KCAN_USB_CMD_SN		6
#define KCAN_USB_CMD_REGISTER	9
#define KCAN_USB_CMD_EXT_VCC	10
#define KCAN_USB_CMD_ERR_FR	11
#define KCAN_USB_CMD_LED	12
#define KCAN_USB_CMD_DEVDATA	30

/* KCAN_USB_CMD_BUS KCAN_USB_SET extension: */
#define KCAN_USB_SET_SILENT_MODE	3

/* bit masks for status/length field in a USB message */
#define STLN_WITH_TIMESTAMP	0x80
#define STLN_INTERNAL_DATA	0x40
#define STLN_EXTENDED_ID	0x20
#define STLN_RTR		0x10
#define STLN_DATA_LENGTH	0x0F         /* mask for length of data bytes */

/* Error-Flags for KCAN-USB */
#define XMT_BUFFER_FULL		0x01
#define CAN_RECEIVE_OVERRUN	0x02
#define BUS_LIGHT		0x04
#define BUS_HEAVY		0x08
#define BUS_OFF			0x10
#define QUEUE_RECEIVE_EMPTY	0x20
#define QUEUE_OVERRUN		0x40
#define QUEUE_XMT_FULL		0x80

#define KCAN_USB_ECHO_ID	0x80

/* timestamp calculation stuff:
 * converting ticks into us is done by scaling the number of ticks per us:
 * as one tick duration is 42.666 us, use 42666 us for 1000 ticks.
 * two methods are proposed here:
 * 1. fast: instead of div, use 2^20 shift method:
 *    (tick_number * 44739) >> 20 <~> (tick_number * 42666) / 1000000
 *    this gives same result with a 10^-7 precision
 * 2. accurate: use the 64-bits division
 *    (tick_number * 1024) / 24000
 */
//#define KCAN_USB_TS_DIV_SHIFTER          20  // shift faster than div
#ifdef KCAN_USB_TS_DIV_SHIFTER
#define KCAN_USB_TS_US_PER_TICK		44739243
#else
#define KCAN_USB_TS_SCALE_MULTIPLIER	42667
#define KCAN_USB_TS_SCALE_DIVISOR	1000
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,8)
#define TICKS(msec)			((msec * HZ) / 1000)
#endif

#define COMMAND_TIMEOUT			500  /* timeout for EP0 control urbs */

/* USB Mass Storage Mode command (FW >= 8.3.0) */
#define	KCAN_USB_SETCAN2FLASH		0xC8

/* kcan-usb parameter get an set function */
typedef struct {
	u8  Function;
	u8  Number;
	u8  Param[14];
} __attribute__ ((packed)) KCAN_USB_PARAM;

static int kcan_usb_devices = 0;

/* functions to handle time construction */
static void kcan_reset_timestamp(struct kcandev *dev)
{
	struct kcan_usb_time *t = &dev->port.usb.time;

	memset(t, '\0', sizeof(struct kcan_usb_time));
}

/* KCAN-USB syncs every ~1.050 s ~23437 ticks */
#define KCAN_USB_SYNC_PERIOD_US		USEC_PER_SEC
#define KCAN_USB_SYNC_PERIOD_TICK	((KCAN_USB_SYNC_PERIOD_US * \
					  KCAN_USB_TS_SCALE_DIVISOR) / \
						KCAN_USB_TS_SCALE_MULTIPLIER)
#ifdef DEBUG_TIMESTAMP
static int already_dbg_wrapp = 0;
#endif

static void kcan_ticks_to_ts(u32 ticks, u32 *ts_low, u32 *ts_high)
{
	u64 ts = ticks;

	ts *= KCAN_USB_TS_SCALE_MULTIPLIER;

	ts = DIV_ROUND_UP_SECTOR_T(ts, KCAN_USB_TS_SCALE_DIVISOR);

	*ts_high = (u32 )(ts >> 32);
	*ts_low = (u32 )ts;
}

static void kcan_updateTimeStampFromWord(struct kcandev *dev, u16 ticks,
						struct kcan_timeval *tv)
{
	struct kcan_usb_time *t = &dev->port.usb.time;
	u32 ts_low, ts_high;
	u32 ticks_count;

	if (!t->sync_ticks_init)
		return;

	ticks_count = t->sync_ticks_high;

	/* if received tick16 is smaller than the last sync tick low 16-bit,
	 * then the tick counter may have wrapped.
	 * Note: during this sync period, we are able to receive several
	 * smaller ticks16
	 */
	if (ticks < t->sync_ticks_low) {

		/* if the new tick is smaller than the sync period,
		 * then the counter has wrapped.
		 */
		if (ticks < KCAN_USB_SYNC_PERIOD_TICK) {

			/* Because the driver might receive event(s)
			 * AFTER having received the calibration msg
			 * *BUT* with a timestamp LOWER than the
			 * ticks of the calibration msg, we must
			 * consider wrapping only if event ticks is
			 * REALLY lower than calibration ticks:
			 * the counter has wrapped if NEXT calibration
			 * ticks will wrapp too
			 */
			u16 next_ticks = t->sync_ticks_low +
						 KCAN_USB_SYNC_PERIOD_TICK;
			if (next_ticks < t->sync_ticks_low) {
				ticks_count += 0x10000;
#ifdef DEBUG_TIMESTAMP
				if (!already_dbg_wrapp) {
					already_dbg_wrapp++;
					pr_info(DEVICE_NAME
						": tick16 wrapped: %u < %u\n",
						ticks, t->sync_ticks_low);
				}
			} else already_dbg_wrapp = 0;
		} else already_dbg_wrapp = 0;
	} else already_dbg_wrapp = 0;
#else
			}
		}
	}
#endif

	/* keep in memory the low 16-bit of the current tick value, 
	 * to detect when 8-bit value wrapps */
	t->ticks16 = ticks;

	/* convert the tick value of this event into µs timestamp: */
	ticks_count += ticks;
	kcan_ticks_to_ts(ticks_count, &ts_low, &ts_high);

#ifdef DEBUG_TIMESTAMP
	pr_info(DEVICE_NAME ": "
		//"ticks=%u ticks_count=%u; "
		"decoding ts=%u-%08u µs\n",
		//ticks, ticks_count,
		ts_high, ts_low);
#endif
	/* then, decode this timestamp according to current clock drift */
	kcan_sync_decode(dev, ts_low, ts_high, tv);
}

static void kcan_updateTimeStampFromByte(struct kcandev *dev, u8 ticks8,
						struct kcan_timeval *tv)
{
	struct kcan_usb_time *t = &dev->port.usb.time;
	u8 hsb = t->ticks16 >> 8;

	/* convert this tick8 into tick16 */
	if (t->sync_ticks_init) {
		const u8 lsb = t->ticks16 & 0xff;

		if (ticks8 < lsb) {
			hsb++;
#ifdef DEBUG_TIMESTAMP
			pr_info(DEVICE_NAME
				": ticks8 wrapped: %u < %u: "
				"tick16=%u\n",
				ticks8, lsb, (u16 )hsb << 8 | ticks8);
#endif
		}

		kcan_updateTimeStampFromWord(dev, (u16 )hsb << 8 | ticks8, tv);
	}
}

static int kcan_usb_sync_times(struct kcandev *dev, u16 ticks, u32 dtv)
{
	struct kcan_usb_time *t = &dev->port.usb.time;
	int err = 0;

	if (t->sync_ticks_init) {

		/* with calibration message, we're certain that this test is
		 * enough to detect wrapping:
		 */
		if (ticks < t->sync_ticks_low) {
			t->sync_ticks_high += 0x10000;
#ifdef DEBUG_TIMESTAMP
			pr_info(DEVICE_NAME ": sync: ticks wrapped: %u < %u: "
				"ticks_high=%u\n",
				ticks, t->sync_ticks_low,
				t->sync_ticks_high);
#endif
		}
	} else {
		t->sync_ticks_init = ticks;
		t->ticks16 = ticks;
	}

	t->sync_ticks_low = ticks;

#ifdef NETDEV_SUPPORT
	/* no need to sync anything in netdev mode since timestamps are given
	 * by socket-can while hw timestamps are raw one
	 */
#else
	{
		const u32 ticks_count = t->sync_ticks_high + t->sync_ticks_low;
		u32 ts_low, ts_high;

		/* convert now ticks into µs and use common sync mechanism */
		kcan_ticks_to_ts(ticks_count, &ts_low, &ts_high);

#ifdef DEBUG_TIMESTAMP
		pr_info(DEVICE_NAME
			": sync: %u tks => ts_high=%u ts_low=%08u\n",
			ticks, ts_high, ts_low);
#endif

		err = kcan_sync_times(dev, ts_low, ts_high, dtv);
	}
#endif
	return err;
}

static int kcan_usb_send_cmd(struct kcandev *dev)
{
	struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);
	KCAN_USB_PARAM *cmd = (KCAN_USB_PARAM *)dev->port.usb.cout_baddr;
	int err, l;

	/* don't do anything with non-existent hardware */
	if (!dev->is_plugged)
		return -ENODEV;

#ifdef DEBUG_USB
	dump_mem("Snd cmd", cmd, sizeof(KCAN_USB_PARAM));
#endif

	err = usb_bulk_msg(usb_if->usb_dev,
			   usb_sndbulkpipe(usb_if->usb_dev,
	                   		usb_if->pipe_cmd_out.ucNumber),
			   cmd, sizeof(KCAN_USB_PARAM), &l, COMMAND_TIMEOUT);
	if (err)
		pr_err(DEVICE_NAME
		       ": err %d submitting cmd %d.%d to KCAN-USB\n",
		       err, cmd->Function, cmd->Number);

	return err;
}

static int kcan_usb_setcontrol_urb(struct kcandev *dev, u8 function, u8 number,
                                   u8 param0, u8 param1, u8 param2, u8 param3)
{
	KCAN_USB_PARAM *cmd = (KCAN_USB_PARAM *)dev->port.usb.cout_baddr;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": %s(): [func=%d num=%d %02X %02X %02X]->EP#%02X\n",
		__func__, function, number, param0, param1, param2,
		kcan_usb_get_if(dev)->pipe_cmd_out.ucNumber);
#endif

	memset(cmd, '\0', sizeof(KCAN_USB_PARAM));

	cmd->Function = function;
	cmd->Number = number;
	cmd->Param[0] = param0;
	cmd->Param[1] = param1;
	cmd->Param[2] = param2;
	cmd->Param[3] = param3;

	return kcan_usb_send_cmd(dev);
}

static int kcan_usb_getcontrol_urb(struct kcandev *dev, u8 function, u8 number,
				   u8 *param0, u8 *param1, u8 *param2,
				   u8 *param3)
{
	struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);
	KCAN_USB_PARAM *cmd = (KCAN_USB_PARAM *)dev->port.usb.cout_baddr;
	int err, l, i;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(cmd %d.%d): <-EP#%02X\n",  __func__,
	        function, number, usb_if->pipe_cmd_in.ucNumber);
#endif

	/* sometimes, need to send the cmd more than once... */
	for (i = 0; i < 3; i++) {

		/* don't do anything with non-existent hardware */
		if (!dev->is_plugged) {
			err = -ENODEV;
			break;
		}

		/* first write function and number to device */
		err = kcan_usb_setcontrol_urb(dev, function, number,
					      (param0) ? *param0 : 0,
					      (param1) ? *param1 : 0,
					      (param2) ? *param2 : 0,
					      (param3) ? *param3 : 0);
		if (err)
			break;

		err = usb_bulk_msg(usb_if->usb_dev,
				   usb_rcvbulkpipe(usb_if->usb_dev,
					   usb_if->pipe_cmd_in.ucNumber),
				   cmd, sizeof(KCAN_USB_PARAM), &l,
				   COMMAND_TIMEOUT);
		if (!err) {
#ifdef DEBUG_USB
			dump_mem("Got rsp", cmd, l);
#endif
			if (param0) {
				*param0 = cmd->Param[0];
				if (param1) {
					*param1 = cmd->Param[1];
					if (param2) {
						*param2 = cmd->Param[2];
						if (param3)
							*param3 = cmd->Param[3];
					}
				}
			}
			break;
		}

		if (!dev->is_plugged)
			err = -ENODEV;

		/* retry only in case of timeout */
		if (err != -ETIMEDOUT)
			break;

#ifdef DEBUG_USB
		pr_err(DEVICE_NAME
		       ": timeout reading cmd %d.%d result from KCAN-USB\n",
		       function, number);
#endif
	}

#ifndef DEBUG_USB
	if (err)
#endif
		pr_err(DEVICE_NAME
		       ": err %d reading cmd %d.%d result from KCAN-USB\n",
		       err, function, number);

	return err;
}

#define KCAN_USB_ERR_ECC	0x01
#define KCAN_USB_ERR_RXERR	0x02
#define KCAN_USB_ERR_TXERR	0x04
#define KCAN_USB_ERR_RXERR_CNT	0x08
#define KCAN_USB_ERR_TXERR_CNT	0x10

static int kcan_usb_set_err_frame(struct kcandev *dev, u8 err_mask)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(err_mask=%02xh)\n", __func__, err_mask);
#endif

	return kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_ERR_FR, KCAN_USB_SET,
				       err_mask, 0, 0, 0);
}

static int kcan_usb_setBTR0BTR1(struct kcandev *dev, u16 wBTR0BTR1)
{
	u8 btr0 = (u8)(wBTR0BTR1 & 0xff);
	u8 btr1 = (u8)((wBTR0BTR1 >> 8) & 0xff);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(wBTR0BTR1=%04xh)\n", __func__, wBTR0BTR1);
#endif

	return kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_BITRATE, KCAN_USB_SET,
				       btr0, btr1, 0, 0);
}

static int kcan_usb_setCANOn(struct kcandev *dev)
{
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_MODE)
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

#ifndef NETDEV_SUPPORT
	/* be sure to resync each time we open */
	kcan_sync_init(dev);
#endif

	err = kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_BUS, KCAN_USB_SET,
				       1, 0, 0, 0);

	if (!err && kcan_usb_get_if(dev)->ucRevision <= 28) {

		/* simulate ERROR_ACTIVE since FW 2.8 does not send any */
		kcan_handle_error_active(dev, NULL);
	}

	return err;
}

static int kcan_usb_setCANOff(struct kcandev *dev)
{
	int err;

#if defined(DEBUG_TRACE) || defined(DEBUG_BUS_MODE)
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	/* wait a bit for last data to be written on CAN bus:
	 * - 5 ms is not enough if any data buffer was almost filled
	 * - 10 ms is not enough for KCAN-USB (fw 2.8)
	 * - 20 ms is ok for KCAN-USB fw 2.8.
	 *
	 * Note: this wait MUST not be interruptible.
	 */
	if (dev->tx_frames_counter > 0)
		mdelay(20);

	/* from WIN driver: fw <= 2.5 need IRQ enable off before setting CAN
	 * to Off */
	if (kcan_usb_get_if(dev)->ucRevision <= 25) {
		err = kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_REGISTER,
					      KCAN_USB_SET, 4,
					      0, 0, 0);
		if (err)
			return err;
	}

	/* CAN to Off */
	err = kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_BUS, KCAN_USB_SET,
				      0, 0, 0, 0);
	if (err)
		return err;

	err = kcan_usb_set_err_frame(dev, 0);
	if (err)
		return err;

	/* code below means that fw 8.x don't need to put SJA1000 into init
	 * mode? WTF? */

	/* or if (err != -ENODEV) { */
	if (dev->is_plugged) {
		struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);

		err = usb_clear_halt(usb_if->usb_dev,
				usb_sndbulkpipe(usb_if->usb_dev,
						usb_if->pipe_cmd_out.ucNumber));
#ifdef DEBUG_USB
		if (err)
			pr_warn(DEVICE_NAME
				": %s(): usb_clear_halt(1) failed err %d\n",
				__func__, err);
#endif

		err = usb_clear_halt(usb_if->usb_dev,
				usb_rcvbulkpipe(usb_if->usb_dev,
						usb_if->pipe_cmd_in.ucNumber));
		if (err) {
#ifdef DEBUG_USB
			pr_warn(DEVICE_NAME
				": %s(): usb_clear_halt(2) failed err %d\n",
				__func__, err);
#endif
			return err;
		}

		mdelay(50);
	}

	/* set SJA1000 in init mode */
	return kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_REGISTER, KCAN_USB_SET,
				       0, 1, 0, 0);
}

static int kcan_usb_setCANSilentMode(struct kcandev *dev, bool onoff)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(onoff=%d)\n", __func__, onoff);
#endif

	return kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_BUS,
				       KCAN_USB_SET_SILENT_MODE, onoff,
				       0, 0, 0);
}

static int kcan_usb_getDeviceNr(struct kcandev *dev, u32 *pdwDeviceNr)
{
	int err;
	u8 dev_nr = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	*pdwDeviceNr = 0;

	err = kcan_usb_getcontrol_urb(dev, KCAN_USB_CMD_DEVID, KCAN_USB_GET,
				      &dev_nr, NULL, NULL, NULL);
        if (err)
		return err;

	dev->flags &= ~KCAN_DEV_USES_ALT_NUM;
	if ((dev_nr != 255)

		/* New versions of KCAN-USB Classic uses 0 as default value:
		 *
		 * New from 8.10:
		 *
		 * - in netdev, usage of devid is driven by assign=devid
		 *   parameter. Therefore, we consider that the user knows what
		 *   he is doing. Moreover, he must be capable of setting
		 *   devid=0 if he wants. Thus, 0x00 ISNOT considered as
		 *   "unspecified" in netdev mode.
		 *
		 * - in chardev, Udev doesn't create symlink when devid=0.
		 *   Thus, the driver doesn't need to consider 0x00 as
		 *   "unspecified" value while Udev does. If one wants that a
		 *   symlink devid=0 be created then he'll have to modifiy
		 *   udev/45-kcan.rules.
		 */
		 ) {

		*pdwDeviceNr = dev_nr;
		dev->flags |= KCAN_DEV_USES_ALT_NUM;
	}

	return err;
}

static int kcan_usb_setDeviceNr(struct kcandev *dev, u32 dwDeviceNr)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	/* KCAN-USB only supports 8-bit device numbers */
	if (dwDeviceNr > 255)
		return -EINVAL;

	return kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_DEVID, KCAN_USB_SET,
				       (u8 )dwDeviceNr, 0, 0, 0);
}

static int kcan_usb_setExtVCCOff(struct kcandev *dev)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	return kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_EXT_VCC, KCAN_USB_SET,
				       0, 0, 0, 0);
}

static int kcan_usb_getSNR(struct kcan_usb_interface *usb_if, u32 *pdwSNR)
{
	struct kcandev *dev = usb_if_dev(usb_if, 0);
	ULCONV SNR;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	memset(&SNR, 0, sizeof(SNR));
	err = kcan_usb_getcontrol_urb(dev, KCAN_USB_CMD_SN, KCAN_USB_GET,
				      &SNR.uc[3], &SNR.uc[2],
				      &SNR.uc[1], &SNR.uc[0]);

	*pdwSNR = SNR.ul;

	return err;
}

/* hw_type (MCU) */
enum {
	KCAN_MCU_USB_KUNHONG_C161,		/* fw < 8.0, *.can files */
	KCAN_MCU_USB_KUNHONG_LPC1346 = 0,	/* fw >= 8.0, MSD */
	KCAN_MCU_USB_LENZE,
	KCAN_MCU_USB_DC1,
	KCAN_MCU_USB_DC1_HUB,

	KCAN_MCU_USB_LPC1346 = 10,	/* v5, MSD */
	KCAN_MCU_USB_LPC51U68,		/* v6, MSD */
};

/* feature_flag */
#define KCAN_USB_FEATURE_SDO_DOWNLOAD	0x00000001
#define KCAN_USB_FEATURE_BUSPOWER	0x00000002
#define KCAN_USB_FEATURE_GUID		0x00000004

#define KCAN_USB_VENDOR_REQ_SIZE_V4	16
#define KCAN_USB_VENDOR_REQ_SIZE_V5	(KCAN_USB_VENDOR_REQ_SIZE_V4 + 16)

struct kcan_usb_vendor_request {
	__le16	hw_type;
	u8	version[3]; /* Main idx= 0 Sub idx=1 ( Build idx=2 ), 2,9,0 */
	u8	time[3];
	u8	datum[4];
	__le32	feature_flag;

	/* v5 includes a GUID */
	u8	guid[16];
} __attribute__ ((packed));

static int kcan_usb_get_hwver(struct kcan_usb_interface *usb_if)
{
	struct kcan_usb_vendor_request req;
	int len;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	len = kcan_usbpro_request(usb_if, USB_VENDOR_REQUEST_INFO,
				  0, &req, sizeof(req));

	if (len >= KCAN_USB_VENDOR_REQ_SIZE_V4) {

		usb_if->mcu = le16_to_cpu(req.hw_type);
		usb_if->hw_ver.major = req.version[0];
		usb_if->hw_ver.minor = req.version[1];
		usb_if->hw_ver.subminor = req.version[2];

		/* Handle possibility of GUID. Copy it only if it is not
		 * already known (which is not possible in case of KCAN-USB,
		 * but we never know...)
		 */
		if ((len >= KCAN_USB_VENDOR_REQ_SIZE_V5) &&
		    (req.feature_flag & KCAN_USB_FEATURE_GUID) &&
		    !(usb_if->adapter->flags & KCAN_ADAPTER_GUID)) {
			usb_if->adapter->flags |= KCAN_ADAPTER_GUID;
			memcpy(usb_if->adapter->guid, req.guid,
			       sizeof(usb_if->adapter->guid));
		}

		return 0;
	}

	return len;
}

static int kcan_usb_get_led(struct kcandev *dev)
{
	u8 led = 0xff;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	err = kcan_usb_getcontrol_urb(dev, KCAN_USB_CMD_LED, KCAN_USB_GET,
				      &led, NULL, NULL, NULL);

	return !err ? led : err;
}

static int kcan_usb_set_led(struct kcandev *dev, int led)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	err = kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_LED, KCAN_USB_SET,
				      (u8 )led, 0, 0, 0);

	return !err ? led : err;
}

static int kcan_usb_identify(struct kcandev *dev, u32 delay_ms)
{
	int err, led0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	/* get current state of the LED */
	led0 = kcan_usb_get_led(dev);
	if (led0 < 0)
		return led0;

	if (delay_ms) {
		unsigned int period_ms = 40;	/* ms */
		int blink_count = delay_ms / period_ms;
		int i, led;

		/* loop switching the state from on to off */
		for (i = 0, led = !led0; i < blink_count; i++, led ^= 1) {
			err = kcan_usb_set_led(dev, led);
			if (err < 0)
				break;

			kcan_msleep_interruptible(period_ms);
		}
	}

	/* restore LED state */
	err = kcan_usb_set_led(dev, led0);

	return !err ? led0 : err;
}

static int kcan_usb_reset(struct kcandev *dev)
{
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s)\n", __func__, dev->adapter->name);
#endif

	/* BUS=OFF */
	err = kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_BUS, KCAN_USB_SET,
				      0, 0, 0, 0);
	if (err)
		return err;

	/* BUS=ON */
	return kcan_usb_setcontrol_urb(dev, KCAN_USB_CMD_BUS, KCAN_USB_SET,
				       1, 0, 0, 0);
}

static int kcan_usb_Init(struct kcandev *dev, u16 btr0btr1,
						u8 use_ext, u8 bListenOnly)
{
	int err = 0;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": %s(btr0btr1=%04xh use_ext=%u bListenOnly=%u)\n",
		__func__, btr0btr1, use_ext, bListenOnly );
#endif

	err = kcan_usb_setBTR0BTR1(dev, btr0btr1);
	if (err)
		goto fail;

	err = kcan_usb_set_err_frame(dev,
				KCAN_USB_ERR_ECC|
				KCAN_USB_ERR_RXERR|KCAN_USB_ERR_TXERR|
				KCAN_USB_ERR_RXERR_CNT|KCAN_USB_ERR_TXERR_CNT);
	if (err)
		pr_info(DEVICE_NAME
			": WARNING: unable to get Rx/Tx counters changes "
			"(err %d)\n", err);

	if (kcan_usb_get_if(dev)->ucRevision > 3) {

		/* set listen only */
		err = kcan_usb_setCANSilentMode(dev, !!bListenOnly);
		if (err)
			goto fail;
	} else {
		/* generate err if one tries to set bListenOnly */
		if (bListenOnly) {
			err = -EINVAL;
			goto fail;
		}
	}

	/* don't know how to handle - walk the save way */
	err = kcan_usb_setExtVCCOff(dev);

	/* prepare for new start of timestamp calculation */
	kcan_reset_timestamp(dev);

	dev->port.usb.dwTelegramCount = 0;

fail:
	return err;
}

/* takes USB-message frames out of ucMsgPtr, decodes and packs them into
 * rx_fifo
 */
static int kcan_usb_decode_msgs(struct kcan_usb_interface *usb_if,
					u8 *ucMsgPtr, int lCurrentLength)
{
	int err = 0;
	int i, j;
	u8 ucMsgPrefix;
	u8 ucMsgLen;         /* number of frames in one USB packet */
	u8 ucStatusLen = 0;  /* status/length entry leading each data frame */
	u8 ucLen;            /* len in bytes of received (CAN) data */
	UWCONV wTimeStamp;
	u8 *ucMsgStart = ucMsgPtr; /* for overflow compare */
#ifndef NETDEV_SUPPORT
	int rwakeup = 0;
#endif
	u8 *org = ucMsgPtr;
	int ts8 = 0;
	struct kcandev *dev = usb_if_dev(usb_if, 0);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%p, %d)\n",
		__func__, ucMsgPtr, lCurrentLength);
#endif

#ifdef DEBUG_DECODE
	dump_mem("received buffer", ucMsgPtr, lCurrentLength);
#endif

	dev->rx_irq_counter++;

	/* get prefix of message and step over */
	ucMsgPrefix = *ucMsgPtr++;

	/* get length of message and step over */
	ucMsgLen = *ucMsgPtr++;

#ifdef DEBUG_DECODE
	pr_info(DEVICE_NAME ": MsgPrefix=%02Xh MsgLen=%u\n",
		ucMsgPrefix, ucMsgLen);
#endif
	for (i = 0; i < ucMsgLen; i++) {
		ULCONV localID;
		struct kcanfd_rxmsg rx;

		ucStatusLen = *ucMsgPtr++;

		ucLen = ucStatusLen & STLN_DATA_LENGTH;

		memset(&rx.msg, '\0', sizeof(rx.msg));

		/* store real DLC read from bus */
		kcanfd_msg_store_dlc(&rx.msg, ucLen);

		/* limit amount of data accordingly */
		rx.msg.data_len = (ucLen > KCANFD_CAN20_MAXDATALEN) ?
				KCANFD_CAN20_MAXDATALEN : ucLen;

#ifdef DEBUG_DECODE
		pr_info(DEVICE_NAME
			": - [%u]: StatusLen=%02X (Status=%d Len=%u)\n",
			i, ucStatusLen,
			!!(ucStatusLen & STLN_INTERNAL_DATA),
			ucLen);
		if (ucStatusLen & STLN_INTERNAL_DATA) {
			pr_info(DEVICE_NAME ": Function=%u Number=%u\n",
				ucMsgPtr[0], ucMsgPtr[1]);

			if (ucStatusLen & STLN_WITH_TIMESTAMP) {
				if (!ts8)
					pr_info(DEVICE_NAME ": Ts=%02x %02x\n",
						ucMsgPtr[2], ucMsgPtr[3]);
				else
					pr_info(DEVICE_NAME ": Ts=%02x\n",
						ucMsgPtr[2]);
			}
		}
#endif

		/* normal CAN messages are always with timestamp */
		if (!(ucStatusLen & STLN_INTERNAL_DATA)) {
			int nRtrFrame;

			rx.msg.type = KCANFD_TYPE_CAN20_MSG;
			rx.msg.flags |= KCANFD_MSG_STD|KCANFD_TIMESTAMP;

			nRtrFrame = ucStatusLen & STLN_RTR;
			if (nRtrFrame)
				rx.msg.flags |= KCANFD_MSG_RTR;

			/* check fw version if SR is supported */
			//if (usb_if->ucRevision >= 41) {
			if (dev->features & KCAN_DEV_ECHO_RDY) {
				if (*ucMsgPtr & 0x01)
					rx.msg.flags |= KCANFD_MSG_SLF;
			}

			if (ucStatusLen & STLN_EXTENDED_ID) {
				rx.msg.flags |= KCANFD_MSG_EXT;

#if defined(__LITTLE_ENDIAN)
				localID.uc[0] = *ucMsgPtr++;
				localID.uc[1] = *ucMsgPtr++;
				localID.uc[2] = *ucMsgPtr++;
				localID.uc[3] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				localID.uc[3] = *ucMsgPtr++;
				localID.uc[2] = *ucMsgPtr++;
				localID.uc[1] = *ucMsgPtr++;
				localID.uc[0] = *ucMsgPtr++;
#endif
				localID.ul >>= 3;
			} else {
				localID.ul = 0;

#if defined(__LITTLE_ENDIAN)
				localID.uc[0] = *ucMsgPtr++;
				localID.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				localID.uc[3] = *ucMsgPtr++;
				localID.uc[2] = *ucMsgPtr++;
#endif
				localID.ul >>= 5;
			}

			rx.msg.id = localID.ul;

			/* read timestamp, 1st timestamp in packet is 16 bit
			 * AND data, following timestamps are 8 bit in length
			 */
			if (!ts8++) {
#if defined(__LITTLE_ENDIAN)
				wTimeStamp.uc[0] = *ucMsgPtr++;
				wTimeStamp.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				wTimeStamp.uc[1] = *ucMsgPtr++;
				wTimeStamp.uc[0] = *ucMsgPtr++;
#endif
				kcan_updateTimeStampFromWord(dev, wTimeStamp.uw,
							     &rx.hwtv);
			} else {
				kcan_updateTimeStampFromByte(dev, *ucMsgPtr++,
							     &rx.hwtv);
			}

			/* SGr FIX:
			 * save in DATA array at most 8 bytes, ignore
			 * all others but ALWAYS increment msg ptr
			 * according to ucLen
			 */
			/* read data */
			j = 0;
			if (!nRtrFrame) {
				while (ucLen--) {
					if (j < KCANFD_CAN20_MAXDATALEN)
						rx.msg.data[j++] = *ucMsgPtr;
					ucMsgPtr++;
				}
			}

			/* there's another byte when SR flag is set
			 * (client handle): if KCAN_USB_ECHO_ID is set, then
			 * it's an ECHO (with ID <= 0x7f)
			 */
			if (rx.msg.flags & KCANFD_MSG_SLF) {
				u8 tmp = *ucMsgPtr++;

				if (tmp & KCAN_USB_ECHO_ID) {
					rx.msg.flags |= KCANFD_MSG_ECHO;
					rx.msg.ctrlr_data[KCANFD_ECHOID] =
						tmp & ~KCAN_USB_ECHO_ID;
				}
			}

			/* only for beauty */
			while (j < KCANFD_CAN20_MAXDATALEN)
				rx.msg.data[j++] = 0;

			err = kcan_xxxdev_rx(dev, &rx);
			if (err < 0) {
				if (net_ratelimit())
					pr_err("%s: kcan_xxxdev_rx(1) failed "
						" err=%d\n",
					       DEVICE_NAME, err);
				goto fail;
			}

#ifndef NETDEV_SUPPORT
			/* successfully enqueued into chardev FIFO */
			if (err > 0)
				rwakeup++;
#endif

		/* Status message */
		} else {
			u8 ucFunction;
			u8 ucNumber;
			u8 dummy;
			u8 ecc;

			/* get function and number */
			ucFunction = *ucMsgPtr++;
			ucNumber = *ucMsgPtr++;

			if (ucStatusLen & STLN_WITH_TIMESTAMP) {

				rx.msg.flags |= KCANFD_TIMESTAMP;

				/* only the 1st packet supplies a word
				 * timestamp
				 */
				if (!ts8++) {
#if defined(__LITTLE_ENDIAN)
					wTimeStamp.uc[0] = *ucMsgPtr++;
					wTimeStamp.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
					wTimeStamp.uc[1] = *ucMsgPtr++;
					wTimeStamp.uc[0] = *ucMsgPtr++;
#endif

					kcan_updateTimeStampFromWord(dev,
							wTimeStamp.uw,
							&rx.hwtv);
				} else
					kcan_updateTimeStampFromByte(dev,
							*ucMsgPtr++,
							&rx.hwtv);
			}

			switch (ucFunction) {

			case 1:
				if ((ucNumber & CAN_RECEIVE_OVERRUN) ||
						(ucNumber & QUEUE_OVERRUN)) {
					kcan_handle_error_ctrl(dev, &rx,
							KCANFD_RX_OVERFLOW);
				}

				if ((ucNumber & XMT_BUFFER_FULL) ||
						(ucNumber & QUEUE_XMT_FULL)) {
					kcan_handle_error_ctrl(dev, &rx,
							KCANFD_TX_OVERFLOW);
				}

				if (ucNumber & BUS_OFF) {
					kcan_handle_busoff(dev, &rx);
				} else {
					int passive = (ucNumber & BUS_HEAVY) &&
						(dev->rx_error_counter >= 128 ||
						 dev->tx_error_counter >= 128);

					if (!kcan_handle_error_status(dev, &rx,
							ucNumber & BUS_HEAVY,
							passive)) {
						/* no error bit (so, no error,
						 * back to active state)
						 */
						kcan_handle_error_active(dev,
									 &rx);
					}
				}

				/* version 3: sometimes the telegram carries 3
				 * additional data without note in ucStatusLen.
				 */
				/* SGr FIX:
				 * save in DATA array at most 8 bytes, ignore
				 * all others but ALWAYS increment msg ptr
				 * according to ucLen
				 */
				for (j = 0; ucLen--; ucMsgPtr++)
					if (j < KCANFD_CAN20_MAXDATALEN)
						rx.msg.data[j++] = *ucMsgPtr;
				break;

			case 2: /* get_analog_value, remove bytes */
				dummy = *ucMsgPtr++;
				dummy = *ucMsgPtr++;
				break;

			case 3: /* get_bus_load, remove byte */
				dev->bus_load = *ucMsgPtr++;
				break;

			case 4: /* only timestamp */
#if defined(__LITTLE_ENDIAN)
				wTimeStamp.uc[0] = *ucMsgPtr++;
				wTimeStamp.uc[1] = *ucMsgPtr++;
#elif defined(__BIG_ENDIAN)
				wTimeStamp.uc[1] = *ucMsgPtr++;
				wTimeStamp.uc[0] = *ucMsgPtr++;
#endif

				/* this specific timestamp is periodically 
				 * sent by the KCAN-USB (more than every 1s):
				 * use it to sync times.
				 */
				kcan_usb_sync_times(dev, wTimeStamp.uw, -50);

#ifdef BUS_STATE_ACTIVE_ON_1ST_TS
				/* consider 1st sync as STATUS=ACTIVE */
				if (dev->bus_state == KCANFD_UNKNOWN)
					kcan_handle_error_active(dev, &rx);
#endif
				break;

			case 5:
				ecc = ucMsgPtr[0];
#ifdef DEBUG_DECODE
				/* Looks like QUEUE_XMT_FULL is not set with
				 * function=5 (ErrorFrame)
				 */
				pr_info(DEVICE_NAME
					": got err_frame len=%u number=%u "
					"[ECC=%02x RXERR=%u TXERR=%u]\n",
					ucLen, ucNumber,
					ucMsgPtr[0],
					ucMsgPtr[1],
					ucMsgPtr[2]);
#endif
				switch (ucNumber) {
				case 0x80: /* counters are increasing */
				case 0x00: /* counters are decreasing */
					dev->rx_error_counter = ucMsgPtr[1];
					dev->tx_error_counter = ucMsgPtr[2];
					break;
				}

				/* sync msg */
				while (ucLen--)
					ucMsgPtr++;

				kcan_handle_error_msg(dev, &rx,
					(ecc & 0xc0) >> 6, (ecc & 0x1f),
					(ecc & 0x20), 0);
				break;

			case 10: /* prepared for future */
				break;

			default:
				pr_err(DEVICE_NAME
					": unexpected function %d, i=%d, "
					"ucStatusLen=0x%02x\n",
					ucFunction, i,
					ucStatusLen);
				dump_mem("unexpected function", org, 64);

				/* unsync? stop the analysis of the entire
				 * packet
				 */
				err = -EILSEQ;
				goto fail;
			}

			/* if an error condition occurred, send an error frame
			 * to the userspace
			 */
			if (rx.msg.type != KCANFD_TYPE_NOP) {

				err = kcan_xxxdev_rx(dev, &rx);
				if (err < 0) {
					if (net_ratelimit())
						pr_err(DEVICE_NAME
							": kcan_xxxdev_rx(2) "
							"failure (err=%d)\n",
							err);
					goto fail;
				}

#ifndef NETDEV_SUPPORT
				/* successfully enqueued into chardev FIFO */
				if (err > 0)
					rwakeup++;
#endif
			}
		}

		/* check for 'read from'-buffer overrun
		 * must be <= dev->port.usb.pipe_read.wDataSz)
		 */
		if ((ucMsgPtr - ucMsgStart) > lCurrentLength) {

			/* sometimes v 3 overrides the buffer by 1 byte */
			if ((usb_if->ucRevision > 3) ||
				((usb_if->ucRevision <= 3)
			&& ((ucMsgPtr - ucMsgStart) > (lCurrentLength + 1)))) {
				err = -EFAULT;
#ifdef __LP64__
				pr_err(DEVICE_NAME
					": Internal Error = %d (%ld, %d)\n",
					err, ucMsgPtr - ucMsgStart,
					lCurrentLength);
#else
				pr_err(DEVICE_NAME
					": Internal Error = %d (%d, %d)\n",
					err, ucMsgPtr - ucMsgStart,
					lCurrentLength);
#endif
				dump_mem("internal error", org, 64);
				goto fail;
			}
		}
	}

#ifndef NETDEV_SUPPORT
	if (rwakeup) {
		kcan_event_signal(&dev->in_event);
	}
#endif
	return 0;
fail:
	dev->nLastError = err;
	dev->dwErrorCounter++;

	return err;
}

/* gets messages out of write-fifo, encodes and puts them into USB buffer
 * ucMsgPtr
 * returns -ENODATA and *pnDataLength > 0  if I made a telegram and no more data
 *                                         are available
 *         -ENODATA and *pnDataLength == 0 if I made no telegram and no more
 *                                         data are available
 *         any ERROR else if something happend
 *         no ERROR if I made a telegram and there are more data available
 */
#define KCAN_USB_FORMAT_CODE	0x02

static int kcan_usb_encode_msgs(struct kcandev *dev, u8 *ucMsgPtr,
							int *pnDataLength)
{
	int nDataLength = *pnDataLength;
	int nBufferTop = nDataLength - 14;  /* buffer fill high water mark */
	u8 *ptr = ucMsgPtr;
	int nMsgCounter = 0;
	int err = 0;
	u8 ucLen;
	ULCONV localID;
	u8 *pucStatusLen;
	u8 *pucMsgCountPtr;
	int j;

	/* indicate no packet */
	*pnDataLength = 0;

	/* put packet type information */
	*ptr++ = KCAN_USB_FORMAT_CODE;
	pucMsgCountPtr = ptr++;  /* fill later the count of messages */

	while ((ptr - ucMsgPtr) < nBufferTop) {
		int nRtrFrame;
		struct kcanfd_txmsg tx;
		u8 dlc;

		/* release fifo buffer and step forward in fifo */
		err = kcan_txfifo_get(dev, &tx);
		if (err == -ENODATA)
			break;
		/* These flags are checked before against dev->features */

		/* get ptr to ucStatusLen byte */
		pucStatusLen = ptr++;

		dlc = ucLen = tx.msg.data_len;
		if (dlc == KCANFD_CAN20_MAXDATALEN) {
			u8 tmp_dlc = kcanfd_msg_read_dlc(&tx.msg);
			if (tmp_dlc > dlc)
				dlc = tmp_dlc;
		}

		*pucStatusLen = dlc;

		nRtrFrame = tx.msg.flags & MSGTYPE_RTR;
		if (nRtrFrame)
			*pucStatusLen |= STLN_RTR;

		j = 0;
		localID.ul = tx.msg.id;

		if (tx.msg.flags & MSGTYPE_EXTENDED) {
			*pucStatusLen |= STLN_EXTENDED_ID;
			localID.ul <<= 3;

			if (tx.msg.flags & (KCANFD_MSG_SLF|KCANFD_MSG_ECHO))
				localID.ul |= 0x01;	/* SRR flag SJA1000 */

			if (tx.msg.flags & MSGTYPE_SINGLESHOT)
				localID.ul |= 0x02;	/* AT flag SJA1000 */

#if defined(__LITTLE_ENDIAN)
			*ptr++ = localID.uc[0];
			*ptr++ = localID.uc[1];
			*ptr++ = localID.uc[2];
			*ptr++ = localID.uc[3];
#elif defined(__BIG_ENDIAN)
			*ptr++ = localID.uc[3];
			*ptr++ = localID.uc[2];
			*ptr++ = localID.uc[1];
			*ptr++ = localID.uc[0];
#endif
		} else {
			localID.ul <<= 5;

			if (tx.msg.flags & (KCANFD_MSG_SLF|KCANFD_MSG_ECHO))
				localID.ul |= 0x01;	/* SRR flag SJA1000 */

			if (tx.msg.flags & MSGTYPE_SINGLESHOT)
				localID.ul |= 0x02;	/* AT flag SJA1000 */

#if defined(__LITTLE_ENDIAN)
			*ptr++ = localID.uc[0];
			*ptr++ = localID.uc[1];
#elif defined(__BIG_ENDIAN)
			*ptr++ = localID.uc[3];
			*ptr++ = localID.uc[2];
#endif
		}

		if (!nRtrFrame) {
			memcpy(ptr, tx.msg.data, tx.msg.data_len);
			ptr += dlc;
		}

		/* save client handle if an ECHO is requested */
		if (tx.msg.flags & KCANFD_MSG_ECHO) {
			*ptr++ = KCAN_USB_ECHO_ID |
				 tx.msg.ctrlr_data[KCANFD_ECHOID];

		} else if (tx.msg.flags & KCANFD_MSG_SLF) {
			*ptr++ = 0x00;
		}

		nMsgCounter++;
	}

	/* generate external nDataLength if I carry payload */
	if ((ptr - ucMsgPtr) > 2) {
		*pnDataLength = nDataLength;

		/* set count of telegrams */
		ptr = ucMsgPtr + nDataLength - 1;
		*ptr = (u8)(dev->port.usb.dwTelegramCount++ & 0xff);

		/* last to do: put count of messages */
		*pucMsgCountPtr = nMsgCounter;

		dev->tx_frames_counter += nMsgCounter;
	} else {
		*pnDataLength = 0;
		*pucMsgCountPtr = 0;
	}

#ifdef DEBUG_ENCODE
	dump_mem("sent buffer", ucMsgPtr, *pnDataLength);
#endif

	return err;
}

static int kcan_usb_set_mass_storage_mode(struct kcan_usb_interface *usb_if)
{
	return kcan_usb_setcontrol_urb(usb_if_dev(usb_if, 0),
					KCAN_USB_SETCAN2FLASH, 0,
					0, 0, 0, 0);
}

#define KCAN_USB_CMD_FRAG_SIZE		11

static int kcan_usb_set_devdata(struct kcandev *dev, int dev_f,
				const u8 *dev_data)
{
	KCAN_USB_PARAM *cmd = (KCAN_USB_PARAM *)dev->port.usb.cout_baddr;
	const u8 id = dev_f >> 16;
	u16 len = 1 + ((dev_f & 0xff00) >> 8);
	int err = 0, os, od = 1;
	u8 p;

	cmd->Function = KCAN_USB_CMD_DEVDATA;
	cmd->Number = KCAN_USB_SET;
	cmd->Param[od++] = id;

	cmd->Param[12] = len & 0xff;
	cmd->Param[13] = len-- >> 8;

	for (p = os = 0; os < len; p++) {
		int l = len - os;

		if (l > (KCAN_USB_CMD_FRAG_SIZE + 1 - od))
			l = KCAN_USB_CMD_FRAG_SIZE + 1 - od;

		cmd->Param[0] = p;
		memcpy(cmd->Param+od, dev_data + os, l);

		/* send packet #p */
		err = kcan_usb_send_cmd(dev);
		if (err)
			break;

		os += l;
		od = 1;
	}

	return err;
}

static int kcan_usb_get_devdata(struct kcandev *dev, int dev_f, u8 *dev_data)
{
	KCAN_USB_PARAM *cmd = (KCAN_USB_PARAM *)dev->port.usb.cout_baddr;
	const int len = ((dev_f & 0xff00) >> 8) + 1;
	int err = -ENODATA, off;
	u8 p;

	cmd->Function = KCAN_USB_CMD_DEVDATA;
	cmd->Number = KCAN_USB_GET;

	for (p = off = 0; off < len; p++) {
		u8 pn = p;
		u16 total_len;
		int l = len - off;

		if (l > KCAN_USB_CMD_FRAG_SIZE)
			l = KCAN_USB_CMD_FRAG_SIZE;

		/* ask for a packet #p */
		err = kcan_usb_getcontrol_urb(dev, KCAN_USB_CMD_DEVDATA,
					      KCAN_USB_GET, &pn,
					      NULL, NULL, NULL);
		if (err)
			break;

		/* check what has been received */
		if ((cmd->Function != KCAN_USB_CMD_DEVDATA) ||
		    (cmd->Number != KCAN_USB_GET) ||
		    (pn != p)) {
			pr_warn(DEVICE_NAME ": wrong rsp f=%u/%u n=%u/%u "
				"p=%u/%u\n",
				cmd->Function, KCAN_USB_CMD_DEVDATA,
				cmd->Number, KCAN_USB_GET,
				pn, p);
			err = -EINVAL;
			break;
		}

		/* get total length from #12 & #13 */
		total_len = cmd->Param[12] | (cmd->Param[13] << 8);
		if (total_len != len) {
			pr_warn(DEVICE_NAME ": wrong len %u/%u\n",
				total_len, len);
			err = -EMSGSIZE;
			break;
		}

		memcpy(dev_data + off, cmd->Param + 1, l);

		off += l;
	}

	return err;
}

/*
 * void kcan_usb_cleanup(struct kcandev *dev)
 *
 * Last chance to submit URB before driver removal.
  * Note: the device is always plugged!
 */
static void kcan_usb_cleanup(struct kcandev *dev)
{
	/* Set CAN bus off here now since we're sure that the request will be */
	/* sent to the usb module */
	kcan_usb_setCANOff(dev);
}

static int kcan_usb_ctrl_init(struct kcandev *dev)
{
	return 0;
}

static void kcan_usb_free(struct kcan_usb_interface *usb_if)
{
	usb_if->adapter = kcan_free_adapter(usb_if->adapter);
	kcan_usb_devices--;
}

/*
 * int kcan_usb_init(struct kcan_usb_interface *usb_if)
 *
 * Do the initialization part of a KCAN-USB adapter.
 */
int kcan_usb_init(struct kcan_usb_interface *usb_if)
{
	u32 hw_ver;

	usb_if->hw_ver.major = (usb_if->ucRevision / 10);
	usb_if->hw_ver.minor = usb_if->ucRevision % 10;
	usb_if->hw_ver.subminor = 0;
	usb_if->hw_ver.extra = 0;

	usb_if->adapter = kcan_alloc_adapter("KCAN-USB", "IPEH-002021/002022",
					     kcan_usb_devices++, 1);
	if (!usb_if->adapter)
		return -ENOMEM;

	/* newer KCAN-USB devices allow to get 3 version numbers */
	if (usb_if->ucRevision >= 21) {
		int err = kcan_usb_get_hwver(usb_if);
		if (!err)
			dev_info(&usb_if->usb_intf->dev,
				"%s (MCU%02xh) fw v%u.%u.%u\n",
				usb_if->adapter->name, usb_if->mcu,
				usb_if->hw_ver.major,
				usb_if->hw_ver.minor, usb_if->hw_ver.subminor);

	} else {
		dev_info(&usb_if->usb_intf->dev, "%s fw v%u.%u\n",
			usb_if->adapter->name, usb_if->hw_ver.major,
			usb_if->hw_ver.minor);
	}

	usb_if->adapter->hw_ver = usb_if->hw_ver;

	/* KCAN-USB gen2 only runs FW 8.x while KCAN-USB gen1 run FW < 7.x
	 *
	 * gen1 (S/N <= 190.000): dev_data available starting from 5.3 (53)
	 * gen2 (S/N >= 200.000): dev_data available starting from 8.6.1 (86)
	 */
	hw_ver = VER_NUM(usb_if->adapter->hw_ver.major,
			 usb_if->adapter->hw_ver.minor,
			 usb_if->adapter->hw_ver.subminor);

	/* gen2:
	 *
	 * Note: customers version starts from 8.3 but some devices
	 * might includes development versions starting from 7.x.
	 * But Windows driver compares with 8.0.
	 */
	if (hw_ver >= VER_NUM(8, 0, 0)) {
		usb_if->device_set_mass_storage_mode =
			kcan_usb_set_mass_storage_mode;

		if (hw_ver >= VER_NUM(8, 6, 1))
			usb_if->device_set_devdata = kcan_usb_set_devdata;

	/* gen1 */
	} else if (hw_ver >= VER_NUM(5, 3, 0))
		usb_if->device_set_devdata = kcan_usb_set_devdata;

	if (usb_if->device_set_devdata)
		usb_if->device_get_devdata = kcan_usb_get_devdata;

	usb_if->device_ctrl_init = kcan_usb_ctrl_init;
	usb_if->device_ctrl_cleanup = kcan_usb_cleanup;
	usb_if->device_ctrl_open = kcan_usb_Init;
	usb_if->device_ctrl_set_bus_on = kcan_usb_setCANOn;
	usb_if->device_ctrl_set_bus_off = kcan_usb_setCANOff;
	usb_if->device_get_snr = kcan_usb_getSNR;
	usb_if->device_ctrl_set_dnr = kcan_usb_setDeviceNr;
	usb_if->device_ctrl_get_dnr = kcan_usb_getDeviceNr;
	usb_if->device_ctrl_msg_encode = kcan_usb_encode_msgs;
	usb_if->device_msg_decode = kcan_usb_decode_msgs;
	usb_if->device_free = kcan_usb_free;

	/* setup identification callback */
	usb_if_dev(usb_if, 0)->device_identify = kcan_usb_identify;

	usb_if_dev(usb_if, 0)->device_reset = kcan_usb_reset;

	return 0;
}
#endif /* USB_SUPPORT */
