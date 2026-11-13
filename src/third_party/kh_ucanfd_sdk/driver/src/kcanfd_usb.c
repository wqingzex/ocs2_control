/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_usbfd.c - the inner parts for KCAN-USB (Pro) FD support
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
/* #define DEBUG */
/* #undef DEBUG */

#include "src/kcan_common.h"

#ifdef USB_SUPPORT

#ifdef DEBUG_USB_LITE
#define DEBUG_TRACE
#define DEBUG_TRACE_ENUM_USB
#define DEBUG_DEVDATA
#endif

#include "src/kcan_fifo.h"
#include "src/kcanfd_usb.h"

#ifdef NETDEV_SUPPORT
#include "src/kcan_netdev.h"		/* for hotplug kcan_netdev_register() */
#else
#include <linux/can/dev.h>
#endif

/* If defined, consider that usb_if enumerate sequentially.
 * If not defined, use usb_if->usb_dev.portnum as usb_if->index.
 * KCAN-USB X6 only.
 */
//#define KCAN_USBIF_SEQ_ENUM

#include "src/kcan_timing.h"		/* timing conversion */
#include "src/kcanfd_usb_fw.h"		/* KCAN-USB (Pro) FD fw structures */
#include "src/kcan_usbpro_fw.h"

/* device state flags */
#define UCAN_USB_SHOULD_WAKEUP		0x00000001UL

#define KCAN_USBFD_FWINFO_REQ	1

#define KCAN_USBFD_TYPE_STD	1
#define KCAN_USBFD_TYPE_EXT	2	/* include EP numbers */

/* hw_type */
enum {
	KCAN_MCU_USBFD_NXP = 1,
	KCAN_MCU_USBPROFD_LPC1778 = 1,

	KCAN_MCU_USBPROFD_LPC4078 = 2,

	KCAN_MCU_USBFD_ATSAM = 10,
	KCAN_MCU_USBPROFD_STM32F733 = 10,
};

#define KCAN_USBFD_VENDOR_INFO_SIZE_V1		28
#define KCAN_USBFD_VENDOR_INFO_SIZE_V234	36
#define KCAN_USBFD_VENDOR_INFO_SIZE_V5		84
#define KCAN_USBFD_VENDOR_INFO_SIZE_X4		46
#define KCAN_USBFD_VENDOR_INFO_SIZE_V6		94

/*
 * Private Data Structures
 */
/* KCAN_USBFD_FWINFO_REQ(1) */
struct kcan_usbfd_fw_info {
	__le16	size_of;	/* sizeof this */
	__le16	type;		/* type of this structure */
	u8	hw_type;	/* Type of hardware (KCAN_USBFD_MCU_xxx) */
	u8	bl_version[3];	/* Bootloader version */
	u8	hw_version;	/* Hardware version (PCB) */
	u8	fw_version[3];	/* Firmware version */
	__le32	dev_id[2];	/* "device id" per CAN */
	__le32	ser_no;		/* S/N */
	__le32	flags;		/* special functions (sizeof(this) = 28) */

	/* v2 (ATSAM MCU for example) */
	u8	cmd_out_ep;	/* ep for cmd */
	u8	cmd_in_ep;	/* ep for replies */
	u8	data_out_ep[2];	/* ep for CANx TX */
	u8	data_in_ep;	/* ep for CAN RX */

	/* v3 */
	u8	can_bus_count;	/* number of CAN */

	/* v4 */
	u8	instance_idx;	/* USB-X6 instance index */
	u8	dummy;		/* sizeof(this) = 36 */

	/* v5 */
	u8	guid[16];		/* GUID */
	u8	fw_version_adv[32];	/* sizeof(this) = 84 */

} __attribute__ ((packed));

/* KCAN_USBFD_FWINFO_REQ(1), v6: 4 CAN channels with per-channel ids */
struct kcan_usbfd_fw_info_v6 {
	__le16	size_of;	/* sizeof this */
	__le16	type;		/* type of this structure */
	u8	hw_type;	/* Type of hardware (KCAN_USBFD_MCU_xxx) */
	u8	bl_version[3];	/* Bootloader version */
	u8	hw_version;	/* Hardware version (PCB) */
	u8	fw_version[3];	/* Firmware version */
	__le32	dev_id[4];	/* "device id" per CAN */
	__le32	ser_no;		/* S/N */
	__le32	flags;		/* special functions (sizeof(this) = 36) */

	/* v2 (ATSAM MCU for example) */
	u8	cmd_out_ep;	/* ep for cmd */
	u8	cmd_in_ep;	/* ep for replies */
	u8	data_out_ep[4];	/* ep for CANx TX */
	u8	data_in_ep;	/* ep for CAN RX */

	/* v3 */
	u8	can_bus_count;	/* number of CAN */

	/* v4 */
	u8	instance_idx;	/* USB-X6 instance index */
	u8	dummy;		/* sizeof(this) = 46 */

	/* v5 */
	u8	guid[16];		/* GUID */
	u8	fw_version_adv[32];	/* sizeof(this) = 94 */

} __attribute__ ((packed));

union kcan_usbfd_fw_info_any {
	struct kcan_usbfd_fw_info v5;
	struct kcan_usbfd_fw_info_v6 v6;
};

static inline u16 kcan_usbfd_fw_info_size(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return le16_to_cpu(pfi->v5.size_of);
}

static inline bool kcan_usbfd_fw_info_is_x4(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_size(pfi) >= KCAN_USBFD_VENDOR_INFO_SIZE_X4 &&
	       kcan_usbfd_fw_info_size(pfi) < KCAN_USBFD_VENDOR_INFO_SIZE_V5;
}

static inline bool kcan_usbfd_fw_info_has_4ch_layout(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_is_x4(pfi) ||
	       kcan_usbfd_fw_info_size(pfi) >= KCAN_USBFD_VENDOR_INFO_SIZE_V6;
}

static inline u8 kcan_usbfd_fw_info_can_count(
				const union kcan_usbfd_fw_info_any *pfi)
{
	if (kcan_usbfd_fw_info_size(pfi) < KCAN_USBFD_VENDOR_INFO_SIZE_V234)
		return 0;

	return kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		pfi->v6.can_bus_count : pfi->v5.can_bus_count;
}

static inline u32 kcan_usbfd_fw_info_ser_no(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return le32_to_cpu(kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		pfi->v6.ser_no : pfi->v5.ser_no);
}

static inline u8 kcan_usbfd_fw_info_dev_id_count(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		ARRAY_SIZE(pfi->v6.dev_id) : ARRAY_SIZE(pfi->v5.dev_id);
}

static inline u32 kcan_usbfd_fw_info_dev_id(
				const union kcan_usbfd_fw_info_any *pfi, u8 idx)
{
	return le32_to_cpu(kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		pfi->v6.dev_id[idx] : pfi->v5.dev_id[idx]);
}

static inline u8 kcan_usbfd_fw_info_data_out_ep_count(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		ARRAY_SIZE(pfi->v6.data_out_ep) :
		ARRAY_SIZE(pfi->v5.data_out_ep);
}

static inline u8 kcan_usbfd_fw_info_data_out_ep(
				const union kcan_usbfd_fw_info_any *pfi, u8 idx)
{
	return kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		pfi->v6.data_out_ep[idx] : pfi->v5.data_out_ep[idx];
}

static inline u8 kcan_usbfd_fw_info_cmd_out_ep(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		pfi->v6.cmd_out_ep : pfi->v5.cmd_out_ep;
}

static inline u8 kcan_usbfd_fw_info_cmd_in_ep(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		pfi->v6.cmd_in_ep : pfi->v5.cmd_in_ep;
}

static inline u8 kcan_usbfd_fw_info_data_in_ep(
				const union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_has_4ch_layout(pfi) ?
		pfi->v6.data_in_ep : pfi->v5.data_in_ep;
}

static inline u8 *kcan_usbfd_fw_info_guid(
				union kcan_usbfd_fw_info_any *pfi)
{
	return kcan_usbfd_fw_info_size(pfi) >= KCAN_USBFD_VENDOR_INFO_SIZE_V6 ?
		pfi->v6.guid : pfi->v5.guid;
}

static int kcan_usbfd_request_fw_info(struct kcan_usb_interface *usb_if,
				      union kcan_usbfd_fw_info_any *pfi)
{
	int len;

	memset(pfi, 0, sizeof(*pfi));
	len = kcan_usbpro_request(usb_if,
				  USB_VENDOR_REQUEST_INFO,
				  KCAN_USBFD_FWINFO_REQ,
				  &pfi->v5, sizeof(pfi->v5));
	if (len < 0)
		return len;

	if (kcan_usbfd_fw_info_size(pfi) < KCAN_USBFD_VENDOR_INFO_SIZE_V6)
		return len;

	memset(pfi, 0, sizeof(*pfi));
	return kcan_usbpro_request(usb_if,
				   USB_VENDOR_REQUEST_INFO,
				   KCAN_USBFD_FWINFO_REQ,
				   pfi, sizeof(*pfi));
}

#ifdef UCAN_USB_OPTION_FAST_FWD
/* enable to globally set the fast-forward option for KCAN-USB FD adapters */
static ushort fast_fwd = 0;
module_param(fast_fwd, ushort, 0444);
MODULE_PARM_DESC(fast_fwd, " set fast forward option in USB device (def=0)");
#endif

static int kcan_chip_devices = 0;
static int kcan_usbfd_devices = 0;
static int kcan_usbprofd_devices = 0;
static int kcan_usbx6_devices = 0;
static int kcan_usbx4_devices = 0;

static const char *str_kcan_usb_x6 = "KCAN-USB X6";
static const char *str_kcan_usb_x4 = "KCAN-USB X4";

#define KCAN_UFD_CMD_EP			0x01
#define KCAN_UFD_LOSPD_PKT_SIZE		64

#define KCAN_USB_FD_SYNC_PERIOD		USEC_PER_SEC

/* copied from kcan_usbpro_timestamp_decode() but use real hw 64 bit value */
static struct kcan_timeval *kcan_usbfd_decode_timestamp(struct kcandev *dev,
						   u32 ts_low, u32 ts_high,
						   struct kcan_timeval *tv)
{
	u64 ts_us = ((u64 )ts_high << 32) + ts_low;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s CAN%u, ts=%u:%u)\n",
		__func__, dev->adapter->name, kcan_idx(dev)+1, ts_high, ts_low);
#endif

	/* simulate sync if calibration msgs aren't used for that */
	if (kcan_usb_get_if(dev)->cm_ignore_count < 0) {

		u64 delta_us = ts_us - dev->time_sync.ts_us;

		/* simulate sync every KCAN_USB_FD_SYNC_PERIOD */
		if (delta_us >= KCAN_USB_FD_SYNC_PERIOD) {

#ifdef KCAN_USBPRO_TS_RESYNC_PERIOD
			/* Note: kcan_sync_times() resets the entire
			 *       time_sync field when ts_us == 0 .
			 */
			if (delta_us >= KCAN_USBPRO_TS_RESYNC_PERIOD)
				dev->time_sync.ts_us = 0;
#endif
			kcan_sync_times64(dev, ts_us, 0);
		}
	}

	kcan_sync_decode64(dev, ts_us, tv);

	return tv;
}

/* static int kcan_usbfd_send_ucan_cmd_ex(struct kcandev *dev, void *rsp_addr,
 *					  int *rsp_size)
 */
static int kcan_usbfd_send_ucan_cmd_ex(struct kcandev *dev, void *rsp_addr,
				       int *rsp_size)
{
	struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);
	int i = 0, err = 0, packet_len;
	void *packet_ptr;
	u16 op_req;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u): ->EP#%02X\n",
		__func__, kcan_idx(dev)+1,
		usb_if->pipe_cmd_out.ucNumber);
#endif

	/* don't do anything with non-existent hardware */
	if (!dev->is_plugged)
		return -ENODEV;

	/* if a packet is not filled completely by commands, the command list
	 * is terminated with an "end of collection" record. */
	if (dev->ucan.cmd_len < dev->ucan.cmd_size) {
		ucan_add_cmd(dev, UCAN_CMD_END_OF_COLLECTION);
	}

#ifdef DEBUG_USB
	//dump_mem("sent cmd", dev->ucan.cmd_head, 32); //dev->ucan.cmd_len);
	dump_mem("sent cmd", dev->ucan.cmd_head, dev->ucan.cmd_len);

	/* firmware is not able to re-assemble 512 bytes buffer in full-speed */
	if ((usb_if->usb_dev->speed != USB_SPEED_HIGH) &&
				(dev->ucan.cmd_len > KCAN_UFD_LOSPD_PKT_SIZE)) {
		pr_warn(DEVICE_NAME
			": large cmd (%dB) is cut for non full-speed USB\n",
			dev->ucan.cmd_len);
	}
#endif

	packet_ptr = dev->ucan.cmd_head;
	packet_len = dev->ucan.cmd_len;

	/* firmware is not able to re-assemble 512 bytes buffer in full-speed */
	if (unlikely((usb_if->usb_dev->speed != USB_SPEED_HIGH) &&
			(packet_len > KCAN_UFD_LOSPD_PKT_SIZE))) {
		packet_len = KCAN_UFD_LOSPD_PKT_SIZE;

#ifdef DEBUG_USB
		pr_warn(DEVICE_NAME
			": packet_len reset to %u bytes due to non-high speed "
			"controler\n", KCAN_UFD_LOSPD_PKT_SIZE);
#endif
	}

	do {
		err = usb_bulk_msg(usb_if->usb_dev,
				   usb_sndbulkpipe(usb_if->usb_dev,
					   usb_if->pipe_cmd_out.ucNumber),
				   packet_ptr, packet_len, NULL,
				   KCAN_USB_CMD_TIMEOUT);
		if (err) {
			pr_err(DEVICE_NAME
			       ": err %d submitting cmd %03xh to %s (%d/3)\n",
			       err, le16_to_cpu(*(__le16 *)dev->ucan.cmd_head),
			       dev->adapter->name, ++i);

			if (!dev->is_plugged)
				err = -ENODEV;

			else if (err == -ETIMEDOUT && i < 3)
				continue;

			break;
		}

		packet_ptr += packet_len;
		dev->ucan.cmd_len -= packet_len;

		if (dev->ucan.cmd_len < KCAN_UFD_LOSPD_PKT_SIZE)
			packet_len = dev->ucan.cmd_len;

	} while (packet_len > 0);

	/* if error or not waiting for any response, stop here */
	if (err || !rsp_addr || !rsp_size)
		return err;

	/* to check with the received rsp opcode */
	op_req = le16_to_cpu(*(__le16 *)dev->ucan.cmd_head);

	for (i = 0; i < 3; i++) {
		u16 op_rsp;

		/* wait for response */
		err = usb_bulk_msg(usb_if->usb_dev,
				   usb_rcvbulkpipe(usb_if->usb_dev,
						usb_if->pipe_cmd_in.ucNumber),
				   dev->ucan.cmd_head, dev->ucan.cmd_size,
				   &dev->ucan.cmd_len, KCAN_USB_CMD_TIMEOUT);

		if (!dev->is_plugged)
			err = -ENODEV;

		if (err < 0) {
			pr_err(DEVICE_NAME
				": %s(): usb_bulk_msg() err %d\n",
				__func__, err);
			break;
		}

		if (!dev->ucan.cmd_len) {
			pr_warn(DEVICE_NAME ": abnormal null response (err=%d)."
				" Retrying...\n",
				err);
			continue;
		}

#ifdef DEBUG_USB
		dump_mem("rcvd rsp", dev->ucan.cmd_head, dev->ucan.cmd_len);
#endif

		/* check if it's the good one */
		op_rsp = le16_to_cpu(*(__le16 *)dev->ucan.cmd_head);
		if (op_rsp != (op_req+1)) {
			pr_warn(DEVICE_NAME
				": %s(): got abnormal rsp 0x%x to req 0x%x\n",
				__func__, op_rsp, op_req);
			continue;
		}

		if (dev->ucan.cmd_len <= *rsp_size)
			*rsp_size = dev->ucan.cmd_len;
#ifdef DEBUG_USB
		else
			pr_warn(DEVICE_NAME
				": %s(): truncated rsp (only %d/%d bytes)\n",
				__func__, *rsp_size, dev->ucan.cmd_len);
#endif
		memcpy(rsp_addr, dev->ucan.cmd_head, *rsp_size);

		/* if fw sends a 512 bytes packet and cmd_size = 512, then it
		 * needs to send another one to stop the transfer. That's the
		 * reason why a 2nd dummy read is needed: with some FW, a 0
		 * length packet is sent by the packet.
		 * With some other (ATSAM MCU), then a 4 bytes packet only is
		 * sent by the FW.
		 * When cmd_size is greater than wPacketSize (=512) then the
		 * transfer entirely fills into the 1st read, therefore, there
		 * is no need to read again...
		 */
		if (dev->ucan.cmd_size == usb_if->pipe_cmd_out.wDataSz) {

			/* dummy read to complete the above read */
			usb_bulk_msg(usb_if->usb_dev,
				usb_rcvbulkpipe(usb_if->usb_dev,
					usb_if->pipe_cmd_in.ucNumber),
				dev->ucan.cmd_head, dev->ucan.cmd_size,
				&dev->ucan.cmd_len, KCAN_USB_CMD_TIMEOUT);

			if (!dev->is_plugged)
				err = -ENODEV;
		}

		break;
	}

	return err;
}

static inline int kcan_usbfd_send_ucan_cmd(struct kcandev *dev)
{
	return kcan_usbfd_send_ucan_cmd_ex(dev, NULL, NULL);
}

/*
 * Hardware Callbacks
 */

/* int ucan_usb_set_can_led(struct kcandev *dev, u8 mode )
 */
static int ucan_usb_set_can_led(struct kcandev *dev, u8 mode)
{
	struct ucan_usb_led *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, mode=%u)\n",
		__func__, kcan_idx(dev)+1, mode);
#endif
	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_LED_SET);
	if (cmd)
		cmd->mode = mode;

	/* send the command */
	return kcan_usbfd_send_ucan_cmd(dev);
}

/* int ucan_usb_set_en_option(struct kcandev *dev, u16 mask, u16 usb_mask)
 */
static int ucan_usb_set_en_option(struct kcandev *dev, u16 mask, u16 usb_mask)
{
	struct ucan_usb_option *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, mask=%04xh, usb_mask=%04xh)\n",
		__func__, kcan_idx(dev)+1, mask, usb_mask);
#endif
	cmd = ucan_add_cmd_set_en_option(ucan_init_cmd(dev), mask, usb_mask);

	/* send the command */
	return kcan_usbfd_send_ucan_cmd(dev);
}

/* int ucan_usb_clr_dis_option(struct kcandev *dev, u16 mask, u16 usb_mask)
 */
static int ucan_usb_clr_dis_option(struct kcandev *dev, u16 mask, u16 usb_mask)
{
	struct ucan_usb_option *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, mask=%04xh, usb_mask=%04xh)\n",
		__func__, kcan_idx(dev)+1, mask, usb_mask);
#endif
	cmd = ucan_add_cmd_clr_dis_option(ucan_init_cmd(dev), mask, usb_mask);

	/* send the command */
	return kcan_usbfd_send_ucan_cmd(dev);
}

/* int kcan_usbfd_get_fw_info(struct kcan_usb_interface *usb_if,
 * 			      int should_print, u32 *bl_version)
 */
static int kcan_usbfd_get_fw_info(struct kcan_usb_interface *usb_if,
				  int should_print, u32 *bl_version)
{
	union kcan_usbfd_fw_info_any fw_info;
	union kcan_usbfd_fw_info_any * const pfi = &fw_info;
	int len;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif
	len = kcan_usbfd_request_fw_info(usb_if, pfi);
	if (len >= 0) {

		int i;

		if (usb_if->adapter) {

			if (should_print) {

				dev_info(&usb_if->usb_intf->dev,
					"%s (MCU%02xh PCB%02Xh) "
					"fw v%d.%d.%d "
					"bl v%d.%d.%d\n",
					usb_if->adapter->name,
					pfi->v5.hw_type, pfi->v5.hw_version,
					pfi->v5.fw_version[0],
					pfi->v5.fw_version[1],
					pfi->v5.fw_version[2],
					pfi->v5.bl_version[0],
					pfi->v5.bl_version[1],
					pfi->v5.bl_version[2]);

#ifdef DEBUG_USB
				if (kcan_usbfd_fw_info_size(pfi) >=
				    KCAN_USBFD_VENDOR_INFO_SIZE_V5) {
					char tmp[4+1+3*KCAN_GUID_LEN+1];
					int l = sprintf(tmp, "GUID=");
					u8 *guid = kcan_usbfd_fw_info_guid(pfi);

					for (i = 0; i < KCAN_GUID_LEN; i++)
						l += sprintf(tmp+l, "%02X", guid[i]);
					sprintf(tmp+l, "\n");

					dev_info(&usb_if->usb_intf->dev, tmp);
				}
#endif
			}

			/* in case of several usb-if are handled by the same
			 * adapter (KCAN-USB X6 for ex)
			 */
			if ((kcan_usbfd_fw_info_size(pfi) >=
			     KCAN_USBFD_VENDOR_INFO_SIZE_V5)  &&
			    !(usb_if->adapter->flags & KCAN_ADAPTER_GUID)) {
				u8 *guid = kcan_usbfd_fw_info_guid(pfi);

				usb_if->adapter->flags |= KCAN_ADAPTER_GUID;
				memcpy(usb_if->adapter->guid, guid,
				       sizeof(usb_if->adapter->guid));
			}
		}

		/* save device id and serial num info for further read */
		usb_if->dwSerialNumber = kcan_usbfd_fw_info_ser_no(pfi);
		usb_if->ucRevision = pfi->v5.hw_version;
		usb_if->hw_ver.major = pfi->v5.fw_version[0];
		usb_if->hw_ver.minor = pfi->v5.fw_version[1];
		usb_if->hw_ver.subminor = pfi->v5.fw_version[2];
		usb_if->hw_ver.extra = 0;
		usb_if->mcu = pfi->v5.hw_type;

		/* copy the EP numbers if they are provided too */
		if (pfi->v5.type == cpu_to_le16(KCAN_USBFD_TYPE_EXT)) {
			usb_if->pipe_cmd_out.ucNumber =
				kcan_usbfd_fw_info_cmd_out_ep(pfi);
			usb_if->pipe_cmd_in.ucNumber =
				kcan_usbfd_fw_info_cmd_in_ep(pfi);
			usb_if->pipe_read.ucNumber =
				kcan_usbfd_fw_info_data_in_ep(pfi);
		}

		{
			u8 reported_can_count = usb_if->can_count;
			u8 id_count = reported_can_count;
			u8 ep_count = reported_can_count;
			u8 fw_can_count = kcan_usbfd_fw_info_can_count(pfi);

			if (fw_can_count)
				reported_can_count = fw_can_count;

			if (id_count > reported_can_count)
				id_count = reported_can_count;
			if (id_count > kcan_usbfd_fw_info_dev_id_count(pfi))
				id_count = kcan_usbfd_fw_info_dev_id_count(pfi);

			if (ep_count > reported_can_count)
				ep_count = reported_can_count;
			if (ep_count > kcan_usbfd_fw_info_data_out_ep_count(pfi))
				ep_count =
					kcan_usbfd_fw_info_data_out_ep_count(pfi);

			for (i = 0; i < usb_if->can_count; i++) {
				struct kcandev *dev = usb_if_dev(usb_if, i);
				u32 dev_id = kcan_usbfd_fw_info_dev_id(pfi, 0);

				if (i < id_count)
					dev_id = kcan_usbfd_fw_info_dev_id(pfi,
									   i);

				dev->port.usb.ucHardcodedDevNr = dev_id;

				/* extended info: get write EP number too */
				if ((pfi->v5.type ==
				     cpu_to_le16(KCAN_USBFD_TYPE_EXT)) &&
				    (i < ep_count))
					dev->port.usb.pipe_write.ucNumber =
						kcan_usbfd_fw_info_data_out_ep(
								pfi, i);
			}
		}

		if (bl_version)
			*bl_version = VER_NUM(pfi->v5.bl_version[0],
					      pfi->v5.bl_version[1],
					      pfi->v5.bl_version[2]);

		return 0;
	}

	return len;
}

/*
 * int kcan_usbfd_get_device_nr(struct kcandev *dev, u32 *p_device_nr)
 */
static int kcan_usbfd_get_device_nr(struct kcandev *dev, u32 *p_device_nr)
{
	struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);
	union kcan_usbfd_fw_info_any * const pfi =
				(union kcan_usbfd_fw_info_any *)dev->ucan.cmd_head;
	int len;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif
	len = kcan_usbfd_request_fw_info(usb_if, pfi);
	if (len >= 0) {
		u8 reported_can_count = usb_if->can_count;
		u8 id_count = reported_can_count;
		u8 fw_can_count = kcan_usbfd_fw_info_can_count(pfi);
		u32 tmp32 = kcan_usbfd_fw_info_dev_id(pfi, 0);

		if (fw_can_count)
			reported_can_count = fw_can_count;

		if (id_count > reported_can_count)
			id_count = reported_can_count;
		if (id_count > kcan_usbfd_fw_info_dev_id_count(pfi))
			id_count = kcan_usbfd_fw_info_dev_id_count(pfi);

		if (dev->can_idx < id_count)
			tmp32 = kcan_usbfd_fw_info_dev_id(pfi, dev->can_idx);

		dev->flags &= ~KCAN_DEV_USES_ALT_NUM;
		if (tmp32 != 0xffffffff)
			dev->flags |= KCAN_DEV_USES_ALT_NUM;

		if (p_device_nr)
			*p_device_nr = tmp32;

		return 0;
	}

	return len;
}

/*
 * int kcan_usbfd_set_device_nr(struct kcandev *dev, u32 device_nr)
 */
static int kcan_usbfd_set_device_nr(struct kcandev *dev, u32 device_nr)
{
	struct ucan_usb_device_id *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, 0x%x)\n",
		__func__, kcan_idx(dev)+1, device_nr);
#endif

	/* ucan_add_cmd() stores dev->can_idx in opcode_channel. */
	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_DEVID_SET);
	if (cmd)
		cmd->device_id = cpu_to_le32(device_nr);

	/* send the command */
	return kcan_usbfd_send_ucan_cmd(dev);
}

/*
 * int kcan_usbfd_get_serial_nr(struct kcan_usb_interface *usb_if,
 *					u32 *p_serial_num)
 *
 * Retrieve serial number from bootloader info
 */
static int kcan_usbfd_get_serial_nr(struct kcan_usb_interface *usb_if,
					u32 *p_serial_num)
{
	/* used cached value read from kcan_usbfd_init() */
	if (p_serial_num)
		*p_serial_num = usb_if->dwSerialNumber;

	return 0;
}

static int kcan_usbfd_set_devdata(struct kcandev *dev, int dev_f,
				  const u8 *dev_data)
{
	struct ucan_usb_devdata *cmd;

#if defined(DEBUG_TRACE) || defined(DEBUG_DEVDATA)
	pr_info(DEVICE_NAME ": %s(CAN%u, dev_f=%08xh)\n",
		__func__, kcan_idx(dev)+1, dev_f);
	dump_mem("dev_data", dev_data, sizeof(cmd->d));
#endif

	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_DEVDATA_SET);
	if (cmd) {
		u16 id = dev_f >> 16;
		u16 len = (dev_f & 0xff00) >> 8;

		cmd->id = cpu_to_le16(id);
		cmd->total_len = cpu_to_le16(len);
		cmd->offset = 0;
		memcpy(cmd->d, dev_data, len);

		dev->ucan.cmd_len = sizeof(struct ucan_usb_devdata);
	}

	/* send the command */
	return kcan_usbfd_send_ucan_cmd(dev);
}

static int kcan_usbfd_get_devdata(struct kcandev *dev, int dev_f, u8 *dev_data)
{
	struct ucan_usb_devdata *cmd_req;
	struct ucan_usb_devdata cmd_rsp;
	u16 len = (dev_f & 0xff00) >> 8;
	u16 id = dev_f >> 16;
	int l, err;

#if defined(DEBUG_TRACE) || defined(DEBUG_DEVDATA)
	pr_info(DEVICE_NAME ": %s(CAN%u, dev_f=%08xh)\n",
		__func__, kcan_idx(dev)+1, dev_f);
#endif

	cmd_req = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_DEVDATA_REQ);
	if (!cmd_req)
		return -ENOMEM;

	cmd_req->id = cpu_to_le16(id);
	cmd_req->total_len = 0;
	cmd_req->offset = 0;

	/* send the command */
	l = sizeof(cmd_rsp);
	err = kcan_usbfd_send_ucan_cmd_ex(dev, &cmd_rsp, &l);
	if (err)
		return err;

	/* check if rsp contains at least a header */
	if (l < offsetof(struct ucan_usb_devdata, d)) {
		pr_err(DEVICE_NAME ": %s: too small %d bytes response\n",
		       dev->adapter->name, l);
		return -EMSGSIZE;
	}

	/* check response */
	if (cmd_rsp.id != cmd_req->id) {
		pr_err(DEVICE_NAME ": %s: invalid id %d in response\n",
		       dev->adapter->name, le16_to_cpu(cmd_rsp.id));
		return -EINVAL;
	}

	l = le16_to_cpu(cmd_rsp.total_len);
	if (l != len+1) {
		pr_err(DEVICE_NAME ": %s: wrong %d bytes payload in response\n",
		       dev->adapter->name, l);
		return -EINVAL;
	}

	/* everything looks ok: give data to user */
	memcpy(dev_data, cmd_rsp.d, l);

#if defined(DEBUG_DEVDATA)
	dump_mem("dev_data", dev_data, sizeof(cmd_rsp.d));
#endif
	return err;
}

/* int ucan_usb_set_clck_domain(struct kcandev *dev, u32 clk_mode)
 */
static int ucan_usb_set_clck_domain(struct kcandev *dev, u32 clk_mode)
{
	struct ucan_usb_clock *cmd;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, 0x%x)\n",
		__func__, kcan_idx(dev)+1, clk_mode);
#endif
	cmd = ucan_add_cmd(ucan_init_cmd(dev), UCAN_USB_CMD_CLK_SET);
	if (cmd)
		cmd->mode = (u8 )clk_mode;

	/* send the command */
	return kcan_usbfd_send_ucan_cmd(dev);
}

/*
 * KCAN-Chip FW v3.3.0 specific:
 */

static int kcan_usb_chip_get_io(struct kcandev *dev,
				struct kcanfd_option *opt, void *c,
				u16 op_req)
{
	struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);
	struct ucan_usb_io_ctrl *cmd = ucan_add_cmd(ucan_init_cmd(dev), op_req);

	int i, l, err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, op_req=%04xh)\n",
		__func__, kcan_idx(dev)+1, op_req);
#endif
	err = kcan_usbfd_send_ucan_cmd(dev);
	if (err)
		return err;

	for (i = 0; i < 3; i++) {

		u16 opcode;
		u32 tmp32;

		/* wait for response */
		err = usb_bulk_msg(usb_if->usb_dev,
				   usb_rcvbulkpipe(usb_if->usb_dev,
						usb_if->pipe_cmd_in.ucNumber),
				   dev->ucan.cmd_head, dev->ucan.cmd_size, &l,
				   KCAN_USB_CMD_TIMEOUT);

		if (!dev->is_plugged) {
			err = -ENODEV;
		}

		if (err < 0) {
			pr_err(DEVICE_NAME
				": %s(): usb_bulk_msg() err %d\n",
				__func__, err);
			break;
		}

		if (!l) {
			pr_warn(DEVICE_NAME ": abnormal null response (err=%d)."
				" Retrying...\n",
				err);
			continue;
		}

#ifdef DEBUG_USB
		dump_mem("rcvd rsp", cmd, l);
#endif

		/* check if it's the good one */
		opcode = le16_to_cpu(cmd->opcode);
		if (opcode != (op_req+1)) {
			pr_warn(DEVICE_NAME
				": %s(): got abnormal rsp 0x%x to req 0x%x\n",
				__func__, opcode, op_req);
			continue;
		}

		opt->size = sizeof(u32);
		tmp32 = le32_to_cpu(cmd->io_val);
		if (kcan_copy_to_user(opt->value, &tmp32, opt->size, c)) {
			pr_err(DEVICE_NAME ": %s(): copy_to_user() failure\n",
				__func__);
			err = -EFAULT;
		}

		/* if fw sends a 512 bytes packet and cmd_size = 512, then it
		 * needs to send another one to stop the transfer. That's the
		 * reason why a 2nd dummy read is needed: with some FW, a 0
		 * length packet is sent by the packet.
		 * With some other (ATSAM MCU), then a 4 bytes packet only is
		 * sent by the FW.
		 * When cmd_size is greater than wPacketSize (=512) then the
		 * transfer entirely fills into the 1st read, therefore, there
		 * is no need to read again...
		 */
		if (dev->ucan.cmd_size == usb_if->pipe_cmd_out.wDataSz) {

			/* dummy read to complete the above read */
			usb_bulk_msg(usb_if->usb_dev,
				usb_rcvbulkpipe(usb_if->usb_dev,
					usb_if->pipe_cmd_in.ucNumber),
				dev->ucan.cmd_head, dev->ucan.cmd_size, &l,
				KCAN_USB_CMD_TIMEOUT);

			if (!dev->is_plugged) {
				err = -ENODEV;
			}
		}
		break;
	}

	return err;
}

static int kcan_usb_chip_set_io(struct kcandev *dev,
				struct kcanfd_option *opt, void *c,
				u16 op_req)
{
	struct ucan_usb_io_ctrl *cmd;
	u32 io_cfg;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u, op_req=%04xh)\n",
		__func__, kcan_idx(dev)+1, op_req);
#endif
	err = kcan_copy_from_user(&io_cfg, opt->value, sizeof(u32), c);
	if (err) {
		pr_err(DEVICE_NAME ": %s(): copy_from_user() failure\n",
			__func__);
		return -EFAULT;
	}

	cmd = ucan_add_cmd(ucan_init_cmd(dev), op_req);
	if (!cmd)
		return -ENOMEM;

	cmd->io_val = cpu_to_le32(io_cfg);
	return kcan_usbfd_send_ucan_cmd(dev);
}

int kcan_usb_chip_get_dig_cfg(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c)
{
	return kcan_usb_chip_get_io(dev, opt, c, UCAN_USB_CMD_DPIN_CFG_REQ);
}

int kcan_usb_chip_set_dig_cfg(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c)
{
	return kcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_CFG_SET);
}

int kcan_usb_chip_get_dig_val(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c)
{
	return kcan_usb_chip_get_io(dev, opt, c, UCAN_USB_CMD_DPIN_VAL_REQ);
}

int kcan_usb_chip_set_dig_val(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c)
{
	return kcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_VAL_SET);
}

int kcan_usb_chip_set_dig_bit(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c)
{
	return kcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_SET_HIGH);
}

int kcan_usb_chip_clr_dig_bit(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c)
{
	return kcan_usb_chip_set_io(dev, opt, c, UCAN_USB_CMD_DPIN_SET_LOW);
}

int kcan_usb_chip_get_ana_val(struct kcandev *dev,
			      struct kcanfd_option *opt, void *c)
{
	return kcan_usb_chip_get_io(dev, opt, c, UCAN_USB_CMD_ANAL_VAL_REQ);
}

/*
 * static int kcan_usbfd_set_clk_domain(struct kcandev *dev,
 *						struct kcanfd_init *pfdi)
 */
static int kcan_usbfd_set_clk_domain(struct kcandev *dev,
				     struct kcanfd_init *pfdi)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif
	/* select the clock for the CAN */
	switch (pfdi->clock_Hz) {
	case 20000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_20MHZ);
		break;
	case 24000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_24MHZ);
		break;
	case 30000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_30MHZ);
		break;
	case 40000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_40MHZ);
		break;
	case 60000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_60MHZ);
		break;
	case 80000000:
		ucan_usb_set_clck_domain(dev, UCAN_USB_CLK_80MHZ);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/* int kcan_usbfd_open_fd(struct kcandev *dev, struct kcanfd_init *pfdi)
 */
static int kcan_usbfd_open_fd(struct kcandev *dev, struct kcanfd_init *pfdi)
{
	unsigned short usb_opt_to_clr = 0, usb_opt_to_set = 0;

#if defined(DEBUG_TRACE) || defined(DEBUG_IRQ)
	pr_info(DEVICE_NAME ": %s(CAN%u, clk=%u Hz)\n",
		__func__, kcan_idx(dev)+1, pfdi->clock_Hz);
#endif

	/* v8.16: IP core options are all set by ucan_device_open_fd() so
	 * only USB specific options have to be handled here
	 */
#ifdef UCAN_USB_OPTION_FAST_FWD
	/* setup fast-forward option */
	if (fast_fwd)
		usb_opt_to_set |= UCAN_USB_OPTION_FAST_FWD;
	else
		usb_opt_to_clr |= UCAN_USB_OPTION_FAST_FWD;
#endif

	return ucan_device_open_fd(dev, pfdi, usb_opt_to_set, usb_opt_to_clr);
}

/* int kcan_usbfd_close(struct kcandev *dev)
 */
static int kcan_usbfd_close(struct kcandev *dev)
{

#ifdef DEBUG_TRACE
	struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);

	pr_info(DEVICE_NAME ": %s(CAN%u): opened_count=%d\n",
		__func__, kcan_idx(dev)+1, usb_if->opened_count);
#endif

	/* v8.16 now handles options to clear
	 * Note: since fast_fwd is set/cleared when opening the device, there is
	 * no need to clear it here.
	 */
	return ucan_device_close(dev);
}

/* do blink LED */
static int kcan_usbfd_identify(struct kcandev *dev, u32 delay_ms)
{
	int err;

	if (delay_ms) {

		/* set (FAST) blinking mode */
		err = ucan_usb_set_can_led(dev, UCAN_USB_LED_FAST);
		if (err)
			return err;

		kcan_msleep_interruptible(delay_ms);
	}

	/* restore "normal" mode */
	err = ucan_usb_set_can_led(dev, UCAN_USB_LED_DEF);

	return !err ? UCAN_USB_LED_DEF : err;
}

/* handle uCAN Rx CAN message */
static int kcan_usbfd_decode_canrx(struct ucan_engine *ucan,
					struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_rx_msg *rm = (struct ucan_rx_msg *)rx_msg;
	struct kcandev *dev = (struct kcandev *)arg;
	struct kcan_timeval tv;
	int err;

	err = ucan_post_canrx_msg(dev, rm, 
			kcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(rm->ts_low),
						    le32_to_cpu(rm->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN error message */
static int kcan_usbfd_decode_error(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_error_msg *er = (struct ucan_error_msg *)rx_msg;
	struct kcandev *dev = (struct kcandev *)arg;
	struct kcan_timeval tv;
	int err;

	err = ucan_post_error_msg(dev, er,
			kcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(er->ts_low),
						    le32_to_cpu(er->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN bus_load message */
static int kcan_usbfd_decode_bus_load(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_bus_load_msg *bl = (struct ucan_bus_load_msg *)rx_msg;
	struct kcandev *dev = (struct kcandev *)arg;

	int err = ucan_post_bus_load_msg(dev, bl,
			/* don't lose time to decode timestamp: bus load events
			 * frequency is very high so use time of day instead
			 */
			NULL);

	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;

	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle uCAN status message */
static int kcan_usbfd_decode_status(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_status_msg *st = (struct ucan_status_msg *)rx_msg;
	struct kcandev *dev = (struct kcandev *)arg;
	struct kcan_timeval tv;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": %s CAN%u > STATUS[ts=%u.%u RB=%u EP=%u EW=%u BO=%u]\n",
		dev->adapter->name, kcan_idx(dev)+1,
		le32_to_cpu(st->ts_low), le32_to_cpu(st->ts_high),
		!!UCAN_STMSG_RB(st), !!UCAN_STMSG_PASSIVE(st),
		!!UCAN_STMSG_WARNING(st), !!UCAN_STMSG_BUSOFF(st));
#endif
	err = ucan_post_status_msg(dev, st,
			kcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(st->ts_low),
						    le32_to_cpu(st->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	/* bus is ok: if tx_engine idle, set it to STOPPED so that user will 
	 * initiate writing on it
	 */
	if ((dev->bus_state != KCANFD_ERROR_BUSOFF) &&
			(dev->locked_tx_engine_state == TX_ENGINE_IDLE))
		kcan_set_tx_engine(dev, TX_ENGINE_STOPPED);

	return err;
}

/* handle uCAN USB overrun message */
static int kcan_usbfd_decode_overrun(struct ucan_engine *ucan,
					struct ucan_msg *rx_msg, void *arg)
{
	struct ucan_usb_ovr_msg *ov = (struct ucan_usb_ovr_msg *)rx_msg;
	struct kcandev *dev = (struct kcandev *)arg;
	struct kcan_timeval tv;
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": got overrun msg: ts=0x%08x-%08x\n",
		le32_to_cpu(ov->ts_low), le32_to_cpu(ov->ts_high));
#endif
	err = ucan_post_overflow_msg(dev,
			kcan_usbfd_decode_timestamp(dev,
						    le32_to_cpu(ov->ts_low),
						    le32_to_cpu(ov->ts_high),
						    &tv));
	if (err > 0)
		dev->port.usb.state |= UCAN_USB_SHOULD_WAKEUP;
	else if (err < 0) {
		dev->nLastError = err;
		dev->dwErrorCounter++;
	}

	return err;
}

/* handle USB calibration message:
 */
static int kcan_usbfd_decode_calibration(struct ucan_engine *ucan,
					 struct ucan_msg *rx_msg, void *arg)
{
	struct kcan_usb_interface *usb_if = (void *)arg;
	struct ucan_usb_ts_msg *ts = (struct ucan_usb_ts_msg *)rx_msg;
	struct timeval host_tv;
	u64 host_tv_ns;
	int bus_frame_index;

	kcan_gettimeofday_ex(&host_tv, &host_tv_ns);
	bus_frame_index = kcan_usbpro_get_frame_number(usb_if);

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%s, ts=%u, fi=%u)\n",
		__func__, usb_if->adapter->name, 
				le32_to_cpu(ts->ts_low),
				le16_to_cpu(ts->usb_frame_index));
#endif

	return kcan_usbpro_handle_calibration(usb_if,
				le32_to_cpu(ts->ts_low),
				le16_to_cpu(ts->usb_frame_index),
				bus_frame_index, &host_tv, host_tv_ns);
}

/* int kcan_usbfd_msg_decode(struct kcan_usb_interface *usb_if,
 *			     u8 *msg_addr, int msg_len)
 *
 * Decode a message received from KCAN-USB (Pro) FD
 */
static int kcan_usbfd_msg_decode(struct kcan_usb_interface *usb_if,
					u8 *msg_addr, int msg_len)
{
	struct kcandev *dev;
	int err = 0, d;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(%d)\n", __func__, msg_len);
#ifdef DEBUG_USB
	dump_mem("received msg", msg_addr, msg_len);
#endif
#endif

	/* do some init for each controller */
	for (d = 0; d < usb_if->can_count; d++) {
		dev = usb_if_dev(usb_if, d);
		dev->port.usb.state &= ~UCAN_USB_SHOULD_WAKEUP;
	}

	/* call default uCAN rx CAN messages handler */
	dev = usb_if_dev(usb_if, 0);
	err = ucan_handle_msgs_buffer(&dev->ucan, msg_addr, msg_len);

	/* check if something must be awake */
	for (d = 0; d < usb_if->can_count; d++) {
		dev = usb_if_dev(usb_if, d);

		if (dev->port.usb.state & UCAN_USB_SHOULD_WAKEUP) {
			dev->rx_irq_counter++;
#ifndef NETDEV_SUPPORT
#ifdef DEBUG_USB
			pr_info(DEVICE_NAME
				": wakeup task reading CAN%u\n", d+1);
#endif
			kcan_event_signal(&dev->in_event);
#endif
		}
	}

	return err;
}

#define USB_VENDOR_REQUEST_wVALUE_SETFKT_MASS_STORAGE	0

/*
 * Switch the device in Mass Storage Mode
 */
static int kcan_usbfd_set_mass_storage_mode(struct kcan_usb_interface *usb_if)
{
	int len = kcan_usbpro_request(usb_if,
			USB_VENDOR_REQUEST_FKT,
			USB_VENDOR_REQUEST_wVALUE_SETFKT_MASS_STORAGE,
			NULL, 0);

	return len >= 0 ? 0 : len;
}

/*
 * void kcan_usbfd_cleanup(struct kcandev *dev)
 *
 * Last chance to submit URB before driver removal.
 */
static void kcan_usbfd_cleanup(struct kcandev *dev)
{
	struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);
	int err;

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(CAN%u)\n", __func__, kcan_idx(dev)+1);
#endif
	/* Sometimes, bus off request can't be submit when driver is removed
	 * so, when the device was not properly closed. So, move the bus off
	 * request here to be sure it is sent.
	 */
	err = ucan_set_bus_off(dev);
	if (err)
		goto lbl_end;

#ifdef USB_VENDOR_REQUEST_wVALUE_SETFKT_INTERFACE_DRIVER_LOADED
	/* No more need to switch off the LEDs by ourselves!
	 * Fw does it when we notify it from driver unload!
	 */
#else
	/* Switch LED off */
	err = ucan_usb_set_can_led(dev, UCAN_USB_LED_OFF);
	if (err)
		goto lbl_end;
#endif

lbl_end:
	/* If last device, tell module that driver is unloaded */
	if (dev->can_idx == (usb_if->can_count-1)) {

		/* just to be sure to ignore pending CM */
		usb_if->cm_ignore_count = 1000;

		/* stop calibration msgs when cleaning up last channel */
		if (!err)
			ucan_usb_clr_dis_option(dev, 0,
						UCAN_USB_OPTION_CALIBRATION);

		/* Tell module the CAN driver is unloaded */
		kcan_usbpro_driver_loaded(usb_if, 0, 0);
	}
}

/*
 * int kcan_usbfd_ctrl_init(struct kcandev *dev)
 *
 * Do CAN controller specific initialization.
 */
static int kcan_usbfd_ctrl_init(struct kcandev *dev)
{
	struct kcan_usb_interface *usb_if = dev->port.usb.usb_if;
	int err;

	/* set LED in default state (end of init phase) */
	err = ucan_usb_set_can_led(dev, UCAN_USB_LED_DEF);

	/* sending data is not allowed for the moment */

	/* can do this here because no isr is running at the moment, and only
	 * one process can do this (actually, the 1st one)
	 */
	kcan_set_tx_engine(dev, TX_ENGINE_IDLE);

	if (dev->can_idx == (usb_if->can_count-1)) {

		/* start calibration msgs when initializing last channel */
		if (!err)
			ucan_usb_set_en_option(dev, 0,
						UCAN_USB_OPTION_CALIBRATION);
	}

	return err;
}

/*
 * void kcan_usbfd_free(struct kcan_usb_interface *usb_if)
 */
static void kcan_usbfd_free(struct kcan_usb_interface *usb_if)
{
#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME ": %s(usb_if->index=%d)\n", __func__, usb_if->index);
#endif
	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case KCAN_USBX6_PRODUCT_ID:
		/* don't free anything until last interface */
		if (usb_if->index)
			return;

		kcan_usbx6_devices--;
		break;
	case KCAN_USBX4_PRODUCT_ID:
		kcan_usbx4_devices--;
		break;
	case KCAN_USBPROFD_PRODUCT_ID:
		kcan_usbprofd_devices--;
		break;
	case KCAN_USBFD_PRODUCT_ID:
		kcan_usbfd_devices--;
		break;
	case KCAN_USBCHIP_PRODUCT_ID:
		kcan_chip_devices--;
		break;

	default:
		break;
	}

	/* release dynamic memory only once */
	usb_if->adapter = kcan_free_adapter(usb_if->adapter);
}

/* interface functions used to send commands / handle msgs to USB/uCAN */
static int (*kcan_usbfd_ucan_handlers[])(struct ucan_engine *,
					 struct ucan_msg *,
					 void *) = {
	[UCAN_MSG_CAN_RX] = kcan_usbfd_decode_canrx,

	/* use this one if rx/tx errors counters are pushed into rx queue
	 * as STATUS messages
	 */
	[UCAN_MSG_ERROR] = kcan_usbfd_decode_error,
	[UCAN_MSG_STATUS] = kcan_usbfd_decode_status,
	[UCAN_MSG_BUSLOAD] = kcan_usbfd_decode_bus_load,

	[UCAN_USB_MSG_CALIBRATION] = kcan_usbfd_decode_calibration,
	[UCAN_USB_MSG_OVERRUN] = kcan_usbfd_decode_overrun,
};

static struct ucan_ops kcan_usbfd_ucan_ops = {
	.set_clk_domain = kcan_usbfd_set_clk_domain,
	.send_cmd = kcan_usbfd_send_ucan_cmd,
	.handle_msg_table = kcan_usbfd_ucan_handlers,
	.handle_msg_size = ARRAY_SIZE(kcan_usbfd_ucan_handlers),
};

/*
 * KCAN-USB X6 normal USB device tree:
 *
 * bus "-" parent.portnum "." portnum
 *
 * To link the 3 USB chip together, then:
 * - bus must be identical
 * - parent.portnum must be identical
 * - portnum must be sequential, but they sometimes aren't:
 *
 * Example with bus=1 parent.portnum=3:
 *
 * $ lsusb -t
 * /:  Bus 01.Port 1: Dev 1, Class=root_hub, Driver=ehci-pci/10p, 480M
 *     |__ Port 3: Dev 2, If 0, Class=Hub, Driver=hub/4p, 480M
 *         |__ Port 1: Dev 5, If 0, Class=(Defined at Interface level), Driver=kcan, 480M
 *         |__ Port 2: Dev 6, If 0, Class=(Defined at Interface level), Driver=kcan, 480M
 *         |__ Port 3: Dev 4, If 0, Class=(Defined at Interface level), Driver=kcan
 *
 * [ 2540.690980] kcan: DevelopmentVersion (le)
 * [ 2540.690993] kcan: driver config [mod] [isa] [pci] [pec] [usb]
 * [ 2540.698065] kcan 1-3.3:1.0: KCAN-USB X6 (01h PCB03h) fw v3.4.2 bl v1.0.2
 * [ 2540.699055] kcan: - usb fd device minor 32 number 0 found
 * [ 2540.700444] kcan: - usb fd device minor 33 number 0 found
 * [ 2540.706622] kcan 1-3.1:1.0: KCAN-USB X6 (01h PCB03h) fw v3.4.2 bl v1.0.2
 * [ 2540.709948] kcan: - usb fd device minor 38 number 0 found
 * [ 2540.711184] kcan: - usb fd device minor 39 number 0 found
 * [ 2540.714185] kcan: - usb fd device minor 40 number 0 found
 * [ 2540.715305] kcan: - usb fd device minor 41 number 0 found
 * [ 2540.715388] usbcore: registered new interface driver kcan
 * [ 2540.715400] kcan: major 247.
 *
 * => In that case, port 5 (kcan32) and port 6 (kcan33) declare first.
 *
 * In a normal Linux system, the USB X6 exports a USB hub first with id =
 * bus "-" parent.portnum:
 * - USB device CAN 1st interface (=CAN1+CAN2) is connected to:
 *   bus "-" parent.portnum ".1" (portnum = 1)
 * - USB device CAN 2nd interface (=CAN3+CAN4) is connected to:
 *   bus "-" parent.portnum ".2" (portnum = 2)
 * - USB device CAN 3rd interface (=CAN5+CAN6) is connected to:
 *   bus "-" parent.portnum ".3" (portnum = 3)
 *
 * In a VM however, the hub can't be filtered. Therefore, the 3 USB interfaces
 * are shown as 3 independant USB devices with id = bus "-" portnum with
 * portnum = { k, k+1, k+2 } (with no guarantee that these portnum are really
 * sequential).
 */
static struct kcan_usb_interface *
		kcan_usbfd_get_same_if(struct kcan_usb_interface *usb_if,
				       u16 product_id)
{
	struct kcan_usb_interface *usb_dev_if = NULL;
	struct list_head *ptr;

#ifdef HANDLE_HOTPLUG
	unsigned long flags;

	kcan_lock_get_irqsave(&kcan_drv.devices_lock, flags);
#endif

#ifdef DEBUG_TRACE_ENUM_USB
	pr_info(DEVICE_NAME ": %s()\n", __func__);
#endif

	list_for_each(ptr, &kcan_drv.devices) {
		struct kcandev *dev = list_entry(ptr, struct kcandev, list_dev);

		if (dev->wType != HW_USB_X6)
			goto lbl_continue;

		usb_dev_if = kcan_usb_get_if(dev);
		if (!usb_dev_if) {
			pr_err(DEVICE_NAME
				": %s(L%u): %s: abnormal NULL usb_if for "
				"CAN%u\n",
				__func__, __LINE__, usb_if->adapter->name,
				kcan_idx(dev)+1);
			continue;
		}

		if (!usb_dev_if->usb_dev) {
			pr_err(DEVICE_NAME
				": %s(L%u): %s: abnormal NULL usb_dev for "
				"CAN%u\n",
				__func__, __LINE__, usb_if->adapter->name,
				kcan_idx(dev)+1);
			goto lbl_continue;
		}
		if (!usb_dev_if->usb_dev->bus) {
			pr_err(DEVICE_NAME
				": %s(L%u): %s: abnormal NULL bus for "
				"CAN%u\n",
				__func__, __LINE__, usb_if->adapter->name,
				kcan_idx(dev)+1);
			goto lbl_continue;
		}

		if (le16_to_cpu(usb_dev_if->usb_dev->descriptor.idProduct) !=
				product_id)
			goto lbl_continue;

		if (usb_dev_if->usb_dev->bus->busnum !=
					usb_if->usb_dev->bus->busnum)
			goto lbl_continue;

		/* see drivers/usb/core/usb.c usb_alloc_dev():
		 *
		 * if devices share the same bus, and the same parent, so they
		 * share the same interface.
		 *
		 * if devices share the same bus and the same port, so they
		 * share the same interface.
		 */
		if (usb_dev_if->usb_dev->route) {

			/* Normal Linux: USB-X6 exports a USB hub connected to
			 * bus "-" parent.portnum
			 */
			if (!usb_dev_if->usb_dev->parent) {
				pr_err(DEVICE_NAME
					": %s(L%u): %s: abnormal NULL bus for "
					"CAN%u\n",
					__func__, __LINE__,
					usb_if->adapter->name,
					kcan_idx(dev)+1);
				goto lbl_continue;
			}

#ifdef DEBUG_TRACE_ENUM_USB
			pr_info(DEVICE_NAME
				": - dev.parent[portnum=%d] vs "
				"usb_if.parent[portnum=%d]\n",
				usb_dev_if->usb_dev->parent->portnum,
				usb_if->usb_dev->parent->portnum);
#endif
			/* is usb_if connected to the same hub? */
			if (usb_dev_if->usb_dev->parent->portnum !=
					usb_if->usb_dev->parent->portnum)

				goto lbl_continue;
		}

#ifdef DEBUG_TRACE_ENUM_USB
		pr_info(DEVICE_NAME
			": - dev[portnum=%d] vs usb_if[portnum=%d]\n",
			usb_dev_if->usb_dev->portnum,
			usb_if->usb_dev->portnum);
#endif

#ifdef KCAN_USBIF_SEQ_ENUM
		/* is usb_if the next one? */
		if (usb_if->usb_dev->portnum == usb_dev_if->usb_dev->portnum+1)
#endif
			break;

lbl_continue:
		usb_dev_if = NULL;
	}

#ifdef HANDLE_HOTPLUG
	kcan_lock_put_irqrestore(&kcan_drv.devices_lock, flags);
#endif
	return usb_dev_if;
}

static int kcan_usbfd_set_xseries_index(struct kcan_usb_interface *usb_if,
					struct kcan_usb_interface *same_if,
					unsigned int max_if)
{
#ifdef KCAN_USBIF_SEQ_ENUM
	/* this considers that usb_if enumerate one after the
	 * other which is not always true.
	 */
	if (same_if) {
		usb_if->index = same_if->index + 1;
		if (usb_if->index >= max_if) {
			pr_err(DEVICE_NAME
			       ": %s ABNORMAL if index %u (max %u)\n",
			       usb_if->adapter->name,
			       usb_if->index, max_if);
			return -ENODEV;
		}
	}
#else
	/* check portnum: must be in range [1..max_if].
	 * Running in a VM, portnum may be [k+1..k+max_if].
	 */
	if (usb_if->usb_dev->route) {
		if (usb_if->usb_dev->portnum > max_if) {
			pr_err(DEVICE_NAME ": %s ABNORMAL portnum %d\n",
			       usb_if->adapter->name,
			       usb_if->usb_dev->portnum);
			return -ENODEV;
		}

		usb_if->index = usb_if->usb_dev->portnum - 1;

	/* No parent: hub was not enumerated: rely on sequence order
	 * only
	 */
	} else if (same_if) {
		usb_if->index = same_if->index +
				usb_if->usb_dev->portnum -
				same_if->usb_dev->portnum;

		if (usb_if->index >= max_if) {
			pr_err(DEVICE_NAME
			       ": %s ABNORMAL portnums %d-%d\n",
			       usb_if->adapter->name,
			       same_if->usb_dev->portnum,
			       usb_if->usb_dev->portnum);
			return -ENODEV;
		}
	}
#endif
	return 0;
}

/*
 * Ugly way of getting EP *before* being able to use device_init()...
 * This leads to send twice the same vendor request (see kcan_usbfd_init() next)
 */
int kcan_usbfd_init_ep(struct kcan_usb_interface *usb_if)
{
	return kcan_usbfd_get_fw_info(usb_if, false, NULL);
}

/*
 * int kcan_usbfd_init(struct kcan_usb_interface *usb_if)
 *
 * Do device specific initialization.
 */
int kcan_usbfd_init(struct kcan_usb_interface *usb_if)
{
	struct kcan_usb_interface *same_if = NULL;
	u32 bl_version;
	int c, err;

	if (!usb_if) {
		pr_info(DEVICE_NAME ": NULL usb_if!\n");
		return -ENODEV;
	}
	if (!usb_if->usb_dev) {
		pr_info(DEVICE_NAME ": NULL usb_if->usb_dev!\n");
		return -ENODEV;
	}

#ifdef DEBUG_TRACE
	pr_info(DEVICE_NAME
		": %s(bus=%d port=%d parent_port=%d can_count=%d)\n",
		__func__,
		usb_if->usb_dev->bus->busnum,
		usb_if->usb_dev->portnum,
		usb_if->usb_dev->parent->portnum,
		usb_if->can_count);
#endif

	/* cm_ignore_count = 0 means that clock_drift will be computed by
	 * calibration messages only: CM ts is not kept as a base to event
	 * timestamps. Only the diff between two ts of CM is kept.
	 * Events timestamps must then be rebased.
	 */
	usb_if->cm_ignore_count = 0;
	usb_if->dev_frame_index = -1;

	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case KCAN_USBPROFD_PRODUCT_ID:
		usb_if->adapter = kcan_alloc_adapter("KCAN-USB Pro FD",
						     "IPEH-004061",
						     kcan_usbprofd_devices++,
						     usb_if->can_count);

		break;

	case KCAN_USBX6_PRODUCT_ID:
		/* search for an usb_if with the same id =
		 * bus "-" [parent.portnum "."] portnum
		 */
		same_if = kcan_usbfd_get_same_if(usb_if,
						 KCAN_USBX6_PRODUCT_ID);

		/* if no usb_if has been found, create one */
		if (!same_if) {
			usb_if->adapter =
				kcan_alloc_adapter(str_kcan_usb_x6,
						   "IPEH-004062/004063/004064",
						   kcan_usbx6_devices++,
						   usb_if->can_count * 3);
		/* Otherwise, an usb_if is already registered: sibling found! */
		} else {
			usb_if->adapter = same_if->adapter;
		}

		if (!usb_if->adapter)
			return -ENOMEM;

		err = kcan_usbfd_set_xseries_index(usb_if, same_if, 3);
		if (err)
			return err;
		break;

	case KCAN_USBX4_PRODUCT_ID:
		usb_if->adapter = kcan_alloc_adapter(str_kcan_usb_x4,
						     "KCAN-USB X4",
						     kcan_usbx4_devices++,
						     usb_if->can_count);
		break;

	case KCAN_USBFD_PRODUCT_ID:
		usb_if->adapter = kcan_alloc_adapter("KCAN-USB FD",
						     "IPEH-004022",
						     kcan_usbfd_devices++,
						     usb_if->can_count);
		break;

	case KCAN_USBCHIP_PRODUCT_ID:
		usb_if->adapter = kcan_alloc_adapter("KCAN-Chip USB",
						     "IPEH-004025",
						     kcan_chip_devices++,
						     usb_if->can_count);
		break;
	}

	if (!usb_if->adapter)
		return -ENOMEM;

	/* Set KCAN-USB (Pro) FD hardware specific callbacks */
	usb_if->device_ctrl_init = kcan_usbfd_ctrl_init;
	usb_if->device_get_snr = kcan_usbfd_get_serial_nr;
	usb_if->device_msg_decode = kcan_usbfd_msg_decode;
	usb_if->device_free = kcan_usbfd_free;

	usb_if->device_ctrl_cleanup = kcan_usbfd_cleanup;
	usb_if->device_ctrl_open_fd = kcan_usbfd_open_fd;
	usb_if->device_ctrl_close = kcan_usbfd_close;
	usb_if->device_ctrl_set_bus_on = ucan_set_bus_on;
	usb_if->device_ctrl_set_bus_off = ucan_set_bus_off;
	usb_if->device_ctrl_set_dnr = kcan_usbfd_set_device_nr;
	usb_if->device_ctrl_get_dnr = kcan_usbfd_get_device_nr;
	usb_if->device_ctrl_msg_encode = ucan_encode_msgs_buffer;

	for (c = 0; c < usb_if->can_count; c++) {
		struct kcandev *dev = usb_if_dev(usb_if, c);
		if (!dev) {
			pr_err(DEVICE_NAME
				": %s: ABNORMAL NULL dev #%u/%u\n",
				usb_if->adapter->name, c, usb_if->can_count);
			return -ENODEV;
		}

		/* setup identification callback */
		dev->device_identify = kcan_usbfd_identify;

		/* set uCAN interface function to send cmds and handle msgs */
		dev->ucan.ops = &kcan_usbfd_ucan_ops;

		/* remember the list of channels in each channel */
		dev->ucan.devs = usb_if->devs;
		dev->ucan.devs_count = usb_if->can_count;

		/* use the allocated commands buffer for building uCAN cmds */
		dev->ucan.cmd_head = dev->port.usb.cout_baddr;
		dev->ucan.cmd_size = dev->port.usb.cout_bsize;
	}

	/* Tell module the CAN driver is loaded */
	err = kcan_usbpro_driver_loaded(usb_if, 0, 1);
	if (err) {
		pr_err(DEVICE_NAME
			": unable to tell %s that driver is loaded (err %d)\n",
			usb_if->adapter->name, err);
		return err;
	}

	/* read fw info */
	err = kcan_usbfd_get_fw_info(usb_if, !same_if, &bl_version);
	if (err) {
		pr_err(DEVICE_NAME
			": unable to read fw info from %s (err %d)\n",
			usb_if->adapter->name, err);
		return err;
	}

	if (VER_NUM(usb_if->hw_ver.major,
		    usb_if->hw_ver.minor,
		    usb_if->hw_ver.subminor) >= VER_NUM(3, 4, 2)) {
		usb_if->device_set_devdata = kcan_usbfd_set_devdata;
		usb_if->device_get_devdata = kcan_usbfd_get_devdata;
	}

	/* init adapter fw version with (only) first interface fw version 
	 * (KCAN-USB X6 includes 3x USB interfaces, each one might run a
	 * separate fw version)
	 */
	if (!usb_if->index)
		usb_if->adapter->hw_ver = usb_if->hw_ver;

	/* setting Mass Storage Mode available for all USB FD devices, except
	 * for KCAN-USB FD and KCAN-USB Pro FD with bootloader version < 2
	 */
	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case KCAN_USBFD_PRODUCT_ID:
	case KCAN_USBPROFD_PRODUCT_ID:
		if (bl_version < VER_NUM(2, 0, 0))
			break;

		/* fall through */
		fallthrough;
	default:
		usb_if->device_set_mass_storage_mode =
			kcan_usbfd_set_mass_storage_mode;
		break;
	}

	switch (le16_to_cpu(usb_if->usb_dev->descriptor.idProduct)) {
	case KCAN_USBPROFD_PRODUCT_ID:

		/* Check HW version; if v4 and FW < 3.4.6 then warn about newer
		 * FW available:
		 */
		if ((usb_if->ucRevision == 4) &&
		    (VER_NUM(usb_if->hw_ver.major,
			     usb_if->hw_ver.minor,
			     usb_if->hw_ver.subminor) < VER_NUM(3, 4, 6))) {

			pr_warn(DEVICE_NAME
				": a new firmware is available for this %s at "
				"www.kunhong-elec.com\n",
				usb_if->adapter->name);
		}
		break;
	}

	return 0;
}
#endif /* USB_SUPPORT */
