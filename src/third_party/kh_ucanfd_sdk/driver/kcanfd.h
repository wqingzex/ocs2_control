/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * CAN-FD extension to Kunhong-Elec CAN products.
 *
 * Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd <www.kunhong-elec.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 *
 * Contact: <support@cdkhdz.com>
 * Author:  Kunhong-Elec driver team <gene@cdkhdz.com>
 */
#ifndef __KCANFD_H__
#define __KCANFD_H__

#if 0
#include <linux/types.h>
#include <linux/ioctl.h>

/* ioctls values */
#define KCAN_MAGIC_NUMBER	'z'
#define KCAN_SEQ_START		0x80

#else

/* include the (old) CAN 2.0 API */
#include "kcan.h"

#endif

/* Constants */
#define KCANFD_CAN20_MAXDATALEN	8
#define KCANFD_CANFD_MAXDATALEN	64

/* enable to define types including array with 0, 1 or more items */
#define __array_of_struct(_n, _x)					\
	_n##s_##_x {							\
		__u32		count;					\
		struct _n	list[_x];				\
	}

#ifdef CONFIG_UBSAN_BOUNDS_STRICT
#define __flex_array_of_struct(_n)					\
	_n##s_0 {							\
		__u32		count;					\
		struct _n	list[];					\
	}

#else
#define __flex_array_of_struct(_n)	__array_of_struct(_n, 0)
#endif

/* Type values */
#define KCANFD_TYPE_NOP		0
#define KCANFD_TYPE_CAN20_MSG	1
#define KCANFD_TYPE_CANFD_MSG	2
#define KCANFD_TYPE_STATUS	3
#define KCANFD_TYPE_ERROR_MSG	4

/* [KCANFD_TYPE_CAN20_MSG]
 * [KCANFD_TYPE_CANFD_MSG]
 *
 * flags bits definition (lowest byte is backward compatible with old MSGTYPE)
 */
#define KCANFD_MSG_STD		0x00000000
#define KCANFD_MSG_RTR		0x00000001
#define KCANFD_MSG_EXT		0x00000002
#define KCANFD_MSG_SLF		0x00000004
#define KCANFD_MSG_SNG		0x00000008
#define KCANFD_MSG_ECHO		0x00000010

/* new bits indicate valid values in the corresponding fields */
#define KCANFD_TIMESTAMP	0x01000000
#define KCANFD_HWTIMESTAMP	0x02000000
#define KCANFD_ERRCNT		0x10000000
#define KCANFD_BUSLOAD		0x20000000
#define KCANFD_OVRCNT		0x40000000

#define KCANFD_DLC_MASK		0x00000F00
#define KCANFD_DLC_SHIFT	8

/* CAN-FD */
#define KCANFD_MSG_BRS		0x00100000
#define KCANFD_MSG_ESI		0x00200000

/* [KCANFD_TYPE_STATUS]
 *
 * id field values:
 */
enum kcanfd_status {

	KCANFD_UNKNOWN,

	/* flags & KCANFD_ERROR_BUS.
	 * event id. is the state of the Bus
	 */
	KCANFD_ERROR_ACTIVE,
	KCANFD_ERROR_WARNING,
	KCANFD_ERROR_PASSIVE,		/* receive only state */
	KCANFD_ERROR_BUSOFF,		/* switched off from the bus */

	/* flags & KCANFD_ERROR_CTRLR|KCANFD_ERROR_INTERNAL.
	 * event id. is one of the following error:
	 */
	KCANFD_RX_EMPTY,
	KCANFD_RX_OVERFLOW,
	KCANFD_RESERVED_1,
	KCANFD_TX_OVERFLOW,

	KCANFD_RESERVED_2,
	KCANFD_BUS_LOAD,

	KCANFD_STATUS_COUNT
};

/* Deprecated "enum kcanfd_status": don't use it anymore */
#define KCANFD_BUS_ERROR	9	/* unused (see KCANFD_TYPE_ERROR_MSG) */
#define KCANFD_TX_EMPTY		7	/* unused */

/* [KCANFD_TYPE_STATUS]
 *
 * flags bits definition: indicate the kind of error/status:
 */
#define KCANFD_ERROR_BUS	0x00000080	/* Bus status */
#define KCANFD_ERROR_PROTOCOL	0x00000100	/* Protocol error */
#define KCANFD_ERROR_CTRLR	0x00000200	/* Controller error */
#define KCANFD_ERROR_INTERNAL	0x00000400	/* Internal error */

/* [KCANFD_TYPE_ERROR_MSG]
 *
 * id field values:
 */
enum kcanfd_error {
	KCANFD_ERRMSG_BIT,	/* SJA1000 ECC register format */
	KCANFD_ERRMSG_FORM,	/* (see data[0] for error code) */
	KCANFD_ERRMSG_STUFF,
	KCANFD_ERRMSG_OTHER,

	KCANFD_ERRMSG_COUNT
};

/* [KCANFD_TYPE_ERROR_MSG]
 *
 * flags bits definition: indicate direction
 */
#define KCANFD_ERRMSG_RX	0x00001000	/* err frame received */
#define KCANFD_ERRMSG_GEN	0x00002000	/* triggered by err generator */

#define KCAN_MAXDATALEN		KCANFD_CAN20_MAXDATALEN
#define KCANFD_MAXDATALEN	KCANFD_CANFD_MAXDATALEN

/* indexes describing content of ctrlr_data array */
enum kcanfd_ctrlr_data {
	KCANFD_RXERRCNT,
	KCANFD_ECHOID = KCANFD_RXERRCNT,	/* KCANFD_MSG_ECHO set */
	KCANFD_TXERRCNT,
	KCANFD_BUSLOAD_UNIT,
	KCANFD_BUSLOAD_DEC,
	KCANFD_MAXCTRLRDATALEN
};

/* define new API with CAN-FD support */
struct kcanfd_msg {
	__u16	type;			/* KCANFD_TYPE_xxx */
	__u16	data_len;		/* true length (not the DLC) */
	__u32	id;			/* CAN / STATUS / ERROR Id. */
	__u32	flags;			/* KCANFD_xxx definitions */
	struct timeval	timestamp;	/* timestamp of the event */
	__u8	ctrlr_data[KCANFD_MAXCTRLRDATALEN];
	__u8	data[KCANFD_MAXDATALEN] __attribute__((aligned(8)));
};

/* messages list base type (flex array of items) */
struct __flex_array_of_struct(kcanfd_msg);

#define kcanfd_msgs		kcanfd_msgs_0

/* rule: 
 * if "bitrate" is not 0, then it is used,
 * if "bitrate" is 0, then all other 4 fields are used instead if all are not 0
 */
struct kcan_bittiming {
	__u32	brp;
	__u32	tseg1;
	__u32	tseg2;
	__u32	sjw;
	__u32	tsam;		/* triple sampling */

	__u32	bitrate;	/* bps */
	__u32	sample_point;	/* in 1/100 th of % (8750 = 87.50%) */
	__u32	tq;		/* Time quantum in ns. */
	__u32	bitrate_real;	/* info only */
};

/* KCANFD_XXX_INIT arg */

/* flags */
#define KCANFD_INIT_LISTEN_ONLY		0x00000001
#define KCANFD_INIT_STD_MSG_ONLY	0x00000002 /* backward compatibility */
#define KCANFD_INIT_FD			0x00000004 /* setup data bitrate too */
#define KCANFD_INIT_FD_NON_ISO		0x00000008 /* CAN-FD non ISO mode */

/* defines how timestamps should be processed by the driver in the messages
 * it gives to the application
 */
#define KCANFD_INIT_TS_HOST_REL		0x00000000 /* rel. to host init time */
#define KCANFD_INIT_TS_DEV_REL		0x00000010 /* rel. to device init time*/
#define KCANFD_INIT_TS_DRV_REL		0x00000020 /* rel. to driver init time*/
#define KCANFD_INIT_TS_FMT_MASK		0x00000030

/* (time is relative) */
#define KCANFD_INIT_TS_ABS		KCANFD_INIT_TS_HOST_REL

/* ask for being notified with KCANFD_TYPE_STATUS[KCANFD_BUS_LOAD]
 * (if the corresponding hardware is able to)
 */
#define KCANFD_INIT_BUS_LOAD_INFO	0x00000100

struct kcanfd_init {
	__u32	flags;
	__u32	clock_Hz;
	struct kcan_bittiming	nominal;
	struct kcan_bittiming	data;
};

/* KCANFD_GET_STATE arg */

struct kcanfd_state {
	__u16	ver_major, ver_minor, ver_subminor;

	struct timeval	tv_init;	/* time the device was initialized */

	enum kcanfd_status	bus_state;	/* CAN bus state */

	__u32	device_id;		/* device id., ffffffff is unused */

	__u32	open_counter;		/* open() counter */
	__u32	filters_counter;	/* count of message filters */

	__u16	hw_type;		/* kcan device type */
	__u16	channel_number;		/* channel number for the device */

	__u16	can_status;		/* same as wCANStatus but NOT CLEARED */
	__u16	bus_load;		/* bus load value, ffff if not given */

	__u32	tx_max_msgs;		/* Tx fifo size in count of msgs */
	__u32	tx_pending_msgs;	/* msgs waiting to be sent */
	__u32	rx_max_msgs;		/* Rx fifo size in count of msgs */
	__u32	rx_pending_msgs;	/* msgs waiting to be read */
	__u32	tx_frames_counter;	/* Tx frames written on device */
	__u32	rx_frames_counter;	/* Rx frames read on device */
	__u32	tx_error_counter;	/* CAN Tx errors counter */
	__u32	rx_error_counter;	/* CAN Rx errors counter */

	__u64	host_time_ns;		/* host time in nanoseconds as it was */
	__u64	hw_time_ns;		/* when hw_time_ns has been received */
};

/* KCANFD_xxx_FILTERS arg */

struct kcanfd_msg_filter {
	__u32	id_from;		/* msgs ID in range [id_from..id_to] */
	__u32	id_to;			/* and flags == msg_flags */
	__u32	msg_flags;		/* will be passed to applications */
};

/* filters list base type (flex array of items) */
struct __flex_array_of_struct(kcanfd_msg_filter);

#define kcanfd_msg_filters		kcanfd_msg_filters_0

/* Device available clocks value */
struct kcanfd_available_clock {
	__u32	clock_Hz;
	__u32	clock_src;
};

/* clocks value list base type (flex array of items) */
struct __flex_array_of_struct(kcanfd_available_clock);

#define kcanfd_available_clocks		kcanfd_available_clocks_0

/* default clock always first */
#define AVCLK_DEFAULT	0

/* CAN-FD bittiming capabilities */
struct kcanfd_bittiming_range {
	__u32	brp_min;
	__u32	brp_max;
	__u32	brp_inc;
	__u32	tseg1_min;
	__u32	tseg1_max;
	__u32	tseg2_min;
	__u32	tseg2_max;
	__u32	sjw_min;
	__u32	sjw_max;
};

/* bittimings ranges list base type (flex array of items) */
struct __flex_array_of_struct(kcanfd_bittiming_range);

#define kcanfd_bittiming_ranges		kcanfd_bittiming_ranges_0

/* bittiming ranges list sequence order */
enum {
	BR_NOMINAL = 0,
	BR_DATA,

	BR_MAX
};

/* Device options */
struct kcanfd_option {
	int	size;
	int	name;
	void *	value;
};

enum {
	KCANFD_OPT_CHANNEL_FEATURES,
	KCANFD_OPT_DEVICE_ID,
	KCANFD_OPT_AVAILABLE_CLOCKS,		/* supersedes */
	KCANFD_OPT_BITTIMING_RANGES,		/* corresponding */
	KCANFD_OPT_DBITTIMING_RANGES,		/* ioctl() below */
	KCANFD_OPT_ALLOWED_MSGS,		/* see _ALLOWED_xxx_ below */
	KCANFD_OPT_ACC_FILTER_11B,
	KCANFD_OPT_ACC_FILTER_29B,
	KCANFD_OPT_IFRAME_DELAYUS,
	KCANFD_OPT_HWTIMESTAMP_MODE,
	KCANFD_OPT_DRV_VERSION,
	KCANFD_OPT_FW_VERSION,
	KCANFD_IO_DIGITAL_CFG,		/* output mode 1: output active */
	KCANFD_IO_DIGITAL_VAL,		/* digital I/O 32-bit value */
	KCANFD_IO_DIGITAL_SET,		/* multiple dig I/O pins to 1=High */
	KCANFD_IO_DIGITAL_CLR,		/* clr multiple dig I/O pins to 0 */
	KCANFD_IO_ANALOG_VAL,		/* get single analog input pin value */
	KCANFD_OPT_MASS_STORAGE_MODE,	/* all USB FD and some USB devices */
	KCANFD_OPT_FLASH_LED,
	KCANFD_OPT_DRV_CLK_REF,
	KCANFD_OPT_LINGER,
	KCANFD_OPT_SELF_ACK,	/* send ACK to self-written frames */
	KCANFD_OPT_BRS_IGNORE,	/* ignore BRS frames (no error frames sent) */

	/* internal use only */
	KCANFD_OPT_DEFERRED_FRM,

	KCANFD_OPT_MAX
};

/* KCANFD_OPT_CHANNEL_FEATURES option:
 * features of a channel
 */
#define KCANFD_FEATURE_FD		0x00000001
#define KCANFD_FEATURE_IFRAME_DELAYUS	0x00000002
#define KCANFD_FEATURE_BUSLOAD		0x00000004
#define KCANFD_FEATURE_HWTIMESTAMP	0x00000008
#define KCANFD_FEATURE_DEVICEID		0x00000010
#define KCANFD_FEATURE_SELFRECEIVE	0x00000020
#define KCANFD_FEATURE_ECHO		0x00000040
#define KCANFD_FEATURE_MSD		0x00000080
#define KCANFD_FEATURE_TS_SOF		0x00000100
#define KCANFD_FEATURE_SELF_ACK		0x00000200
#define KCANFD_FEATURE_BRS_IGN		0x00000400
#define KCANFD_FEATURE_DEVDATA		0x00000800
#define KCANFD_FEATURE_NEW_FW_AV	0x00001000

/* KCANFD_OPT_ALLOWED_MSGS option:
 * bitmask of allowed message an application is able to receive
 */
#define KCANFD_ALLOWED_MSG_CAN		0x00000001
#define KCANFD_ALLOWED_MSG_RTR		0x00000002
#define KCANFD_ALLOWED_MSG_EXT		0x00000004
#define KCANFD_ALLOWED_MSG_STATUS	0x00000010
#define KCANFD_ALLOWED_MSG_ERROR	0x00000100
#define KCANFD_ALLOWED_MSG_ALL		0xffffffff
#define KCANFD_ALLOWED_MSG_NONE		0x00000000

/* KCANFD_OPT_HWTIMESTAMP_MODE option:
 * 0	off (host timestamp at the time the event has been saved)
 * 1	on (host time base + device raw time offset)
 * 2	on (host time base + device time offset handling clock drift)
 * 3	on (device timestamp in struct timeval format)
 * 4	reserved
 * 5	same as 1 + ts is generated at SOF rather than at EOF (if hw allows it)
 * 6	same as 2 + ts is generated at SOF rather than at EOF (if hw allows it)
 * 7	same as 3 + ts is generated at SOF rather than at EOF (if hw allows it)
 */
enum {
	KCANFD_OPT_HWTIMESTAMP_OFF,
	KCANFD_OPT_HWTIMESTAMP_ON,
	KCANFD_OPT_HWTIMESTAMP_COOKED,
	KCANFD_OPT_HWTIMESTAMP_RAW,
	KCANFD_OPT_HWTIMESTAMP_RESERVED_4,
	KCANFD_OPT_HWTIMESTAMP_SOF_ON,
	KCANFD_OPT_HWTIMESTAMP_SOF_COOKED,
	KCANFD_OPT_HWTIMESTAMP_SOF_RAW,

	KCANFD_OPT_HWTIMESTAMP_MAX
};

/* KCANFD_OPT_XXX_VERSION major, minor and subminor fields */
#define KCANFD_OPT_VER_MAJ(v)		(((v) >> 24) & 0xff)
#define KCANFD_OPT_VER_MIN(v)		(((v) >> 16) & 0xff)
#define KCANFD_OPT_VER_SUB(v)		(((v) >> 8) & 0xff)

/* KCANFD_OPT_LINGER option gives the timeout value used by the driver to
 * wait for the pending data to be transmitted:
 * < 0	the driver estimates itself the maximum necessary time to wait until the
 *	queue is empty
 * 0	the driver does not wait until the queue is empty before closing
 *      everything.
 * > 0	this specifies the timeout period in ms.
 */
#define KCANFD_OPT_LINGER_AUTO		(-1)
#define KCANFD_OPT_LINGER_NOWAIT	0

/* KCANFD_RESET argument flags */
#define	KCANFD_RESET_RXFIFO		0x00000001
#define	KCANFD_RESET_TXFIFO		0x00000002
#define	KCANFD_RESET_CTRLR		0x00000004
#define KCANFD_RESET_ALL		0xffffffff

/* DLC handling */
static inline __u8 kcanfd_msg_read_dlc(struct kcanfd_msg *msg)
{
	return ((msg->flags & KCANFD_DLC_MASK) >> KCANFD_DLC_SHIFT);
}

static inline void kcanfd_msg_store_dlc(struct kcanfd_msg *msg, __u8 dlc)
{
	msg->flags &= ~KCANFD_DLC_MASK;
	msg->flags |= ((__u32 )dlc << KCANFD_DLC_SHIFT) & KCANFD_DLC_MASK;
}

/* ioctls codes */
#define KCANFD_SEQ_START		0x90

enum {
	KCANFD_SEQ_SET_INIT = KCANFD_SEQ_START,
	KCANFD_SEQ_GET_INIT,
	KCANFD_SEQ_GET_STATE,
	KCANFD_SEQ_ADD_FILTERS,
	KCANFD_SEQ_GET_FILTERS,
	KCANFD_SEQ_SEND_MSG,
	KCANFD_SEQ_RECV_MSG,
	KCANFD_SEQ_SEND_MSGS,
	KCANFD_SEQ_RECV_MSGS,
	KCANFD_SEQ_GET_AVAILABLE_CLOCKS,	/* deprecated, use related */
	KCANFD_SEQ_GET_BITTIMING_RANGES,	/* options above instead */
	KCANFD_SEQ_GET_OPTION,
	KCANFD_SEQ_SET_OPTION,
	KCANFD_SEQ_RESET,
};

#define KCANFD_SET_INIT		_IOW(KCAN_MAGIC_NUMBER, KCANFD_SEQ_SET_INIT,\
					struct kcanfd_init)

#define KCANFD_GET_INIT		_IOR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_GET_INIT,\
					struct kcanfd_init)

#define KCANFD_GET_STATE	_IOR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_GET_STATE,\
					struct kcanfd_state)

#define KCANFD_ADD_FILTERS	_IOW(KCAN_MAGIC_NUMBER,			       \
				     KCANFD_SEQ_ADD_FILTERS,		       \
				     struct kcanfd_msg_filters)

#define KCANFD_GET_FILTERS	_IOWR(KCAN_MAGIC_NUMBER,		       \
				      KCANFD_SEQ_GET_FILTERS,		       \
				      struct kcanfd_msg_filters)

#define KCANFD_SEND_MSG		_IOW(KCAN_MAGIC_NUMBER, KCANFD_SEQ_SEND_MSG,\
					struct kcanfd_msg)

#define KCANFD_RECV_MSG		_IOR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_RECV_MSG,\
					struct kcanfd_msg)

#define KCANFD_SEND_MSGS	_IOWR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_SEND_MSGS,\
					struct kcanfd_msgs)

#define KCANFD_RECV_MSGS	_IOWR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_RECV_MSGS,\
					struct kcanfd_msgs)

#define KCANFD_GET_AVAILABLE_CLOCKS	_IOWR(KCAN_MAGIC_NUMBER,\
					      KCANFD_SEQ_GET_AVAILABLE_CLOCKS,\
					      struct kcanfd_available_clocks)

#define KCANFD_GET_BITTIMING_RANGES	_IOWR(KCAN_MAGIC_NUMBER,\
					      KCANFD_SEQ_GET_BITTIMING_RANGES,\
					      struct kcanfd_bittiming_ranges)

#define KCANFD_GET_OPTION	_IOWR(KCAN_MAGIC_NUMBER, KCANFD_SEQ_GET_OPTION,\
					struct kcanfd_option)
#define KCANFD_SET_OPTION	_IOW(KCAN_MAGIC_NUMBER, KCANFD_SEQ_SET_OPTION,\
					struct kcanfd_option)

#define KCANFD_RESET		_IOW(KCAN_MAGIC_NUMBER, KCANFD_SEQ_RESET,\
					unsigned long)

#endif
