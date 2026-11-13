/* SPDX-License-Identifier: GPL-2.0 */
/*
 * kcan_main.h - global defines to include in all files this module is made of
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
#ifndef __KCAN_MAIN_H__
#define __KCAN_MAIN_H__

/* INCLUDES */
#include "src/kcan_common.h"

#include <linux/types.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/time.h>

#ifdef LINUX_26
#include <linux/device.h>
#endif

#ifdef DEBUG
#define DEBUG_TX_ENG
#endif

#ifdef PCI_SUPPORT
#include <linux/pci.h>

#define KCAN_PCI_MINOR_BASE	0	/* the base of all pci device minors */
#endif

#ifdef OLD_DEVNUM_SCHEME
/* kcan <= v8.5.1:
 * 0	7	PCI/PCIe
 * 8	15	ISA/PC104
 * 16	23	DNG SP
 * 24	31	DNG EPP
 * 32	39	USB
 * 40	47	PC-CARD
 */
#ifdef ISA_SUPPORT
#define ISA_MINOR_BASE		8
#endif

#ifdef DONGLE_SUPPORT
#define KCAN_DNG_SP_MINOR_BASE	16	/* SP devs minors starting point */
#define KCAN_DNG_EPP_MINOR_BASE	24	/* EPP devs minors starting point */
#endif

#else
/* now:
 * 0	31	PCI/PCIe
 * 32	63	USB
 * 64	71	PC-CARD
 * 72	79	ISA/PC104
 * 80	87	DNG SP
 * 88	95	DNG EPP
 */ 
#ifdef ISA_SUPPORT
#define ISA_MINOR_BASE		72
#endif

#ifdef DONGLE_SUPPORT
#define KCAN_DNG_SP_MINOR_BASE	80	/* SP devs minors starting point */
#define KCAN_DNG_EPP_MINOR_BASE	88	/* EPP devs minors starting point */
#endif
#endif

#include <asm/atomic.h>

#ifdef PARPORT_SUBSYSTEM
#include <linux/parport.h>
#endif

#ifdef PCIEC_SUPPORT
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#endif

#ifdef USB_SUPPORT
#include <linux/usb.h>

#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 4, 19)
typedef struct urb urb_t, *purb_t;
#endif

#define KCAN_USB_MINOR_BASE	32	/* USB dev minors starting point */

#endif

#ifdef PCCARD_SUPPORT

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 37)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 36)
#include <pcmcia/cs_types.h>
#endif
#include <pcmcia/cs.h>
#endif

#include <pcmcia/cistpl.h>
#include <pcmcia/ds.h>

#ifdef OLD_DEVNUM_SCHEME
#define PCCARD_MINOR_BASE	40
#else
#define PCCARD_MINOR_BASE	64
#endif

#define KCAN_USB_MINOR_END	(PCCARD_MINOR_BASE-1)

#endif	/* PCCARD_SUPPORT */

/* compute index of last USB entry according to selected supports */
#ifndef KCAN_USB_MINOR_END

#ifdef OLD_DEVNUM_SCHEME
#define KCAN_USB_MINOR_END	-1
#else

#ifdef ISA_SUPPORT
#define KCAN_USB_MINOR_END	(ISA_MINOR_BASE-1)

#elif defined(DONGLE_SUPPORT)
#define KCAN_USB_MINOR_END	(KCAN_DNG_SP_MINOR_BASE-1)
#else
#define KCAN_USB_MINOR_END	-1
#endif
#endif	/* OLD_DEVNUM_SCHEME */
#endif	/* KCAN_USB_MINOR_END */

#ifdef NETDEV_SUPPORT
#include <linux/netdevice.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
#include <linux/can/dev.h>
#endif
#endif

struct kcanusr;

/* PF_CAN is part of the Linux Mainline Kernel since v2.6.25
 * For older Kernels the KCAN driver includes the needed
 * defines from private files src/can.h and src/error.h
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 25)
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#else /* before 2.6.25 kcan netdev contains private includes */
#include <src/can.h>
#include <src/error.h>

#define ARPHRD_CAN	280	/* to be moved to include/linux/if_arp.h */
#define ETH_P_CAN	0x000C	/* to be moved to include/linux/if_ether.h */
#endif

/* fix overlap in namespace between socketcan can/error.h and kcan.h */
#define CAN_ERR_BUSOFF_NETDEV	CAN_ERR_BUSOFF
#undef CAN_ERR_BUSOFF

#include <kcan.h>
#include <kcanfd.h>

#include "src/kcan_fifo.h"
#include "src/kcan_timing.h"

/* DEFINES */
#define CHANNEL_SINGLE	0	/* this is a single channel device */
#define CHANNEL_MASTER	1	/* multi channel device, master device */
#define CHANNEL_SLAVE	2	/* multi channel device, this is slave */

#define READBUFFER_SIZE		360	/* buffers used in readr/write call */
#define WRITEBUFFER_SIZE	512

#define KCAN_MAJOR	0	/* use dynamic major alloc, 91 otherwise */

/* parameter wBTR0BTR1:
 * bitrate codes of BTR0/BTR1 registers
 */
#define CAN_BAUD_1M	0x0014	/*   1 MBit/s */
#define CAN_BAUD_500K	0x001C	/* 500 kBit/s */
#define CAN_BAUD_250K	0x011C	/* 250 kBit/s */
#define CAN_BAUD_125K	0x031C	/* 125 kBit/s */
#define CAN_BAUD_100K	0x432F	/* 100 kBit/s */
#define CAN_BAUD_50K	0x472F	/*  50 kBit/s */
#define CAN_BAUD_20K	0x532F	/*  20 kBit/s */
#define CAN_BAUD_10K	0x672F	/*  10 kBit/s */
#define CAN_BAUD_5K	0x7F7F	/*   5 kBit/s */

/* Activity states */
#define ACTIVITY_NONE		0	/* LED off           - set when the channel is created or deleted */
#define ACTIVITY_INITIALIZED	1	/* LED on            - set when the channel is initialized */
#define ACTIVITY_IDLE		2	/* LED slow blinking - set when the channel is ready to receive or transmit */
#define ACTIVITY_XMIT		3	/* LED fast blinking - set when the channel has received or transmitted */

/* this is how kcan saves version major+minor+subminor into a 32-bit value */
#define KCAN_MAKE_VERSION(x, y, z)	VER_NUM(x, y, z)

#define KCAN_SF_DEVDATA		(KCAN_SF_MAX-1)
#define SF_GET_DEVDATA		KCAN_SF_GET(KCAN_SF_DEVDATA)
#define SF_SET_DEVDATA		KCAN_SF_SET(KCAN_SF_DEVDATA)

/* this structure holds various channel properties */
typedef struct chn_props {
	u8 ucExternalClock : 1;	/* supplied with a external clock */
	u8 ucMasterDevice  : 2; /* clock master, slave, single */
} CHN_PROPS;

/* helper for fast conversion between SJA1000 and host data ordering */
typedef union {
	u8  uc[4];
	u32 ul;
} ULCONV;

typedef union {
	u8  uc[2];
	u16 uw;
} UWCONV;

/* uCAN device programming interface */
struct ucan_engine;
struct ucan_msg;
struct kcandev;
struct ucan_ops {

	int (*set_clk_domain)(struct kcandev *, struct kcanfd_init *);

	/* Tx path: commands and msgs sending interface */
	int (*send_cmd)(struct kcandev *);
	int (*send_msg)(struct kcandev *);

	/* Rx path: all rx messages handlers are stored into a table */
	int (**handle_msg_table)(struct ucan_engine *,
				 struct ucan_msg *, void *);
	int handle_msg_size;

	/* handler of msgs which ID is outside handle_msg_table[] */
	int (*handle_private_msg)(struct ucan_engine *,
				  struct ucan_msg *, void *);
};

struct ucan_engine {
	struct ucan_ops *	ops;		/* uCAN cmd/msg ops */
	struct kcandev **	devs;		/* uCAN channels */
	int			devs_count;	/* count of uCAN channels */

	void *	cmd_head;	/* buffer used to save uCAN cmds */
	int	cmd_size;	/* size of this buffer in bytes */
	int	cmd_len;	/* length of the cmd (in bytes) */

	u8	frag_rec[128];	/* ALIGN(sizeof(struct ucan_rx_msg)+64,4)=92 */
	int	frag_size;
};

#define ucan_dev(u, i)		((u)->devs[i])

#ifdef PCIEC_SUPPORT
#define PCIEC_CHANNELS	2	/* maximum KCAN-PCIExpressCard channel number */

typedef struct {
	void __iomem *gpoutenable;	/* vaddr for bit-banging interface */
	void __iomem *gpin;
	void __iomem *gpout;
	struct	kcandev *dev[PCIEC_CHANNELS];	/* associated channels */

	struct i2c_adapter		adapter;	/* i2c adapter */
	struct i2c_algo_bit_data	algo_data;	/* bit banging if */

	u8	VCCenable;		/* reflection of VCCEN */
	u8	PCA9553_LS0Shadow;      /* Shadow reg holding LEDs state */
	int	run_activity_timer_cyclic;	/* sync flag stop conditions */

	struct delayed_work	activity_timer;	/* scan for activity timer */
} KCAN_PCIEC_CARD;
#endif

struct kcan_msi {
	int	msi_requested;
	int	msi_assigned;
};

struct ucan_pci_page {
	void *		vbase;
	dma_addr_t	lbase;
	u32		offset;
	u32		size;
};

/* timestamp sync in µs */
struct kcan_time_sync {

	/* SYNC variables */
	struct timeval	tv;		/* host time at sync time */
	u64		tv_ns;		/* (used for PCIe FD) */
	u64		ts_us;		/* last sync'ed device timestamp */

	u64		ttv_us;		/* count of host_us */
	u64		tts_us;		/* count of device_us */
	long		clock_drift;

	/* EVT variables (cm_inactivity_count MUST be the 1st one) */
	u32		cm_inactivity_count;

	struct timeval	evt_base_tv;
	u64		evt_base_us;
	u32		evt_time_lag;

	u32		ts_fixed;
};

/*
 * Enable cooked host/device time synchronization.
 *
 * 0: default to raw device hardware timestamps
 * 1: enable calibration-based host/device timestamp synchronization
 */
#ifndef KCAN_ENABLE_TIME_SYNC
#define KCAN_ENABLE_TIME_SYNC	0
#endif

/* 32-bit area describing the content of the beginning of Rx DMA area of the
 * PCI boards running uCAN
 */
struct ucan_pci_irq_status {
#ifdef __LITTLE_ENDIAN
	uint	irq_tag:4;
	uint	rx_cnt:7;
	uint	:5;
	uint	lnk:1;
	uint	:15;
#else
	uint	:15;
	uint	lnk:1;
	uint	:5;
	uint	rx_cnt:7;
	uint	irq_tag:4;
#endif
};

typedef struct {
	u32	dwConfigPort;		/* the configuration port, PCI only */
	void __iomem *can_port_addr;    /* virtual address of port */
	void __iomem *bar0_cfg_addr;	/* vaddr of the config port */
	struct pci_dev *pciDev;		/* remember the hosting PCI card */
#ifdef PCIEC_SUPPORT
	KCAN_PCIEC_CARD *card;		/* point to a card structure */
#endif
	/* PCIe uCAN specific */
	u64	ucan_cmd;

	u32	irq_tag;
	u32	irq_not_for_me;
	struct ucan_pci_irq_status irq_status;

	dma_addr_t rx_dma_laddr;
	void *	rx_dma_vaddr;

	dma_addr_t tx_dma_laddr;
	void *	tx_dma_vaddr;

	u16	tx_pages_free;
	u16	tx_page_index;

	struct ucan_pci_page *tx_pages;


} PCI_PORT;

typedef struct {
#ifdef PARPORT_SUBSYSTEM
	struct pardevice *pardev;	/* associated parallel port */
#endif
	u16	wEcr;			/* ECR register in case of EPP */
	u8	ucOldDataContent;	/* overwritten contents of port regs */
	u8	ucOldControlContent;
	u8	ucOldECRContent;

	kcan_lock_t	lock;	/* shared access to chip registers */
} DONGLE_PORT;

#ifdef KCAN_HANDLE_IRQ_SHARING
typedef struct {
	struct list_head	item;	/* link for items with same irq level */
	struct kcandev *	dev;	/* device with the same irq level */
} SAME_IRQ_ITEM;

typedef struct {
	struct list_head	same_irq_items;	/* list of SAME_IRQ_ITEM's */

	u16		same_irq_count;		/* count of devices */
	u16		same_irq_active;	/* count of active irqs */
} SAME_IRQ_LIST;
#endif

typedef struct {
#ifdef KCAN_HANDLE_IRQ_SHARING
	SAME_IRQ_ITEM	same;	/* each ISA_PORT belongs to one SAME_IRQ_LIST */
	SAME_IRQ_LIST	anchor;	/* the anchor for one irq level */

	SAME_IRQ_LIST *	my_anchor;	/* list of items for the same irq */
#endif
} ISA_PORT;

#ifdef PCCARD_SUPPORT
struct kcan_pccard;
typedef struct {
	struct kcan_pccard *card;	/* points to the associated pccard */
} PCCARD_PORT;
#endif

#ifdef USB_SUPPORT
struct kcan_usb_time {
	u16	ticks16;

	u16	sync_ticks_init;
	u32	sync_ticks_low;
	u32	sync_ticks_high;
};

typedef struct {
	u8	ucNumber;		/* number (or address) of endpoint */
	u16	wDataSz;		/* supported max data transfer length */
} KCAN_ENDPOINT;

struct kcan_usb_interface;
typedef struct kcan_usb_port {
	struct kcan_usb_interface *usb_if;

	u8	ucHardcodedDevNr;

	u32	dwTelegramCount;	/* counter for telegrams */

	struct kcan_usb_time	time;

	struct urb *	write_data;	/* pointer to write data urb */

	u8 *	write_buffer_addr;	/* buffer for to write data */

	KCAN_ENDPOINT pipe_write;

	u32	state;

	struct urb *	urb_cmd_async;		/* async. cmd URB */
	struct urb *	urb_cmd_sync;		/* sync cmd URB */
	atomic_t	cmd_async_complete;	/* flag set when async cmd  */
	atomic_t	cmd_sync_complete;	/* flag set when sync cmd  */
						/* finished */
	u8 *		cout_baddr;		/* command buffer address */
	int		cout_bsize;		/* command buffer size */

} USB_PORT;
#endif /* USB_SUPPORT */

struct kcan_version {
	int		major;
	int		minor;
	int		subminor;
	int		extra;
};

#define KCAN_GUID_LEN		16		/* 128-bit Unique Identifier */

#define KCAN_ADAPTER_GUID	0x00000001	/* adapter handles a GUID */

struct kcan_adapter {
	u32		flags;
	const char *	name;
	const char *	part_num;
	int		index;
	int		can_count;
	int		opened_count;
	struct kcan_version hw_ver;
	u8		guid[KCAN_GUID_LEN];
};

typedef struct __array_of_struct(kcanfd_available_clock, 1)
	kcanfd_mono_clock_device;

struct kcanfd_options {
	int req_size;
	int (*get)(struct kcandev *dev, struct kcanfd_option *, void *arg);
	int (*set)(struct kcandev *dev, struct kcanfd_option *, void *arg);
};

/* these structs enable to apply clock drift outside ISR, but when the message
 * is read by user.
 */
struct kcan_timeval {
	struct timeval tv;		/* host base time */
	u64 ts_us;			/* event hw time */
	u64 tv_us;			/* hw base time */
	u32 ts_mode;			/* cooking mode */
	long clock_drift;		/* clock drift */
};

struct kcanfd_rxmsg {
	struct kcanfd_msg msg;
	struct kcan_timeval hwtv;
};

struct __flex_array_of_struct(kcanfd_rxmsg);

#define kcanfd_rxmsgs	kcanfd_rxmsgs_0

struct kcanfd_txmsg {
	struct kcanfd_msg msg;
	struct list_head link;
	struct timeval delay;
};

struct __flex_array_of_struct(kcanfd_txmsg);

#define kcanfd_txmsgs	kcanfd_txmsgs_0

/* flags */
#define KCAN_DEV_USES_ALT_NUM	0x00000002
#define KCAN_DEV_IGNORE_RX	0x00000004
#define KCAN_DEV_LINKED		0x00000008
#define KCAN_DEV_CLEANED	0x00000010
#define KCAN_DEV_STATIC		0x00000020
#define KCAN_DEV_OPENED		0x00000040
#define KCAN_DEV_BUS_ON		0x00000080
#define KCAN_DEV_FREE_IRQ0	0x00000100
#define KCAN_DEV_MSI_SHARED	0x00000200
#define KCAN_DEV_CLEANING	0x00000400
#define KCAN_DEV_CLOSING	0x00000800
#define KCAN_DEV_CTRLR_FATAL	0x00001000
#define KCAN_DEV_TS_SOF		0x02000000
#define KCAN_DEV_SELF_ACK	0x04000000
#define KCAN_DEV_BRS_IGN	0x08000000

/* features */
#define KCAN_DEV_BUSLOAD_RDY	0x00010000
#define KCAN_DEV_ERRCNT_RDY	0x00020000
#define KCAN_DEV_MSD_RDY	0x00040000
#define KCAN_DEV_TXPAUSE_RDY	0x00080000
#define KCAN_DEV_HWTS_RDY	0x00100000
#define KCAN_DEV_HWTSC_RDY	0x00200000
#define KCAN_DEV_SLF_RDY	0x00400000
#define KCAN_DEV_ECHO_RDY	0x00800000
#define KCAN_DEV_FD_RDY		0x01000000
#define KCAN_DEV_TS_SOF_RDY	0x02000000
#define KCAN_DEV_SELF_ACK_RDY	0x04000000
#define KCAN_DEV_BRS_IGN_RDY	0x08000000
#define KCAN_DEV_SNG_RDY	0x10000000
#define KCAN_DEV_DEVDATA	0x20000000
#define KCAN_DEV_NEW_FW_AV	0x40000000

#define KCAN_DEV_SJA1000_RDY	(KCAN_DEV_ERRCNT_RDY|\
				 KCAN_DEV_SLF_RDY|\
				 KCAN_DEV_SNG_RDY)

#define TX_ENGINE_CLOSED	0
#define TX_ENGINE_IDLE		1
#define TX_ENGINE_STARTED	2
#define TX_ENGINE_STOPPED	3
#define TX_ENGINE_BUSY		4

#define MAX_WAIT_UNTIL_CLOSE	1000

/* this bit indicates that hw timestamps should be measured at the moment of SOF
 * rather than at the moment of EOF (default)
 */
#define KCANFD_OPT_HWTIMESTAMP_SOF	KCANFD_OPT_HWTIMESTAMP_RESERVED_4
#define KCANFD_OPT_HWTIMESTAMP_MASK	(KCANFD_OPT_HWTIMESTAMP_SOF - 1)

struct kcandev {
	struct list_head	list_dev; /* link anchor for list of devices */

	int	nOpenPaths;	/* number of open paths linked to the device */
	int	nMajor;		/* device major (USB devices have their own) */
	int	nMinor;		/* the associated minor */
	char *	type;		/* the literal type of the device, info only */

	u16	wType;		/* (number type) to distinguish sp and epp */
	u16	wInitStep;	/* device specific init state */

	int	can_idx;	/* in case of multi-CAN board/adapter */
	int	opened_index;	/* open sequence order */

	struct kcan_adapter *adapter;	/* link to the real device */
	struct kcan_version *hw_ver;

	u32	dwPort;		/* the port of the transport layer */

	u32	ts_mode;

	u32	flags;
	u32	features;
	u32	tx_iframe_delay_us;

	unsigned long bus_load_ind_period;

	struct timer_list bus_load_timer;
	u32 bus_load_count;
	u32 bus_load_total;

	u32	allowed_msgs;
	u32	sysclock_Hz;
	const struct kcanfd_available_clocks *clocks_list;
	const struct kcanfd_bittiming_range *bittiming_caps;
	const struct kcanfd_bittiming_range *dbittiming_caps;

	const struct kcanfd_options *	option;
	struct kcan_time_sync	time_sync;	/* used to sync clocks */

	int	linger_opt_value;
	int	linger_cur_value;

	struct device *		sysfs_dev;
	struct attribute **	sysfs_attrs;

	u32	device_alt_num;
	union {
		struct {
#ifdef __LITTLE_ENDIAN
			u32	mask;
			u32	code;
#else
			u32	code;
			u32	mask;
#endif
		};
		u64	value64;
	} acc_11b, acc_29b;

#ifdef NETDEV_SUPPORT
	struct net_device *netdev;	/* reference to net device for AF_CAN */
	struct delayed_work restart_work;
#endif

	struct ucan_engine ucan;	/* ref to the uCAN engine */

	union {
		DONGLE_PORT	dng;	/* private data of the various ports */
		ISA_PORT	isa;
		PCI_PORT	pci;
#ifdef PCCARD_SUPPORT
		PCCARD_PORT	pccard;
#endif
#ifdef USB_SUPPORT
		USB_PORT	usb;
#endif
	} port;

	struct chn_props	props;	/* various channel properties */

	/* read a register */
	u8   (*readreg)(struct kcandev *dev, u8 port);

	/* write a register */
	void (*writereg)(struct kcandev *dev, u8 port, u8 data);

	/* cleanup the interface */
	int  (*cleanup)(struct kcandev *dev);

	/* called at open of a path (open()) */
	int  (*open)(struct kcandev *dev);

	/* called at release of a path (close()) */
	int  (*release)(struct kcandev *dev);

	/* install the interrupt handler */
	int  (*req_irq)(struct kcandev *dev, struct kcanusr *ctx);

	void (*lock_irq)(struct kcandev *dev, kcan_lock_irqsave_ctxt *pflags);
	void (*unlock_irq)(struct kcandev *dev, kcan_lock_irqsave_ctxt *pflags);

	/* release the interrupt */
	void (*free_irq)(struct kcandev *dev, struct kcanusr *ctx);

	/* open the device itself */
	int  (*device_open)(struct kcandev *dev, u16 btr0btr1,
						u8 bExtended, u8 bListenOnly);
	int  (*device_open_fd)(struct kcandev *dev, struct kcanfd_init *pfdi);

	/* reset the controller only */
	int  (*device_reset)(struct kcandev *dev);

	/* release the device itself */
	void (*device_release)(struct kcandev *dev);

	/* identify device */
	int (*device_identify)(struct kcandev *dev, u32 delay_ms);

	/* write the device */
	int  (*device_write)(struct kcandev *dev, struct kcanusr *ctx);

	/* interface to set or get special parameters from the device */
	int  (*device_params)(struct kcandev *dev, TPEXTRAPARAMS *params);

#ifndef NETDEV_SUPPORT
	kcan_event_t	in_event;

	FIFO_MANAGER	rx_fifo;	/* manages the read fifo */
	struct kcanfd_rxmsg *rx_msgs;
#endif
	kcan_event_t	out_event;

	FIFO_MANAGER	tx_fifo;	/* manages the write fifo */
	struct kcanfd_txmsg *tx_msgs;

	unsigned	locked_tx_engine_state;

	kcan_lock_t	wlock;	/* mutual exclusion lock for write invocation */
	kcan_lock_t	isr_lock;	/* in isr */

	kcan_mutex_t	mutex;

#ifndef NO_RT
	struct rtdm_device *	rtdm_dev;
	rtdm_irq_t	irq_handle;	/* mandatory parameter in for Xenomai */
#endif

	int	bus_load;

	int	nLastError;	/* last error written */
	enum kcanfd_status	bus_state;
	enum kcanfd_error	bus_error;
	u32	dwErrorCounter;		/* counts all fatal errors */
	u32	rx_irq_counter;
	u32	tx_irq_counter;
	u32	rx_frames_counter;	/* Rx frames read on device */
	u32	tx_frames_counter;	/* Tx frames written on device */
	u32	rx_bytes_counter;
	u32	tx_bytes_counter;

	struct kcanfd_init	def_init_settings;
	struct timeval		init_timestamp;
	struct kcanfd_init	init_settings;

	void *	filter;		/* ID filter - currently associated to device */

	u16	wCANStatus;	/* status of CAN chip */
	u8	is_plugged;	/* the device is PhysicallyInstalled */
	u8	ucActivityState;	/* state of a channel activity */

	u8	rx_error_counter;	/* Rx errors counter */
	u8	tx_error_counter;	/* Tx errors counter */

	struct {
#ifdef FIFO_PRE_ROUTINE
		struct kcanfd_rxmsg status;
		struct kcanfd_rxmsg error;
#endif
		int	bus_state;
		int	bus_load;
		int	bus_error;
		int	rxerr;
		int	txerr;
	} posted;

	u16	wIrq;		/* the associated irq */
};

#ifdef USB_SUPPORT
struct kcan_usb_interface {
	struct kcan_adapter *adapter;
	struct usb_device *usb_dev;	/* Kernel USB device */
	struct usb_interface *usb_intf;

	int	can_count;
	int	index;
	int	opened_count;

	u32	state;
	int	removing_driver;
	int	cm_ignore_count;	/* nb of CM to ignore before handling */

	u8	ucHardcodedDevNr;
	u32	dwSerialNumber;		/* Serial number of device */
	u8	ucRevision;		/* the revision number of  */
	u16	mcu;

	struct kcan_version	hw_ver;

	atomic_t	r_active_urbs;	/* note all active urbs for reading */
	atomic_t	w_active_urbs;	/* note all active urbs for writing */

	struct urb *	urb_cmd_sync;		/* sync cmd URB */
	atomic_t	cmd_sync_complete;	/* flag set when sync cmd  */
						/* finished */

	int		read_packet_size;	/* packet read buffer size */
	int		write_packet_size;	/* packet write buffer size */

	int		read_buffer_size;
	int		write_buffer_size;

	struct urb *	read_data;		/* pointer to read data urb */
	u8 *		read_buffer_addr[2];	/* read data transfer buffers */

	/* USB pipes to/from CAN controller(s) */
	KCAN_ENDPOINT pipe_cmd_in;
	KCAN_ENDPOINT pipe_cmd_out;
	KCAN_ENDPOINT pipe_read;

	int  (*device_init)(struct kcan_usb_interface *);

	int  (*device_get_snr)(struct kcan_usb_interface *, u32 *);
	int  (*device_msg_decode)(struct kcan_usb_interface *, u8 *, int );
	void (*device_free)(struct kcan_usb_interface *);
	int  (*device_set_mass_storage_mode)(struct kcan_usb_interface *);

	int  (*device_ctrl_init)(struct kcandev *dev);
	void (*device_ctrl_cleanup)(struct kcandev *dev);
	int  (*device_ctrl_open)(struct kcandev *dev, u16, u8, u8 );
	int  (*device_ctrl_open_fd)(struct kcandev *dev,
					struct kcanfd_init *pfdi);
	int  (*device_ctrl_close)(struct kcandev *dev);
	int  (*device_ctrl_set_bus_on)(struct kcandev *dev);
	int  (*device_ctrl_set_bus_off)(struct kcandev *dev);
	int  (*device_ctrl_set_dnr)(struct kcandev *dev, u32);
	int  (*device_ctrl_get_dnr)(struct kcandev *dev, u32 *);
	int  (*device_ctrl_msg_encode)(struct kcandev *dev, u8 *, int *);

#ifdef KCAN_SF_DEVDATA
	int  (*device_set_devdata)(struct kcandev *dev, int, const u8 *);
	int  (*device_get_devdata)(struct kcandev *dev, int, u8 *);
#endif

	int 	dev_frame_index;
	int	bus_frame_index;

	u64	uptime_us;

	u32	cm_ts_low;

	int	frag_rec_offset;

#ifdef CONFIG_UBSAN_BOUNDS_STRICT
	struct kcandev *devs[];		/* a device for each CAN controller */
#else
	struct kcandev *devs[0];	/* a device for each CAN controller */
#endif
};

#define usb_if_dev(u, i)		((u)->devs[i])

#endif

struct kcanusr {

#ifdef NETDEV_SUPPORT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	struct net_device_stats		stats;	/* standard netdev statistics */

#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 31)
	struct can_priv			can;	/* must be the 1st one */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3, 6, 0)
	/* consider playing with bitrates only for 3.6+ and CAN-FD */
	struct can_bittiming_const	bt_const;
	struct can_bittiming_const	dbt_const;
#endif
#endif

#endif /* NETDEV_SUPPORT */

	int	open_flags;

#ifdef NETDEV_SUPPORT
	u8	tx_hwts_enabled;
	u8	rx_hwts_enabled;
	u8	tx_hwts_next_mm;
	u8	tx_hwts_exhausted_reported;
	u16	tx_hwts_outstanding;
	spinlock_t tx_hwts_lock;
	u8	tx_hwts_echo_state[256];
#endif

	u8	pcReadBuffer[READBUFFER_SIZE];	/* used in read() call */
	u8 *	pcReadPointer;	/* points into current read data rest */
	int	nReadRest;	/* rest of data left to read */

	int	nTotalReadCount;        /* for test only */

	u8	pcWriteBuffer[WRITEBUFFER_SIZE];/* used in write() call */
	u8 *	pcWritePointer;	/* work pointer into buffer */
	int	nWriteCount;	/* count of written data bytes */

#ifdef NO_RT
	struct file *			filep;		/* back linkage */
#elif !defined(XENOMAI3)
	struct rtdm_dev_context *	context;	/* back linkage */
#endif

	struct kcandev *dev;	/* pointer to related device */
};

struct kcan_driver {
	int	nMajor;		/* the major number of Kcan interfaces */
	u16	wDeviceCount;	/* count of found devices */
	u16	wInitStep;	/* driver specific init state */

	struct timeval		sInitTime;	/* time when init was called */
	struct list_head	devices;	/* list of devices */
#ifdef HANDLE_HOTPLUG
	kcan_lock_t		devices_lock;	/* devices list mutex */
#endif
	u8 *	szVersionString;	/* driver version string */

#ifdef PCCARD_SUPPORT
#ifndef LINUX_24
	struct pcmcia_driver	pccarddrv;	/* pccard driver structure */
#endif
#endif

#ifdef USB_SUPPORT
	struct usb_driver	usbdrv;		/* usb driver structure */
#endif

#ifdef PCI_SUPPORT
	struct pci_driver	pci_drv;	/* pci driver structure */
#endif

#ifdef ISA_SUPPORT
	/* legacy platform driver */
	struct device_driver	legacy_driver_isa;
#endif

#ifdef DONGLE_SUPPORT
	/* legacy platform driver */
	struct device_driver	legacy_driver_dongle;
#endif

	struct class *	class;	/* the associated class of kcan devices */
};

#ifndef NO_RT
struct kcan_rtdm_dev {
	struct list_head	list_dev;
	struct rtdm_device *	device;
};
#endif

/* the global driver object */
extern struct kcan_driver kcan_drv;
extern ushort txqhiwat;

extern const struct kcanfd_bittiming_range sja1000_capabilities;

extern const kcanfd_mono_clock_device sja1000_clocks;

/* Global functions */

#ifdef USB_SUPPORT

/* this function is global for USB adapters */
static struct kcan_usb_interface *kcan_usb_get_if(const struct kcandev *dev)
{
	return dev->port.usb.usb_if;
}

/* Before calling this function, some fields needs to be set:
 * - can_idx *and* wType (which is done by kcan_alloc_dev())
 *
 * And for KCAN-USB X6 devices:
 * - dev->port.usb.usb_if (which is done next to kcan_alloc_dev())
 * - can_count (which is done before entering kcan_alloc_dev() loop)
 * (see kcan_usb_plugin() in kcan_usb_core.c)
 * - usb_if->index (which is done by kcanfd_device_init())
 */
static inline int kcan_idx(const struct kcandev *dev)
{
	int c = dev->can_idx;

	if (dev->wType == HW_USB_X6) {
		struct kcan_usb_interface *usb_if = kcan_usb_get_if(dev);

		c += usb_if->index * usb_if->can_count;
	}
	return c;
}
#else
#define kcan_idx(dev)	((dev)->can_idx)
#endif

/* request time in msec, fast */
u32 get_mtime(void);

void kcan_init_version(struct kcan_version *ver);
void kcan_init_adapter(struct kcan_adapter *pa, const char *name,
		       const char *part_num, int index, int can_count);

void *__kcan_alloc_adapter_ex(const char *name, const char *part_num, int index,
			      int can_count, int extra_size);

#ifdef DEBUG
static inline void *_kcan_alloc_adapter_ex(char *f, int l, const char *name,
					   const char *part_num, int index,
					   int can_count, int extra_size)
{
	void *p = __kcan_alloc_adapter_ex(name, part_num, index, can_count,
					  extra_size);
	pr_info(DEVICE_NAME ": %s(%u): %s()=%p\n", f, l, __func__, p);
	return p;
}

#define kcan_alloc_adapter_ex(n, p, i, c, e)	\
		_kcan_alloc_adapter_ex(__FILE__, __LINE__, n, p, i, c, e)
#define kcan_alloc_adapter(n, p, i, c)		\
		_kcan_alloc_adapter_ex(__FILE__, __LINE__, n, p, i, c, 0)
#else
#define kcan_alloc_adapter_ex(n, p, i, c, e)	\
		__kcan_alloc_adapter_ex(n, p, i, c, e)

static inline struct kcan_adapter *kcan_alloc_adapter(const char *name,
						      const char *part_num,
						      int index, int can_count)
{
	return kcan_alloc_adapter_ex(name, part_num, index, can_count, 0);
}
#endif

struct kcan_adapter *__kcan_free_adapter(struct kcan_adapter *pa);

#ifdef DEBUG
static inline struct kcan_adapter *_kcan_free_adapter(char *f, int l,
						      struct kcan_adapter *p)
{
	pr_info(DEVICE_NAME ": %s(%u): %s(%p)\n", f, l, __func__, p);
	return __kcan_free_adapter(p);
}

#define kcan_free_adapter(p)	_kcan_free_adapter(__FILE__, __LINE__, p)
#else
#define kcan_free_adapter(p)	__kcan_free_adapter(p)
#endif

void kcan_set_dev_adapter(struct kcandev *dev, struct kcan_adapter *pa);

struct kcandev *__kcan_alloc_dev(char *type_str, u16 type, int index);

#ifdef DEBUG
static inline struct kcandev *_kcan_alloc_dev(char *f, int l, char *type_str,
					      u16 type, int index)
{
	struct kcandev *p = __kcan_alloc_dev(type_str, type, index);
	pr_info(DEVICE_NAME ": %s(%u): %s()=%p\n", f, l, __func__, p);
	return p;
}

#define kcan_alloc_dev(s, t, i)	_kcan_alloc_dev(__FILE__, __LINE__, s, t, i)
#else
#define kcan_alloc_dev(s, t, i)	__kcan_alloc_dev(s, t, i)
#endif

struct kcandev *__kcan_free_dev(struct kcandev *dev);

#ifdef DEBUG
static inline struct kcandev *_kcan_free_dev(char *f, int l, struct kcandev *p)
{
	pr_info(DEVICE_NAME ": %s(%u): %s(%p)\n", f, l, __func__, p);
	return __kcan_free_dev(p);
}

#define kcan_free_dev(p)	_kcan_free_dev(__FILE__, __LINE__, p)
#else
#define kcan_free_dev(p)	__kcan_free_dev(p)
#endif

void __kcan_destroy_dev(struct kcandev *dev);

#ifdef DEBUG
static inline void _kcan_destroy_dev(char *f, int l, struct kcandev *dev)
{
	pr_info(DEVICE_NAME ": %s(%u): %s(%p)\n", f, l, __func__, dev);
	__kcan_destroy_dev(dev);
}

#define kcan_destroy_dev(p)	_kcan_destroy_dev(__FILE__, __LINE__, p)
#else
#define kcan_destroy_dev(p)	__kcan_destroy_dev(p)
#endif

static inline void kcan_lock_dev(struct kcandev *dev)
{
#ifdef NO_RT
	kcan_mutex_lock(&dev->mutex);
#endif
}

static inline void kcan_unlock_dev(struct kcandev *dev)
{
#ifdef NO_RT
	kcan_mutex_unlock(&dev->mutex);
#endif
}

static inline int kcan_trylock_dev(struct kcandev *dev)
{
#ifdef NO_RT
	return kcan_mutex_trylock(&dev->mutex);
#else
	return 1;
#endif
}

void kcan_add_dev_in_list_ex(struct kcandev *dev, u32 flags);
static inline void kcan_add_dev_in_list(struct kcandev *dev)
{
	kcan_add_dev_in_list_ex(dev, 0);
}

struct kcandev *kcan_remove_dev_from_list(struct kcandev *dev);
int kcan_is_device_in_list(struct kcandev *dev);
int kcan_find_free_minors(struct kcandev *pdev, int from, int until);

#define KCAN_DEVICE_ATTR(_v, _name, _show) \
	struct device_attribute kcan_dev_attr_##_v = \
				__ATTR(_name, S_IRUGO, _show, NULL)

#define KCAN_DEVICE_ATTR_RW(_v, _name, _show, _store) \
	struct device_attribute kcan_dev_attr_##_v = \
				__ATTR(_name, S_IRUGO|S_IWUSR, _show, _store)

static inline struct kcandev *to_kcandev(struct device *dev)
{
	return (struct kcandev *)dev_get_drvdata(dev);
}

static inline void kcan_free_irq(struct kcandev *dev)
{
#ifdef NO_RT
	free_irq(dev->wIrq, dev);
#else
	rtdm_irq_free(&dev->irq_handle);
#endif
}

void kcan_soft_init_ex(struct kcandev *dev,
			const struct kcanfd_available_clocks *clocks,
			const struct kcanfd_bittiming_range *pc,
			u32 features);

static inline void kcan_soft_init(struct kcandev *dev)
{
	kcan_soft_init_ex(dev,
			(struct kcanfd_available_clocks *)&sja1000_clocks,
			&sja1000_capabilities, KCAN_DEV_SJA1000_RDY);
}

void dump_mem(const char *prompt, const void *p, int l);

static inline void __kcan_set_tx_engine(struct kcandev *dev, int tx_eng)
{
	dev->locked_tx_engine_state = tx_eng;
}

#ifdef DEBUG_TX_ENG
static inline void kcan_set_tx_engine_dbg(struct kcandev *dev, int tx_eng,
						const char *f, int l)
{
	if (dev->locked_tx_engine_state != tx_eng)
		pr_info(DEVICE_NAME
			": %s(l=%u): CAN%u TX engine goes to %u\n",
			f, l, kcan_idx(dev)+1, tx_eng);

	__kcan_set_tx_engine(dev, tx_eng);
}

#define kcan_set_tx_engine(d, s)	kcan_set_tx_engine_dbg(d, s, \
							__func__, __LINE__)
#else
#define kcan_set_tx_engine(d, s)	__kcan_set_tx_engine(d, s)
#endif

int __kcan_dev_start_writing(struct kcandev *dev, struct kcanusr *ctx);

void kcan_sync_init(struct kcandev *dev);
int kcan_sync_decode(struct kcandev *dev, u32 ts_low, u32 ts_high,
		     struct kcan_timeval *hwtv);
int kcan_sync_times_host(struct kcandev *dev, u32 ts_low, u32 ts_high,
			 int tv_off, const struct timeval *host_tv,
			 u64 host_tv_ns);
int kcan_sync_times(struct kcandev *dev, u32 ts_low, u32 ts_high, int tv_off);

static inline int kcan_sync_decode64(struct kcandev *dev, u64 ts64,
				     struct kcan_timeval *hwtv)
{
	return kcan_sync_decode(dev, ts64 & 0xffffffff, ts64 >> 32, hwtv);
}

static inline int kcan_sync_times64(struct kcandev *dev, u64 ts64, int tv_off)
{
	return kcan_sync_times(dev, ts64 & 0xffffffff, ts64 >> 32, tv_off);
}

static inline int kcan_sync_times64_host(struct kcandev *dev, u64 ts64,
					 int tv_off,
					 const struct timeval *host_tv,
					 u64 host_tv_ns)
{
	return kcan_sync_times_host(dev, ts64 & 0xffffffff, ts64 >> 32,
				    tv_off, host_tv, host_tv_ns);
}

struct kcanfd_msg *kcan_sync_timestamps(struct kcandev *dev,
					struct kcanfd_rxmsg *pqm);

int kcan_post_bus_state(struct kcandev *dev);
int kcan_set_bus_state(struct kcandev *dev, enum kcanfd_status bus_state);
void kcan_copy_err_counters(struct kcandev *dev, struct kcanfd_rxmsg *pf);
int kcan_get_dev_features(struct kcandev *dev, u32 *features);

int kcan_handle_busoff(struct kcandev *dev, struct kcanfd_rxmsg *pf);
void kcan_handle_error_active(struct kcandev *dev, struct kcanfd_rxmsg *pf);
int kcan_handle_error_status(struct kcandev *dev, struct kcanfd_rxmsg *pf,
				int err_warning, int err_passive);
int kcan_handle_error_ctrl(struct kcandev *dev, struct kcanfd_rxmsg *pf,
				int err_ctrl);
void kcan_handle_error_msg(struct kcandev *dev, struct kcanfd_rxmsg *pf,
			int err_type, u8 err_code, int err_rx, int err_gen);
void kcan_handle_error_internal(struct kcandev *dev, struct kcanfd_rxmsg *pf,
				int err_internal);
void kcan_handle_error_protocol(struct kcandev *dev, struct kcanfd_rxmsg *pf,
				int err_protocol);

void kcan_soft_error_active(struct kcandev *dev);

int kcan_set_status_bit(struct kcandev *dev, u16 bits);
int kcan_clear_status_bit(struct kcandev *dev, u16 bits);

void kcan_cleanup_dev(struct kcandev *dev);

u16 sja1000_bitrate(u32 dwBitRate, u32 sample_pt, u32 sjw);

/* get bitrate in bps */
static inline u32 kcan_get_bps(u32 clk_Hz, struct kcan_bittiming *pbt)
{
	return clk_Hz / (pbt->brp * (1 + pbt->tseg1 + pbt->tseg2));
}

int kcan_init_rxmsg(struct kcandev *dev, struct kcanfd_rxmsg *rx,
		    __u16 type, __u32 id, __u32 flags);

int kcan_chardev_rx(struct kcandev *dev, struct kcanfd_rxmsg *cf);
int kcan_txfifo_get(struct kcandev *dev, struct kcanfd_txmsg *tx);

void kcan_sysfs_dev_node_create_ex(struct kcandev *dev, struct device *parent);
void kcan_sysfs_dev_node_destroy(struct kcandev *dev);

static inline void kcan_sysfs_dev_node_create(struct kcandev *dev)
{
	kcan_sysfs_dev_node_create_ex(dev, NULL);
}

void remove_dev_list(void);

#define kcanfd_dump_bittiming(pbt, c)	\
		__kcanfd_dump_bittiming(__FILE__, __LINE__, pbt, c)

static inline void __kcanfd_dump_bittiming(const char *fn, int l,
				struct kcan_bittiming *pbt, u32 clock_Hz)
{
	u32 btr = pbt->bitrate;
	char unit = ' ';

	if (btr >= 1000000) {
		btr /= 1000000;
		unit = 'M';
	} else if (btr >= 1000) {
		btr /= 1000;
		unit = 'k';
	}
	pr_info(DEVICE_NAME ": %s(%u): clk=%uMHz btr=%u%cbps sp=%u "
		"[BRP=%u TSEGx=%u/%u SJW=%u TQ=%u]\n",
		fn, l, clock_Hz/1000000, btr, unit, pbt->sample_point,
		pbt->brp, pbt->tseg1, pbt->tseg2, pbt->sjw, pbt->tq);
}

static inline u16 kcan_bittiming_to_btr0btr1(struct kcan_bittiming *pbt)
{
	u16 btr0btr1;

	btr0btr1 = ((pbt->sjw - 1) & 0x3) << 14;
	btr0btr1 |= ((pbt->brp - 1) & 0x3f) << 8;
	btr0btr1 |= (!!pbt->tsam) << 7;
	btr0btr1 |= ((pbt->tseg2 - 1) & 0x7) << 4;
	btr0btr1 |= ((pbt->tseg1 - 1) & 0xf);

	return btr0btr1;
}

int strtounit(char *str, u32 *pv, char *units);

const struct kcanfd_options *kcan_inherit_options_from(
				struct kcanfd_options *child_opts,
				const struct kcanfd_options *parent_opts);

int kcan_sysfs_add_attr(struct device *dev, struct attribute *attrs);
int kcan_sysfs_add_attrs(struct device *dev, struct attribute **attrs);
void kcan_sysfs_del_attr(struct device *dev, struct attribute *attrs);
void kcan_sysfs_del_attrs(struct device *dev, struct attribute **attrs);

#endif /* __KCAN_MAIN_H__ */
