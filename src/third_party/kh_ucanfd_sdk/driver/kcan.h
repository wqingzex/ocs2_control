/* SPDX-License-Identifier: LGPL-2.1-only */
/*
 * constants and definitions to access the drivers
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
 * Contact:     <support@cdkhdz.com>
 * Maintainer:  Kunhong-Elec driver team <gene@cdkhdz.com>
 */
#ifndef __KCAN_H__
#define __KCAN_H__

#include <linux/types.h>
#include <linux/ioctl.h>

#if defined(DWORD) || defined(WORD) || defined(BYTE)
#error "double define for DWORD, WORD, BYTE found"
#endif

#ifdef __KERNEL__
#define DWORD  u32
#define WORD   u16
#define BYTE   u8
#else
#define DWORD  __u32
#define WORD   __u16
#define BYTE   __u8
#endif

/*
 * parameter wHardwareType, used by open
 */
#define HW_ISA             1 // not supported with LINUX, 82C200 chip
#define HW_DONGLE_SJA      5
#define HW_DONGLE_SJA_EPP  6
#define HW_DONGLE_PRO      7
#define HW_DONGLE_PRO_EPP  8
#define HW_ISA_SJA         9 // use this also for PC/104
#define HW_PCI            10 // this kind of PCI carries SJA1000 chips
#define HW_USB            11 // don't know if this is common over kunhong products
#define HW_PCCARD         12 // not aligned to other OS
#define HW_USB_PRO        13
#define HW_USB_PRO_FD     17	/* same as Device ID (why not?) */
#define HW_USB_FD         18	/* same as Device ID (why not?) */
#define HW_PCIE_FD        19	/* this kind of PCIe runs uCAN FPGA */
#define HW_USB_X6         20	/* same as Device ID (why not?) */
#define HW_USB_X4         32	/* same as Device ID (why not?) */

/* compatibility */
#define HW_PCI_FD         HW_PCIE_FD

/*
 * mask for standard and extended CAN identifiers
 */
#define CAN_MAX_STANDARD_ID     0x7ff
#define CAN_MAX_EXTENDED_ID     0x1fffffff

/*
 * error codes
 */
#define CAN_ERR_OK             0x0000  // no error
#define CAN_ERR_XMTFULL        0x0001  // transmit buffer full
#define CAN_ERR_OVERRUN        0x0002  // overrun in receive buffer
#define CAN_ERR_BUSLIGHT       0x0004  // bus error, errorcounter limit reached
#define CAN_ERR_BUSHEAVY       0x0008  // bus error, errorcounter limit reached
#define CAN_ERR_BUSOFF         0x0010  // bus error, 'bus off' state entered
#define CAN_ERR_QRCVEMPTY      0x0020  // receive queue is empty
#define CAN_ERR_QOVERRUN       0x0040  // receive queue overrun
#define CAN_ERR_QXMTFULL       0x0080  // transmit queue full
#define CAN_ERR_REGTEST        0x0100  // test of controller registers failed
#define CAN_ERR_NOVXD          0x0200  // Win95/98/ME only
#define CAN_ERR_RESOURCE       0x2000  // can't create resource
#define CAN_ERR_ILLPARAMTYPE   0x4000  // illegal parameter
#define CAN_ERR_ILLPARAMVAL    0x8000  // value out of range
#define CAN_ERRMASK_ILLHANDLE  0x1C00  // wrong handle, handle error

/*
 * MSGTYPE bits of element MSGTYPE in structure TKCANMsg
 */
#define MSGTYPE_STANDARD      0x00     // marks a standard frame
#define MSGTYPE_RTR           0x01     // marks a remote frame
#define MSGTYPE_EXTENDED      0x02     // declares a extended frame
#define MSGTYPE_SELFRECEIVE   0x04     // self-received message
#define MSGTYPE_SINGLESHOT    0x08     // single-shot message
#define MSGTYPE_STATUS        0x80     // used to mark a status TKCANMsg

/*
 * maximum length of the version string (attention: used in driver too)
 */
#define VERSIONSTRING_LEN     64

/*
 * structures to communicate via ioctls
 */
typedef struct kcan_init {
	WORD wBTR0BTR1;        // merged BTR0 and BTR1 register of the SJA1000
	BYTE ucCANMsgType;     // 11 or 29 bits - put MSGTYPE_... in here
	BYTE ucListenOnly;     // listen only mode when != 0
} TKCANInit;             // for KCAN_INIT

typedef struct kcan_msg {
	DWORD ID;              // 11/29 bit code
	BYTE  MSGTYPE;         // bits of MSGTYPE_*
	BYTE  LEN;             // count of data bytes (0..8)
	BYTE  DATA[8];         // data bytes, up to 8
} TKCANMsg;              // for KCAN_WRITE_MSG

typedef struct kcan_rd_msg {
	TKCANMsg Msg;          // the above message
	DWORD    dwTime;       // a timestamp in msec, read only
	WORD     wUsec;        // remainder in micro-seconds
} TKCANRdMsg;            // for KCAN_READ_MSG

typedef struct kcan_status {
	WORD  wErrorFlag;      // same as in TPDIAG, is cleared in driver after access
	int   nLastError;      // is cleared in driver after access
} TPSTATUS;              // for KCAN_GET_STATUS

typedef struct kcan_diag {
	WORD  wType;           // the type of interface hardware - see HW_....
	DWORD dwBase;          // the base address or port of this device
	WORD  wIrqLevel;       // the irq level of this device
	DWORD dwReadCounter;   // counts all reads to this device from start
	DWORD dwWriteCounter;  // counts all writes
	DWORD dwIRQcounter;    // counts all interrupts
	DWORD dwErrorCounter;  // counts all errors
	WORD  wErrorFlag;      // gathers all errors
	int   nLastError;      // the last local error for this device
	int   nOpenPaths;      // number of open paths for this device
	char  szVersionString[VERSIONSTRING_LEN]; // driver version string
} TPDIAG;                // for KCAN_DIAG, in opposition to KCAN_GET_STATUS nothing is cleared

typedef struct kcan_btr0btr1 {
	DWORD dwBitRate;       // in + out, bitrate in bits per second
	WORD  wBTR0BTR1;       // out only: the result
} TPBTR0BTR1;

typedef struct kcan_ext_status {
	WORD  wErrorFlag;      // same as in TPDIAG, is cleared in driver after access
	int   nLastError;      // is cleared in driver after access
	int   nPendingReads;   // count of unread telegrams
	int   nPendingWrites;  // count of unsent telegrams
} TPEXTENDEDSTATUS;      // for KCAN_GET_ESTATUS

typedef struct kcan_msg_filter {
	DWORD FromID;          // First CAN ID to accept
	DWORD ToID;            // Last CAN ID to accept
	BYTE  MSGTYPE;         // bits of MSGTYPE_*
} TPMSGFILTER;

/*
 * currently available sub-functions
 */
#define KCAN_SF_SET(f)		(int )((f) << 1)
#define KCAN_SF_GET(f)		(KCAN_SF_SET(f) - 1)

/* 32-bit func compatibles */
#define KCAN_SF_SERIALNUMBER	1
#define KCAN_SF_DEVICENO	2
#define KCAN_SF_FWVERSION	3
#define KCAN_SF_MAX32		63

/* 64 bytes func */
#define KCAN_SF_ADAPTERNAME	65
#define KCAN_SF_PARTNUM		66
#define KCAN_SF_MAX		127

#define SF_GET_SERIALNUMBER	KCAN_SF_GET(KCAN_SF_SERIALNUMBER)
#define SF_GET_HCDEVICENO	KCAN_SF_GET(KCAN_SF_DEVICENO)
#define SF_SET_HCDEVICENO	KCAN_SF_SET(KCAN_SF_DEVICENO)
#define SF_GET_FWVERSION	KCAN_SF_GET(KCAN_SF_FWVERSION)
#define SF_GET_ADAPTERNAME	KCAN_SF_GET(KCAN_SF_ADAPTERNAME)
#define SF_GET_PARTNUM		KCAN_SF_GET(KCAN_SF_PARTNUM)

#define KCAN_SF_DATA_MAXLEN	64

typedef struct kcan_extra_params {
	int   nSubFunction;
	union {
		DWORD	dwSerialNumber;
		BYTE	ucHCDeviceNo;
		BYTE	ucDevData[KCAN_SF_DATA_MAXLEN];
	} func;
} TPEXTRAPARAMS;

/*
 * some predefines for ioctls
 */
#define KCAN_MAGIC_NUMBER  'z'
#define MYSEQ_START        0x80

/*
 * ioctls control codes
 */
#define KCAN_INIT           _IOWR(KCAN_MAGIC_NUMBER, MYSEQ_START,     TKCANInit)
#define KCAN_WRITE_MSG      _IOW (KCAN_MAGIC_NUMBER, MYSEQ_START + 1, TKCANMsg)
#define KCAN_READ_MSG       _IOR (KCAN_MAGIC_NUMBER, MYSEQ_START + 2, TKCANRdMsg)
#define KCAN_GET_STATUS     _IOR (KCAN_MAGIC_NUMBER, MYSEQ_START + 3, TPSTATUS)
#define KCAN_DIAG           _IOR (KCAN_MAGIC_NUMBER, MYSEQ_START + 4, TPDIAG)
#define KCAN_BTR0BTR1       _IOWR(KCAN_MAGIC_NUMBER, MYSEQ_START + 5, TPBTR0BTR1)
#define KCAN_GET_EXT_STATUS _IOR (KCAN_MAGIC_NUMBER, MYSEQ_START + 6, TPEXTENDEDSTATUS)
#define KCAN_MSG_FILTER     _IOW (KCAN_MAGIC_NUMBER, MYSEQ_START + 7, TPMSGFILTER)
#define KCAN_EXTRA_PARAMS   _IOWR(KCAN_MAGIC_NUMBER, MYSEQ_START + 8, TPEXTRAPARAMS)

#endif
