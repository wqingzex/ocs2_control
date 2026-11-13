#!/bin/bash
# SPDX-License-Identifier: GPL 2.0
#
# This script checks KCAN USB devices minor before creating their symlink.
# With some kernel configurations (such as CONFIG_USB_DYNAMIC_MINORS=y),
# USB core doesn't take user minor base into account but starts from 0.
# As a result, the original KCAN rule will create a symlink named /dev/kcan0
# instead of /dev/kcan32, for the first KCAN USB device added to the system...
# This may conflict with any other "internal" KCAN device which is also 
# plugged into the same host.
# Thus, workaround here is to check if usb device minor is smaller than 32.
# If it is not, nothing is done (e.g. outputs the given minor).
# Otherwise, the script outputs the first free number, starting from 32 (x is 
# free <=> no /dev/kcanx does exist)
# Next rule key should take "%c" (instead of %m) into its formatted string.
#
# This Bash script is to be executed with the following udev rule:
# KERNEL=="kcanusb*", PROGRAM="kcan_usb_minor_check.bash %m", SYMLINK+="kcan%c", MODE="0666"
# 
# Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd <www.kunhong-elec.com>
#
# Contact: <support@cdkhdz.com>
# Author:  Kunhong-Elec driver team <gene@cdkhdz.com>
#
MAX_USB_MINORS=256      # see drivers/usb/core/file.c
KCAN_USB_MINOR_BASE=32  # see driver/src/kcan_main.h

if [ -n "$1" ] && [ $1 -lt ${KCAN_USB_MINOR_BASE} ]; then
	let "m=${KCAN_USB_MINOR_BASE}+$1"
else
	m=$1
fi

echo $m

exit 0
