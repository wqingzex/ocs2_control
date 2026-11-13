# SPDX-License-Identifier: LGPL-2.1-only
#
# Makefile - global Makefile (USB + netdev only)
#
# Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd <www.kunhong-elec.com>
#
# Contact:       <support@cdkhdz.com>
# Maintainer:    Kunhong-Elec driver team <gene@cdkhdz.com>
#

#export ARCH=arm
#export CROSS_COMPILE=...

define do-make-driver
@$(MAKE) -C driver $1
endef

define do-make-test
@$(MAKE) -C KH-socket-can-test $1
endef

define do-make-fwtool
@$(MAKE) -C tools/fw_tool $1
endef

define pcan-unload
@echo "Disabling automatic loading driver..."
-sudo rmmod peak_usb 2>/dev/null || true
-sudo rmmod pcan 2>/dev/null || true
@mkdir -p /etc/modprobe.d/
@echo "blacklist peak_usb" > /etc/modprobe.d/peak_usb_blacklist.conf
@echo "blacklist pcan" > /etc/modprobe.d/pcan_blacklist.conf
@echo "Automatic loading disabled."
endef

define kcan-load
-sudo modprobe kcan 2>/dev/null || true
endef

define blacklist-clean
@echo "Removing blacklist files..."
-sudo rm -f /etc/modprobe.d/peak_usb_blacklist.conf
-sudo rm -f /etc/modprobe.d/pcan_blacklist.conf
@echo "Blacklist files removed."
endef

all:
	$(call do-make-driver, all)
	$(call do-make-driver, tools)
	$(call do-make-test, all)
	$(call do-make-fwtool, all)

netdev: all

clean:
	$(call do-make-driver, clean)
	$(call do-make-test, clean)
	$(call do-make-fwtool, clean)

install:
	$(pcan-unload)
	$(call do-make-driver, install)
	$(call do-make-test, install)
	$(call do-make-fwtool, install)
	$(kcan-load)

install_with_dkms:
	$(pcan-unload)
	$(call do-make-driver, install_with_dkms)
	$(call do-make-fwtool, install)
	$(kcan-load)

uninstall:
	$(call do-make-driver, uninstall)
	$(call do-make-test, uninstall)
	$(call do-make-fwtool, uninstall)
	$(blacklist-clean)

# DO NOT DELETE
