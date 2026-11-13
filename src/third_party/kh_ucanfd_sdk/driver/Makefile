# SPDX-License-Identifier: GPL-2.0
#
# Makefile for Kunhong-Elec CAN interfaces driver (USB + netdev only)
#
# Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd <www.kunhong-elec.com>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
# Contact:      <support@cdkhdz.com>
# Maintainer:   Kunhong-Elec driver team <gene@cdkhdz.com>
#
# targets:
# all / netdev       build the driver (USB + netdev mode)
# tools              build optional userspace helper tools
# clean              clean up
# install            install the driver
# install_with_dkms  install using dkms
# uninstall          uninstall the driver
# debug              build with debug enabled
#
# make options:
# KERNEL_LOCATION=...  kernel source path (for cross-compile)
# VERBOSE=1            verbose build output
# DKMS=DKMS_SUPPORT    use dkms for install
# DBG=DEBUG            enable debug
#

# override default CC = cc
CC = $(CROSS_COMPILE)gcc

DRV = kcan

# Hardcoded: USB + netdev + NO_RT
DBG     = NO_DEBUG
MOD     = MODVERSIONS
USB     = USB_SUPPORT
NET     = NETDEV_SUPPORT
RT      = NO_RT

KCAN_BUILD_OPTS := MOD USB NET RT

MAKE_OPTS = $(foreach o,$(KCAN_BUILD_OPTS),$(o)=$(value $(o)))
KCAN_OPTS = $(foreach o,$(KCAN_BUILD_OPTS),-D$(value $(o)))

# System tools
DEPMOD = depmod
MODPROBE = modprobe
UDEVADM = udevadm

# dkms stuff
DKMS_DRV = kunhong-linux-driver
DKMS_VER = 1.4.1
DKMS_CONF = dkms.conf
DKMS_TREE = /var/lib/dkms
DKMS_BIN = $(shell which dkms 2> /dev/null)
ifeq ($(DKMS),DKMS_SUPPORT)
ifneq ($(DKMS_BIN),)
USE_DKMS = yes
endif
endif

#
# please modify only here if your kernel path is elsewhere located
#
KERNEL_LOCATION = $(DKMS_KERNEL_DIR)

#
# probe various standard kernel locations
#
ifeq ($(KERNEL_LOCATION),)
define KERNEL_LOCATION_1
$(shell UR=`uname -r`; \
	for d in \
	/lib/modules/$$UR/build \
	/lib/modules/$$UR/source \
	; do \
		if [ -d $$d ]; then \
			r=$$(readlink -f $$d 2>/dev/null || echo $$d); \
			if [ -f "$$r/Makefile" ] && [ -d "$$r/include" ] && { [ -d "$$r/scripts" ] || [ -d "$$r/arch" ]; }; then echo $$r; break; fi; \
		elif [ -h $$d ]; then \
			r=$$(readlink -f $$d); \
			[ -n "$$r" ] && [ -f "$$r/Makefile" ] && [ -d "$$r/include" ] && { [ -d "$$r/scripts" ] || [ -d "$$r/arch" ]; } && echo $$r && break; \
		fi; \
	done)
endef
ifeq ($(KERNEL_LOCATION_1),)
$(error "Unable to automatically find the Kernel headers.\
These are mandatory to build the kcan driver. Please set the KERNEL_LOCATION\
variable to the root directory of your own built Kernel.")
else
KERNEL_LOCATION = $(KERNEL_LOCATION_1)
endif
endif

#
# Get Linux OS release
#
-include $(DESTDIR)/etc/os-release
OS_RELEASE_ID = $(NAME)
ifeq ($(OS_RELEASE_ID),)
OS_RELEASE_ID = UNKNOWN_LINUX
endif
OS_RELEASE_VER = $(VERSION_ID)
ifeq ($(OS_RELEASE_VER),)
OS_RELEASE_VER = X.Y.Z
endif

#
# determine where to find the kernel built header files
#
define HEADER_PATH
$(shell if [ -d $(KERNEL_LOCATION)/include/generated ]; then \
		echo "$(KERNEL_LOCATION)/include/generated"; \
	else \
		echo "$(KERNEL_LOCATION)/include/linux"; \
	fi)
endef

define VERSION_H_DIR
$(shell if [ -d $(HEADER_PATH)/uapi ]; then \
		echo "$(HEADER_PATH)/uapi"; \
	else \
		echo "$(KERNEL_LOCATION)/include"; \
	fi)
endef

HASVERSION_H = $(shell [ -f $(VERSION_H_DIR)/linux/version.h ] && echo yes)

ifneq ($(HASVERSION_H),yes)
KERNEL_VERSION := $(shell uname -r)
else

KERNEL_VERSION := $(shell cpp -E -dM -I$(KERNEL_LOCATION)/include \
                              $(VERSION_H_DIR)/linux/version.h | \
                              grep UTS_RELEASE | sed -e 's;[^"]*"\(.*\)";\1;g')

ifeq ($(KERNEL_VERSION),)
KERNEL_VERSION := $(shell cpp -E -dM -I$(KERNEL_LOCATION)/include \
                              $(HEADER_PATH)/utsrelease.h | \
                              grep UTS_RELEASE | sed -e 's;[^"]*"\(.*\)";\1;g')
endif
endif

ifeq ($(KERNEL_VERSION),)
$(error "Unable to read Kernel version string from $(KERNEL_LOCATION)")
endif

#
# where the driver should be installed
#
ifeq ($(KERNEL_LOCATION),)
MODULES_DIR = $(DESTDIR)/lib/modules/$(shell uname -r)
else
MODULES_DIR = $(DESTDIR)/lib/modules/$(KERNEL_VERSION)
endif

INSTALL_LOCATION = $(MODULES_DIR)/misc

#
# get extracted kernel VERSION and PATCHLEVEL for comparison
#
KVERSION := $(shell echo $(KERNEL_VERSION) | sed -e 's/^\([1-9][0-9]*\)\..*/\1/' )
KPATCHLEVEL := $(shell echo $(KERNEL_VERSION) | sed -e 's/^$(KVERSION)\.0*\([0-9][0-9]*\).*/\1/' )
SUBLEVELTMP := $(shell echo $(KERNEL_VERSION) | sed -e 's/^$(KVERSION)\.0*$(KPATCHLEVEL)\.0*\([0-9][0-9]*\).*/\1/' )

ifeq ($(SUBLEVELTMP),$(KERNEL_VERSION))
SUBLEVEL=0
else
SUBLEVEL=$(SUBLEVELTMP)
endif

KVERNUM := $(shell expr $(KVERSION) \* 10000 + $(KPATCHLEVEL) \* 100 + $(SUBLEVEL))

USE_KBUILD = $(shell [ $(KVERNUM) -ge 20600 ] && echo yes)

SRC     = src
PWD     = $(shell pwd)

ifeq ($(DBG),DEBUG)
VERBOSE = 1
else
VERBOSE = 0
endif

# native support of CONFIG_COMPAT can be disabled with:
# $ make COMPAT_SUPPORT=NO
ifeq ($(COMPAT_SUPPORT),NO)
KCAN_OPTS += -DNO_COMPAT
MAKE_OPTS += COMPAT_SUPPORT=NO
endif

# Test current system for UDEV support
define UDEV
$(shell if [ -d $(DESTDIR)/etc/udev/rules.d ]; then \
		echo yes; \
	else \
		echo no; \
	fi )
endef

#****************************************************************************
# netdev: check for CAN restart timer vs delayed_work
#
ifeq ($(OS_RELEASE_ID),"CentOS Linux")
DRV_CFLAGS += -Dndo_change_mtu=ndo_change_mtu_rh74
endif

K_4_8 := $(shell [ $(KVERNUM) -le 40800 ] && echo yes)
ifeq ($(K_4_8),yes)
DEV_H := $(KERNEL_LOCATION)/include/linux/can/dev.h
HAS_CAN_DEV_H := $(shell [ -f $(DEV_H) ] && echo yes)

ifneq ($(HAS_CAN_DEV_H),yes)
DEV_H := $(dir $(KERNEL_LOCATION))source/include/linux/can/dev.h
HAS_CAN_DEV_H := $(shell [ -f $(DEV_H) ] && echo yes)
endif

ifeq ($(HAS_CAN_DEV_H),yes)
DRV_CFLAGS += $(shell  awk '/^struct can_priv {/ { in_can_priv=1 } /^};/ { in_can_priv=0 } /^[ \t]*struct[ \t]*timer_list[ \t]*restart_timer/ { if (in_can_priv) print "-DLINUX_CAN_RESTART_TIMER" }' $(DEV_H))
else
$(warning $(DEV_H) not found: netdev mode might fail to compile)
endif
endif

ETHTOOL_H := $(firstword $(wildcard $(KERNEL_LOCATION)/include/linux/ethtool.h $(dir $(KERNEL_LOCATION))source/include/linux/ethtool.h))
NETDEVICE_H := $(firstword $(wildcard $(KERNEL_LOCATION)/include/linux/netdevice.h $(dir $(KERNEL_LOCATION))source/include/linux/netdevice.h $(KERNEL_LOCATION)/include/uapi/linux/netdevice.h $(dir $(KERNEL_LOCATION))source/include/uapi/linux/netdevice.h))
NET_TSTAMP_H := $(firstword $(wildcard $(KERNEL_LOCATION)/include/linux/net_tstamp.h $(dir $(KERNEL_LOCATION))source/include/linux/net_tstamp.h $(KERNEL_LOCATION)/include/uapi/linux/net_tstamp.h $(dir $(KERNEL_LOCATION))source/include/uapi/linux/net_tstamp.h))
CAN_DEV_API_H := $(firstword $(wildcard $(KERNEL_LOCATION)/include/linux/can/dev.h $(dir $(KERNEL_LOCATION))source/include/linux/can/dev.h))

define NETDEV_OP_HAS
$(shell [ -n "$(NETDEVICE_H)" ] && awk '/^struct[[:space:]]+net_device_ops[[:space:]]*\{/ { in_struct=1; next } in_struct && /$(1)/ { found=1 } in_struct && /^};/ { exit } END { if (found) print yes }' $(NETDEVICE_H))
endef

HAS_NDO_ETH_IOCTL := $(call NETDEV_OP_HAS,ndo_eth_ioctl)
HAS_NDO_HWTSTAMP_GET := $(call NETDEV_OP_HAS,ndo_hwtstamp_get)
HAS_NDO_HWTSTAMP_SET := $(call NETDEV_OP_HAS,ndo_hwtstamp_set)
HAS_KERNEL_ETHTOOL_TS_INFO := $(shell [ -n "$(ETHTOOL_H)" ] && grep -Eq 'struct[[:space:]]+kernel_ethtool_ts_info[[:space:]]*\{' $(ETHTOOL_H) && echo yes)
HAS_KERNEL_HWTSTAMP_CONFIG := $(shell [ -n "$(NET_TSTAMP_H)" ] && grep -Eq 'struct[[:space:]]+kernel_hwtstamp_config[[:space:]]*\{' $(NET_TSTAMP_H) && echo yes)
HAS_CAN_ETH_IOCTL_HWTS := $(shell [ -n "$(CAN_DEV_API_H)" ] && grep -Eq 'can_eth_ioctl_hwts' $(CAN_DEV_API_H) && echo yes)

ifeq ($(HAS_KERNEL_ETHTOOL_TS_INFO),yes)
DRV_CFLAGS += -DKCAN_HAS_KERNEL_ETHTOOL_TS_INFO
endif

ifeq ($(HAS_NDO_ETH_IOCTL),yes)
ifeq ($(HAS_CAN_ETH_IOCTL_HWTS),yes)
DRV_CFLAGS += -DKCAN_HAS_NDO_ETH_IOCTL
endif
endif

ifeq ($(HAS_NDO_ETH_IOCTL),yes)
ifeq ($(HAS_CAN_ETH_IOCTL_HWTS),yes)
ifeq ($(HAS_NDO_HWTSTAMP_GET),yes)
ifeq ($(HAS_NDO_HWTSTAMP_SET),yes)
ifeq ($(HAS_KERNEL_HWTSTAMP_CONFIG),yes)
DRV_CFLAGS += -DKCAN_HAS_NDO_HWTSTAMP_OPS
endif
endif
endif
endif
endif

#****************************************************************************
#
# Object files: core + USB + netdev
#
kcan-objs := $(SRC)/kcan_main.o $(SRC)/kcan_fops.o $(SRC)/kcan_fifo.o $(SRC)/kcan_filter.o
kcan-objs += $(SRC)/kcan_parse.o $(SRC)/kcan_sja1000.o $(SRC)/kcan_common.o $(SRC)/kcan_timing.o
kcan-objs += $(SRC)/kcanfd_core.o $(SRC)/kcanfd_ucan.o

# USB support
kcan-objs += $(SRC)/kcan_usb_core.o $(SRC)/kcan_usb.o
kcan-objs += $(SRC)/kcan_usbpro.o
kcan-objs += $(SRC)/kcanfd_usb.o

# netdev support
kcan-objs += $(SRC)/kcan_netdev.o

BINDIR = /usr/local/bin
TOOLS = kcan-settings
TOOLS_CFLAGS = -D$(RT) -I. -g
TOOLS_LDFLAGS =

ifeq ($(USE_KBUILD),yes)

MODCONF := $(shell if test -d "$(DESTDIR)/etc/modprobe.d" ; then echo "$(DESTDIR)/etc/modprobe.d/$(DRV).conf" ; else echo "$(DESTDIR)/etc/modules.conf.local" ; fi)
ifeq ($(UDEV),yes)
MODINST = "install $(DRV) $(MODPROBE) --ignore-install $(DRV)"
else
MODINST = "install $(DRV) $(MODPROBE) --ignore-install $(DRV); $(BINDIR)/kcan_make_devices 2"
endif

TARGET = $(DRV).ko
obj-m := $(DRV).o

# In kbuild phase, PWD points to kernel tree, so use M (module dir).
# Keep PWD as fallback for direct/non-kbuild invocations.
KCAN_CFLAGS := -I$(if $(M),$(M),$(PWD)) -D$(DBG) $(KCAN_OPTS) $(DRV_CFLAGS)
# Use ccflags-y only. EXTRA_CFLAGS support was removed in newer kbuild.
ccflags-y += $(KCAN_CFLAGS)

# suppress date-time warning for gcc >= 4.9
GCC_VERMAJ := $(shell $(CC) -dumpversion | cut -d. -f1)
GCC_VERMIN := $(shell $(CC) -dumpversion | cut -d. -f2)

DATE_TIME_WARNING := $(shell if [ $(GCC_VERMAJ) -gt 4 ]; then echo y; else echo n; fi)
ifeq ($(DATE_TIME_WARNING),n)
DATE_TIME_WARNING := $(shell if [ $(GCC_VERMAJ) -eq 4 ] && [ $(GCC_VERMIN) -ge 9 ]; then echo y; else echo n; fi)
endif
ifeq ($(DATE_TIME_WARNING),y)
ccflags-y += -Wno-date-time
endif

CMDLINE = $(MAKE) -C $(KERNEL_LOCATION) $(CMD_OPTS) V=$(VERBOSE) modules

define USE_SUBDIRS
$(shell [ $(KVERNUM) -ge 20605 ] && [ $(KVERNUM) -lt 50000 ] && echo yes)
endef

ifneq ($(USE_SUBDIRS),yes)
CMDLINE += M=$(PWD)
else
CMDLINE += SUBDIRS=$(PWD)
endif

#****************************************************************************
.PHONY: $(DKMS_CONF)
all: message $(DKMS_CONF)
	$(CMDLINE)

tools: $(TOOLS)

$(TOOLS): %: $(SRC)/%.c
	$(CC) $(TOOLS_CFLAGS) $< $(TOOLS_LDFLAGS) -o $@

depend:

#****************************************************************************
# !USE_KBUILD (kernel < 2.6)
else

CC      = gcc
LD      = ld
INC     = $(KERNEL_LOCATION)/include
DEP     = .depend

MODCONF = "$(DESTDIR)/etc/modules.conf"
MODINST = "post-install $(DRV) $(BINDIR)/kcan_make_devices 2"

TARGET = $(DRV).o
CFLAGS  = -O2 -D__KERNEL__ -DMODULE -Wall -I$(INC) -I. -D$(DBG) $(KCAN_OPTS)

all: message $(TARGET) $(DKMS_CONF)

$(TARGET): $(kcan-objs)
	$(LD) -r $^ -o $@

ifeq ($(DEP),$(wildcard $(DEP)))
depend:
	makedepend -f$(DEP) -- $(CFLAGS) -- $(addsuffix .c, $(basename $(kcan-objs))) -I$(INC)
include $(DEP)
else
depend:
	touch $(DEP)
	makedepend -f$(DEP) -- $(CFLAGS) -- $(addsuffix .c, $(basename $(kcan-objs))) -I$(INC)
endif

endif
# !USE_KBUILD
#****************************************************************************

$(DKMS_CONF):
	@echo "PACKAGE_NAME=\"$(DKMS_DRV)\"" > $(DKMS_CONF)
	@echo "PACKAGE_VERSION=\"$(DKMS_VER)\"" >> $(DKMS_CONF)
	@echo "CLEAN=\"make clean\"" >> $(DKMS_CONF)
	@echo "MAKE[0]=\"make DKMS_KERNEL_DIR=\$$kernel_source_dir $(MAKE_OPTS)\"" >> $(DKMS_CONF)
	@echo "BUILT_MODULE_NAME[0]=\"$(DRV)\"" >> $(DKMS_CONF)
	@echo "BUILT_MODULE_LOCATION[0]=\".\"" >> $(DKMS_CONF)
	@echo "DEST_MODULE_LOCATION[0]=\"/updates\"" >> $(DKMS_CONF)
	@echo "AUTOINSTALL=\"yes\"" >> $(DKMS_CONF)

clean:
	rm -f $(SRC)/*.o $(SRC)/*~ $(SRC)/.*cmd *.o .*.o *.ko *~ .*cmd  $(DRV).mod.c Module.symvers modules.order .cache.mk $(DRV).mod $(TOOLS)

ifneq ($(USE_KBUILD),yes)
mrproper: clean
	rm -f $(DEP)
endif

#********** install / uninstall *********************
compat:
	@if [ -f $(DESTDIR)/etc/modprobe.d/$(DRV) ] ; then \
		mv $(DESTDIR)/etc/modprobe.d/$(DRV) $(DESTDIR)/etc/modprobe.d/$(DRV).conf ; \
		echo "Info: Moved /etc/modprobe.d/$(DRV) to /etc/modprobe.d/$(DRV).conf for compatibility reason."; \
	fi

.PHONY: install install_files install_udev install_module

ifeq ($(DESTDIR_DEV),)
DESTDIR_DEV = $(DESTDIR)
endif

ifneq ($(DESTDIR),)
DEPMOD_OPTS = --basedir $(DESTDIR)
endif
ifneq ($(KERNEL_VERSION),)
DEPMOD_OPTS += $(KERNEL_VERSION)
endif

install: install_files install_udev install_module

install_files: compat tools
	@mkdir -p $(DESTDIR)$(BINDIR)
	@cp -f kcanosdiag.sh $(DESTDIR)$(BINDIR)
	@chmod 755 $(DESTDIR)$(BINDIR)/kcanosdiag.sh
	@cp -f lskcan $(DESTDIR)$(BINDIR)
	@chmod 755 $(DESTDIR)$(BINDIR)/lskcan
	@cp -f kcan_monitor $(DESTDIR)$(BINDIR)
	@chmod 755 $(DESTDIR)$(BINDIR)/kcan_monitor
	@if [ -f kcan-settings ]; then \
		cp -f kcan-settings $(DESTDIR)$(BINDIR); \
		chmod 755 $(DESTDIR)$(BINDIR)/kcan-settings; \
	fi
	@mkdir -p $(DESTDIR_DEV)/usr/include
	@cp -f kcan.h kcanfd.h $(DESTDIR_DEV)/usr/include
	@chmod 644 $(DESTDIR_DEV)/usr/include/kcan.h $(DESTDIR_DEV)/usr/include/kcanfd.h

	@if [ ! -f $(MODCONF) ] || test -z $(shell grep -ls $(DRV) $(MODCONF)); then\
		mkdir -p $(shell dirname $(MODCONF));\
		cp $(MODCONF) $(MODCONF)~ 2> /dev/null;\
		if [ -f kcan.conf ]; then\
			cp kcan.conf $(MODCONF);\
			(echo;\
			 echo "$(MODINST)";\
			 echo;\
			) >> $(MODCONF);\
		else\
			(echo "# kcan - automatic made entry, begin --------";\
			 echo "# if required add options and remove comment";\
			 echo "$(MODINST)";\
			 echo "# kcan - automatic made entry, end ----------";\
			 echo;\
			 ) > $(MODCONF);\
		fi;\
	else\
		echo "Info: Left current '$(DRV)'-entry in $(MODCONF) untouched.";\
	fi

install_udev:
ifeq ($(UDEV),yes)
	mkdir -p $(DESTDIR)$(BINDIR)
	cp -f udev/kcan_usb_minor_check.bash $(DESTDIR)$(BINDIR)
	chmod 744 $(DESTDIR)$(BINDIR)/kcan_usb_minor_check.bash
	cp -f udev/45-kcan.rules $(DESTDIR)/etc/udev/rules.d
	@echo "Info: Copied 45-kcan.rules to $(DESTDIR)/etc/udev/rules.d."
	@for m in $(shell awk '/^blacklist/ { print $$2 }' udev/blacklist-kunhong.conf); do\
		rmmod $$m 2> /dev/null || true;\
	done
	cp -f udev/blacklist-kunhong.conf $(shell dirname $(MODCONF))
	chmod 644 $(shell dirname $(MODCONF))/blacklist-kunhong.conf
	@echo "Info: mainline drivers removed and blacklisted in"
	@echo "      $(shell dirname $(MODCONF))/blacklist-kunhong.conf"
ifeq ($(DESTDIR),)
	$(UDEVADM) control --reload-rules
endif
else
	@echo "Info: No UDEV installation found."
endif

DKMS_STDERR = /tmp/$(DKMS_DRV)-$(DKMS_VER)-dkms.stderr

install_module: $(TARGET)
	@-rmmod $(DRV) 2> /dev/null || true
	@-find $(MODULES_DIR) -name $(TARGET) -delete
ifeq ($(USE_DKMS),yes)
	@echo "- Cleaning previous DKMS state..."
	-$(DKMS_BIN) remove $(DKMS_DRV)/$(DKMS_VER) --all 2>/dev/null || true
	-rm -rf $(DESTDIR)$(DKMS_TREE)/$(DKMS_DRV)
	-rm -f $(DESTDIR)/usr/src/$(DKMS_DRV)-$(DKMS_VER)
	@echo "- Installing via DKMS..."
	ln -s $(PWD) $(DESTDIR)/usr/src/$(DKMS_DRV)-$(DKMS_VER)
	$(DKMS_BIN) add $(DKMS_DRV)/$(DKMS_VER)
	$(DKMS_BIN) install --force $(DKMS_DRV)/$(DKMS_VER) -k $(KERNEL_VERSION)
else
	@echo "- manually installing $(DRV) under $(INSTALL_LOCATION)..."
	mkdir -p $(INSTALL_LOCATION)
	cp -f $(TARGET) $(INSTALL_LOCATION)/$(TARGET)
	@echo "- Building dependencies..."
	$(DEPMOD) $(DEPMOD_OPTS)
endif

dkms_status:
	@echo "Getting dkms status for $(DKMS_DRV) $(DKMS_VER) for $(KERNEL_VERSION):" && echo
	$(DKMS_BIN) status $(DKMS_DRV)/$(DKMS_VER) -k $(KERNEL_VERSION) 2> $(DKMS_STDERR) | grep -q -e "^$(DKMS_DRV), $(DKMS_VER), $(KERNEL_VERSION), .*: installed$$" && [ $$? -eq 0 ] && echo "=> Ok" || echo "=> NOk"

.PHONY: uninstall uninstall_files uninstall_module
uninstall: uninstall_module uninstall_files

uninstall_files:
	-rm -f	$(DESTDIR_DEV)/usr/include/kcan.h \
		$(DESTDIR_DEV)/usr/include/kcanfd.h \
		$(DESTDIR)$(BINDIR)/kcanosdiag.sh \
		$(DESTDIR)$(BINDIR)/kcan_monitor \
		$(DESTDIR)$(BINDIR)/kcan-settings \
		$(DESTDIR)$(BINDIR)/lskcan \
		$(DESTDIR)$(BINDIR)/kcan_make_devices \
		$(DESTDIR)$(BINDIR)/kcanfdtst \
		$(DESTDIR)/etc/udev/rules.d/45-kcan.rules \
		$(DESTDIR)$(BINDIR)/kcan_usb_minor_check.bash \
		$(shell dirname $(MODCONF))/blacklist-kunhong.conf \
		$(MODCONF)

uninstall_module:
	-rmmod $(DRV) 2> /dev/null || true
	if [ -f $(INSTALL_LOCATION)/$(TARGET) ]; then\
		rm -f $(INSTALL_LOCATION)/$(TARGET);\
		$(DEPMOD) $(DEPMOD_OPTS);\
	fi
ifneq ($(DKMS_BIN),)
	[ ! -d $(DESTDIR)/var/lib/dkms/$(DKMS_DRV) ] || \
		$(DKMS_BIN) remove $(DKMS_DRV)/$(DKMS_VER) --all || true
endif
	-rm -f	$(DESTDIR)/usr/src/$(DKMS_DRV)-$(DKMS_VER) \
		$(DKMS_STDERR)
	-rm -rf $(DESTDIR)/var/lib/dkms/$(DKMS_DRV)

# informations during build of driver
.PHONY: message
message:
	@echo "***"
	@echo "*** Making $(DRV) driver in netdev mode (USB only)"
	@echo "***"
	@echo "*** Host OS Release=$(OS_RELEASE_ID) v$(OS_RELEASE_VER)"
	@echo "*** Host machine kernel version=$(shell uname -r)"
	@echo "*** Driver kernel version=$(KERNEL_VERSION) ($(KVERSION).$(KPATCHLEVEL).$(SUBLEVEL))"
	@echo "*** Path to kernel sources=$(KERNEL_LOCATION)"
	@echo "*** use KBUILD=$(USE_KBUILD)"
	@echo "*** use DKMS=$(USE_DKMS)"
	@echo "*** $(CC) version=$(shell $(CC) -dumpversion)"
	@echo "***"

debug_kernel_version:
	@echo "KERNEL_LOCATION=$(KERNEL_LOCATION)"
	@echo "HEADER_PATH=$(HEADER_PATH)"
	@echo "Major=$(KVERSION) Minor=$(KPATCHLEVEL) Subminor=$(SUBLEVEL)"
	@echo "KVERNUM=$(KVERNUM)"

netdev: all

install_with_dkms:
	$(MAKE) DKMS=DKMS_SUPPORT install

debug:
	$(MAKE) DBG=DEBUG

debug_usb_lite debug-usb-lite:
	$(MAKE) DBG=DEBUG_USB_LITE

debug_usb debug-usb:
	$(MAKE) DBG=DEBUG_USB

# DO NOT DELETE
