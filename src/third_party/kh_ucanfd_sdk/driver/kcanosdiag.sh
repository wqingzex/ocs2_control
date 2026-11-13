#!/bin/sh
#
# kcanosdiag.sh
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
# Author: Kunhong-Elec driver team <gene@cdkhdz.com>
# Contact: <support@cdkhdz.com>
# Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd <www.kunhong-elec.com>
#
# History:
#
# 1.0.6 - Kunhong-Elec driver team <gene@cdkhdz.com>
# - add cat blacklist-kunhong.conf
#
VERSION=1.0.0

CAT=/bin/cat
DATE=/bin/date
DMESG=/usr/bin/dmesg
FIND=/usr/bin/find
GREP=/bin/grep
LSCPU=/bin/lscpu
LSB_RELEASE=/etc/lsb-release
LSMOD=/bin/lsmod
LSKCAN=/usr/local/bin/lskcan
LSPCI=/bin/lspci
LSUSB=/bin/lsusb
MODINFO=/usr/sbin/modinfo
MODPROBE=/sbin/modprobe
PS=/bin/ps
KCANINFO=/usr/local/bin/kcaninfo
RMMOD=/sbin/rmmod
UNAME=/bin/uname

SYSFS_MODPARAM=/sys/module/kcan/parameters
SYSFS_KCAN=/sys/class/kcan

#LOG=/tmp/kcanosdiag-`$DATE +"%Y%m%d-%H%M%S"`.log
LOG=./kcanosdiag-$VERSION-`$DATE +"%Y%m%d_%H%M%S"`.log

[ -z "$EUID" ] && EUID=`id -u`
step=0

run()
{
	echo --------------------------------------------------------------------------------
	step=`expr $step + 1`
	echo "STEP $step # $*"
	if test ! -x $1; then
		bin=`basename $1`
		echo "Warning: $1 not found! Trying with $bin only:"
		cmd=`echo $* | sed -e "s#$1#$bin#"`
	else
		cmd="$*"
	fi

	eval $cmd
	return $?

	if test -x $1; then
		#eval $*
		eval $cmd
		return $?
	fi
	echo "'$1' not found: can't run '$*'"
	return 2
}

#-- main

echo "$0 v$VERSION"
[ $EUID -ne 0 ] && echo "Error: you must be root" && exit 1

#-- Redirect stdout/stderr to $LOG:
# Save stdout to fd=3
exec 3<&1

# Open stdout as $LOG file for write.
exec 1>$LOG

# Redirect standard error to standard output
exec 2>&1

#-- Start
echo "$0 v$VERSION"
run $DATE +\"%F %R:%S %s\"
run $CAT /proc/uptime
run $CAT $LSB_RELEASE
run $UNAME -a
run $CAT /proc/cmdline
run $LSCPU
run $CAT /proc/interrupts
run $PS -ef

#-- USB specific
run $LSUSB
run $LSUSB -v -d 0c72:

#-- kcan specific
run $FIND /lib/modules -name "kcan.ko"
for p in `$FIND /lib/modules -name "kcan.ko"`; do
	run $MODINFO $p
done
run $LSMOD "|" $GREP -e "^kunhong"
run $LSMOD "|" $GREP -e "^kcan"
run $RMMOD kcan
run $LSMOD "|" $GREP -e "^kcan"
run $DMESG

run $CAT /etc/modprobe.d/kcan.conf
run $CAT /etc/modprobe.d/blacklist-kunhong.conf
run $MODPROBE kcan
run $DMESG "|" $GREP kcan
run $CAT /proc/kcan
run $LSKCAN -t -T -i
run $CAT $SYSFS_KCAN/kcan*/adapter_name
run $CAT $SYSFS_KCAN/kcan*/adapter_partnum
run $CAT $SYSFS_MODPARAM/fdusemsi
run $CAT $SYSFS_MODPARAM/usemsi
run $CAT $SYSFS_MODPARAM/rxqsize
run $CAT $SYSFS_MODPARAM/txqsize
run $CAT $SYSFS_MODPARAM/drvclkref
run $CAT $SYSFS_MODPARAM/deftsmode
run $CAT $SYSFS_MODPARAM/fast_fwd

#-- kcanbasic
run $KCANINFO --all

#-- End: restore stdout from 3 and close 3
exec 1<&3 3<&-

echo "Done."
if test ! -f $LOG; then
	echo "Error: $LOG not found!"
	exit 2
fi

echo "Please send $LOG to <vx:sysmax-cn/email:support@cdkhdz.com>"
exit 0
