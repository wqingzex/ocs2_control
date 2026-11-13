#!/bin/bash

# KH-UCANFD Cross-compilation Configuration File
# Please modify the following parameters according to your actual situation

# Target architecture (arm64, arm, x86_64, etc.)
export ARCH=arm64

# Cross-compilation toolchain prefix
# Example paths:
# ARM64: /path/to/gcc-arm-aarch64-linux-gnu/bin/aarch64-linux-gnu-
# ARM32: /path/to/gcc-arm-arm-linux-gnueabihf/bin/arm-linux-gnueabihf-
export CROSS_COMPILE=/path/to/your/toolchain/bin/aarch64-linux-gnu-

# Kernel source path
# Note: Kernel source needs to be configured and prepared first
export KERNEL_LOCATION=/path/to/your/kernel-source/

# Additional compilation parameters (optional)
export EXTRA_CFLAGS=""

# Common cross-compilation toolchain path examples:
# Ubuntu/Debian: /usr/bin/aarch64-linux-gnu-
# Linaro: /opt/linaro/gcc-linaro-*/bin/aarch64-linux-gnu-
# ARM Development Studio: /arm/developmentstudio-*/sw/gcc/bin/aarch64-none-elf-
# Yocto: /path/to/build/tmp/work/*/linux-*/recipe-sysroot-native/usr/bin/aarch64-poky-linux/aarch64-poky-linux-

# Kernel source acquisition methods:
# 1. Copy from target device: /lib/modules/$(uname -r)/build
# 2. Download corresponding version source: https://www.kernel.org/pub/linux/kernel/
# 3. Use kernel source provided by chip manufacturer
