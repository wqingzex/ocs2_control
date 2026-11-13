#!/bin/bash
#
# KCAN Auto Firmware Upgrade Script
# 
# This script automatically upgrades KCAN device firmware by:
# 1. Entering bootloader mode (device becomes USB mass storage) [optional with --skip-bootloader]
# 2. Identifying the USB mass storage device
# 3. Copying firmware file to the device
# 4. Waiting for device to restart
# 5. Verifying the upgraded firmware version
#
# Usage: kcan_auto_upgrade.sh <can_interface> <firmware_file> [options]
#
# Options:
#   --skip-bootloader  Skip entering bootloader mode (device already in upgrade mode)
#
# Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd

set -euo pipefail

# Color output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW_UPDATER="${SCRIPT_DIR}/kcan_fw_tool"
PY_ENTRY="${SCRIPT_DIR}/kcan_fw_upgrade.py"
if [ ! -f "$PY_ENTRY" ]; then
    # When installed, python entry may be installed as `kcan_fw_upgrade` (no .py suffix).
    PY_ENTRY="${SCRIPT_DIR}/kcan_fw_upgrade"
fi
if [ ! -f "$PY_ENTRY" ]; then
    echo -e "${RED}[ERROR]${NC} Python updater entry not found (expected kcan_fw_upgrade.py or kcan_fw_upgrade) in $SCRIPT_DIR" >&2
    exit 1
fi
SYSTEM_FW_DIR="/usr/lib/firmware"
LOCAL_FW_DIR="${SCRIPT_DIR}/../fw_images"
MOUNT_POINT="/tmp/kcan_upgrade_mount"
TIMEOUT_WAIT_USB=5           # Wait up to 5 seconds for USB device to appear (device appears quickly ~1s)
TIMEOUT_WAIT_RESTART=60      # Wait up to 60 seconds for device to restart
TIMEOUT_CHECK_INTERVAL=1     # Check interval in seconds

# KCAN OTA USB device identification (sysfs + /dev/disk/by-id)
# idVendor=34cc, idProduct=ffff; Product: Kunhong OTA Device
USB_ID_VENDOR="34cc|395e"
USB_ID_PRODUCT="ffff"
BY_ID_VENDOR="Kunhong|Chengdu_Kunhong"
BY_ID_PRODUCT="OTA_Device"

# Global variables
CAN_INTERFACE=""
FIRMWARE_FILE=""
VERBOSE=0
SKIP_VERSION_CHECK=0
SKIP_BOOTLOADER=0

#============================================================================
# Utility Functions
#============================================================================

log_info() {
    echo -e "${BLUE}[INFO]${NC} $*"
}

log_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $*"
}

log_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $*"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $*" >&2
}

log_debug() {
    if [ "$VERBOSE" -eq 1 ]; then
        echo -e "${BLUE}[DEBUG]${NC} $*"
    fi
}

print_usage() {
    cat << EOF
KCAN Auto Firmware Upgrade Script

Usage:
  Normal mode:        $0 <can_interface> <firmware_file> [options]
                      $0 lsfw [pattern]
  Skip bootloader:    $0 <firmware_file> --skip-bootloader [can_interface] [options]

Arguments:
  can_interface      CAN interface name (e.g., can0)
                      - Required in normal mode
                      - Optional with --skip-bootloader (only needed for version check)
  firmware_file      Path to firmware file (.enc file), or a filename/glob that
                      exists under /usr/lib/firmware (e.g. KH-UCANFD-*.enc).
                      Or literal `auto` to auto-select from system/local manifest JSON [required]

Options:
  -v, --verbose              Verbose output
  -s, --skip-version         Skip version verification after upgrade
  -b, --skip-bootloader      Skip entering bootloader mode (device already in upgrade mode)
                              When used, can_interface is optional and can be specified after firmware_file
  -h, --help                 Show this help message

Examples:
  # Normal upgrade (enters bootloader automatically, requires CAN interface)
  $0 can0 test/fw_image/KH-UCANFD-G2-0x60000-X4-8.1.5-260206.enc
  
  # Auto upgrade (select newest compatible firmware from fw_images/)
  $0 can0 auto
  
  # Upgrade device already in bootloader/OTA mode (no CAN interface needed)
  $0 test/fw_image/KH-UCANFD-G2-0x60000-X4-8.1.5-260206.enc --skip-bootloader
  
  # Skip bootloader + skip version check (no CAN interface needed)
  $0 test/test/fw_image/KH-UCANFD-G2-0x60000-X4-8.1.5-260206.enc --skip-bootloader --skip-version
  
  # Skip bootloader but verify version (CAN interface needed)
  $0 test/test/fw_image/KH-UCANFD-G2-0x60000-X4-8.1.5-260206.enc --skip-bootloader can0
  
  # Verbose output
  $0 can0 test/test/fw_image/KH-UCANFD-G2-0x60000-X4-8.1.5-260206.enc -v

Device Identification (minimal deps: sysfs + by-id):
  - /dev/disk/by-id/usb-Kunhong_Kunhong_OTA_Device_*-0:0
  - or /sys/block/sd* with idVendor=34cc, idProduct=ffff

EOF
}

#============================================================================
# Device Identification Functions (sysfs + by-id, no lsblk)
#============================================================================

read_usb_ids_from_sysfs() {
    local dev_name="$1"
    local dev_path p vendor product
    [ -d "/sys/block/$dev_name" ] || return 1
    [ -L "/sys/block/$dev_name/device" ] || return 1
    dev_path=$(readlink -f "/sys/block/$dev_name/device" 2>/dev/null) || return 1
    p="$dev_path"
    while [ -n "$p" ] && [ "$p" != "/" ]; do
        if [ -f "$p/idVendor" ] && [ -f "$p/idProduct" ]; then
            vendor=$(cat "$p/idVendor" 2>/dev/null | tr -d ' \n')
            product=$(cat "$p/idProduct" 2>/dev/null | tr -d ' \n')
            echo "$vendor $product"
            return 0
        fi
        p=$(dirname "$p")
    done
    return 1
}

find_by_sysfs() {
    local b ids vendor product
    for b in /sys/block/sd*; do
        [ -d "$b" ] || continue
        b=${b##*/}
        ids=$(read_usb_ids_from_sysfs "$b" 2>/dev/null) || continue
        read -r vendor product <<< "$ids"
        if echo "$vendor" | grep -qE "^($USB_ID_VENDOR)$" && [ "$product" = "$USB_ID_PRODUCT" ]; then
            echo "$b"
            return 0
        fi
    done
    return 1
}

find_by_disk_id() {
    local by_id="/dev/disk/by-id" link target name vendor_pattern
    [ -d "$by_id" ] || return 1
    
    # Split BY_ID_VENDOR by | and check each pattern
    IFS='|' read -ra vendors <<< "$BY_ID_VENDOR"
    for vendor_pattern in "${vendors[@]}"; do
        for link in "$by_id"/usb-"${vendor_pattern}"_*"${BY_ID_PRODUCT}"*; do
            [ -L "$link" ] || continue
            name=$(basename "$link")
            [[ "$name" == *"${BY_ID_PRODUCT}"* ]] || continue
            target=$(readlink -f "$link" 2>/dev/null)
            [ -n "$target" ] || continue
            target=${target##*/}
            [[ "$target" == sd* ]] || continue
            echo "$target"
            return 0
        done
    done
    return 1
}

# Find KCAN OTA upgrade device (by-id first, then sysfs)
find_upgrade_device() {
    local dev_name=""
    log_debug "Scanning for upgrade USB device (by-id + sysfs)..." >&2
    dev_name=$(find_by_disk_id 2>/dev/null) || true
    [ -z "$dev_name" ] && dev_name=$(find_by_sysfs 2>/dev/null) || true
    if [ -n "$dev_name" ]; then
        log_success "Found upgrade device: /dev/$dev_name" >&2
        echo "/dev/$dev_name"
        return 0
    fi
    return 1
}

# Wait for USB device to appear
# Note: Device appears quickly (~1s) after entering bootloader mode
wait_for_usb_device() {
    local elapsed=0
    local device=""
    
    log_info "Waiting for USB mass storage device to appear (timeout: ${TIMEOUT_WAIT_USB}s)..." >&2
    log_debug "Device should appear quickly (~1s) after entering bootloader mode" >&2

    while [ $elapsed -lt $TIMEOUT_WAIT_USB ]; do
        # Refresh udev database (quick check)
        udevadm settle --timeout=0.5 >/dev/null 2>&1 || true
        
        device=$(find_upgrade_device)
        if [ -n "$device" ]; then
            log_success "USB device appeared: $device (after ${elapsed}s)" >&2
            echo "$device"
            return 0
        fi
        
        sleep "$TIMEOUT_CHECK_INTERVAL"
        elapsed=$((elapsed + TIMEOUT_CHECK_INTERVAL))
        
        if [ $elapsed -ge $TIMEOUT_WAIT_USB ]; then
            break
        fi
    done
    
    log_error "Timeout waiting for USB device to appear (${elapsed}s)"
    log_info "Checking dmesg for device status..."
    if command -v dmesg >/dev/null 2>&1; then
        log_info "Recent USB/Kunhong device messages:"
        sudo dmesg | tail -20 | grep -iE "kunhong|ota|usb.*34cc|usb.*ffff" || true
    fi
    return 1
}

# Mount USB device
mount_upgrade_device() {
    local device="$1"
    local retries=5
    local retry_delay=2
    local mount_output=""
    
    log_info "Mounting device $device to $MOUNT_POINT..."
    
    # Create mount point
    mkdir -p "$MOUNT_POINT"
    
    # Unmount if already mounted
    umount "$MOUNT_POINT" 2>/dev/null || true
    
    # Retry mounting (device may need time to stabilize)
    while [ $retries -gt 0 ]; do
        # Verify device exists (check both block device and by-id)
        local dev_to_mount="$device"
        if [ ! -b "$device" ] && [ ! -c "$device" ]; then
            # Device node doesn't exist, try to find via by-id
            log_debug "Device node $device not found, trying to find via by-id..."
            local by_id_link
            for link in /dev/disk/by-id/usb-Kunhong_Kunhong_OTA_Device*; do
                if [ -L "$link" ]; then
                    dev_to_mount=$(readlink -f "$link" 2>/dev/null)
                    if [ -n "$dev_to_mount" ] && [ -b "$dev_to_mount" ]; then
                        log_debug "Found device via by-id: $dev_to_mount"
                        break
                    fi
                fi
            done
            
            if [ ! -b "$dev_to_mount" ] && [ ! -c "$dev_to_mount" ]; then
                log_debug "Device not found, waiting ${retry_delay}s... ($retries attempts left)"
                sleep "$retry_delay"
                retries=$((retries - 1))
                continue
            fi
        fi
        
        # Small delay to ensure device is ready
        sleep 1
        
        # Mount device (use simple rw option)
        mount_output=$(mount -t vfat "$dev_to_mount" "$MOUNT_POINT" -o rw 2>&1)
        local mount_ret=$?
        if [ $mount_ret -eq 0 ]; then
            log_success "Device mounted successfully"
            return 0
        else
            log_debug "Mount attempt failed: $mount_output"
            if [ $retries -gt 1 ]; then
                log_debug "Retrying in ${retry_delay}s... ($((retries - 1)) attempts left)"
                sleep "$retry_delay"
            fi
        fi
        retries=$((retries - 1))
    done
    
    log_error "Failed to mount device $device after retries"
    if [ "$VERBOSE" -eq 1 ] && [ -n "$mount_output" ]; then
        echo "Last mount error: $mount_output" >&2
    fi
    return 1
}

# Unmount USB device
# Note: Device may disappear immediately after firmware copy (device reboots),
# so unmount failure is expected and not an error
unmount_upgrade_device() {
    log_info "Unmounting device..."
    
    # Check if mount point is still mounted
    if mountpoint -q "$MOUNT_POINT" 2>/dev/null; then
        if umount "$MOUNT_POINT" 2>/dev/null; then
            log_success "Device unmounted"
            rmdir "$MOUNT_POINT" 2>/dev/null || true
            return 0
        else
            log_warning "Failed to unmount (device may have disappeared/rebooted)"
            # Device may have rebooted and disappeared, try to clean up
            rmdir "$MOUNT_POINT" 2>/dev/null || true
            return 0  # Not an error, device likely rebooted
        fi
    else
        log_debug "Mount point not mounted (device may have already rebooted)"
        rmdir "$MOUNT_POINT" 2>/dev/null || true
        return 0
    fi
}

#============================================================================
# Firmware Upgrade Functions
#============================================================================

# Enter bootloader mode
# Note: KCAN device will immediately disappear and enumerate as USB disk (~1s)
enter_bootloader_mode() {
    log_info "Entering bootloader mode on $CAN_INTERFACE..."
    log_debug "Device will immediately disappear from CAN and enumerate as USB disk (~1s)"
    
    if [ ! -f "$FW_UPDATER" ]; then
        log_error "Firmware updater not found: $FW_UPDATER"
        log_error "Please build it first: cd $(dirname "$FW_UPDATER") && make"
        return 1
    fi
    
    # Use non-interactive mode by piping 'y' to confirm
    local bootloader_output
    bootloader_output=$(echo "y" | "$FW_UPDATER" "$CAN_INTERFACE" bl 2>&1)
    local bootloader_ret=$?
    
    if [ $bootloader_ret -eq 0 ] || echo "$bootloader_output" | grep -qiE "RST|BL|disappear"; then
        log_success "Bootloader command sent, device will reset..."
        log_debug "KCAN device should disappear and USB disk should appear shortly"
        return 0
    else
        log_error "Failed to enter bootloader mode"
        if [ "$VERBOSE" -eq 1 ]; then
            log_debug "Bootloader output: $bootloader_output"
        fi
        # Check dmesg for clues
        if command -v dmesg >/dev/null 2>&1; then
            log_info "Checking dmesg for device status..."
            sudo dmesg | tail -10 | grep -iE "kunhong|ota|kcan|usb.*34cc" || true
        fi
        return 1
    fi
}

# Copy firmware file to device
# Note: U盘模式只能写入，无法读出比较。拷贝固件后设备会立即重启，U盘消失。
# 如果拷贝后U盘没有消失，说明升级失败。
copy_firmware() {
    local firmware_file="$1"
    local dest_file="$MOUNT_POINT/$(basename "$firmware_file")"
    local device_before="$2"  # Device path before copy
    
    log_info "Copying firmware file..."
    log_debug "Source: $firmware_file"
    log_debug "Destination: $dest_file"
    log_debug "Device: $device_before"
    
    if [ ! -f "$firmware_file" ]; then
        log_error "Firmware file not found: $firmware_file"
        return 1
    fi
    
    # Get source file size for logging
    local src_size=$(stat -f%z "$firmware_file" 2>/dev/null || stat -c%s "$firmware_file" 2>/dev/null)
    log_info "Firmware file size: $src_size bytes"
    
    # Copy file
    if cp "$firmware_file" "$dest_file"; then
        log_success "Firmware file copied successfully"
        
        # Sync to ensure data is written
        sync
        log_debug "File system synced"
        
        # Wait for device to detect firmware and start rebooting
        # Device should reboot immediately after detecting firmware file
        log_info "Waiting for device to detect firmware and reboot..."
        
        # Check multiple times to see if device disappears
        local check_count=0
        local max_checks=5
        local device_disappeared=0
        
        while [ $check_count -lt $max_checks ]; do
            sleep 1
            check_count=$((check_count + 1))
            
            # Check if device still exists
            local device_still_exists=0
            if [ -b "$device_before" ] || [ -c "$device_before" ]; then
                device_still_exists=1
            else
                # Check via by-id
                for link in /dev/disk/by-id/usb-Kunhong_Kunhong_OTA_Device*; do
                    if [ -L "$link" ]; then
                        local target=$(readlink -f "$link" 2>/dev/null)
                        if [ "$target" = "$device_before" ]; then
                            device_still_exists=1
                            break
                        fi
                    fi
                done
            fi
            
            if [ $device_still_exists -eq 0 ]; then
                device_disappeared=1
                log_success "Device (U盘) disappeared after ${check_count}s - device is rebooting (upgrade triggered successfully)"
                break
            fi
            
            log_debug "Device still present, checking again... (${check_count}/${max_checks})"
        done
        
        if [ $device_disappeared -eq 0 ]; then
            log_error "Device (U盘) still exists after firmware copy - upgrade FAILED!"
            log_error "Expected: Device should reboot immediately and U盘 should disappear"
            log_error "Actual: U盘 still present after ${max_checks}s, device did not detect firmware or reboot"
            log_error "Possible causes:"
            log_error "  1. Firmware file format incorrect"
            log_error "  2. Firmware file not recognized by bootloader"
            log_error "  3. Device hardware issue"
            
            # Check dmesg for clues
            if command -v dmesg >/dev/null 2>&1; then
                log_info "Checking dmesg for device status..."
                sudo dmesg | tail -30 | grep -iE "kunhong|ota|usb.*34cc|usb.*ffff|error|fail" || true
            fi
            
            return 1
        fi
        
        return 0
    else
        log_error "Failed to copy firmware file"
        return 1
    fi
}

# Wait for device to restart and CAN interface to reappear
wait_for_device_restart() {
    local elapsed=0
    
    log_info "Waiting for device to restart and CAN interface to reappear..."
    log_info "This may take up to ${TIMEOUT_WAIT_RESTART} seconds..."
    log_info "Note: Device reboots immediately after firmware copy, USB disk disappears quickly"
    
    # Wait a bit for device to unmount/reboot
    # Device may have already rebooted and USB disk disappeared
    sleep 2
    
    # Unmount if still mounted (device may have already rebooted, so failure is OK)
    unmount_upgrade_device || true
    
    while [ $elapsed -lt $TIMEOUT_WAIT_RESTART ]; do
        # Check if CAN interface exists and is up
        if ip link show "$CAN_INTERFACE" >/dev/null 2>&1; then
            # Try to get version to verify device is ready
            if "$FW_UPDATER" "$CAN_INTERFACE" version >/dev/null 2>&1; then
                log_success "Device restarted and CAN interface is ready (after ${elapsed}s)"
                sleep 2  # Give it a bit more time to stabilize
                return 0
            fi
        fi
        
        sleep "$TIMEOUT_CHECK_INTERVAL"
        elapsed=$((elapsed + TIMEOUT_CHECK_INTERVAL))
        
        if [ $((elapsed % 10)) -eq 0 ]; then
            log_info "Still waiting... (${elapsed}s/${TIMEOUT_WAIT_RESTART}s)"
            # Check dmesg for device status
            if [ "$VERBOSE" -eq 1 ] && command -v dmesg >/dev/null 2>&1; then
                log_debug "Checking dmesg for device status..."
                sudo dmesg | tail -10 | grep -iE "kunhong|ota|kcan|usb.*34cc|can0" || true
            fi
        fi
    done
    
    log_warning "Timeout waiting for device restart (${elapsed}s)"
    log_info "Checking dmesg for device status..."
    if command -v dmesg >/dev/null 2>&1; then
        sudo dmesg | tail -20 | grep -iE "kunhong|ota|kcan|usb.*34cc|can0" || log_info "No relevant messages found"
    fi
    log_warning "Continuing anyway (device may still be restarting)..."
    return 0
}

# Get firmware version
get_firmware_version() {
    log_info "Getting firmware version..."
    
    if "$FW_UPDATER" "$CAN_INTERFACE" version; then
        return 0
    else
        log_error "Failed to get firmware version"
        return 1
    fi
}

#============================================================================
# Main Function
#============================================================================

main() {
    local device=""
    local step_num=1
    local total_steps=5
    
    log_info "=========================================="
    log_info "KCAN Auto Firmware Upgrade"
    log_info "=========================================="
    if [ -n "$CAN_INTERFACE" ]; then
        log_info "CAN Interface: $CAN_INTERFACE"
    else
        log_info "CAN Interface: Not specified (not needed)"
    fi
    log_info "Firmware File: $FIRMWARE_FILE"
    if [ "$SKIP_BOOTLOADER" -eq 1 ]; then
        log_info "Mode: Device already in upgrade mode (skipping bootloader entry)"
        total_steps=4
    fi
    log_info ""
    
    # Step 1: Enter bootloader mode (skip if --skip-bootloader)
    if [ "$SKIP_BOOTLOADER" -eq 0 ]; then
        log_info "Step ${step_num}/${total_steps}: Entering bootloader mode..."
        if ! enter_bootloader_mode; then
            log_error "Failed to enter bootloader mode"
            exit 1
        fi
        step_num=$((step_num + 1))
    else
        log_info "Skipping bootloader entry (device already in upgrade mode)"
    fi
    
    # Step 2: Wait for USB device to appear
    log_info ""
    log_info "Step ${step_num}/${total_steps}: Waiting for USB mass storage device..."
    device=$(wait_for_usb_device)
    if [ -z "$device" ]; then
        log_error "USB device not found"
        if [ "$SKIP_BOOTLOADER" -eq 1 ]; then
            log_error "Make sure device is already in upgrade/bootloader mode"
        fi
        exit 1
    fi
    step_num=$((step_num + 1))
    
    # Step 3: Mount device and copy firmware
    log_info ""
    log_info "Step ${step_num}/${total_steps}: Mounting device and copying firmware..."
    if ! mount_upgrade_device "$device"; then
        log_error "Failed to mount device"
        exit 1
    fi
    
    # Cleanup on exit
    trap 'unmount_upgrade_device || true' EXIT
    
    # Copy firmware - device should reboot immediately after copy
    # Note: U盘模式只能写入，无法读出。拷贝后设备会立即重启，U盘消失。
    # 如果拷贝后U盘没有消失，说明升级失败。
    if ! copy_firmware "$FIRMWARE_FILE" "$device"; then
        log_error "Failed to copy firmware or device did not reboot"
        unmount_upgrade_device || true
        exit 1
    fi
    step_num=$((step_num + 1))
    
    # Device should have rebooted and U盘 disappeared
    # Try to unmount (may fail if device already rebooted - this is OK)
    log_info ""
    log_info "Step ${step_num}/${total_steps}: Device should have rebooted (U盘 disappeared)..."
    log_info "Note: Device reboots immediately after firmware copy, U盘 disappears"
    unmount_upgrade_device || true
    step_num=$((step_num + 1))
    
    # Step 4: Wait for device restart
    log_info ""
    log_info "Step ${step_num}/${total_steps}: Waiting for device to restart..."
    if [ -n "$CAN_INTERFACE" ]; then
        wait_for_device_restart
    else
        log_warning "CAN interface not available, skipping device restart check"
        log_info "Please manually verify device has restarted"
    fi
    
    # Step 5: Verify firmware version
    if [ "$SKIP_VERSION_CHECK" -eq 0 ]; then
        if [ -n "$CAN_INTERFACE" ]; then
            log_info ""
            log_info "Verifying firmware version..."
            get_firmware_version
        else
            log_warning "CAN interface not specified, skipping version verification"
        fi
    else
        log_warning "Skipping version verification (--skip-version)"
    fi
    
    log_info ""
    log_success "=========================================="
    log_success "Firmware upgrade completed successfully!"
    log_success "=========================================="
}

#============================================================================
# Parse Arguments
#============================================================================

# First pass: parse all options to determine if --skip-bootloader is used
SKIP_BOOTLOADER=0
VERBOSE=0
SKIP_VERSION_CHECK=0
POSITIONAL_ARGS=()

while [[ $# -gt 0 ]]; do
    case $1 in
        -v|--verbose)
            VERBOSE=1
            shift
            ;;
        -s|--skip-version)
            SKIP_VERSION_CHECK=1
            shift
            ;;
        -b|--skip-bootloader)
            SKIP_BOOTLOADER=1
            shift
            ;;
        -h|--help)
            print_usage
            exit 0
            ;;
        -*)
            log_error "Unknown option: $1"
            print_usage
            exit 1
            ;;
        *)
            # Collect positional arguments
            POSITIONAL_ARGS+=("$1")
            shift
            ;;
    esac
done

# lsfw: query installed firmware (no device interaction required)
if [ ${#POSITIONAL_ARGS[@]} -ge 1 ]; then
    cmd_lower="${POSITIONAL_ARGS[0],,}"
    if [[ "$cmd_lower" == "lsfw" || "$cmd_lower" == "ls-fw" ]]; then
        pattern="${POSITIONAL_ARGS[1]-}"
        python_positional=("lsfw")
        if [ -n "$pattern" ]; then
            python_positional+=("$pattern")
        fi
        python_flags=()
        if [ "$VERBOSE" -eq 1 ]; then
            python_flags+=("-v")
        fi
        python3 "$PY_ENTRY" "${python_positional[@]}" "${python_flags[@]}"
        exit $?
    fi
fi

# Parse positional arguments based on --skip-bootloader flag
if [ ${#POSITIONAL_ARGS[@]} -lt 1 ]; then
    log_error "Firmware file not specified"
    print_usage
    exit 1
fi

if [ "$SKIP_BOOTLOADER" -eq 1 ]; then
    # With --skip-bootloader: first positional arg is firmware file
    FIRMWARE_FILE="${POSITIONAL_ARGS[0]}"
    # Optional second arg: CAN interface (for version check)
    if [ ${#POSITIONAL_ARGS[@]} -ge 2 ]; then
        CAN_INTERFACE="${POSITIONAL_ARGS[1]}"
    fi
else
    # Normal mode: first positional arg is CAN interface, second is firmware file
    if [ ${#POSITIONAL_ARGS[@]} -lt 2 ]; then
        log_error "CAN interface and firmware file required when not using --skip-bootloader"
        print_usage
        exit 1
    fi
    CAN_INTERFACE="${POSITIONAL_ARGS[0]}"
    FIRMWARE_FILE="${POSITIONAL_ARGS[1]}"
fi

# Auto mode: allow firmware_file to be literal "auto"
AUTO_MODE=0
if [ "${FIRMWARE_FILE,,}" = "auto" ]; then
    AUTO_MODE=1
fi

# Validate inputs
if [ -z "$FIRMWARE_FILE" ]; then
    log_error "Firmware file not specified"
    print_usage
    exit 1
fi

# CAN interface validation
if [ "$SKIP_BOOTLOADER" -eq 0 ]; then
    # Normal mode: CAN interface required
    if [ -z "$CAN_INTERFACE" ]; then
        log_error "CAN interface required when not using --skip-bootloader"
        print_usage
        exit 1
    fi
elif [ "$SKIP_VERSION_CHECK" -eq 0 ] && [ -z "$CAN_INTERFACE" ]; then
    # Skipping bootloader but version check enabled - warn if no CAN interface
    log_warning "CAN interface not specified. Version check requires CAN interface."
    log_warning "Use --skip-version to skip version verification, or specify CAN interface."
    SKIP_VERSION_CHECK=1
fi

if [ "$AUTO_MODE" -eq 0 ] && [ ! -f "$FIRMWARE_FILE" ]; then
    # Friendly mode:
    # - user passes a filename/glob (e.g. KH-UCANFD-*.enc or KH-UCANFD-*8.5*.enc)
    # - we resolve it under system/local firmware dirs before copying.
    token="$FIRMWARE_FILE"
    token="${token//\*\*/\*}"  # treat '**' as '*'

    # If user passed something that looks like a path, keep strict behavior.
    if [[ "$token" == *"/"* ]]; then
        log_error "Firmware file not found: $FIRMWARE_FILE"
        exit 1
    fi

    resolved_path="$(
        python3 "$PY_ENTRY" resolvefw "$token" 2>&1
    )" || {
        log_error "resolvefw failed: $resolved_path"
        exit 1
    }

    if [ -z "$resolved_path" ] || [ ! -f "$resolved_path" ]; then
        log_error "Resolved firmware file invalid: $resolved_path"
        exit 1
    fi

    FIRMWARE_FILE="$resolved_path"
fi

# Check if running as root (needed for mount operations)
if [ "$EUID" -ne 0 ]; then
    log_error "This script must be run as root (for mount operations)"
    log_error "Please run: sudo $0 $*"
    exit 1
fi

# In auto mode delegate selection to Python script (reads kcan_fw_manifest.json)
if [ "$AUTO_MODE" -eq 1 ]; then
    log_info "Auto mode: selecting firmware automatically..."
    # Important: "$@" has been consumed by option parsing above, so rebuild args
    # from parsed variables (CAN_INTERFACE / FIRMWARE_FILE / flags).
    python_positional=()
    python_flags=()

    if [ "$SKIP_BOOTLOADER" -eq 1 ]; then
        # Python expects: <firmware_token> [can_interface]
        python_positional+=("auto")
        if [ -n "$CAN_INTERFACE" ]; then
            python_positional+=("$CAN_INTERFACE")
        fi
        python_flags+=("--skip-bootloader")
    else
        # Python expects: <can_interface> <firmware_token>
        python_positional+=("$CAN_INTERFACE")
        python_positional+=("auto")
    fi

    if [ "$VERBOSE" -eq 1 ]; then
        python_flags+=("-v")
    fi
    if [ "$SKIP_VERSION_CHECK" -eq 1 ]; then
        python_flags+=("-s")
    fi

    python3 "$PY_ENTRY" "${python_positional[@]}" "${python_flags[@]}"
    exit $?
fi

# Run main function
main
