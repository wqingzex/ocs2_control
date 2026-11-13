#!/usr/bin/env python3
#
# KCAN Auto Firmware Upgrade Script (Python Version)
# 
# This script automatically upgrades KCAN device firmware by:
# 1. Entering bootloader mode (device becomes USB mass storage)
# 2. Identifying the USB mass storage device
# 3. Copying firmware file to the device
# 4. Waiting for device to restart
# 5. Verifying the upgraded firmware version
#
# Usage: kcan_auto_upgrade.py <can_interface> <firmware_file> [options]
#
# Copyright (C) 2022-2026 Chengdu Kunhong Electronic Technology Co., Ltd

import os
import sys
import time
import subprocess
import shutil
import argparse
import stat
import re
import json
from pathlib import Path

# Color output
class Colors:
    RED = '\033[0;31m'
    GREEN = '\033[0;32m'
    YELLOW = '\033[1;33m'
    BLUE = '\033[0;34m'
    NC = '\033[0m'  # No Color

# Configuration
SCRIPT_DIR = Path(__file__).parent.absolute()
FW_UPDATER = SCRIPT_DIR / "kcan_fw_tool"
# Prefer system-installed firmware resources when available.
# In installed mode:
#   - firmware images: /usr/lib/firmware/*.enc
#   - manifest:        /usr/lib/firmware/kcan_fw_manifest.json
FW_IMAGE_DIR_LOCAL = (SCRIPT_DIR.parent / "fw_images").resolve()
FW_IMAGE_DIR_SYS = Path("/usr/lib/firmware")
FW_MANIFEST_PATH_SYS = (FW_IMAGE_DIR_SYS / "kcan_fw_manifest.json").resolve()
FW_MANIFEST_PATH_LOCAL = (FW_IMAGE_DIR_LOCAL / "fw_manifest.json").resolve()

if FW_MANIFEST_PATH_SYS.exists():
    FW_IMAGE_DIR = FW_IMAGE_DIR_SYS
    FW_MANIFEST_PATH_DEFAULT = FW_MANIFEST_PATH_SYS
else:
    FW_IMAGE_DIR = FW_IMAGE_DIR_LOCAL
    FW_MANIFEST_PATH_DEFAULT = FW_MANIFEST_PATH_LOCAL
MOUNT_POINT = Path("/tmp/kcan_upgrade_mount")
TIMEOUT_WAIT_USB = 30          # Wait up to 30 seconds for USB device to appear
TIMEOUT_WAIT_RESTART = 60      # Wait up to 60 seconds for device to restart
TIMEOUT_CHECK_INTERVAL = 2     # Check interval in seconds

# KCAN OTA USB device identification (sysfs + /dev/disk/by-id)
USB_ID_VENDOR = "34cc|395e"
USB_ID_PRODUCT = "ffff"
BY_ID_VENDOR = "Kunhong|Chendu_Kunhong"
BY_ID_PRODUCT = "OTA_Device"

def log_info(msg):
    print(f"{Colors.BLUE}[INFO]{Colors.NC} {msg}")

def log_success(msg):
    print(f"{Colors.GREEN}[SUCCESS]{Colors.NC} {msg}")

def log_warning(msg):
    print(f"{Colors.YELLOW}[WARNING]{Colors.NC} {msg}")

def log_error(msg):
    print(f"{Colors.RED}[ERROR]{Colors.NC} {msg}", file=sys.stderr)

def log_debug(msg, verbose=False):
    if verbose:
        print(f"{Colors.BLUE}[DEBUG]{Colors.NC} {msg}")

def run_cmd(cmd, check=True, capture_output=True, verbose=False):
    """Run shell command and return result"""
    log_debug(f"Running: {' '.join(cmd) if isinstance(cmd, list) else cmd}", verbose)
    try:
        result = subprocess.run(
            cmd if isinstance(cmd, list) else cmd.split(),
            check=check,
            capture_output=capture_output,
            text=True,
            timeout=30
        )
        return result.stdout.strip() if capture_output else None
    except subprocess.CalledProcessError as e:
        log_error(f"Command failed: {e}")
        if e.stdout:
            log_debug(f"stdout: {e.stdout}", verbose)
        if e.stderr:
            log_debug(f"stderr: {e.stderr}", verbose)
        raise
    except subprocess.TimeoutExpired:
        log_error("Command timed out")
        raise

def _read_usb_ids_from_sysfs(dev_name):
    """Walk up from /sys/block/sdX/device to find idVendor, idProduct."""
    device_path = Path(f"/sys/block/{dev_name}/device")
    if not device_path.exists() or not device_path.is_symlink():
        return None
    try:
        real = device_path.resolve()
        p = real
        while p != Path("/") and p.parts:
            v = p / "idVendor"
            pid = p / "idProduct"
            if v.exists() and pid.exists():
                vendor = v.read_text().strip()
                product = pid.read_text().strip()
                return (vendor, product)
            p = p.parent
    except Exception:
        pass
    return None

def _find_by_disk_id():
    """Find device by /dev/disk/by-id/usb-Kunhong_Kunhong_OTA_Device_*."""
    by_id = Path("/dev/disk/by-id")
    if not by_id.is_dir():
        return None
    # Split BY_ID_VENDOR by | and check each pattern
    for vendor_pattern in BY_ID_VENDOR.split('|'):
        for link in by_id.glob(f"usb-{vendor_pattern}*"):
            if not link.is_symlink():
                continue
            if BY_ID_PRODUCT not in link.name:
                continue
            try:
                target = link.resolve()
                name = target.name
                if name.startswith("sd"):
                    return name
            except Exception:
                pass
    return None

def _find_by_sysfs():
    """Find device by scanning /sys/block/sd* for idVendor=34cc|395e, idProduct=ffff."""
    for b in Path("/sys/block").glob("sd*"):
        if not b.is_dir():
            continue
        dev_name = b.name
        ids = _read_usb_ids_from_sysfs(dev_name)
        if ids and re.match(f"^({USB_ID_VENDOR})$", ids[0]) and ids[1] == USB_ID_PRODUCT:
            return dev_name
    return None

def find_upgrade_device(verbose=False):
    """Find KCAN OTA upgrade device (by-id first, then sysfs)."""
    log_debug("Scanning for upgrade USB device (by-id + sysfs)...", verbose)
    dev_name = _find_by_disk_id()
    if not dev_name:
        dev_name = _find_by_sysfs()
    if dev_name:
        device = Path(f"/dev/{dev_name}")
        log_success(f"Found upgrade device: {device}")
        return device
    return None

def wait_for_usb_device(verbose=False):
    """Wait for USB device to appear"""
    log_info(f"Waiting for USB mass storage device to appear (timeout: {TIMEOUT_WAIT_USB}s)...")
    
    # Baseline snapshot: used for "fallback" detection when vendor/product matching fails.
    # This prevents the upgrade flow from getting stuck even if OTA USB IDs differ.
    baseline_sd = set()
    try:
        for b in Path("/sys/block").glob("sd*"):
            if b.is_dir():
                baseline_sd.add(b.name)
    except Exception:
        pass

    elapsed = 0
    while elapsed < TIMEOUT_WAIT_USB:
        # Refresh udev database
        try:
            run_cmd(["udevadm", "settle", "--timeout=1"], check=False, verbose=False)
        except:
            pass
        
        device = find_upgrade_device(verbose)
        if device and device.exists():
            log_success(f"USB device appeared: {device}")
            return device

        # Fallback: if no known OTA device matched, try to pick the newly appeared sdX.
        try:
            current_sd = set()
            for b in Path("/sys/block").glob("sd*"):
                if b.is_dir():
                    current_sd.add(b.name)
            new_sd = sorted(current_sd - baseline_sd)
            if new_sd:
                # Try prefer known OTA match among new devices (may still fail due to sysfs quirks).
                for dev_name in new_sd:
                    ids = _read_usb_ids_from_sysfs(dev_name)
                    if ids and re.match(f"^({USB_ID_VENDOR})$", ids[0]) and ids[1] == USB_ID_PRODUCT:
                        device = Path(f"/dev/{dev_name}")
                        log_warning(f"Fallback matched OTA device by ids: {device}")
                        return device
                # Otherwise just take the first newly appeared device.
                dev_name = new_sd[0]
                device = Path(f"/dev/{dev_name}")
                log_warning(f"No OTA device matched by ids; using newly appeared device as fallback: {device}")
                return device
        except Exception:
            pass
        
        time.sleep(1)
        elapsed += 1
        
        if elapsed % 5 == 0:
            log_info(f"Still waiting... ({elapsed}s/{TIMEOUT_WAIT_USB}s)")
    
    log_error("Timeout waiting for USB device to appear")
    return None

def mount_upgrade_device(device, verbose=False):
    """Mount USB device"""
    log_info(f"Mounting device {device} to {MOUNT_POINT}...")
    
    # Create mount point
    MOUNT_POINT.mkdir(parents=True, exist_ok=True)
    
    # Unmount if already mounted
    try:
        run_cmd(["umount", str(MOUNT_POINT)], check=False, verbose=False)
    except:
        pass
    
    # Mount device
    try:
        uid = os.getuid()
        gid = os.getgid()
        run_cmd([
            "mount", "-t", "vfat", str(device), str(MOUNT_POINT),
            "-o", f"rw,uid={uid},gid={gid}"
        ], verbose=verbose)
        log_success("Device mounted successfully")
        return True
    except Exception as e:
        log_error(f"Failed to mount device {device}: {e}")
        return False

def unmount_upgrade_device(verbose=False):
    """Unmount USB device"""
    log_info("Unmounting device...")
    
    try:
        run_cmd(["umount", str(MOUNT_POINT)], check=False, verbose=verbose)
        log_success("Device unmounted")
        try:
            MOUNT_POINT.rmdir()
        except:
            pass
        return True
    except Exception as e:
        log_warning(f"Failed to unmount (may already be unmounted): {e}")
        return False

def enter_bootloader_mode(can_interface, verbose=False):
    """Enter bootloader mode"""
    log_info(f"Entering bootloader mode on {can_interface}...")
    
    if not FW_UPDATER.exists():
        log_error(f"Firmware updater not found: {FW_UPDATER}")
        log_error(f"Please build it first: cd {FW_UPDATER.parent} && make")
        return False
    
    try:
        # Use non-interactive mode by passing -y to skip confirmation.
        # This matches: `kcan_fw_tool can0 bl -y`
        process = subprocess.Popen(
            [str(FW_UPDATER), "-y", can_interface, "bl"],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        stdout, stderr = process.communicate(timeout=10)

        combined = (stdout or "") + "\n" + (stderr or "")
        lc = combined.lower()
        if process.returncode == 0 or "bl+rst" in lc or "bootloader" in lc or "reset" in lc:
            log_success("Bootloader command sent, device should reset...")
            return True

        log_error("Bootloader command failed to trigger reset.")
        if verbose:
            log_error(f"stdout: {stdout}")
            log_error(f"stderr: {stderr}")
        else:
            log_debug(f"stdout: {stdout}", verbose)
            log_debug(f"stderr: {stderr}", verbose)
        return False
    except Exception as e:
        log_error(f"Failed to enter bootloader mode: {e}")
        return False

def copy_firmware(firmware_file, verbose=False):
    """Copy firmware file to device"""
    firmware_path = Path(firmware_file)
    dest_file = MOUNT_POINT / firmware_path.name
    
    log_info("Copying firmware file...")
    log_debug(f"Source: {firmware_path}", verbose)
    log_debug(f"Destination: {dest_file}", verbose)
    
    if not firmware_path.exists():
        log_error(f"Firmware file not found: {firmware_path}")
        return False
    
    try:
        # Copy file
        shutil.copy2(firmware_path, dest_file)
        log_success("Firmware file copied successfully")
        
        # Sync to ensure data is written
        run_cmd(["sync"], verbose=False)
        log_debug("File system synced", verbose)

        # OTA upgrade media is write-only from the device point of view.
        # The device may reboot immediately, so the destination file may disappear.
        # Do not attempt to stat/verify dest_file.
        return True
    except Exception as e:
        log_error(f"Failed to copy firmware file: {e}")
        return False

def wait_for_device_restart(can_interface, verbose=False):
    """Wait for device to restart and CAN interface to reappear"""
    log_info("Waiting for device to restart and CAN interface to reappear...")
    log_info(f"This may take up to {TIMEOUT_WAIT_RESTART} seconds...")
    
    # Wait a bit for device to unmount
    time.sleep(3)
    
    # Unmount if still mounted
    unmount_upgrade_device(verbose)
    
    elapsed = 0
    while elapsed < TIMEOUT_WAIT_RESTART:
        # Check if CAN interface exists and is up
        try:
            run_cmd(["ip", "link", "show", can_interface], check=False, verbose=False)
            
            # Try to get version to verify device is ready
            result = run_cmd([str(FW_UPDATER), can_interface, "version"], check=False, verbose=False)
            if result and "Version:" in result:
                log_success("Device restarted and CAN interface is ready")
                time.sleep(2)  # Give it a bit more time to stabilize
                return True
        except:
            pass
        
        time.sleep(TIMEOUT_CHECK_INTERVAL)
        elapsed += TIMEOUT_CHECK_INTERVAL
        
        if elapsed % 10 == 0:
            log_info(f"Still waiting... ({elapsed}s/{TIMEOUT_WAIT_RESTART}s)")
    
    log_warning("Timeout waiting for device restart, but continuing...")
    return True

def get_firmware_version(can_interface, verbose=False):
    """Get firmware version"""
    log_info("Getting firmware version...")
    
    try:
        output = run_cmd([str(FW_UPDATER), can_interface, "version"], verbose=verbose)
        print(output)
        return True
    except Exception as e:
        log_error(f"Failed to get firmware version: {e}")
        return False

def _parse_version_3(s):
    parts = s.strip().split(".")
    if len(parts) != 3:
        raise ValueError(f"Invalid version3: {s}")
    return tuple(int(p) for p in parts)

def _parse_version_4(s):
    parts = s.strip().split(".")
    if len(parts) != 4:
        raise ValueError(f"Invalid version4: {s}")
    return tuple(int(p) for p in parts)

def _parse_kcan_fw_tool_versions(version_output):
    """
    Parse `kcan_fw_tool <if> version` output.
    Expected output sections:
      === Firmware Version ===
        Version: X.Y.Z
      === BL Version ===
        Version: A.B.C.D
    """
    fw_m = re.search(
        r"===\s*Firmware Version\s*===.*?Version:\s*([0-9]+)\.([0-9]+)\.([0-9]+)",
        version_output,
        flags=re.S,
    )
    bl_m = re.search(
        r"===\s*BL Version\s*===.*?Version:\s*([0-9]+)\.([0-9]+)\.([0-9]+)\.([0-9]+)",
        version_output,
        flags=re.S,
    )
    if not fw_m:
        raise ValueError("Failed to parse firmware version from kcan_fw_tool output")

    fw_ver = (int(fw_m.group(1)), int(fw_m.group(2)), int(fw_m.group(3)))
    if bl_m:
        bl_ver = (int(bl_m.group(1)), int(bl_m.group(2)), int(bl_m.group(3)), int(bl_m.group(4)))
        return fw_ver, bl_ver

    # Boot version is optional (older firmware may not implement GET_BOOT_VER).
    return fw_ver, None

def query_current_versions(can_interface, verbose=False):
    """Return (fw_ver_3_tuple, bl_ver_4_tuple)."""
    out = run_cmd([str(FW_UPDATER), can_interface, "version"], verbose=verbose)
    return _parse_kcan_fw_tool_versions(out)

def get_device_channels_from_can_interface(can_interface):
    """
    Infer hardware type (channel count) from USB idProduct.
    Mapping (from driver):
      0x0012 -> FD (1ch)
      0x0011 -> FDPRO (2ch)
      0x0020 -> KCAN_X4 (4ch)
      0x0014 -> X6 (6ch, hidden / not auto-upgraded)
    """
    iface_path = Path(f"/sys/class/net/{can_interface}/device")
    if not iface_path.exists():
        raise RuntimeError(f"Cannot find sysfs path for {can_interface}: {iface_path}")

    # Known product IDs -> channels
    product_to_channels = {
        0x0012: 1,
        0x0011: 2,
        0x0020: 4,
        0x0014: 6,
    }

    try:
        cur = iface_path.resolve()
        while True:
            id_vendor = cur / "idVendor"
            id_product = cur / "idProduct"
            if id_vendor.exists() and id_product.exists():
                pid_hex = id_product.read_text().strip()
                pid = int(pid_hex, 16)
                return product_to_channels.get(pid, None)
            if cur == cur.parent:
                break
            cur = cur.parent
    except Exception as e:
        raise RuntimeError(f"Failed to detect USB product id for {can_interface}: {e}")

    return None

def _boot_compatible_ok(entry, current_bl_ver):
    boot_any = entry.get("boot_compatible", None)
    if boot_any is None or boot_any == "any":
        return True
    if current_bl_ver is None:
        # 设备当前固件不支持读取 boot 版本（例如 8.4.1）。
        # 自动升级场景宁可“放行”也不要因为无法校验而阻断升级。
        return True
    if boot_any == "exact":
        bl_list = entry.get("boot_compatible_list", [])
        return ".".join(map(str, current_bl_ver)) in bl_list

    # List form: {"boot_compatible_list": [...]}
    if "boot_compatible_list" in entry:
        return ".".join(map(str, current_bl_ver)) in entry.get("boot_compatible_list", [])

    # Range form: min/max
    bmin = entry.get("boot_compatible_min", None)
    bmax = entry.get("boot_compatible_max", None)
    if bmin is not None and bmax is not None:
        min_t = _parse_version_4(bmin)
        max_t = _parse_version_4(bmax)
        return min_t <= current_bl_ver <= max_t

    # Unknown schema: be conservative (do not block).
    return True

def select_firmware_from_manifest(can_interface, manifest_path, verbose=False):
    """
    Select the newest compatible firmware from manifest.
    Returns Path to selected firmware file, or None if no upgrade needed.
    """
    if not Path(manifest_path).exists():
        raise FileNotFoundError(f"Manifest not found: {manifest_path}")

    with open(manifest_path, "r", encoding="utf-8") as f:
        manifest = json.load(f)

    device_channels = get_device_channels_from_can_interface(can_interface)
    if device_channels is None:
        raise RuntimeError(f"Unsupported/unknown device USB product for {can_interface}")
    if device_channels == 6:
        raise RuntimeError("X6 device auto-upgrade is not supported (hidden mode)")

    current_fw_ver, current_bl_ver = query_current_versions(can_interface, verbose=verbose)

    def _semver_cmp_key(vt):
        return vt[0], vt[1], vt[2]

    candidates = []
    for img in manifest.get("images", []):
        img_channels = img.get("device_channels", None)
        if img_channels is None:
            continue
        # Compatibility rule:
        # - If firmware requires more channels than hardware provides, skip.
        # - Otherwise allow (e.g. KCAN_X4(4ch) can run FDPRO(2ch) firmware).
        if img_channels > device_channels:
            continue
        if device_channels % img_channels != 0:
            continue

        if not _boot_compatible_ok(img, current_bl_ver):
            continue

        fw_version_str = img.get("fw_version", None)
        filename = img.get("filename", None)
        if not fw_version_str or not filename:
            continue

        fw_ver_t = _parse_version_3(fw_version_str)
        if fw_ver_t <= current_fw_ver:
            continue

        firmware_path = Path(FW_IMAGE_DIR) / filename
        if not firmware_path.exists():
            continue

        candidates.append((fw_ver_t, firmware_path))

    if not candidates:
        return None

    # Choose the newest FW version (max tuple)
    candidates.sort(key=lambda x: _semver_cmp_key(x[0]))
    chosen_fw_ver, chosen_path = candidates[-1]

    return chosen_path

def list_installed_firmware(pattern=None, manifest_path=str(FW_MANIFEST_PATH_DEFAULT), verbose=False):
    """List installed firmware images under FW_IMAGE_DIR with metadata from manifest."""
    search_dir = Path(FW_IMAGE_DIR)
    if not search_dir.exists():
        log_warning(f"Firmware directory not found: {search_dir}")
        return 0

    manifest = {}
    by_filename = {}
    manifest_file = Path(manifest_path)
    if manifest_file.exists():
        with open(manifest_file, "r", encoding="utf-8") as f:
            manifest = json.load(f)
        for img in manifest.get("images", []):
            fn = img.get("filename", None)
            if fn:
                by_filename[fn] = img
    else:
        log_warning(f"Manifest not found: {manifest_file}")

    # Collect candidates
    candidates = []
    if pattern:
        token = str(pattern)
        # Treat '**' as '*' for single-directory filename matching.
        if "/" not in token:
            token = token.replace("**", "*")
        candidates = sorted(search_dir.glob(token))
    else:
        candidates = sorted(search_dir.glob("*.enc"))

    print(f"Installed firmware (dir: {search_dir})")
    if not candidates:
        print("  (no .enc firmware found)")
        return 0

    for p in candidates:
        fn = p.name
        entry = by_filename.get(fn, None)
        if entry:
            fw_ver = entry.get("fw_version", "unknown")
            dev_ch = entry.get("device_channels", "unknown")
            boot_compat = entry.get("boot_compatible", "unknown")
            print(f"- {fn}")
            print(f"  fw_version: {fw_ver}")
            print(f"  device_channels: {dev_ch}")
            print(f"  boot_compatible: {boot_compat}")
        else:
            print(f"- {fn}")
            print("  (no manifest entry found)")
    return 0

def resolve_firmware_token(firmware_token, manifest_path=str(FW_MANIFEST_PATH_DEFAULT), verbose=False):
    """Resolve a user token to an actual firmware .enc path.

    token can be:
      - an existing file path
      - an installed firmware filename
      - a glob pattern like 'KH-UCANFD-*.enc' (searched under FW_IMAGE_DIR)
    """
    token = str(firmware_token).strip()
    # Direct path: keep current behavior
    p = Path(token)
    if p.exists() and p.is_file():
        return p

    # If it looks like a path, fail fast.
    if "/" in token:
        raise FileNotFoundError(f"Firmware file not found: {token}")

    # Token without '/', search in firmware directory.
    # Treat '**' as '*' for matching single directory filenames.
    if "**" in token and "/" not in token:
        token = token.replace("**", "*")

    search_dir = Path(FW_IMAGE_DIR)
    matches = sorted(search_dir.glob(token))

    if not matches:
        # Exact filename without wildcard might still be requested.
        exact = search_dir / token
        if exact.exists() and exact.is_file():
            return exact
        raise FileNotFoundError(f"Firmware '{firmware_token}' not found under {search_dir}")

    if len(matches) == 1:
        return matches[0]

    # Multiple matches: choose newest version when manifest provides metadata.
    manifest_file = Path(manifest_path)
    by_filename = {}
    if manifest_file.exists():
        with open(manifest_file, "r", encoding="utf-8") as f:
            manifest = json.load(f)
        for img in manifest.get("images", []):
            fn = img.get("filename", None)
            if fn:
                by_filename[fn] = img

    candidates = []
    for m in matches:
        entry = by_filename.get(m.name, None)
        fw_version_str = entry.get("fw_version", None) if entry else None
        if fw_version_str:
            try:
                fw_ver_t = _parse_version_3(fw_version_str)
                candidates.append((fw_ver_t, m))
            except Exception:
                pass

    if candidates:
        # fw_version is parsed into (major, minor, revision); lexicographical compare matches semver ordering here.
        candidates.sort(key=lambda x: x[0])
        chosen = candidates[-1][1]
        if verbose:
            matched_names = ", ".join([x.name for x in matches])
            log_warning(f"Multiple firmware matched '{firmware_token}'. Choosing newest: {chosen.name}. Matches: {matched_names}")
        return chosen

    # Fallback: ask user to specify a more exact pattern.
    matched_names = ", ".join([x.name for x in matches])
    raise RuntimeError(f"Multiple firmware matched '{firmware_token}'. Please specify exact filename. Matches: {matched_names}")

def main():
    argv = sys.argv[1:]
    verbose = False
    skip_version = False
    skip_bootloader = False
    manifest_path = str(FW_MANIFEST_PATH_DEFAULT)

    positional = []
    i = 0
    while i < len(argv):
        a = argv[i]
        if a in ("-h", "--help"):
            print("Usage:")
            print("  Normal:         kcan_fw_upgrade.py <can_if> <firmware.enc|auto> [options]")
            print("  Skip bootloader: kcan_fw_upgrade.py <firmware.enc|auto> --skip-bootloader [can_if] [options]")
            print("  Query:          kcan_fw_upgrade.py lsfw [pattern]   (list installed firmware under /usr/lib/firmware)")
            print("Options:")
            print("  -v, --verbose           Verbose output")
            print("  -s, --skip-version     Skip version verification after upgrade")
            print("  -b, --skip-bootloader  Skip entering bootloader mode")
            print(f"  --manifest PATH        Manifest JSON (default: {manifest_path})")
            return
        if a in ("-v", "--verbose"):
            verbose = True
            i += 1
            continue
        if a in ("-s", "--skip-version"):
            skip_version = True
            i += 1
            continue
        if a in ("-b", "--skip-bootloader"):
            skip_bootloader = True
            i += 1
            continue
        if a == "--manifest":
            if i + 1 >= len(argv):
                log_error("--manifest requires a value")
                sys.exit(1)
            manifest_path = argv[i + 1]
            i += 2
            continue
        if a.startswith("-"):
            log_error(f"Unknown option: {a}")
            sys.exit(1)
        positional.append(a)
        i += 1

    # lsfw: list installed firmware images (no device interaction required)
    if positional and str(positional[0]).lower() in ("lsfw", "ls-fw"):
        pattern = positional[1] if len(positional) >= 2 else None
        return list_installed_firmware(pattern=pattern, manifest_path=manifest_path, verbose=verbose)

    # resolvefw: resolve token (filename/glob) -> absolute .enc path
    if positional and str(positional[0]).lower() in ("resolvefw", "resolve-fw", "resolve"):
        if len(positional) < 2:
            log_error("resolvefw requires a token (filename or glob), e.g. resolvefw KH-UCANFD-*.enc")
            sys.exit(1)
        firmware_token = positional[1]
        try:
            resolved = resolve_firmware_token(
                firmware_token,
                manifest_path=manifest_path,
                verbose=False,  # ensure resolver outputs only the path
            )
            print(str(resolved))
            return 0
        except Exception as e:
            log_error(str(e))
            sys.exit(1)

    if skip_bootloader:
        if len(positional) < 1:
            log_error("Firmware file or 'auto' required when using --skip-bootloader")
            sys.exit(1)
        firmware_token = positional[0]
        can_interface = positional[1] if len(positional) >= 2 else None
    else:
        if len(positional) < 2:
            log_error("CAN interface and firmware file are required")
            sys.exit(1)
        can_interface = positional[0]
        firmware_token = positional[1]

    # Check if running as root (needed for mount operations)
    if os.geteuid() != 0:
        log_error("This script must be run as root (for mount operations)")
        log_error(f"Please run: sudo {sys.argv[0]} {' '.join(sys.argv[1:])}")
        sys.exit(1)

    # Resolve firmware path (auto or explicit file)
    firmware_path = None
    auto_mode = str(firmware_token).lower() == "auto"
    if auto_mode:
        if not can_interface:
            log_error("Auto mode requires CAN interface for version/device detection")
            sys.exit(1)
        try:
            log_info(f"Auto mode: selecting firmware from manifest: {manifest_path}")
            firmware_path = select_firmware_from_manifest(can_interface, manifest_path, verbose=verbose)
        except Exception as e:
            log_error(f"Auto selection failed: {e}")
            sys.exit(1)

        if firmware_path is None:
            log_success("Already up-to-date: no compatible newer firmware found.")
            return 0
    else:
        try:
            firmware_path = resolve_firmware_token(firmware_token, manifest_path=manifest_path, verbose=verbose)
        except Exception as e:
            log_error(str(e))
            sys.exit(1)

    log_info("=" * 42)
    log_info("KCAN Auto Firmware Upgrade")
    log_info("=" * 42)
    log_info(f"CAN Interface: {can_interface if can_interface else '(not provided)'}")
    log_info(f"Firmware File: {firmware_path}")
    log_info(f"Manifest: {manifest_path}" if auto_mode else "")
    if skip_bootloader:
        log_info("Mode: Skip bootloader")
    log_info("")

    device = None
    try:
        # Step 1: Enter bootloader mode
        if not skip_bootloader:
            if not can_interface:
                log_error("CAN interface required to enter bootloader mode")
                sys.exit(1)
            log_info("Step 1/5: Entering bootloader mode...")
            if not enter_bootloader_mode(can_interface, verbose):
                log_error("Failed to enter bootloader mode")
                sys.exit(1)
        else:
            log_info("Step 1/5: Skipping bootloader entry (--skip-bootloader)")

        # Step 2: Wait for USB device to appear
        log_info("")
        log_info("Step 2/5: Waiting for USB mass storage device...")
        device = wait_for_usb_device(verbose)
        if not device:
            log_error("USB device not found")
            sys.exit(1)

        # Step 3: Mount device and copy firmware
        log_info("")
        log_info("Step 3/5: Mounting device and copying firmware...")
        if not mount_upgrade_device(device, verbose):
            log_error("Failed to mount device")
            sys.exit(1)

        try:
            if not copy_firmware(firmware_path, verbose):
                log_error("Failed to copy firmware")
                sys.exit(1)
        finally:
            # Unmount device (device will handle upgrade and restart)
            log_info("")
            log_info("Step 4/5: Unmounting device (device will upgrade and restart)...")
            unmount_upgrade_device(verbose)

        # Step 4: Wait for device restart
        log_info("")
        log_info("Step 5/5: Waiting for device to restart...")
        if can_interface:
            wait_for_device_restart(can_interface, verbose)
        else:
            log_warning("CAN interface not provided; skipping device restart check")

        # Step 5: Verify firmware version
        if not skip_version and can_interface:
            log_info("")
            log_info("Verifying firmware version...")
            get_firmware_version(can_interface, verbose)
        else:
            if skip_version:
                log_warning("Skipping version verification (--skip-version)")

        log_info("")
        log_success("=" * 42)
        log_success("Firmware upgrade completed successfully!")
        log_success("=" * 42)

    except KeyboardInterrupt:
        log_warning("\nInterrupted by user")
        if device:
            unmount_upgrade_device(verbose)
        sys.exit(1)
    except Exception as e:
        log_error(f"Unexpected error: {e}")
        if verbose:
            import traceback
            traceback.print_exc()
        if device:
            unmount_upgrade_device(verbose)
        sys.exit(1)

if __name__ == "__main__":
    main()
