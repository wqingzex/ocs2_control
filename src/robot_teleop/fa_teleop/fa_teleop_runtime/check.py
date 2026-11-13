"""Validate a teleop YAML and print what will be started.

Usage: python3 -m fa_teleop_runtime.check <config.yaml> [ws_root]  (exit 0 valid, 2 config error)"""
from __future__ import annotations

import argparse
import os
import sys

from w2_payload.devices.cameras import camera_specs
from .config import ConfigError, load
from .robot import controller_launch


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="check", description="Validate a robot config and print what will start")
    parser.add_argument("config")
    parser.add_argument("ws_root", nargs="?", default=os.getcwd())
    parser.add_argument("--set", dest="overrides", action="append", default=[], metavar="SECTION.FIELD=VALUE")
    args = parser.parse_args(sys.argv[1:] if argv is None else argv)
    path, ws_root = args.config, args.ws_root
    try:
        cfg = load(path, args.overrides)
    except ConfigError as exc:
        print(f"[配置错误] {path}: {exc}", file=sys.stderr)
        return 2

    launch_file, args = controller_launch(cfg, os.path.abspath(path))

    arg_text = " ".join(f"{k}:={v}" for k, v in args.items())
    print(f"配置: {path}")
    print(f"  运控: ros2 launch robot_bringup {launch_file} {arg_text}")
    xr = cfg["xr"]
    if xr["enable"]:
        suffix = "（自动启动 PC Service）" if xr["start_pc_service"] else ""
        print(f"  VR:   xr_bridge{suffix}")
        mode = "网线" if xr["network_mode"] == "ethernet" else "Wi-Fi"
        print(f"        网络: {mode}，网卡: {xr['network_interface'] or '初始化时选择'}，头显 ID: {xr['device_id'] or '尚未绑定'}")
        from .xr_network import network_addresses
        addresses = [entry for entry in network_addresses(xr["network_mode"])
                     if not xr["network_interface"] or entry.interface == xr["network_interface"]]
        for entry in addresses:
            print(f"        PC Service 地址: {entry.address} ({entry.interface})")
        if not addresses:
            print("        所选网络暂无可用 IPv4 地址，请检查网线/Wi-Fi 或重新配置 XR 网卡")
    else:
        print("  VR:   关闭")
    cams = camera_specs(cfg)
    cam_text = ", ".join("{}({})".format(c.name, c.args["serial_number"]) for c in cams) if cams else "关闭"
    print(f"  相机: {cam_text}")
    import yaml
    from w2_payload.setup.state import confirmed
    with open(path, encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    for section, title, enabled in (('cameras', '相机画面与位置', bool(cams)),
                                     ('hand', '左右手实物', cfg['robot']['hand'] != 'none')):
        if enabled:
            status = '已确认' if confirmed(raw, section) else '尚未确认或配置已变化'
            command = 'cam' if section == 'cameras' else 'hand'
            print(f"  {title}: {status}（./scripts/setup.sh {command}）")
    warning = scheme_mismatch_warning(cfg)
    if warning:
        sys.stdout.flush()  # keep the warning after the summary
        print(f"  [警告] {warning}", file=sys.stderr)
    return 0


def _robot_vr_scheme(robot_name: str):
    """vr_control_scheme from the robot's teleop_command.yaml, or None if it cannot be read."""
    try:
        import yaml
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory(f"{robot_name}_description")
        with open(os.path.join(share, "config", "ocs2", "teleop_command.yaml"), encoding="utf-8") as handle:
            params = yaml.safe_load(handle)["/**"]["ros__parameters"]
        return params.get("vr_control_scheme", "legacy")
    except Exception:  # package not built / file layout differs: nothing to compare against
        return None


def scheme_mismatch_warning(cfg, robot_scheme=None):
    """xr.control_scheme and the robot's vr_control_scheme should be the same."""
    if not cfg["xr"]["enable"]:
        return None
    from .config import ENTRIES
    robot_name = ENTRIES[cfg["robot"]["entry"]][1]
    scheme = robot_scheme if robot_scheme is not None else _robot_vr_scheme(robot_name)
    profile = cfg["xr"]["control_scheme"]
    if scheme is None or scheme == profile:
        return None
    return (f"xr.control_scheme={profile}，但 {robot_name}_description 的 teleop_command.yaml "
            f"vr_control_scheme={scheme}：两者应一致（simple 需要两边都设为 simple）")


if __name__ == "__main__":
    sys.exit(main())
