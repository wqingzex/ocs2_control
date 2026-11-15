"""Validate a teleop YAML and print what will be started.

Usage: python3 -m teleop_app.check <config.yaml> [ws_root]  (exit 0 valid, 2 config error)"""
from __future__ import annotations

import argparse
import os
import sys

from w2_camera.camera import camera_specs
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
        print(f"  XR:   xr_bridge{suffix}")
        mode = "网线" if xr["network_mode"] == "ethernet" else "Wi-Fi"
        print(f"        网络: {mode}，网卡: {xr['network_interface'] or '初始化时选择'}，头显 ID: {xr['device_id'] or '尚未绑定'}")
        import yaml
        from xr_teleop.xr_network import arm_ip, network_addresses, without_arm_link
        with open(path, encoding='utf-8') as handle:
            raw = yaml.safe_load(handle) or {}
        addresses = [entry for entry in without_arm_link(network_addresses(xr["network_mode"]), arm_ip(raw))
                     if not xr["network_interface"] or entry.interface == xr["network_interface"]]
        for entry in addresses:
            print(f"        PC Service 地址: {entry.address} ({entry.interface})")
        if not addresses:
            print("        所选网络暂无可用 IPv4 地址，请检查网线/Wi-Fi 或重新配置 XR 网卡")
    else:
        print("  XR:   关闭")
    cams = camera_specs(cfg)
    cam_text = ", ".join("{}({})".format(c.name, c.args["serial_number"]) for c in cams) if cams else "关闭"
    print(f"  相机: {cam_text}")
    import yaml
    from w2_payload.config_edit import confirmed
    import w2_camera.camera_setup, w2_hand.hand_setup  # noqa: F401  (they register how a mapping is fingerprinted)
    with open(path, encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    for section, title, enabled in (('cameras', '相机画面与位置', bool(cams)),
                                     ('hand', '左右手实物', cfg['robot']['hand'] != 'none')):
        if enabled:
            status = '已确认' if confirmed(raw, section) else '尚未确认或配置已变化'
            command = 'cam' if section == 'cameras' else 'hand'
            print(f"  {title}: {status}（./scripts/setup.sh {command}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
