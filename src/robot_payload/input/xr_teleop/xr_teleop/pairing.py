"""XR headset pairing (./scripts/setup.sh xr): choose the network adapter, start the PC Service and the bridge,
wait for the headset and save its id.

    python3 -m xr_teleop.pairing --config C --ws-root W [--yes] [--xr-timeout S] xr"""
from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Dict, List, Optional, Tuple
from pathlib import Path

from robot_runtime.supervisor import Supervisor, UnitError
from robot_runtime.units import UnitSet
from w2_payload import config_edit
from w2_payload.config import load_lenient
from w2_payload.device_setup import Io

from .config import EXTENSION
from .units import xr_units
from .xr_network import arm_ip, network_addresses, without_arm_link

# ---- VR --------------------------------------------------------------------------------------------

def host_ips() -> List[str]:
    return [f"{entry.address} ({entry.interface})" for entry in network_addresses()]


def _xr_network(config_path: str, xr: Dict, io: Io) -> bool:
    mode = io.ask("XR 连接方式：ethernet（网线）| wifi（无线）", xr["network_mode"]).lower()
    mode = {"网线": "ethernet", "有线": "ethernet", "wi-fi": "wifi"}.get(mode, mode)
    if mode not in ("ethernet", "wifi"):
        io.error("请选择 ethernet（网线）或 wifi（无线）")
        return False
    arm = arm_ip(config_edit.load_rt(config_path)[1])
    addresses = without_arm_link(network_addresses(mode), arm)  # the cable to the arm is not offered
    if not addresses:
        io.error(f"没有找到可用的{'有线' if mode == 'ethernet' else 'Wi-Fi'} IPv4 地址；请检查网卡和网络连接"
                 f"（和机械臂 {arm} 同网段的网卡不会列出）")
        return False
    for index, entry in enumerate(addresses, 1):
        io.info(f"  [{index}] {entry.interface}: {entry.address}")
    preferred = next((index for index, entry in enumerate(addresses, 1)
                      if entry.interface == xr["network_interface"]), 1)
    answer = io.ask("选择 XR 使用的网卡编号", str(preferred)) if len(addresses) > 1 else "1"
    if not answer.isdigit() or not 1 <= int(answer) <= len(addresses):
        io.error(f"无效网卡编号: {answer}")
        return False
    selected = addresses[int(answer) - 1]
    xr.update(network_mode=mode, network_interface=selected.interface)
    config_edit.set_fields(config_path, "xr", {"network_mode": mode, "network_interface": selected.interface})
    io.info(f"在 Pico 的 XRoboToolkit App 中选择 PC Service 地址：{selected.address}（{selected.interface}）")
    io.info("头显 App：Tracking 至少勾选 Head 或 Controller 一项，开启 Data & Control 的 Send（本次无需开启 Data Collection）")
    interface_path = (Path("/sys/class/net") / selected.interface).resolve()
    if (Path("/.dockerenv").exists() and "/devices/virtual/" in str(interface_path)
            and os.environ.get("W2_CONTAINER_NETWORK") != "host"):
        io.warn("当前为容器地址；头显必须能访问 PC Service，Docker 桥接网络不会自动转发局域网发现广播")
    return True


def step_xr(config_path: str, ws_root: str, io: Io, timeout_s: Optional[float] = None, probe_factory=None) -> bool:
    cfg = load_lenient(config_path, EXTENSION)
    xr = cfg["xr"]
    if not xr["enable"]:
        answer = io.ask("XR 当前关闭，是否现在启用并配对头显？y/n", "y")
        if answer.lower() not in ("y", "yes", "是"):
            io.info("按选择保持 XR 关闭")
            return True
        config_edit.set_fields(config_path, "xr", {"enable": True})
        cfg = load_lenient(config_path, EXTENSION)
        xr = cfg["xr"]
    if xr["start_pc_service"] and not os.path.isfile(xr["pc_service_script"]):
        io.warn(f"未安装 XRoboToolkit PC Service（{xr['pc_service_script']}）：./scripts/setup.sh host")
        return False
    if not _xr_network(config_path, xr, io):
        return False
    expected_id = xr["device_id"]
    rebind = False
    if expected_id:
        io.info(f"已绑定头显 ID: {expected_id}；日常启动会自动匹配该设备")
        rebind = io.ask("是否更换绑定头显？y/n", "n").lower() in ("y", "yes", "是")
        if rebind:
            xr["device_id"] = ""  # Save a new ID only after successful tracking reception.
            expected_id = ""

    units = UnitSet(xr_units(cfg))
    sup = Supervisor(units, os.path.join(ws_root, "run"), os.path.join(ws_root, "logs"))
    if rebind and sup.is_running("xr"):
        io.error("XR 模块正在运行，请先停止 XR 模块再更换绑定头显")
        return False
    started = [n for n in units.start_order(["xr"]) if not sup.is_running(n)]
    try:
        for name in started:
            sup.start(name)
        if probe_factory is None:
            from .probe import XrProbe
            probe_factory = lambda: XrProbe("xr_teleop_pairing")  # noqa: E731
        probe = probe_factory()
        try:
            deadline = time.monotonic() + timeout_s if timeout_s is not None and timeout_s > 0 else None
            io.info("等待头显连接（不限时，Ctrl+C 取消）..." if deadline is None
                    else f"等待头显连接（最多 {timeout_s:.0f} s，Ctrl+C 取消）...")
            while deadline is None or time.monotonic() < deadline:
                device_id = getattr(probe, "xr_device_id", "")
                if probe.xr_connected and device_id and (not expected_id or expected_id == device_id):
                    if not expected_id:
                        answer = io.ask(f"收到头显 {device_id} 的追踪数据，是否绑定？y/n", "y")
                        if answer.lower() not in ("y", "yes", "是"):
                            io.error("未绑定头显，配对未完成")
                            return False
                    config_edit.set_fields(config_path, "xr", {"device_id": device_id})
                    io.info(f"头显已连接并保存 ID: {device_id}；日常启动自动匹配，断线后等待同一设备恢复")
                    return True
                time.sleep(0.5)
        finally:
            probe.close()
        io.error("未收到目标头显的追踪数据和设备 ID；确认头显 App 已连接上述地址、正在发送追踪数据且网络可达；日志见 logs/xr/latest.log")
        return False
    except UnitError as exc:
        io.warn(str(exc))
        return False
    finally:
        for name in reversed(started):  # leave the system as we found it
            try:
                sup.stop(name)
            except UnitError as exc:
                io.warn(str(exc))


def xr_status(raw) -> Tuple[bool, str]:
    """(needs pairing, reason): nothing to do when XR is off or a headset id is already saved."""
    xr = raw.get("xr") or {}
    if xr.get("enable") is False:
        return False, "VR 已关闭（xr.enable: false）"
    if xr.get("device_id"):
        return False, f"已绑定头显 {xr['device_id']}"
    return True, "头显尚未配对"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="setup.sh")
    parser.add_argument("--config", required=True)
    parser.add_argument("--ws-root", required=True)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--xr-timeout", type=float, default=None, help="可选等待秒数；不指定或 0 为不限时")
    parser.add_argument("--status", action="store_true", help="only print whether pairing is needed (exit 1 if so)")
    parser.add_argument("step", choices=["xr"])
    args = parser.parse_args(argv)
    if args.status:
        needed, reason = xr_status(config_edit.load_rt(args.config)[1])
        print(f"{'需配置' if needed else '已配置'}：{reason}")
        return 1 if needed else 0
    io = Io(assume_yes=args.yes)
    try:
        ok = step_xr(args.config, args.ws_root, io, args.xr_timeout)
    except KeyboardInterrupt:
        io.info("已取消初始化")
        return 130
    except (RuntimeError, ValueError) as exc:
        io.error(str(exc))
        ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
