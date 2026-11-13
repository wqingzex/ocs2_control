"""Shared camera and hand initialization used by setup.sh."""
from __future__ import annotations
import argparse
import os
from pathlib import Path
import sys
from typing import Callable, Dict, List
from ..config import edit as config_edit
from ..devices import cameras as devices
from ..config import load_lenient, normalize_hand_drive

class Io:
    """Console I/O; tests replace ask()."""

    def __init__(self, assume_yes: bool = False, out=None):
        self.assume_yes = assume_yes
        self.out = out or sys.stdout

    def info(self, msg: str) -> None:
        print(f">>> {msg}", file=self.out)

    def warn(self, msg: str) -> None:
        print(f"[WARN] {msg}", file=self.out)

    def error(self, msg: str) -> None:
        print(f"[ERROR] {msg}", file=self.out)

    def ask(self, question: str, default: str) -> str:
        if self.assume_yes or not sys.stdin.isatty():
            return default
        answer = input(f"{question} [{default}]: ").strip()
        return answer or default


# ---- cameras -----------------------------------------------------------------------------------

def step_cameras(config_path: str, ws_root: str, io: Io,
                 scanner: Callable[[], List[devices.Device]] = devices.scan, runner=None) -> bool:
    from .device_setup import CameraSetup
    from .state import confirmed
    from .wizard import run_wizard
    _, raw = config_edit.load_rt(config_path)
    if (raw.get("cameras") or {}).get("enable") is False:
        answer = io.ask("相机当前关闭，是否现在配置并启用？y/n", "y")
        if answer.lower() not in ("y", "yes", "是"):
            io.info("按选择保持相机关闭")
            return True
    io.info("扫描 Orbbec 相机 ...")
    try:
        devs = scanner()
    except RuntimeError as exc:
        io.error(str(exc))
        return False
    if not devs:
        io.error("没有发现相机，初始化已停止。请检查 USB/GMSL 连接、设备权限及占用情况；容器需映射相机 USB 设备。")
        return False
    for i, d in enumerate(devs, 1):
        io.info(f"  [{i}] {d.model:<24} 序列号 {d.serial:<16} 连接 {d.connection or '-':<8} 端口 {d.usb_port or '-'}")
    for serial, node in devices.inaccessible_usb_devices():
        io.warn(f'检测到相机 {serial}，但当前环境无法访问 {node}；检查权限或容器 USB 映射，热插拔后设备编号可能变化')
    if len(devs) < len(devices.ROLES):
        io.warn(f"只发现 {len(devs)} 台相机（正常为 {len(devices.ROLES)} 台）")
    if io.assume_yes:
        cams = raw.get('cameras') or {}
        saved = {value.get('serial') for role, value in cams.items()
                 if role in devices.ROLES and isinstance(value, dict) and value.get('enable', True)}
        if confirmed(raw, 'cameras') and saved.issubset({d.serial for d in devs}):
            io.info('复用已通过画面确认的相机配置')
            return True
        io.error('首次相机配置或设备映射变化需要在窗口中确认；未保存配置，请去掉 --yes 重试')
        return False
    session = CameraSetup(config_path, ws_root, devs)
    try:
        ok = (runner or run_wizard)(session, io)
        if ok:
            io.info(f'相机画面与位置已确认，已写入 {config_path}')
        else:
            io.error('相机配置已取消，未保存本次修改')
        return ok
    finally:
        session.close()


# ---- hand ----------------------------------------------------------------------------------------

def serial_ports() -> List[str]:
    from ..devices.hand_probe import serial_devices
    return [entry['path'] for entry in serial_devices()]


def load_raw_robot(config_path: str) -> Dict:
    _, data = config_edit.load_rt(config_path)
    return dict(data.get("robot") or {})


def step_hand(config_path: str, io: Io, ports: Callable[[], List[str]] = serial_ports,
              *, ws_root=None, runner=None) -> bool:
    from .device_setup import HandSetup
    from .state import confirmed
    from .wizard import run_wizard
    _, raw = config_edit.load_rt(config_path)
    if io.assume_yes:
        if confirmed(raw, 'hand'):
            robot, hw = raw.get('robot') or {}, raw.get('hardware') or {}
            if robot.get('hand') != 'none' and normalize_hand_drive(robot.get('hand_drive', 'usb')) == 'usb':
                available = {os.path.realpath(path) for path in ports()}
                if any(os.path.realpath(hw.get('usb_' + side + '_port', '')) not in available for side in ('left', 'right')):
                    io.error('已确认的手部串口当前不可用，请检查连接或容器设备映射')
                    return False
            io.info('复用已确认左右实物的手部配置')
            return True
        io.error('首次手部配置或设备映射变化需要交互确认左右手；未保存配置，请去掉 --yes 重试')
        return False
    root = ws_root or str(Path(config_path).resolve().parent.parent)
    entries = lambda: [{'path': path, 'device': os.path.realpath(path), 'stable': '/dev/serial/' in path} for path in ports()]
    session = HandSetup(config_path, root, ports=entries)
    try:
        ok = (runner or run_wizard)(session, io)
        if ok:
            io.info(f'左右手配置已确认，已写入 {config_path}')
        else:
            io.warn('手部配置已取消，未保存本次修改')
        return ok
    finally:
        session.close()



def main(argv=None):
    parser = argparse.ArgumentParser(prog='setup.sh')
    parser.add_argument('--config', required=True)
    parser.add_argument('--ws-root', required=True)
    parser.add_argument('--yes', action='store_true')
    parser.add_argument('--reset', action='store_true', help='dyn: back to the default load (empties both overrides)')
    parser.add_argument('step', choices=['cameras', 'hand', 'dyn'])
    args = parser.parse_args(argv)
    io = Io(assume_yes=args.yes)
    try:
        if args.step == 'dyn':
            from . import dyn
            ok = dyn.reset_dyn(args.config, args.ws_root, io) if args.reset else dyn.step_dyn(args.config, args.ws_root, io)
        else:
            ok = step_cameras(args.config, args.ws_root, io) if args.step == 'cameras' else step_hand(args.config, io, ws_root=args.ws_root)
    except KeyboardInterrupt:
        io.info('已取消初始化')
        return 130
    except (RuntimeError, ValueError, OSError) as exc:
        io.error(str(exc))
        ok = False
    if not ok and args.step == 'cameras':
        io.error('相机配置失败，已退出；重试本步：./scripts/setup.sh cam；从本步继续到末尾：./scripts/setup.sh cam --from')
    return 0 if ok else 1

if __name__ == '__main__':
    sys.exit(main())
