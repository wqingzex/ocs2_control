"""Validate the shared robot configuration (robot, cameras) without importing teleoperation or XR SDKs."""
import sys
import yaml

from ..config import ConfigError, load
from .state import confirmed


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    if len(args) != 1:
        print('用法：python3 -m w2_payload.setup.check config/robot.local.yaml', file=sys.stderr)
        return 2
    path = args[0]
    try:
        cfg = load(path)
    except ConfigError as exc:
        print(f'[配置错误] {path}: {exc}', file=sys.stderr)
        return 2
    with open(path, encoding='utf-8') as handle:
        raw = yaml.safe_load(handle) or {}
    print(f'配置: {path}')
    robot = cfg['robot']
    print(f"  机器人: {robot['entry']}，硬件模式: {robot['hardware']}")
    print(f"  手: {robot['hand']}，接线: {robot['hand_drive']}")
    hardware = raw.get('hardware') or {}
    if robot['hand'] != 'none' and robot['hand_drive'] == 'usb':
        print(f"    左手: {hardware.get('usb_left_port', '/dev/ttyUSB0')}")
        print(f"    右手: {hardware.get('usb_right_port', '/dev/ttyUSB2')}")
    cams = cfg['cameras']
    for role in ('head', 'left_wrist', 'right_wrist'):
        camera = cams[role]
        description = f"{camera['serial']}，旋转 {camera['rotation']}°" if cams['enable'] and camera['enable'] else '关闭'
        print(f'  相机 {role}: {description}')
    for section, title, enabled in (('cameras', '相机画面与位置', cams['enable']), ('hand', '左右手实物', robot['hand'] != 'none')):
        if enabled:
            status = '已确认' if confirmed(raw, section) else '尚未确认或配置已变化'
            command = 'cam' if section == 'cameras' else 'hand'
            print(f'  {title}: {status}（./scripts/setup.sh {command}）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
