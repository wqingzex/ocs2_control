"""Navigation launch (start.sh nav) and joystick discovery (console), the former quick_start.sh menus.

    python3 -m teleop_app.navigation --config C --ws-root W [--list] [--map ID] [--map-only]"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
from typing import Any, Dict, List, Optional, Tuple

from .config import ConfigError, load_lenient

NAV_LAUNCH = ("ros2", "launch", "robot_navigation", "navigation_isaac_gt.launch.py")
JOYSTICK_LINE = re.compile(r"^\s*(\d+)\s*:\s*([^:]+?)\s*:")
ENUMERATE_TIMEOUT_S = 3.0


try:
    from ament_index_python.packages import PackageNotFoundError
    from ament_index_python.packages import get_package_share_directory as _share_directory
except ImportError:  # no ROS environment (unit tests): treat every package as missing
    class PackageNotFoundError(Exception):  # type: ignore[no-redef]
        pass

    def _share_directory(name: str) -> str:
        raise PackageNotFoundError(name)


class NavigationError(RuntimeError):
    """The navigation launch cannot be built (message is user facing)."""


def missing_dependency() -> Optional[str]:
    """nav2_bringup is loaded by the navigation launch file but only installed on demand."""
    try:
        _share_directory("nav2_bringup")
    except PackageNotFoundError:
        return "缺少 nav2_bringup：./scripts/setup.sh deps，或 sudo apt install ros-jazzy-nav2-bringup"
    return None


def list_maps(cfg: Dict[str, Any], ws_root: str) -> List[Dict[str, Any]]:
    """Maps of the standard search chain (needs the sourced ROS workspace)."""
    from robot_navigation.config import list_navigation_maps
    nav = cfg["navigation"]
    return list_navigation_maps(robot_name=nav["robot"], map_config_package=nav["map_config_package"],
                                ws_dir=ws_root)


def launch_command(cfg: Dict[str, Any], maps: List[Dict[str, Any]], map_id: Optional[str],
                   map_only: Optional[bool]) -> Tuple[str, ...]:
    """ros2 launch command; map_id / map_only override the config (None = use the config)."""
    nav = cfg["navigation"]
    if not maps:
        raise NavigationError("没有找到地图：确认已编译 fiveages_w2_description / robot_navigation")
    ids = [m["id"] for m in maps]
    chosen = map_id or nav["map"] or next((m["id"] for m in maps if m.get("default")), "")
    if not chosen:
        raise NavigationError(f"没有默认地图，请用 --map 指定，可选: {', '.join(ids)}")
    if chosen not in ids:
        raise NavigationError(f"未知地图 '{chosen}'，可选: {', '.join(ids)}")
    cmd = NAV_LAUNCH + (f"robot:={nav['robot']}", f"map:={chosen}")
    if nav["map_config_package"]:
        cmd += (f"map_config_package:={nav['map_config_package']}",)
    if nav["nav_config_package"]:
        cmd += (f"nav_config_package:={nav['nav_config_package']}",)
    if nav["map_only"] if map_only is None else map_only:
        cmd += ("nav2_profile:=map_only",)
    return cmd


def parse_joysticks(text: str) -> List[Dict[str, Any]]:
    """Parse `ros2 run joy joy_enumerate_devices` output: lines like '0 : name : true : ...'."""
    return [{"id": int(m.group(1)), "name": m.group(2)}
            for m in (JOYSTICK_LINE.match(line) for line in text.splitlines()) if m]


def enumerate_joysticks() -> List[Dict[str, Any]]:
    try:
        out = subprocess.run(["ros2", "run", "joy", "joy_enumerate_devices"], capture_output=True, text=True,
                             check=False, timeout=ENUMERATE_TIMEOUT_S).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    return parse_joysticks(out)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="start.sh nav", description="Start navigation (Ctrl+C stops)")
    parser.add_argument("--config", required=True)
    parser.add_argument("--ws-root", required=True)
    parser.add_argument("--list", action="store_true", help="list the available maps and exit")
    parser.add_argument("--map", dest="map_id", help="map id (default: config navigation.map, then the default map)")
    parser.add_argument("--map-only", action="store_true", default=None, help="static map only, no point cloud")
    args = parser.parse_args(argv)
    try:
        cfg = load_lenient(args.config)
    except ConfigError as exc:
        print(f"[配置错误] {args.config}: {exc}", file=sys.stderr)
        return 2
    try:
        maps = list_maps(cfg, args.ws_root)
        if args.list:
            for item in maps:
                print(f"{item['id']}{' (默认)' if item.get('default') else ''}  [{item.get('source', '')}]")
            return 0 if maps else 1
        cmd = launch_command(cfg, maps, args.map_id, args.map_only)
        missing = missing_dependency()
        if missing:
            raise NavigationError(missing)
    except (NavigationError, ImportError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    print(">>> " + " ".join(cmd[3:]), flush=True)
    os.execvp(cmd[0], list(cmd))


if __name__ == "__main__":
    sys.exit(main())
