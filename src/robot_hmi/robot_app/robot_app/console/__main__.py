"""Run the web console:  python3 -m robot_app.console --config C --ws-root W [--host H] [--port P]"""
from __future__ import annotations

import argparse
import os
import sys

from .server import Console, create_app


def static_dir(ws_root: str) -> str:
    """The console page lives in <ws_root>/src/robot_hmi/robot_app/web, in the robot_app package directory but
    outside the Python package."""
    return os.path.join(ws_root, "src", "robot_hmi", "robot_app", "web")


def access_urls(host_ips, port: int):
    """Addresses other computers can open: one per network interface (host_ips: "ip (iface)")."""
    return [f"http://{entry.split()[0]}:{port}/" for entry in host_ips]


DEFAULT_HOST = "127.0.0.1"  # the console can start and stop the robot: reachable from other computers only on request


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="start.sh console")
    parser.add_argument("--config", required=True)
    parser.add_argument("--ws-root", required=True)
    parser.add_argument("--host", default=DEFAULT_HOST,
                        help=f"listen address (default: {DEFAULT_HOST}, this computer only; "
                             "0.0.0.0 = all interfaces, no login, plain HTTP: robot LAN only)")
    parser.add_argument("--port", type=int, default=8765)
    return parser


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    try:
        import uvicorn
    except ImportError:
        print("[ERROR] 缺少 uvicorn/fastapi：sudo apt install python3-uvicorn python3-fastapi（或 rosdep install）",
              file=sys.stderr)
        return 1

    def probe_factory():
        from ..ros_probe import RosProbe
        return RosProbe("robot_app_console")

    console = Console(args.config, args.ws_root, probe_factory=probe_factory)
    app = create_app(console, static_dir(args.ws_root))

    from xr_teleop.pairing import host_ips
    local_only = args.host in ("127.0.0.1", "localhost")
    console.access_urls = [] if local_only else access_urls(host_ips(), args.port)
    print(">>> ocs2_control 控制台（Ctrl+C 退出；已启动的模块不受影响）")
    if not local_only:
        print("[WARN] 没有登录：同一网络里能访问这个端口的人都能控制机器人，只在机器人内网使用", file=sys.stderr)
    for url in [f"http://127.0.0.1:{args.port}/"] + console.access_urls:
        print(f"    {url}")
    if console.probe_error:
        print(f"[WARN] {console.probe_error}", file=sys.stderr)
    sys.stdout.flush()  # the URL must reach logs even when stdout is not a terminal
    uvicorn.run(app, host=args.host, port=args.port, log_level="warning")
    if console.probe is not None:
        console.probe.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
