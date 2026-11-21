"""XR headset: how to start it, and whether it is really up.

The PC Service daemon (start, stop, tracked by process name), the units robot_app supervises (the service and the bridge
node), and the readiness verdict read from the bridge log. The bridge itself is in robot_teleop/xr_teleop, the pairing
step in robot_setup/xr_setup."""
from __future__ import annotations

import configparser
from contextlib import ExitStack
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, List, NamedTuple, Optional, Set

from ament_index_python.packages import get_package_prefix
from robot_config.config import teleop_type, xr_settings
from robot_runtime.units import Unit
from xr_teleop.xr_network import network_addresses

# ---- PC Service: start / stop (runService.sh daemonises it and exits at once) ----------------------------------------
# Started only if not running; on shutdown only the PIDs we started are stopped.

# Linux comm is truncated to 15 characters: RoboticsServiceProcess -> RoboticsService
PROCESS_COMM = "RoboticsService"
PICO_TCP_PORT = 63901


def _is_zombie(pid: int) -> bool:
    """True when the process is dead but not reaped yet (or gone): a container's PID 1 often never reaps."""
    try:
        return Path(f"/proc/{pid}/stat").read_text().rsplit(")", 1)[1].split()[0] == "Z"
    except (OSError, IndexError):
        return True


def running_pids() -> Set[int]:
    """PIDs of the running service. A zombie is not running: counting it made start() believe the service was up."""
    result = subprocess.run(["pgrep", "-x", PROCESS_COMM], capture_output=True, text=True, check=False)
    return {pid for pid in map(int, result.stdout.split()) if not _is_zombie(pid)}


def _lan_endpoint(script: str, interface: str) -> tuple[str, int]:
    for entry in network_addresses():
        if entry.interface == interface:
            config = configparser.ConfigParser(interpolation=None)
            try:
                config.read(Path(script).with_name("setting.ini"))
                port = config.getint("TCP", "TcpBindPort", fallback=PICO_TCP_PORT)
            except (configparser.Error, ValueError) as exc:
                raise RuntimeError(f"无效的 PC Service TCP 端口配置：{exc}") from exc
            if not 0 < port <= 65535:
                raise RuntimeError(f"无效的 PC Service TCP 端口：{port}")
            return entry.address, port
    raise RuntimeError(f"XR 配置的网卡 {interface} 没有可用的局域网 IPv4 地址")


def _listening_on(address: str, interface: str, port: int, pids: Set[int]) -> bool:
    accepted = {f"{address}%{interface}:{port}", f"[::ffff:{address}]%{interface}:{port}"}
    try:
        result = subprocess.run(["ss", "-Hltnp", f"sport = :{port}"], capture_output=True, text=True, check=True)
    except (OSError, subprocess.CalledProcessError) as exc:
        raise RuntimeError(f"无法检查 PC Service 的 TCP {port} 监听：{exc}") from exc
    endpoints = {line.split()[3] for line in result.stdout.splitlines()
                 if any(int(pid) in pids for pid in re.findall(r"\bpid=(\d+)", line))}
    return bool(endpoints) and endpoints <= accepted


def start_pc_service(script: str, timeout_s: float = 3.0, log_path: Optional[str] = None, *,
                     network_interface: str = "") -> List[int]:
    """Start the service if needed; return the PIDs this call started ([] if already running).

    Its output (stdout and stderr of the script, and of the service if it keeps the descriptors) goes to log_path.

    Raises RuntimeError if the script is missing or the process does not appear in time."""
    pids = running_pids()
    endpoint = _lan_endpoint(script, network_interface) if network_interface else None
    if pids:
        if endpoint and not _listening_on(endpoint[0], network_interface, endpoint[1], pids):
            raise RuntimeError(f"PC Service 已运行，但未绑定 {network_interface}；请重启 PC Service")
        return []
    if not os.path.isfile(script):
        raise RuntimeError(
            f"[XR-005] XRoboToolkit PC Service not found: {script} (install the PC Service deb, "
            f"or set teleop.xr.start_pc_service: false)")
    with ExitStack() as stack:
        log = stack.enter_context(open(log_path, "ab")) if log_path else subprocess.DEVNULL
        env, fds = None, ()
        if endpoint:
            address, port = endpoint
            library = Path(get_package_prefix("xr_teleop")) / "lib/xr_teleop/libxrt_lan_bind.so"
            if not library.is_file():
                raise RuntimeError(f"缺少 PC Service 网卡绑定库 {library}，请重新编译 xr_teleop")
            bind_file = stack.enter_context(library.open("rb"))
            fds = (bind_file.fileno(),)
            previous = os.environ.get("LD_PRELOAD", "")
            env = dict(os.environ, FA_XRT_LAN_IP=address, FA_XRT_LAN_INTERFACE=network_interface,
                       FA_XRT_PICO_TCP_PORT=str(port),
                       LD_PRELOAD=f"/proc/self/fd/{fds[0]}" + (f":{previous}" if previous else ""))
            if log_path:
                log.write(f"Pico PC Service: {address}:{port} ({network_interface})\n".encode())
                log.flush()
        subprocess.Popen(["bash", script], stdin=subprocess.DEVNULL, stdout=log,
                         stderr=subprocess.STDOUT, start_new_session=True, env=env, pass_fds=fds)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pids = running_pids()
        if pids and (not endpoint or _listening_on(endpoint[0], network_interface, endpoint[1], pids)):
            return sorted(pids)
        time.sleep(0.2)
    if endpoint:
        stop_pc_service(sorted(pids))
        raise RuntimeError(f"PC Service 未在 {endpoint[0]}:{endpoint[1]} ({network_interface}) 上就绪；请查看 PC Service 日志")
    raise RuntimeError(
        f"{PROCESS_COMM} did not start within {timeout_s:.0f}s (desktop build needs DISPLAY; "
        f"try running {script} manually)")


def stop_pc_service(pids: List[int], timeout_s: float = 3.0) -> None:
    """Stop the given PIDs (SIGTERM, then SIGKILL)."""
    alive = [pid for pid in pids if pid in running_pids()]
    for pid in alive:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and any(pid in running_pids() for pid in alive):
        time.sleep(0.2)
    for pid in alive:
        if pid in running_pids():
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass


# ---- units ------------------------------------------------------------------------------------------------------------

PROXY_VARIABLES = ("http_proxy", "https_proxy", "all_proxy", "grpc_proxy",
                   "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "GRPC_PROXY")


class PcServiceDriver:
    """runService.sh daemonises the service and exits at once, so it is tracked by process name, not by group."""

    def __init__(self, script: str, network_interface: str = ""):
        self.script = script
        self.network_interface = network_interface

    def running(self) -> bool:
        return bool(running_pids())

    def pids(self) -> List[int]:
        return sorted(running_pids())

    def start(self, log_path: str) -> List[int]:
        return start_pc_service(self.script, log_path=log_path, network_interface=self.network_interface)

    def stop(self, pids: List[int]) -> None:
        stop_pc_service(pids)


def xr_units(cfg: Dict[str, Any]) -> Dict[str, Unit]:
    if teleop_type(cfg) != "xr":
        return {}
    xr = xr_settings(cfg)
    units: Dict[str, Unit] = {}
    if xr["start_pc_service"]:
        units["pc_service"] = Unit(name="pc_service", title="PC Service", cmd=(),
                                   driver=PcServiceDriver(xr["pc_service_script"], xr["network_interface"]))
    units["xr"] = Unit(
        name="xr",
        title="XR",
        cmd=("ros2", "run", "xr_teleop", "xr_bridge_node", "--ros-args",
             "-r", "__node:=xr_target_node",
             "-p", f"frame_timeout_ms:={int(xr['frame_timeout_ms'])}",
             "-p", f"reconnect_interval_s:={float(xr['reconnect_interval_s'])}",
             "-p", f"device_id:={json.dumps(xr['device_id'])}"),
        depends=("pc_service",) if "pc_service" in units else (),
        topics=("/teleop/left_ee_pose", "/teleop/connection_status"),
        # gRPC honors proxy variables on loopback, where the local PC Service listens.
        unset_env=PROXY_VARIABLES,
    )
    return units


# ---- readiness --------------------------------------------------------------------------------------------------------
# Is the XR bridge really receiving the headset? `up xr` used to print "已启动" as soon as the process existed, while the
# bridge may still be waiting: for the PC Service, or for a headset that is not connected to it.
# The bridge log tells: "PC Service 已连接" once the SDK reaches the service, "接收头显追踪数据" at the first frame.

READY, FAILED, PENDING = "ready", "failed", "pending"  # same values as robot_app.control_ready

SERVICE_CONNECTED = "SDK 事件: PC Service 已连接"
HEADSET_SELECTED = re.compile(r"头显选择：(\S+)")
HEADSET_DATA = re.compile(r"接收头显追踪数据，设备 ID: (\S+)")
ERROR_LINE = re.compile(r"\[XR-00\d\][^\n]*")
MAX_ERRORS = 2


class Verdict(NamedTuple):
    state: str          # READY | FAILED | PENDING
    reasons: List[str]  # READY: the headset id; PENDING: what it waits for; FAILED: why the bridge is gone


def judge(log_text: str, alive: bool) -> Verdict:
    """READY at the first headset frame; FAILED when the bridge exited; otherwise PENDING with what is missing."""
    data = HEADSET_DATA.search(log_text)
    if data and alive:
        return Verdict(READY, [data.group(1)])
    errors = list(dict.fromkeys(m.group(0).strip() for m in ERROR_LINE.finditer(log_text)))[-MAX_ERRORS:]
    if not alive:
        return Verdict(FAILED, ["XR 桥进程已退出"] + errors)
    selected = HEADSET_SELECTED.search(log_text)
    wanted = selected.group(1) if selected else "头显"
    if SERVICE_CONNECTED not in log_text:
        return Verdict(PENDING, ["桥还没有连上 PC Service（127.0.0.1:60061）"] + errors)
    return Verdict(PENDING, [f"桥已连上 PC Service，但没有收到头显发来的追踪数据（等待 {wanted}）"])
