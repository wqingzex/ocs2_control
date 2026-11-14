"""The units of the XR headset: the PC Service daemon and the bridge node."""
from __future__ import annotations

import json
from typing import Any, Dict, List

from robot_runtime.units import Unit

from . import pc_service


PROXY_VARIABLES = ("http_proxy", "https_proxy", "all_proxy", "grpc_proxy",
                   "HTTP_PROXY", "HTTPS_PROXY", "ALL_PROXY", "GRPC_PROXY")


class PcServiceDriver:
    """runService.sh daemonises the service and exits at once, so it is tracked by process name, not by group."""

    def __init__(self, script: str):
        self.script = script

    def running(self) -> bool:
        return bool(pc_service.running_pids())

    def pids(self) -> List[int]:
        return sorted(pc_service.running_pids())

    def start(self, log_path: str) -> List[int]:
        return pc_service.start(self.script, log_path=log_path)

    def stop(self, pids: List[int]) -> None:
        pc_service.stop(pids)


def xr_units(cfg: Dict[str, Any]) -> Dict[str, Unit]:
    xr = cfg["xr"]
    units: Dict[str, Unit] = {}
    if xr["enable"] and xr["start_pc_service"]:
        units["pc_service"] = Unit(name="pc_service", title="PC Service", cmd=(),
                                   driver=PcServiceDriver(xr["pc_service_script"]))
    if xr["enable"]:
        units["xr"] = Unit(
            name="xr",
            title="VR",
            cmd=("ros2", "run", "xr_teleop", "xr_bridge_node", "--ros-args",
                 "-r", "__node:=xr_target_node",
                 "-p", f"frame_timeout_ms:={int(xr['frame_timeout_ms'])}",
                 "-p", f"reconnect_interval_s:={float(xr['reconnect_interval_s'])}",
                 "-p", f"device_id:={json.dumps(xr['device_id'])}",
                 "-p", f"control_scheme:={xr['control_scheme']}"),
            depends=("pc_service",) if "pc_service" in units else (),
            topics=("/teleop/left_ee_pose", "/teleop/connection_status"),
            # The bridge's gRPC client talks to the PC Service on 127.0.0.1. gRPC obeys http_proxy even for loopback
            # (no_proxy is not enough), so a proxy inherited from the shell swallows the connection.
            unset_env=PROXY_VARIABLES,
        )
    return units
