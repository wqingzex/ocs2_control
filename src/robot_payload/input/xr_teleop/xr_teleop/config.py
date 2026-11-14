"""The xr section of config/robot.local.yaml: defaults and checks. The headset, PC Service and network belong here."""
from __future__ import annotations

import re
from typing import Any, Dict

from w2_payload.config import Extension, expect

NAME_RE = re.compile(r"^[A-Za-z0-9_.-]*$")  # network interface names

DEFAULTS: Dict[str, Any] = {
    "xr": {
        "enable": True,
        "start_pc_service": True,
        "pc_service_script": "/opt/apps/roboticsservice/runService.sh",
        "frame_timeout_ms": 300,
        "reconnect_interval_s": 3.0,
        # legacy: same events as the former xr_target_node | simple: single keys, use together
        # with vr_control_scheme: simple in the robot's teleop_command.yaml (grip clutch)
        "control_scheme": "legacy",
        "network_mode": "ethernet",
        "network_interface": "",
        "device_id": "",
    },
}
# Renamed fields: old configs keep working (xr.button_profile is now xr.control_scheme).
RENAMED = {"xr": {"button_profile": "control_scheme"}}


def _check_xr(cfg: Dict[str, Any]) -> None:
    xr = cfg["xr"]
    expect(xr["network_mode"] in ("ethernet", "wifi"), "'xr.network_mode' must be 'ethernet' (网线) or 'wifi'")
    expect(bool(NAME_RE.fullmatch(xr["network_interface"])), "'xr.network_interface' is invalid")
    expect(not any(ord(c) < 32 or ord(c) == 127 for c in xr["device_id"]),
           "'xr.device_id' contains control characters")
    expect(xr["frame_timeout_ms"] > 0, "'xr.frame_timeout_ms' must be > 0")
    expect(xr["reconnect_interval_s"] > 0, "'xr.reconnect_interval_s' must be > 0")
    expect(xr["control_scheme"] in ("legacy", "simple"),
           f"'xr.control_scheme' must be 'legacy' or 'simple', got {xr['control_scheme']!r}")


EXTENSION = Extension(defaults=DEFAULTS, renamed=RENAMED, checks=(_check_xr,))
