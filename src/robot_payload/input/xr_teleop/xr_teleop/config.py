"""The xr section of config/robot.local.yaml: defaults and checks. The headset, PC Service and network belong here."""
from __future__ import annotations

import re
from typing import Any, Dict

from fa_w2_payload.config import Extension, expect

NAME_RE = re.compile(r"^[A-Za-z0-9_.-]*$")  # network interface names

DEFAULTS: Dict[str, Any] = {
    "xr": {
        "enable": True,
        "start_pc_service": True,
        "pc_service_script": "/opt/apps/roboticsservice/runService.sh",
        "frame_timeout_ms": 300,
        "reconnect_interval_s": 3.0,
        "network_mode": "ethernet",
        "network_interface": "",
        "device_id": "",
    },
}


def _check_xr(cfg: Dict[str, Any]) -> None:
    xr = cfg["xr"]
    expect(xr["network_mode"] in ("ethernet", "wifi"), "'xr.network_mode' must be 'ethernet' (网线) or 'wifi'")
    expect(bool(NAME_RE.fullmatch(xr["network_interface"])), "'xr.network_interface' is invalid")
    expect(not any(ord(c) < 32 or ord(c) == 127 for c in xr["device_id"]),
           "'xr.device_id' contains control characters")
    expect(xr["frame_timeout_ms"] > 0, "'xr.frame_timeout_ms' must be > 0")
    expect(xr["reconnect_interval_s"] > 0, "'xr.reconnect_interval_s' must be > 0")


EXTENSION = Extension(defaults=DEFAULTS, checks=(_check_xr,))
