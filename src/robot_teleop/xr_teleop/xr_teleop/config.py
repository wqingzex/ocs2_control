"""The xr section of config/robot.local.yaml: its checks (the keys and their values are in robot_config's template)."""
from __future__ import annotations

import re
from typing import Any, Dict

from robot_config.config import Extension, expect, xr_settings

NAME_RE = re.compile(r"^[A-Za-z0-9_.-]*$")  # network interface names

def _check_xr(cfg: Dict[str, Any]) -> None:
    xr = xr_settings(cfg)
    expect(xr["network_mode"] in ("ethernet", "wifi"), "'teleop.xr.network_mode' must be 'ethernet' (网线) or 'wifi'")
    expect(bool(NAME_RE.fullmatch(xr["network_interface"])), "'teleop.xr.network_interface' is invalid")
    expect(not any(ord(c) < 32 or ord(c) == 127 for c in xr["device_id"]),
           "'teleop.xr.device_id' contains control characters")
    expect(xr["frame_timeout_ms"] > 0, "'teleop.xr.frame_timeout_ms' must be > 0")
    expect(xr["reconnect_interval_s"] > 0, "'teleop.xr.reconnect_interval_s' must be > 0")


EXTENSION = Extension(checks=(_check_xr,))
