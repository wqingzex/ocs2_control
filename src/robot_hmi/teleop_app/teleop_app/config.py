"""Teleop configuration: the xr / joystick / navigation sections and robot.entry on top of w2_payload's."""
from __future__ import annotations

import re
from typing import Any, Callable, Dict, Iterable, Optional

from w2_payload import config as common
from w2_payload.config import ConfigError, Extension, expect
from xr_teleop.config import DEFAULTS as XR_DEFAULTS, EXTENSION as XR_EXTENSION

JOYSTICK_RE = re.compile(r"^(/dev/input/js\d+)?$")
NAME_RE = re.compile(r"^[A-Za-z0-9_.-]*$")  # map ids and ROS package names

# entry -> (whole-robot launch file in robot_bringup, robot name); same entries as the former quick_start menu
ENTRIES = {
    "split_body": ("split_body.launch.py", "fiveages_w2"),
    "dual_arm": ("dual_arm.launch.py", "m6_ccs"),
    "whole_body": ("whole_body.launch.py", "fiveages_w2"),
}

SECTION_DEFAULTS: Dict[str, Any] = {
    "joystick": {
        "device": "",  # "" = let joystick_teleop pick; or /dev/input/jsN
    },
    "log": {
        "mode": "normal",    # normal | debug: see src/robot_log/README.md; fixed for a run, change it and restart
    },
    "record": {
        "dir": "",           # "" = <workspace>/bags
        "extra_topics": [],  # added to every recording, e.g. ["/tf"]
    },
    "navigation": {
        "robot": "fiveages_w2",
        "map": "",                  # "" = the map flagged default
        "map_only": False,          # True: static map only, no point cloud processing
        "map_config_package": "",   # "" = search the standard chain
        "nav_config_package": "",
    },
}
DEFAULTS: Dict[str, Any] = {**common.DEFAULTS, **SECTION_DEFAULTS, **XR_DEFAULTS}
TOPIC_RE = re.compile(r"^/[A-Za-z0-9_]+(/[A-Za-z0-9_]+)*$")


def _check_record(record: Dict[str, Any]) -> None:
    expect(not any(ord(c) < 32 or ord(c) == 127 for c in record["dir"]), "'record.dir' contains control characters")
    topics = record["extra_topics"]
    expect(isinstance(topics, list) and all(isinstance(t, str) and TOPIC_RE.match(t) for t in topics),
           f"'record.extra_topics' must be a list of topic names like /tf, got {topics!r}")


def _check_log(log: Dict[str, Any]) -> None:
    expect(log["mode"] in ("normal", "debug"), f"'log.mode' must be 'normal' or 'debug', got {log['mode']!r}")


def _check_teleop(cfg: Dict[str, Any]) -> None:
    entry = cfg["robot"]["entry"]
    expect(entry in ENTRIES, f"'robot.entry' must be one of {list(ENTRIES)}, got {entry!r}")
    device = cfg["joystick"]["device"]
    expect(bool(JOYSTICK_RE.match(device)), f"'joystick.device' must be empty or /dev/input/jsN, got {device!r}")
    for key in ("robot", "map", "map_config_package", "nav_config_package"):
        value = cfg["navigation"][key]
        expect(bool(NAME_RE.match(value)), f"'navigation.{key}' has invalid characters: {value!r}")
    _check_log(cfg["log"])
    _check_record(cfg["record"])


EXTENSION = Extension(defaults=SECTION_DEFAULTS, checks=(_check_teleop,)).merge(XR_EXTENSION)


def _edit(overrides: Iterable[str]) -> Optional[Callable[[Dict[str, Any]], Dict[str, Any]]]:
    assignments = list(overrides)
    if not assignments:
        return None
    from .overrides import apply_overrides
    return lambda raw: apply_overrides(raw, assignments)


def validate(cfg: Dict[str, Any]) -> Dict[str, Any]:
    return common.validate(cfg, EXTENSION)


def load(path: str, overrides: Iterable[str] = ()) -> Dict[str, Any]:
    """Read and validate; overrides are --set assignments for this run only."""
    return common.load(path, EXTENSION, _edit(overrides))


def load_lenient(path: str, overrides: Iterable[str] = ()) -> Dict[str, Any]:
    return common.load_lenient(path, EXTENSION, _edit(overrides))


__all__ = ["ConfigError", "DEFAULTS", "ENTRIES", "JOYSTICK_RE", "load", "load_lenient", "validate"]
