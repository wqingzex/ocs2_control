"""Teleop configuration: the xr section and robot.entry on top of robot_config's."""
from __future__ import annotations

import re
from typing import Any, Callable, Dict, Iterable, Optional

from robot_config import config as common
from robot_config.config import ConfigError, Extension, expect
from xr_teleop.config import EXTENSION as XR_EXTENSION

# entry -> (whole-robot launch file in robot_startup, robot name); same entries as the former quick_start menu
ENTRIES = {
    "split_body": ("split_body.launch.py", "fa_w2"),
    "dual_arm": ("dual_arm.launch.py", "m6_ccs"),
    "whole_body": ("whole_body.launch.py", "fa_w2"),
}

# entry -> the arm drivers that have a robot model there (the registered drivers are robot_config.config.ARM_TYPES)
ENTRY_ARM_TYPES = {"split_body": ("marvin",), "dual_arm": ("marvin",), "whole_body": ("marvin",)}

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
    arm = cfg["robot"]["arm"]["type"]
    expect(arm in ENTRY_ARM_TYPES[entry],
           f"'robot.arm.type' {arm!r} has no robot model for 'robot.entry' {entry!r} (supported: {list(ENTRY_ARM_TYPES[entry])})")
    _check_log(cfg["log"])
    _check_record(cfg["record"])


EXTENSION = Extension(checks=(_check_teleop,)).merge(XR_EXTENSION)


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


__all__ = ["ConfigError", "ENTRIES", "load", "load_lenient", "validate"]
