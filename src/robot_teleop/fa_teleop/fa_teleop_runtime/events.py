"""logs/events.log: one JSON line per event (unit start / stop / exit, mode changes) so the per-unit logs can be
lined up in time. Events are rare, so every write opens, appends and closes: nothing stays open or buffered,
and a full disk or a read-only log directory never stops teleoperation."""
from __future__ import annotations

import os
import re
import socket
import subprocess
from typing import Any, Dict

from robot_log_tools.events import EventLog  # noqa: F401 - the timeline is logging infrastructure, shared with the log keeper

KEEP_UNIT_LOGS = 20          # newest <stamp>.log files kept per unit
_STAMPED = re.compile(r"^\d{8}_\d{6}\.log$")
ROBOT_FIELDS = ("entry", "hardware", "hand", "hand_drive")


def prune_unit_logs(unit_dir: str, keep: int = KEEP_UNIT_LOGS) -> None:
    """Delete the oldest <stamp>.log files of one unit beyond `keep` (latest.log is a link and stays)."""
    try:
        stamped = sorted(name for name in os.listdir(unit_dir) if _STAMPED.match(name))
    except OSError:
        return
    for name in stamped[:-keep] if keep > 0 else stamped:
        try:
            os.remove(os.path.join(unit_dir, name))
        except OSError:
            pass


def git_revision(root: str) -> str:
    def git(*args: str) -> str:
        return subprocess.run(["git", "-C", root, *args], capture_output=True, text=True, timeout=3).stdout.strip()
    try:
        revision = git("rev-parse", "--short", "HEAD")
        return revision + (" +modified" if git("status", "--porcelain") else "") if revision else "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def session_info(ws_root: str, cfg: Dict[str, Any]) -> Dict[str, Any]:
    """What a later reader needs to know about this run: code version, host and the hardware selection."""
    robot = cfg.get("robot") or {}
    return {"git": git_revision(ws_root), "host": socket.gethostname(),
            "robot": {key: robot.get(key) for key in ROBOT_FIELDS}}
