"""What a later reader of logs/events.log needs to know about a run: code version, host, hardware selection."""
from __future__ import annotations

import socket
import subprocess
from typing import Any, Dict

ROBOT_FIELDS = ("entry", "hardware", "end_effector", "end_effector_bus")


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
