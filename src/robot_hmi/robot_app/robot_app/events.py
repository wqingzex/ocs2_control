"""What a later reader of logs/events.log needs to know about a run: code version, host, hardware selection."""
from __future__ import annotations

import socket
import subprocess
from typing import Any, Dict

def robot_fields(robot: Dict[str, Any]) -> Dict[str, Any]:
    """The selection a later reader of the log needs: entry, run mode, arm and end effector with its bus."""
    eef = robot.get("end_effector") or {}
    return {"entry": robot.get("entry"), "hardware": robot.get("hardware"),
            "arm": (robot.get("arm") or {}).get("type"),
            "end_effector": eef.get("type"), "end_effector_bus": (eef.get("interface") or {}).get("bus")}


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
            "robot": robot_fields(robot)}
