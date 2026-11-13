"""Start / stop the XRoboToolkit PC Service (runService.sh daemonises it and exits at once).

Started only if not running; on shutdown only the PIDs we started are stopped."""
from __future__ import annotations

import os
import signal
import subprocess
import time
from typing import List, Optional, Set

# Linux comm is truncated to 15 characters: RoboticsServiceProcess -> RoboticsService
PROCESS_COMM = "RoboticsService"


def running_pids() -> Set[int]:
    result = subprocess.run(["pgrep", "-x", PROCESS_COMM], capture_output=True, text=True, check=False)
    return {int(pid) for pid in result.stdout.split()}


def start(script: str, timeout_s: float = 3.0, log_path: Optional[str] = None) -> List[int]:
    """Start the service if needed; return the PIDs this call started ([] if already running).

    Its output (stdout and stderr of the script, and of the service if it keeps the descriptors) goes to log_path.

    Raises RuntimeError if the script is missing or the process does not appear in time."""
    if running_pids():
        return []
    if not os.path.isfile(script):
        raise RuntimeError(
            f"[XR-005] XRoboToolkit PC Service not found: {script} (install the PC Service deb, "
            f"or set xr.start_pc_service: false)")
    if log_path:
        with open(log_path, "ab") as log:
            subprocess.Popen(["bash", script], stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
    else:
        subprocess.Popen(["bash", script], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        pids = running_pids()
        if pids:
            return sorted(pids)
        time.sleep(0.2)
    raise RuntimeError(
        f"{PROCESS_COMM} did not start within {timeout_s:.0f}s (desktop build needs DISPLAY; "
        f"try running {script} manually)")


def stop(pids: List[int], timeout_s: float = 3.0) -> None:
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
