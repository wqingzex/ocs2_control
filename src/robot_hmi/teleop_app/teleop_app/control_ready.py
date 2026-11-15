"""Did the controller launch really come up? `up control` used to print "已启动" as soon as the process existed, even
when every controller spawner had died (a second controller_manager on the network, a hardware that does not answer).

The launch log tells: each controller is loaded by a spawner process, which ends with "process has finished cleanly"
once the controller is active, or with "process has died" when it could not be loaded or activated."""
from __future__ import annotations

import re
import time
from pathlib import Path
from typing import Callable, List, NamedTuple

READY, FAILED, PENDING = "ready", "failed", "pending"
DEFAULT_TIMEOUT_S = 120.0  # the real robot needs about 10 s, hand drivers retry for much longer when they do not answer
POLL_INTERVAL_S = 0.5

SPAWNER_STARTED = re.compile(r"\[INFO\] \[(spawner-\d+)\]: process started")
SPAWNER_FINISHED = re.compile(r"\[INFO\] \[(spawner-\d+)\]: process has finished cleanly")
SPAWNER_DIED = re.compile(r"\[ERROR\] \[(spawner-\d+)\]: process has died")
NODE_DIED = re.compile(r"\[ERROR\] \[(ros2_control_node-\d+)\]: process has died")
SPAWNER_REASON = re.compile(r"\[spawner-\d+\] \[(?:ERROR|FATAL)\][^\n]*")
NODE_REASON = re.compile(r"\[ros2_control_node-\d+\] (?:Exception thrown[^\n]*|\[ERROR\][^\n]*)")
MAX_REASONS = 4


class Verdict(NamedTuple):
    state: str          # READY | FAILED | PENDING
    reasons: List[str]  # log lines that explain a failure


def _reasons(text: str, died: List[str]) -> List[str]:
    """The log lines that say why: errors of the spawners and the controller node's own errors."""
    lines: List[str] = []
    # In log order: the first error is the cause (a hand that does not answer), the spawner errors only follow from it.
    for match in sorted(list(SPAWNER_REASON.finditer(text)) + list(NODE_REASON.finditer(text)), key=lambda m: m.start()):
        line = match.group(0).strip()
        if line not in lines:
            lines.append(line)
    return lines[:MAX_REASONS] or [f"{name} 退出" for name in died]


def judge(log_text: str, alive: bool) -> Verdict:
    """READY when every spawner that started finished cleanly; FAILED when one died or the launch is gone."""
    started = set(SPAWNER_STARTED.findall(log_text))
    finished = set(SPAWNER_FINISHED.findall(log_text))
    died = sorted(set(SPAWNER_DIED.findall(log_text)) | set(NODE_DIED.findall(log_text)))
    if died:
        return Verdict(FAILED, _reasons(log_text, died))
    if not alive:
        return Verdict(FAILED, ["运控进程已退出"] + _reasons(log_text, []))
    if started and started <= finished:
        return Verdict(READY, [])
    return Verdict(PENDING, [])


def wait_ready(log_path: str, alive: Callable[[], bool], timeout_s: float = DEFAULT_TIMEOUT_S,
               sleep: Callable[[float], None] = time.sleep, now: Callable[[], float] = time.monotonic,
               judge: Callable[[str, bool], Verdict] = judge) -> Verdict:
    """Poll the log until the verdict is not PENDING; PENDING is returned when the timeout passes.
    judge decides from the log text (the controller launch by default, the XR bridge for xr_teleop.xr_ready)."""
    deadline = now() + timeout_s
    while True:
        try:
            text = Path(log_path).read_text(encoding="utf-8", errors="replace")
        except OSError:
            text = ""
        verdict = judge(text, alive())
        if verdict.state != PENDING or now() >= deadline:
            return verdict
        sleep(POLL_INTERVAL_S)
