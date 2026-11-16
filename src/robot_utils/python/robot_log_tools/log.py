"""Logging for the Python processes of the robot software, in the same form as the C++ side.

    from robot_log_tools import log
    logger = log.get_logger("incident")
    logger.info("watching %s", path)
    log.fault(logger, "XR-006", "headset frames stopped for %d ms", 300)       # [WARN] ... [XR-006] headset frames ...

What it fixes is the form, so a line from Python reads like one from a C++ node and the incident watcher and the
fault code registry treat both alike:
  * a line is   [SEVERITY] [YYYY-mm-dd HH:MM:SS.mmm] [name]: message     (severity DEBUG INFO WARN ERROR FATAL)
  * a fault line carries its registry code first: "[XR-006] ..."          (log.fault checks the code is registered)
  * the log mode is the one of the run: ROBOT_LOG_MODE=debug shows DEBUG lines, otherwise INFO and above
Output goes to stdout, flushed per line, which the supervisor writes to logs/<unit>/.

Health of a hop that is Python (how often something happens, the longest silence) is counted by RateStats and
written one line per window by HealthReporter, in the form robot_log/health.hpp writes:
    stats = log.RateStats();  stats.record()                       # at every event, from any thread
    log.HealthReporter(logger, "my_unit", stats).start()           # [HEALTH] my_unit window=10.0s count=.. rate_hz=.. gap_max_ms=..
The hops of the chain that need it today (control cycle, XR bridge, target_generator, hardware read/write) are C++ and
use robot_log/health.hpp; no Python unit reports health yet.
"""
from __future__ import annotations

import logging
import os
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Callable, Optional, Set, TextIO

from robot_log_tools import fault_codes

LOG_MODE_ENV = "ROBOT_LOG_MODE"          # set for every unit by the supervisor; the C++ side reads the same variable
_SEVERITY = {logging.DEBUG: "DEBUG", logging.INFO: "INFO", logging.WARNING: "WARN", logging.ERROR: "ERROR",
             logging.CRITICAL: "FATAL"}

_registered: Optional[Set[str]] = None


def health_window_s() -> float:
    """10 s in normal mode, 1 s in debug (the same as robot_log::healthWindowSeconds())."""
    return 1.0 if log_mode() == "debug" else 10.0


def log_mode() -> str:
    """"debug" or "normal" (anything else counts as normal)."""
    return "debug" if os.environ.get(LOG_MODE_ENV, "").strip().lower() == "debug" else "normal"


class RosFormatter(logging.Formatter):
    """[INFO] [2026-11-13 10:00:00.123] [name]: message"""

    def format(self, record: logging.LogRecord) -> str:
        moment = datetime.fromtimestamp(record.created)
        text = (f"[{_SEVERITY.get(record.levelno, record.levelname)}] "
                f"[{moment:%Y-%m-%d %H:%M:%S}.{int(record.msecs):03d}] [{record.name}]: {record.getMessage()}")
        if record.exc_info:
            text += "\n" + self.formatException(record.exc_info)
        return text


class _FlushingHandler(logging.StreamHandler):
    """Flushes per line. Without a stream of its own it writes to whatever sys.stdout is at that moment (it can be
    replaced after the logger was made: a redirect, a test)."""

    def __init__(self, stream: Optional[TextIO] = None):
        super().__init__(stream or sys.stdout)
        self._follow_stdout = stream is None

    def emit(self, record: logging.LogRecord) -> None:
        if self._follow_stdout:
            self.stream = sys.stdout
        super().emit(record)
        self.flush()


def get_logger(name: str, stream: Optional[TextIO] = None) -> logging.Logger:
    """The logger of a component. Asking twice for the same name gives the same logger (no duplicated lines)."""
    logger = logging.getLogger(name)
    if not any(isinstance(h, _FlushingHandler) for h in logger.handlers):
        handler = _FlushingHandler(stream)
        handler.setFormatter(RosFormatter())
        logger.addHandler(handler)
        logger.propagate = False
    logger.setLevel(logging.DEBUG if log_mode() == "debug" else logging.INFO)
    return logger


def registered_codes() -> Set[str]:
    global _registered
    if _registered is None:
        try:
            _registered = {fault.code for fault in fault_codes.load()}
        except (OSError, fault_codes.FaultRegistryError):
            _registered = set()
    return _registered


def fault(logger: logging.Logger, code: str, message: str, *args: object, level: int = logging.WARNING) -> None:
    """Log a line that carries a fault code. A code that is not in the registry is still logged (a log call must not
    fail), with a note, so a typo shows up in the log and in the tests."""
    codes = registered_codes()
    note = "" if (not codes or code in codes) else " (unregistered fault code)"
    logger.log(level, f"[{code}] {message}{note}", *args)


# ---- health ---------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class RateSnapshot:
    count: int = 0
    rate_hz: float = 0.0
    gap_max_ms: float = 0.0


class RateStats:
    """Counts events of one kind: how many, and the longest silence between two. Safe from several threads."""

    def __init__(self, clock: Callable[[], float] = time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._count = 0
        self._last: Optional[float] = None
        self._gap_max = 0.0

    def record(self, now: Optional[float] = None) -> None:
        now = self._clock() if now is None else now
        with self._lock:
            if self._last is not None and now > self._last:
                self._gap_max = max(self._gap_max, now - self._last)
            self._last = now
            self._count += 1

    def take(self, window_s: float) -> RateSnapshot:
        """The numbers since the last take(), and a new window. The time of the last event is kept: a silence that
        spans two windows is still seen."""
        with self._lock:
            snap = RateSnapshot(self._count, self._count / window_s if window_s > 0 else 0.0, self._gap_max * 1000.0)
            self._count, self._gap_max = 0, 0.0
            return snap


def format_rate_line(name: str, window_s: float, snap: RateSnapshot, extra: str = "") -> str:
    """The line robot_log::formatRateLine() writes (the same text, tested against it)."""
    return (f"[HEALTH] {name} window={window_s:.1f}s count={snap.count} rate_hz={snap.rate_hz:.1f} "
            f"gap_max_ms={snap.gap_max_ms:.1f}" + (f" {extra}" if extra else ""))


class HealthReporter:
    """Writes one health line per window from a thread of its own (never from the code being measured).
    INFO while healthy; WARN with `code` when the longest silence of the window reached `gap_warn_ms`."""

    def __init__(self, logger: logging.Logger, name: str, stats: RateStats, extra: Optional[Callable[[], str]] = None,
                 gap_warn_ms: Optional[float] = None, code: Optional[str] = None, window_s: Optional[float] = None):
        self._logger, self._name, self._stats, self._extra = logger, name, stats, extra
        self._gap_warn_ms, self._code = gap_warn_ms, code
        self._window_s = health_window_s() if window_s is None else window_s
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def report_once(self) -> str:
        snap = self._stats.take(self._window_s)
        line = format_rate_line(self._name, self._window_s, snap, self._extra() if self._extra else "")
        if self._gap_warn_ms is not None and self._code and snap.gap_max_ms >= self._gap_warn_ms:
            fault(self._logger, self._code, "%s", line)
        else:
            self._logger.info("%s", line)
        return line

    def start(self) -> "HealthReporter":
        self._thread = threading.Thread(target=self._run, name=f"health-{self._name}", daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        while not self._stop.wait(self._window_s):
            self.report_once()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
