"""Start / stop / inspect units as process groups; state in <state_dir>/<unit>.json survives restarts.

Logs: <log_dir>/<unit>/<YYYYmmdd_HHMMSS>.log + latest.log. Stop: SIGINT to the group, then SIGTERM, SIGKILL."""
from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Callable, Dict, List, Optional

from robot_log_tools.events import EventLog

from . import processes as run_state
from .events import prune_unit_logs
from .units import Unit, UnitSet

# Lines that look like errors but are expected on shutdown (exit code -2 = stopped by SIGINT).
_NOISE = ("pal_statistics", "exit code -2")
# Every unit logs the same way: severity, wall-clock date and time (to the millisecond) and node name on each line,
# no colour codes, no buffering. rcutils has no time-of-day-only token, so the date comes with it.
# Defaults only: a value already in the environment or set by the unit wins.
LOG_ENV = {"RCUTILS_CONSOLE_OUTPUT_FORMAT": "[{severity}] [{date_time_with_ms}] [{name}]: {message}",
           "RCUTILS_COLORIZED_OUTPUT": "0", "PYTHONUNBUFFERED": "1"}


class UnitError(RuntimeError):
    """A unit could not be started or stopped (message is user facing)."""


@dataclass
class UnitStatus:
    name: str
    title: str
    running: bool
    pid: Optional[int] = None
    started_at: Optional[str] = None
    uptime_s: Optional[float] = None
    log: Optional[str] = None
    last_error: Optional[str] = None

    def to_dict(self) -> Dict:
        return asdict(self)



class Supervisor:
    def __init__(self, units: UnitSet, state_dir: str, log_dir: str, env: Optional[Dict[str, str]] = None):
        self.units = units
        self.state_dir = state_dir
        self.log_dir = log_dir
        self.env = env
        self._children: Dict[str, subprocess.Popen] = {}  # reaped by poll() in long-lived callers
        self._stopping: set = set()  # units being stopped on purpose: their exit is not an incident
        self._exit_lock = threading.Lock()  # several console polls can notice the same exit at once
        os.makedirs(state_dir, exist_ok=True)
        os.makedirs(log_dir, exist_ok=True)
        self.events = EventLog(log_dir)

    # ---- state files ------------------------------------------------------------------------
    def _read_state(self, name: str) -> Optional[Dict]:
        return run_state.read_state(self.state_dir, name)

    def _write_state(self, name: str, state: Dict) -> None:
        run_state.write_state(self.state_dir, name, state)

    def _clear_state(self, name: str) -> None:
        run_state.clear_state(self.state_dir, name)

    # ---- queries -------------------------------------------------------------------------------
    def is_running(self, name: str) -> bool:
        unit = self.units[name]
        child = self._children.get(name)  # may be popped by stop() in another thread
        if child is not None:
            child.poll()
        if unit.driver is not None:
            return unit.driver.running()
        return run_state.is_running(self._read_state(name))

    def status(self, name: str) -> UnitStatus:
        unit = self.units[name]
        state = self._read_state(name) or {}
        running = self.is_running(name)
        if state and not running and unit.driver is None:
            self._note_exit(name, state)
        result = UnitStatus(name=name, title=unit.title, running=running, log=state.get("log"),
                            last_error=self.last_error(name))
        if unit.driver is not None:
            pids = sorted(unit.driver.pids())
            result.pid = pids[0] if pids else None
            return result
        if running:
            result.pid = state.get("pid")
            result.started_at = state.get("started_at")
            result.uptime_s = round(time.time() - state.get("started_epoch", time.time()), 1)
        return result

    def _note_exit(self, name: str, state: Dict) -> None:
        """A unit that is gone although nobody stopped it: record it once, with its exit code and last error."""
        with self._exit_lock:
            state = self._read_state(name) or {}  # re-read: another poll may have recorded it meanwhile
            if not state or state.get("exit_recorded") or name in self._stopping:
                return
            child = self._children.get(name)
            code = child.poll() if child is not None else None  # only known to the process that started it
            try:
                signal_name = signal.Signals(-code).name if code is not None and code < 0 else None
            except ValueError:
                signal_name = None
            self.events.record("exit", name, pid=state.get("pid"), code=code, signal=signal_name,
                               last_error=self.last_error(name))
            self._write_state(name, {**state, "exit_recorded": True})

    def status_all(self) -> List[UnitStatus]:
        return [self.status(n) for n in self.units.names()]

    def log_path(self, name: str) -> Optional[str]:
        path = os.path.join(self.log_dir, name, "latest.log")
        return path if os.path.exists(path) else None

    def log_tail(self, name: str, lines: int = 200) -> List[str]:
        path = self.log_path(name)
        if not path:
            return []
        with open(path, "rb") as handle:
            handle.seek(0, os.SEEK_END)
            handle.seek(max(0, handle.tell() - 256 * 1024))
            data = handle.read().decode("utf-8", errors="replace")
        return data.splitlines()[-lines:]

    def last_error(self, name: str) -> Optional[str]:
        for line in reversed(self.log_tail(name, 400)):
            if ("[ERROR]" in line or "Traceback" in line or "process has died" in line) and \
                    not any(n in line for n in _NOISE):
                return line.strip()[:300]
        return None

    # ---- start / stop --------------------------------------------------------------------------
    def start(self, name: str) -> UnitStatus:
        unit = self.units[name]
        if self.is_running(name):
            return self.status(name)
        missing = [d for d in unit.depends if d in self.units and not self.is_running(d)]
        if missing and unit.on_demand:
            titles = "、".join(self.units[d].title for d in missing)
            raise UnitError(f"{unit.title} 需要先启动{titles}（不会自动启动）")
        if missing:
            raise UnitError(f"{unit.title} 依赖的模块未运行: {', '.join(missing)}（先启动它们，或用 up 自动按依赖启动）")
        if unit.driver is not None:
            log_path = self._new_log_path(name)
            try:
                owned = unit.driver.start(log_path)
            except RuntimeError as exc:
                raise UnitError(str(exc)) from exc
            logged = bool(owned) and os.path.exists(log_path)      # nothing was written if it was running already
            if logged:
                self._link_latest(name, log_path)
            self._write_state(name, {"owned_pids": owned, "log": log_path if logged else None,
                                     "started_at": datetime.now().isoformat(timespec="seconds")})
            self.events.record("start", name, pids=owned, log=log_path if logged else None)
            return self.status(name)
        status = self._spawn(unit)
        if unit.ready_log and status.log:
            run_state.wait_for_log(status.log, unit.ready_log, lambda: self.is_running(name), unit.ready_timeout_s)
        return self.status(name)

    def _new_log_path(self, name: str) -> str:
        """logs/<unit>/<stamp>.log for a new run (the directory made, the oldest runs removed)."""
        unit_log_dir = os.path.join(self.log_dir, name)
        os.makedirs(unit_log_dir, exist_ok=True)
        prune_unit_logs(unit_log_dir)
        return os.path.join(unit_log_dir, f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.log")

    def _link_latest(self, name: str, log_path: str) -> None:
        latest = os.path.join(os.path.dirname(log_path), "latest.log")
        if os.path.lexists(latest):
            os.remove(latest)
        os.symlink(os.path.basename(log_path), latest)

    def _spawn(self, unit: Unit) -> UnitStatus:
        log_path = self._new_log_path(unit.name)
        env = dict(LOG_ENV)
        env.update(self.env if self.env is not None else os.environ)
        env.update(dict(unit.env))
        for name in unit.unset_env:
            env.pop(name, None)
        with open(log_path, "ab") as log:
            try:
                proc = subprocess.Popen(list(unit.cmd), stdin=subprocess.DEVNULL, stdout=log,
                                        stderr=subprocess.STDOUT, start_new_session=True, env=env)
            except FileNotFoundError as exc:
                raise UnitError(f"{unit.title} 启动失败：找不到命令 {unit.cmd[0]}（是否已 source ROS 工作区？）") from exc
        self._link_latest(unit.name, log_path)
        self._children[unit.name] = proc
        self._write_state(unit.name, {
            "pid": proc.pid, "start_ticks": run_state.start_ticks(proc.pid), "log": log_path,
            "cmd": list(unit.cmd), "started_at": datetime.now().isoformat(timespec="seconds"),
            "started_epoch": time.time(),
        })
        self.events.record("start", unit.name, pid=proc.pid, log=log_path)
        return self.status(unit.name)

    def stop(self, name: str, timeout_s: float = 10.0,
             pre_stop: Optional[Callable[[str], None]] = None) -> UnitStatus:
        """Stop a unit. pre_stop(name) runs first while it is still running (e.g. FSM -> HOLD)."""
        was_running = self.is_running(name)
        self._stopping.add(name)
        try:
            status = self._stop(name, timeout_s, pre_stop)
        finally:
            self._stopping.discard(name)
        if was_running:
            self.events.record("stop", name)
        return status

    def _stop(self, name: str, timeout_s: float, pre_stop: Optional[Callable[[str], None]]) -> UnitStatus:
        unit = self.units[name]
        if pre_stop is not None and self.is_running(name):
            pre_stop(name)
        if unit.driver is not None:
            state = self._read_state(name) or {}
            unit.driver.stop(state.get("owned_pids", []))  # only what we started
            self._clear_state(name)
            return self.status(name)
        state = self._read_state(name)
        if state and self.is_running(name):
            pgid = state["pid"]
            for sig, wait in ((signal.SIGINT, timeout_s), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
                try:
                    os.killpg(pgid, sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + wait
                while time.monotonic() < deadline and run_state.group_alive(pgid):
                    child = self._children.get(name)
                    if child is not None:
                        child.poll()
                    time.sleep(0.1)
                if not run_state.group_alive(pgid):
                    break
            if run_state.group_alive(pgid):
                raise UnitError(f"{unit.title} 停止失败：进程组 {pgid} 仍在运行")
        child = self._children.pop(name, None)
        if child is not None:
            child.poll()
        self._clear_state(name)
        return self.status(name)

    def up(self, names: Optional[List[str]] = None) -> List[UnitStatus]:
        return [self.start(n) for n in self.units.launch_order(names)]

    def down(self, names: Optional[List[str]] = None,
             pre_stop: Optional[Callable[[str], None]] = None) -> List[UnitStatus]:
        return [self.stop(n, pre_stop=pre_stop) for n in self.units.stop_order(names)]
