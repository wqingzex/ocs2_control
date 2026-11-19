"""Who started a unit and whether it still runs, and setup's own subprocess trees.

Run-state files <run_dir>/<name>.json ({pid, start_ticks, ...}) are shared by the code that starts units and the
code that only needs to know they are idle. OwnedDrivers owns setup's subprocesses, including descendants that
create new sessions."""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path
from types import SimpleNamespace
from typing import Any, Dict, List, Optional


def _read_stat(pid: int) -> Optional[List[str]]:
    """Fields of /proc/<pid>/stat after the command name (None if the process does not exist)."""
    try:
        with open(f"/proc/{pid}/stat", encoding="utf-8") as handle:
            return handle.read().rsplit(")", 1)[1].split()
    except (OSError, IndexError, ValueError):
        return None


def start_ticks(pid: int) -> Optional[int]:
    """Process start time (None if the process does not exist or is a zombie)."""
    fields = _read_stat(pid)
    if fields is None or fields[0] == "Z":
        return None
    return int(fields[19])  # field 22 overall: starttime


def group_alive(pgid: int) -> bool:
    """True if any non-zombie process is still in the process group."""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    for entry in os.listdir("/proc"):
        if entry.isdigit():
            fields = _read_stat(int(entry))
            if fields is not None and int(fields[2]) == pgid and fields[0] != "Z":
                return True
    return False


def state_path(run_dir: str, name: str) -> str:
    return os.path.join(run_dir, f"{name}.json")


def read_state(run_dir: str, name: str) -> Optional[Dict[str, Any]]:
    try:
        with open(state_path(run_dir, name), encoding="utf-8") as handle:
            state = json.load(handle)
    except (OSError, json.JSONDecodeError):
        return None
    return state if isinstance(state, dict) else None


def write_state(run_dir: str, name: str, state: Dict[str, Any]) -> None:
    tmp = state_path(run_dir, name) + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(state, handle, indent=1)
    os.replace(tmp, state_path(run_dir, name))


def clear_state(run_dir: str, name: str) -> None:
    try:
        os.remove(state_path(run_dir, name))
    except FileNotFoundError:
        pass


def is_running(state: Optional[Dict[str, Any]]) -> bool:
    """The unit of a state file runs while its leader (same start time) or any process of its group lives."""
    try:
        pid = int(state["pid"])
    except (KeyError, TypeError, ValueError):
        return False
    ticks = start_ticks(pid)
    if ticks is None:
        return group_alive(pid)  # leader gone: children may still be alive
    return ticks == state.get("start_ticks")  # different start time = PID reused


def wait_for_log(path, marker, alive, timeout_s, interval_s=0.1):
    """Wait until the log file contains marker. False when alive() turns false or the timeout passes first."""
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            if marker in Path(path).read_text(encoding='utf-8', errors='replace'):
                return True
        except OSError:
            pass
        if not alive() or time.monotonic() >= deadline:
            return False
        time.sleep(interval_s)


def process_stat(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return None if fields[0] == 'Z' else (int(fields[19]), int(fields[2]))
    except (OSError, IndexError, ValueError):
        return None


def require_idle(ws_root, names):
    run_dir = str(Path(ws_root) / 'run')
    for name in names:
        if is_running(read_state(run_dir, name)):
            raise ValueError('模块仍在运行，请先自行停止后再配置：' + name)


class OwnedDrivers:
    def __init__(self, log_root):
        self.log_root = Path(log_root)
        self.commands, self.entries = {}, {}

    def names(self):
        return list(self.entries)

    def configure(self, name, command):
        self.commands[name] = command

    def start(self, name, ready_log='', ready_timeout_s=20.0):
        """Start the command; with ready_log, return once the log shows it (or it exited, or the timeout passed)."""
        directory = self.log_root / name
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / (str(time.time_ns()) + '.log')
        with path.open('ab') as output:
            process = subprocess.Popen(self.commands[name], stdin=subprocess.DEVNULL,
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        self.entries[name] = {'process': process, 'log': path,
                              'tracked': {}, 'gone': False}
        current = process_stat(process.pid)
        if current:
            self.entries[name]['tracked'][process.pid] = current[0]
        if ready_log:
            wait_for_log(path, ready_log, lambda: self.is_running(name), ready_timeout_s)

    def is_running(self, name):
        entry = self.entries.get(name)
        if not entry or entry['gone']:
            return False
        entry['process'].poll()
        snapshot = {}
        for path in Path('/proc').iterdir():
            if path.name.isdigit():
                try:
                    fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
                    if fields[0] != 'Z':
                        snapshot[int(path.name)] = (int(fields[19]), int(fields[1]))
                except (OSError, IndexError, ValueError):
                    continue
        tracked = {pid: ticks for pid, ticks in entry['tracked'].items()
                   if pid in snapshot and snapshot[pid][0] == ticks}
        # Keep already-seen descendants even if they later become orphans.
        changed = True
        while changed:
            changed = False
            for pid, (ticks, parent) in snapshot.items():
                if parent in tracked and pid not in tracked:
                    tracked[pid] = ticks
                    changed = True
        entry['tracked'] = tracked
        entry['gone'] = not tracked
        return bool(tracked)

    def status(self, name):
        entry = self.entries.get(name)
        error = ''
        if entry:
            try:
                lines = entry['log'].read_text(errors='replace').splitlines()[-25:]
                error = next((line[-500:] for line in reversed(lines)
                              if '[error]' in line.lower() or 'error:' in line.lower()), '')
            except OSError:
                pass
        return SimpleNamespace(running=self.is_running(name), last_error=error)

    def stop(self, name):
        if name not in self.entries:
            return
        entry = self.entries[name]
        process = entry['process']
        # A driver killed while it is still streaming can come back without delivering frames (camera_setup uses
        # the same 10 s), so SIGINT gets the long grace and the kill comes last.
        for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 3), (signal.SIGKILL, 1)):
            if not self.is_running(name):
                break
            for pid, ticks in list(entry['tracked'].items()):
                current = process_stat(pid)
                if current and current[0] == ticks:
                    try:
                        os.kill(pid, sig)
                    except ProcessLookupError:
                        pass
            deadline = time.monotonic() + wait
            while self.is_running(name) and time.monotonic() < deadline:
                time.sleep(.05)
        process.poll()
        if self.is_running(name):
            raise RuntimeError(f'相机预览进程未完全退出：{name}')
