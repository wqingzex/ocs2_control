"""One command for "something went wrong, collect what is needed": logs/diag/diag_<stamp>.tar.gz and, beside it,
diag_<stamp>.report.txt (the same report that is inside, to read without unpacking).

    ./scripts/start.sh diag [--minutes N] [reason]

It gathers the last N minutes (default 10) before now: the lines of every unit log, the fault codes and the health lines
in them, and what each module received and sent (the flight recorder rings, which survive a crashed module), analysed
for the first hop that went wrong (robot_log_tools.evidence). Besides that: the event timeline, the run states, the
config in use, the machine (clock, memory, disk, network counters, kernel messages). console.token is never included.
Nothing runs all the time; this is done by hand, after the fact, and costs nothing while teleoperating."""
from __future__ import annotations

import io
import os
import shutil
import subprocess
import tarfile
import tempfile
from datetime import datetime
from typing import Callable, List, Optional

from robot_log_tools import evidence

from .events import git_revision

MAX_FILE_BYTES = 64 * 1024 * 1024   # a larger file goes in cut to its end
MACHINE_COMMANDS = (["date"], ["timedatectl", "status"], ["free", "-m"], ["df", "-h", "."], ["ip", "-s", "link"],
                    ["bash", "-c", "dmesg 2>&1 | tail -n 100"])


def _run(cmd: List[str], timeout: float = 5.0) -> str:
    try:
        done = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        return (done.stdout + done.stderr).strip()
    except (OSError, subprocess.SubprocessError) as exc:
        return f"(unavailable: {exc})"


def clock_warning(run: Callable[[List[str]], str] = _run) -> Optional[str]:
    """A note when the system clock is not synchronised. The modules are lined up by the wall clock (logs, flight
    recorder rings), so a clock that jumps makes the times of a diagnosis wrong. None when synchronised or unknown
    (no timedatectl, as in a container)."""
    answer = run(["timedatectl", "show", "-p", "NTPSynchronized", "--value"]).strip().lower()
    if answer == "no":
        return ("系统时钟没有和时间服务器同步（NTPSynchronized=no）：各模块的日志和记录靠系统时钟对齐，"
                "时钟被调整后诊断里的时间会错位。可检查 timedatectl status 和网络时间服务")
    return None


def ros_info() -> str:
    return "\n".join(f"$ {' '.join(cmd)}\n{_run(cmd)}\n"
                     for cmd in (["ros2", "node", "list"], ["ros2", "topic", "list"]))


def machine_info() -> str:
    return "\n".join(f"$ {' '.join(cmd)}\n{_run(cmd)}\n" for cmd in MACHINE_COMMANDS)


def _add_bytes(tar: tarfile.TarFile, arcname: str, data: bytes) -> None:
    item = tarfile.TarInfo(arcname)
    item.size, item.mtime = len(data), int(datetime.now().timestamp())
    tar.addfile(item, io.BytesIO(data))


def _add_file(tar: tarfile.TarFile, path: str, arcname: str, limit: int = MAX_FILE_BYTES) -> None:
    try:
        with open(path, "rb") as handle:
            size = os.fstat(handle.fileno()).st_size
            if size > limit:
                handle.seek(size - limit)
            _add_bytes(tar, arcname, handle.read(limit))
    except OSError:
        pass  # vanished or unreadable: the rest of the bundle is still worth having


def report_path(bundle: str) -> str:
    return bundle[: -len(".tar.gz")] + ".report.txt"


def make_bundle(ws_root: str, config_path: str, minutes: float = evidence.DEFAULT_MINUTES, reason: str = "",
                ros: Optional[Callable[[], str]] = None, machine: Optional[Callable[[], str]] = None) -> str:
    """Build the bundle; returns its path (the report is beside it, see report_path)."""
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_dir, name = os.path.join(ws_root, "logs"), f"diag_{stamp}"
    out_dir = os.path.join(log_dir, "diag")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, name + ".tar.gz")
    work = tempfile.mkdtemp(prefix=".work_", dir=out_dir)
    try:
        report = evidence.gather(log_dir, work, minutes, reason)
        with tarfile.open(path, "w:gz") as tar:
            info = [f"time: {datetime.now().isoformat(timespec='seconds')}", f"git: {git_revision(ws_root)}",
                    f"config: {config_path}", f"reason: {reason}", f"window: last {minutes:g} minutes", "",
                    "$ git status --short", _run(["git", "-C", ws_root, "status", "--short"]), "",
                    (machine or machine_info)(), "", (ros or ros_info)()]
            _add_bytes(tar, f"{name}/info.txt", "\n".join(info).encode("utf-8"))
            for root, _dirs, files in os.walk(work):
                for file in sorted(files):
                    full = os.path.join(root, file)
                    _add_file(tar, full, f"{name}/{os.path.relpath(full, work)}")
            run_dir = os.path.join(ws_root, "run")
            for state in sorted(os.listdir(run_dir)) if os.path.isdir(run_dir) else []:
                _add_file(tar, os.path.join(run_dir, state), f"{name}/run/{state}", 64 * 1024)
            _add_file(tar, config_path, f"{name}/config/{os.path.basename(config_path)}", 1024 * 1024)
    finally:
        shutil.rmtree(work, ignore_errors=True)
    with open(report_path(path), "w", encoding="utf-8") as handle:
        handle.write(report)
    return path
