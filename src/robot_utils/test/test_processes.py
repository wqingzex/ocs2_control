"""Run-state files: written by whoever starts a unit, read by anyone who needs to know it is idle."""
import os
import subprocess
import sys
import time

import pytest

from robot_runtime import processes as run_state
from robot_runtime.processes import require_idle


def test_state_file_round_trip_and_clear(tmp_path):
    run_state.write_state(str(tmp_path), "control", {"pid": 1, "log": "a.log"})
    assert run_state.read_state(str(tmp_path), "control") == {"pid": 1, "log": "a.log"}
    run_state.clear_state(str(tmp_path), "control")
    run_state.clear_state(str(tmp_path), "control")  # clearing twice is fine
    assert run_state.read_state(str(tmp_path), "control") is None


@pytest.mark.parametrize("text", ["", "{broken", "[1, 2]"])
def test_unreadable_state_counts_as_no_state(tmp_path, text):
    (tmp_path / "x.json").write_text(text)
    assert run_state.read_state(str(tmp_path), "x") is None
    assert not run_state.is_running(None)


def test_running_follows_pid_and_start_time():
    me = {"pid": os.getpid(), "start_ticks": run_state.start_ticks(os.getpid())}
    assert run_state.is_running(me)
    assert not run_state.is_running({**me, "start_ticks": me["start_ticks"] + 1})  # reused pid
    assert not run_state.is_running({"owned_pids": [1]})


def test_unit_stops_counting_as_running_when_its_process_ends():
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"], start_new_session=True)
    state = {"pid": child.pid, "start_ticks": run_state.start_ticks(child.pid)}
    try:
        assert run_state.is_running(state)
    finally:
        child.kill()
        child.wait()
    deadline = time.monotonic() + 2
    while run_state.is_running(state) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not run_state.is_running(state)


def test_require_idle_reads_the_shared_state_files(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    require_idle(str(tmp_path), ["control"])  # no state file: idle
    run_state.write_state(str(run), "control", {"pid": os.getpid(), "start_ticks": run_state.start_ticks(os.getpid())})
    with pytest.raises(ValueError, match="control"):
        require_idle(str(tmp_path), ["control"])


def test_wait_for_log_returns_when_the_text_appears_and_gives_up_when_the_process_is_gone(tmp_path):
    import threading
    log = tmp_path / "unit.log"
    log.write_text("starting\n")
    threading.Timer(0.2, lambda: log.write_text("starting\nInitialize device cost: 1 ms\n")).start()
    assert run_state.wait_for_log(str(log), "Initialize device cost", lambda: True, 3.0)
    assert not run_state.wait_for_log(str(log), "never printed", lambda: False, 3.0)
    assert not run_state.wait_for_log(str(tmp_path / "missing.log"), "x", lambda: True, 0.3)
