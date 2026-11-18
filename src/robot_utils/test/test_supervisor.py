"""Tests for robot_runtime.units and robot_runtime.supervisor (no ROS needed)."""
import json
import os
import time

import pytest

from robot_runtime.supervisor import Supervisor, UnitError
from robot_runtime.units import Unit, UnitSet


def test_dependency_cycle_is_reported():
    cyclic = UnitSet({"a": Unit("a", "A", ("true",), depends=("b",)), "b": Unit("b", "B", ("true",), depends=("a",))})
    with pytest.raises(ValueError, match="成环"):
        cyclic.start_order()


SLEEPER = ("sh", "-c", "echo started; trap 'echo got-int; exit 0' INT; while true; do sleep 0.05; done")


STUBBORN = ("sh", "-c", "trap '' INT TERM; while true; do sleep 0.05; done")


@pytest.fixture
def make_supervisor(tmp_path):
    created = []

    def factory(units):
        sup = Supervisor(UnitSet(units), str(tmp_path / "run"), str(tmp_path / "logs"))
        created.append(sup)
        return sup

    yield factory
    for sup in created:  # never leave processes behind
        for name in sup.units.names():
            try:
                sup.stop(name, timeout_s=1.0)
            except UnitError:
                pass


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.05)
    return False


def test_start_writes_state_and_log_then_stop_cleans_up(make_supervisor, tmp_path):
    sup = make_supervisor({"a": Unit("a", "模块A", SLEEPER)})
    status = sup.start("a")
    assert status.running and status.pid
    assert wait_for(lambda: "started" in "\n".join(sup.log_tail("a")))
    state = json.loads((tmp_path / "run" / "a.json").read_text())
    assert state["pid"] == status.pid and state["start_ticks"]
    assert os.path.islink(tmp_path / "logs" / "a" / "latest.log")

    assert not sup.stop("a").running
    assert not (tmp_path / "run" / "a.json").exists()
    assert "got-int" in "\n".join(sup.log_tail("a"))  # stopped via SIGINT, cleanly


def test_status_survives_a_new_supervisor_instance(make_supervisor):
    first = make_supervisor({"a": Unit("a", "A", SLEEPER)})
    pid = first.start("a").pid
    second = make_supervisor({"a": Unit("a", "A", SLEEPER)})
    assert second.status("a").running and second.status("a").pid == pid
    second.stop("a")
    assert not first.is_running("a")


def test_crashed_unit_is_reported_with_its_last_error(make_supervisor):
    crash = ("sh", "-c", "echo '[ERROR] [x]: driver failed'; exit 3")
    sup = make_supervisor({"a": Unit("a", "A", crash)})
    sup.start("a")
    assert wait_for(lambda: not sup.is_running("a"))
    status = sup.status("a")
    assert not status.running
    assert "driver failed" in status.last_error


def test_reused_pid_is_not_mistaken_for_the_unit(make_supervisor, tmp_path):
    sup = make_supervisor({"a": Unit("a", "A", SLEEPER)})
    sup.start("a")
    state_file = tmp_path / "run" / "a.json"
    state = json.loads(state_file.read_text())
    sup.stop("a")
    state["pid"] = os.getpid()  # a live process that is not the unit
    state_file.write_text(json.dumps(state))
    assert not sup.is_running("a")


def test_sigint_resistant_unit_is_escalated_to_kill(make_supervisor):
    sup = make_supervisor({"a": Unit("a", "A", STUBBORN)})
    sup.start("a")
    started = time.monotonic()
    assert not sup.stop("a", timeout_s=0.3).running
    assert time.monotonic() - started < 6.0


def test_start_refuses_when_dependency_is_down(make_supervisor):
    sup = make_supervisor({"dep": Unit("dep", "依赖", SLEEPER), "a": Unit("a", "A", SLEEPER, depends=("dep",))})
    with pytest.raises(UnitError, match="依赖的模块未运行: dep"):
        sup.start("a")
    sup.up(["a"])  # up starts dependencies first
    assert sup.is_running("dep") and sup.is_running("a")
    sup.down(["dep"])  # stopping a dependency stops its dependents too
    assert not sup.is_running("a") and not sup.is_running("dep")


def test_on_demand_unit_does_not_start_its_dependency(make_supervisor):
    sup = make_supervisor({"control": Unit("control", "运控", SLEEPER),
                           "rviz": Unit("rviz", "RViz", SLEEPER, depends=("control",), on_demand=True)})
    with pytest.raises(UnitError, match="需要先启动运控"):
        sup.up(["rviz"])
    assert not sup.is_running("control")
    sup.up(["control"])
    sup.up(["rviz"])
    assert sup.is_running("rviz")


def test_pre_stop_hook_runs_before_the_process_is_stopped(make_supervisor):
    sup = make_supervisor({"control": Unit("control", "运控", SLEEPER)})
    sup.start("control")
    seen = []
    sup.stop("control", pre_stop=lambda name: seen.append((name, sup.is_running(name))))
    assert seen == [("control", True)]




def test_unit_environment_is_passed_to_the_process(make_supervisor):
    printer = ("sh", "-c", "echo DISPLAY=$DISPLAY; trap 'exit 0' INT; while true; do sleep 0.05; done")
    sup = make_supervisor({"a": Unit("a", "A", printer, env=(("DISPLAY", ":7"),))})
    sup.start("a")
    assert wait_for(lambda: "DISPLAY=:7" in "\n".join(sup.log_tail("a")))


def test_missing_command_is_a_readable_error(make_supervisor):
    sup = make_supervisor({"a": Unit("a", "A", ("definitely-not-a-command-xyz",))})
    with pytest.raises(UnitError, match="找不到命令"):
        sup.start("a")


def events_of(tmp_path):
    path = tmp_path / "logs" / "events.log"
    return [json.loads(line) for line in path.read_text().splitlines()] if path.exists() else []


def test_units_log_with_one_format_unless_the_environment_says_otherwise(make_supervisor, monkeypatch):
    monkeypatch.setenv("RCUTILS_COLORIZED_OUTPUT", "1")
    printer = ("sh", "-c", "echo FMT=$RCUTILS_CONSOLE_OUTPUT_FORMAT COLOR=$RCUTILS_COLORIZED_OUTPUT "
                           "UNBUF=$PYTHONUNBUFFERED; trap 'exit 0' INT; while true; do sleep 0.05; done")
    sup = make_supervisor({"a": Unit("a", "A", printer)})
    sup.start("a")
    assert wait_for(lambda: "UNBUF=1" in "\n".join(sup.log_tail("a")))
    text = "\n".join(sup.log_tail("a"))
    assert "{severity}" in text and "{date_time_with_ms}" in text and "COLOR=1" in text  # an existing value is kept


def test_start_and_stop_are_recorded_in_the_event_log(make_supervisor, tmp_path):
    sup = make_supervisor({"a": Unit("a", "A", SLEEPER)})
    pid = sup.start("a").pid
    sup.stop("a", timeout_s=2.0)
    start, stop = events_of(tmp_path)
    assert (start["kind"], start["unit"], start["pid"]) == ("start", "a", pid) and "time" in start
    assert (stop["kind"], stop["unit"]) == ("stop", "a")
    sup.stop("a")  # stopping what is not running records nothing more
    assert len(events_of(tmp_path)) == 2


def test_an_unexpected_exit_is_recorded_once_with_code_and_last_error(make_supervisor, tmp_path):
    crasher = ("sh", "-c", "echo '[ERROR] [controller]: solver diverged'; exit 3")
    sup = make_supervisor({"a": Unit("a", "A", crasher)})
    sup.start("a")
    assert wait_for(lambda: not sup.is_running("a"))
    sup.status("a")
    sup.status("a")  # polled again: still one event
    exits = [e for e in events_of(tmp_path) if e["kind"] == "exit"]
    assert len(exits) == 1
    assert exits[0]["unit"] == "a" and exits[0]["code"] == 3 and "solver diverged" in exits[0]["last_error"]


def test_a_requested_stop_is_not_reported_as_an_exit(make_supervisor, tmp_path):
    sup = make_supervisor({"a": Unit("a", "A", SLEEPER)})
    sup.start("a")
    sup.stop("a", timeout_s=2.0)
    sup.status("a")
    assert [e["kind"] for e in events_of(tmp_path)] == ["start", "stop"]


def test_only_the_newest_logs_of_a_unit_are_kept(make_supervisor, tmp_path):
    from robot_runtime.supervisor import KEEP_UNIT_LOGS
    unit_dir = tmp_path / "logs" / "a"
    unit_dir.mkdir(parents=True)
    old = [f"2020010{i // 10}_0000{i % 10:02d}.log" for i in range(KEEP_UNIT_LOGS + 5)]
    for name in old:
        (unit_dir / name).write_text("x")
    sup = make_supervisor({"a": Unit("a", "A", SLEEPER)})
    sup.start("a")
    remaining = sorted(p.name for p in unit_dir.glob("2*.log"))
    assert len(remaining) == KEEP_UNIT_LOGS + 1  # the kept ones plus the log of this start
    assert old[0] not in remaining and old[-1] in remaining
    assert (unit_dir / "latest.log").is_symlink()


def test_the_event_log_rolls_over_and_survives_an_unwritable_directory(tmp_path):
    from robot_runtime.supervisor import MAX_BYTES, EventLog
    log = EventLog(str(tmp_path))
    (tmp_path / "events.log").write_text("x" * (MAX_BYTES + 1))
    log.record("start", "a")
    assert (tmp_path / "events.log.1").exists() and len((tmp_path / "events.log").read_text().splitlines()) == 1
    EventLog(str(tmp_path / "missing" / "dir")).record("start", "a")  # must not raise


def test_polls_from_several_threads_record_one_exit(make_supervisor, tmp_path):
    import threading
    sup = make_supervisor({"a": Unit("a", "A", ("sh", "-c", "exit 4"))})
    sup.start("a")
    assert wait_for(lambda: not sup.is_running("a"))
    pollers = [threading.Thread(target=lambda: [sup.status("a") for _ in range(20)]) for _ in range(6)]
    for thread in pollers:
        thread.start()
    for thread in pollers:
        thread.join()
    assert [e["kind"] for e in events_of(tmp_path)].count("exit") == 1


def test_start_returns_after_the_ready_text_is_logged(make_supervisor):
    slow = ("sh", "-c", "sleep 0.5; echo ready-now; sleep 30")
    sup = make_supervisor({"a": Unit("a", "A", slow, ready_log="ready-now", ready_timeout_s=10.0)})
    started = time.monotonic()
    sup.start("a")
    assert 0.4 < time.monotonic() - started < 5.0
    assert "ready-now" in "\n".join(sup.log_tail("a"))


def test_start_does_not_wait_for_a_unit_that_exited(make_supervisor):
    sup = make_supervisor({"a": Unit("a", "A", ("sh", "-c", "exit 1"), ready_log="never", ready_timeout_s=20.0)})
    started = time.monotonic()
    sup.start("a")
    assert time.monotonic() - started < 5.0


def test_units_start_one_after_another_when_they_declare_a_ready_text(make_supervisor, tmp_path):
    """The second unit is spawned only after the first printed its ready text."""
    # The time is written before "ready" is printed: the second unit starts on that text and the test then reads it.
    first = ("sh", "-c", f"sleep 0.6; date +%s.%N > {tmp_path}/first; echo ready; sleep 30")
    second = ("sh", "-c", f"date +%s.%N > {tmp_path}/second; sleep 30")
    sup = make_supervisor({"a": Unit("a", "A", first, ready_log="ready", ready_timeout_s=10.0),
                           "b": Unit("b", "B", second)})
    sup.up(["a", "b"])
    # The shell creates the file before date writes it: wait for the content, not for the file.
    assert wait_for(lambda: (tmp_path / "second").exists() and (tmp_path / "second").read_text().strip())
    assert float((tmp_path / "second").read_text()) >= float((tmp_path / "first").read_text())


def test_variables_a_unit_must_not_inherit_are_removed(tmp_path):
    from robot_runtime.supervisor import Supervisor
    printer = ("sh", "-c", "echo PROXY=[$http_proxy] KEEP=[$KEEP]; trap 'exit 0' INT; while true; do sleep 0.05; done")
    unit = Unit("a", "A", printer, unset_env=("http_proxy",))
    sup = Supervisor(UnitSet({"a": unit}), str(tmp_path / "run"), str(tmp_path / "logs"),
                     env={"http_proxy": "http://127.0.0.1:8080", "KEEP": "yes", "PATH": os.environ["PATH"]})
    try:
        sup.start("a")
        assert wait_for(lambda: "PROXY=[] KEEP=[yes]" in "\n".join(sup.log_tail("a")))
    finally:
        sup.stop("a", timeout_s=1.0)
