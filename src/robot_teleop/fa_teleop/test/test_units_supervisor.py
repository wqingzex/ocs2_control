"""Tests for fa_teleop_runtime.units and fa_teleop_runtime.supervisor (no ROS needed)."""
import json
import os
import time

import pytest

from fa_teleop_runtime import validate
from fa_teleop_runtime.supervisor import Supervisor, UnitError
from fa_teleop_runtime.units import Unit, UnitSet, build_units

SERIALS = {"head": {"serial": "CP111"}, "left_wrist": {"serial": "CP222"}, "right_wrist": {"serial": "CP333"}}


def make_cfg(**sections):
    base = {"cameras": {k: dict(v) for k, v in SERIALS.items()}}
    for key, value in sections.items():
        base.setdefault(key, {}).update(value)
    return validate(base)


# --- unit generation ----------------------------------------------------------------------------

def test_headset_id_is_passed_as_a_string_even_when_it_is_numeric():
    units = build_units(make_cfg(xr={"device_id": "123456"}), "/c.yaml", "/ws")
    assert 'device_id:="123456"' in units["xr"].cmd

def test_all_units_are_generated_from_the_config():
    units = build_units(make_cfg(), "/ws/config/robot.local.yaml", "/ws")
    assert units.names() == ["pc_service", "control", "rviz", "joystick", "xr",
                             "camera_left_wrist", "camera_right_wrist", "camera_head"]
    assert units["xr"].depends == ("pc_service",)
    assert "__node:=xr_target_node" in units["xr"].cmd
    assert "only:=camera_head" in units["camera_head"].cmd
    assert units["control"].cmd[:4] == ("ros2", "launch", "robot_bringup", "split_body.launch.py")


def test_disabled_parts_produce_no_units():
    units = build_units(make_cfg(xr={"enable": False}, cameras={"enable": False}), "/c.yaml", "/ws")
    assert units.names() == ["control", "rviz", "joystick"]


def test_xr_without_managed_pc_service_has_no_dependency():
    units = build_units(make_cfg(xr={"start_pc_service": False}), "/c.yaml", "/ws")
    assert "pc_service" not in units
    assert units["xr"].depends == ()


def test_start_order_adds_dependencies_first_and_stop_order_dependents_first():
    units = build_units(make_cfg(), "/c.yaml", "/ws")
    assert units.start_order(["xr"]) == ["pc_service", "xr"]
    assert units.stop_order(["pc_service"]) == ["xr", "pc_service"]
    full = units.stop_order()
    assert full.index("xr") < full.index("pc_service")


def test_unknown_unit_names_the_choices():
    with pytest.raises(KeyError, match="可选"):
        build_units(make_cfg(), "/c.yaml", "/ws")["camera_tail"]


def test_dependency_cycle_is_reported():
    cyclic = UnitSet({"a": Unit("a", "A", ("true",), depends=("b",)), "b": Unit("b", "B", ("true",), depends=("a",))})
    with pytest.raises(ValueError, match="成环"):
        cyclic.start_order()


# --- supervisor ---------------------------------------------------------------------------------

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


def test_joystick_is_an_on_demand_unit_that_needs_control():
    units = build_units(make_cfg(joystick={"device": "/dev/input/js2"}), "/c.yaml", "/ws")
    joystick = units["joystick"]
    assert joystick.on_demand and joystick.depends == ("control",)
    assert joystick.cmd[:4] == ("ros2", "launch", "arms_teleop", "joystick_teleop.launch.py")
    assert "joy_dev:=/dev/input/js2" in joystick.cmd
    auto = build_units(make_cfg(), "/c.yaml", "/ws")["joystick"]
    assert not any(a.startswith("joy_dev") for a in auto.cmd)
    assert "joystick" not in build_units(make_cfg(), "/c.yaml", "/ws").start_order()


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


class FakeProbe:
    def __init__(self, state, hold_ok=True):
        self.state, self.hold_ok, self.hold_requests, self.closed = state, hold_ok, 0, False

    def wait_fsm(self, _timeout):
        return self.state

    def request_hold(self, _timeout):
        self.hold_requests += 1
        if self.hold_ok:
            self.state = 2
        return self.hold_ok

    def close(self):
        self.closed = True


def run_hook(state, answers, assume_yes=False, hold_ok=True):
    from fa_teleop_runtime.units_cli import make_hold_hook
    probe, asked = FakeProbe(state, hold_ok), []

    def ask(question):
        asked.append(question)
        return answers.pop(0)

    hook = make_hold_hook(assume_yes, ask, probe_factory=lambda: probe)
    try:
        hook("control")
        error = None
    except UnitError as exc:
        error = str(exc)
    return probe, asked, error


def test_stopping_control_in_hold_needs_no_confirmation():
    probe, asked, error = run_hook(state=2, answers=[])
    assert asked == [] and error is None and probe.hold_requests == 0 and probe.closed


def test_stopping_control_in_ocs2_asks_then_requests_hold():
    probe, asked, error = run_hook(state=3, answers=[True])
    assert "OCS2" in asked[0] and error is None and probe.hold_requests == 1


def test_declining_the_confirmation_aborts_the_stop():
    probe, _, error = run_hook(state=3, answers=[False])
    assert "已取消" in error and probe.hold_requests == 0


def test_hold_timeout_asks_again():
    _, asked, error = run_hook(state=3, answers=[True, False], hold_ok=False)
    assert len(asked) == 2 and "未回到 HOLD" in error


def test_yes_flag_skips_questions():
    probe, asked, error = run_hook(state=3, answers=[], assume_yes=True)
    assert asked == [] and error is None and probe.hold_requests == 1


def test_hook_ignores_other_units():
    from fa_teleop_runtime.units_cli import make_hold_hook
    make_hold_hook(False, lambda q: False, probe_factory=lambda: pytest.fail("probe must not be created"))("xr")


def test_status_table_lists_every_unit(make_supervisor):
    import io
    from fa_teleop_runtime.units_cli import print_table
    sup = make_supervisor({"a": Unit("a", "模块A", SLEEPER), "b": Unit("b", "模块B", SLEEPER)})
    sup.start("a")
    out = io.StringIO()
    print_table(sup.status_all(), {"a": "x=30.0"}, out=out)
    text = out.getvalue()
    assert "a(模块A)" in text and "运行" in text and "x=30.0" in text and "b(模块B)" in text and "停止" in text


def test_unit_environment_is_passed_to_the_process(make_supervisor):
    printer = ("sh", "-c", "echo DISPLAY=$DISPLAY; trap 'exit 0' INT; while true; do sleep 0.05; done")
    sup = make_supervisor({"a": Unit("a", "A", printer, env=(("DISPLAY", ":7"),))})
    sup.start("a")
    assert wait_for(lambda: "DISPLAY=:7" in "\n".join(sup.log_tail("a")))


def test_display_env_keeps_an_existing_display_and_falls_back_to_local_x(monkeypatch, tmp_path):
    from fa_teleop_runtime import units as units_mod
    assert units_mod.display_env({"DISPLAY": ":1"}) == ()
    monkeypatch.setattr(units_mod.os.path, "exists", lambda p: p == "/tmp/.X11-unix/X0")
    (tmp_path / ".Xauthority").write_text("x")
    env = dict(units_mod.display_env({"HOME": str(tmp_path)}))
    assert env["DISPLAY"] == ":0" and env["XAUTHORITY"] == str(tmp_path / ".Xauthority")


def test_rviz_is_a_separate_on_demand_unit():
    units = build_units(make_cfg(), "/c.yaml", "/ws")
    control, rviz = units["control"].cmd, units["rviz"].cmd
    assert "launch_mode:=control_only" in control and "launch_mode:=rviz_only" in rviz
    # same robot / hand arguments, only the mode differs
    assert [a for a in control if not a.startswith("launch_mode")] == [a for a in rviz if not a.startswith("launch_mode")]
    assert units["rviz"].depends == ("control",) and units["rviz"].on_demand
    assert "rviz" not in units.start_order()          # up / "全部启动" leaves it out
    assert units.launch_order(["rviz"]) == ["rviz"]  # never starts the controllers by itself
    assert units.launch_order(["xr", "rviz"]) == ["pc_service", "xr", "rviz"]
    assert units.stop_order(["control"]) == ["joystick", "rviz", "control"]  # dependents stop first
    assert "rviz" in units.stop_order()               # "全部停止" still closes it


def test_missing_command_is_a_readable_error(make_supervisor):
    sup = make_supervisor({"a": Unit("a", "A", ("definitely-not-a-command-xyz",))})
    with pytest.raises(UnitError, match="找不到命令"):
        sup.start("a")


# --- logging: uniform format, event timeline, retention -----------------------------------------

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
    from fa_teleop_runtime.events import KEEP_UNIT_LOGS
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
    from fa_teleop_runtime import events
    from robot_log_tools.events import MAX_BYTES
    log = events.EventLog(str(tmp_path))
    (tmp_path / "events.log").write_text("x" * (MAX_BYTES + 1))
    log.record("start", "a")
    assert (tmp_path / "events.log.1").exists() and len((tmp_path / "events.log").read_text().splitlines()) == 1
    events.EventLog(str(tmp_path / "missing" / "dir")).record("start", "a")  # must not raise


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


# ---- PC Service output goes to logs/pc_service/ --------------------------------------------------------------------
def fake_pc_service(monkeypatch, tmp_path, already_running=False):
    """A runService.sh that prints, and a process table that shows the service once the script has run."""
    from fa_teleop_runtime import pc_service
    marker = tmp_path / "service_started"
    script = tmp_path / "runService.sh"
    script.write_text(f"#!/bin/bash\necho 'service says hello'\necho 'service complains' >&2\ntouch {marker}\n")
    monkeypatch.setattr(pc_service, "running_pids", lambda: {4242} if already_running or marker.exists() else set())
    return script


def test_the_pc_service_output_is_kept_in_its_own_log(make_supervisor, tmp_path, monkeypatch):
    import time
    script = fake_pc_service(monkeypatch, tmp_path)
    sup = make_supervisor({"pc_service": Unit("pc_service", "PC Service", (), kind="pc_service")})
    sup.pc_service_script = str(script)
    sup.start("pc_service")
    latest = tmp_path / "logs" / "pc_service" / "latest.log"
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline and "complains" not in (latest.read_text() if latest.exists() else ""):
        time.sleep(0.05)
    text = latest.read_text()
    assert "service says hello" in text and "service complains" in text      # stdout and stderr, as they are
    assert latest.is_symlink() and sup.log_path("pc_service") is not None
    assert sup.status("pc_service").log is not None and sup.log_tail("pc_service")[0] == "service says hello"


def test_a_pc_service_that_was_running_already_makes_no_log(make_supervisor, tmp_path, monkeypatch):
    script = fake_pc_service(monkeypatch, tmp_path, already_running=True)
    sup = make_supervisor({"pc_service": Unit("pc_service", "PC Service", (), kind="pc_service")})
    sup.pc_service_script = str(script)
    sup.start("pc_service")
    assert not (tmp_path / "logs" / "pc_service" / "latest.log").exists()
