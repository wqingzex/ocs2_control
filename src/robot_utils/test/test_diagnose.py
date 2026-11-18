"""diagnose: the analysis of the blackbox rings (the first hop that went wrong in an incident) and the evidence bundle
(the window of the logs, the fault codes and the health in it, and the report that ties it together)."""
import csv
import os
from datetime import datetime, timedelta

from robot_log_tools import diagnose, error_code
from robot_log_tools.diagnose import Stream
from test_blackbox_decoder import make_ring

MS = 1_000_000
T0 = 1_700_000_000_000_000_000


def stream(module, name, times_ms, aux=0, values=None, type_id=1, start_ms=0.0):
    values = values or [(0.0,)] * len(times_ms)
    auxes = aux if isinstance(aux, list) else [aux] * len(times_ms)
    return Stream(module, name, type_id, [T0 + int(t * MS) for t in times_ms], [int(t * MS) for t in times_ms],
                  auxes, list(values), T0 + int(start_ms * MS))


def regular(count, period_ms=10.0, start=0.0):
    return [start + i * period_ms for i in range(count)]


def test_summary_of_a_regular_stream():
    m = diagnose.summary(stream("m", "s", regular(101)))
    assert m["count"] == 101 and abs(m["rate_hz"] - 100.0) < 1e-6 and m["median_ms"] == 10.0 and m["max_gap_ms"] == 10.0


def test_a_hole_in_a_stream_is_a_gap_at_the_time_it_ended():
    times = regular(50) + regular(50, start=490 + 300)        # a 300 ms hole after the 50th record
    [gap] = diagnose.find_gaps(stream("xr_bridge", "xr_frame_in", times))
    assert gap.kind == "gap" and gap.wall_ns == T0 + int(790 * MS) and gap.detail.startswith("300 ms")


def test_a_steady_stream_and_a_slow_or_short_one_have_no_gaps():
    assert diagnose.find_gaps(stream("m", "s", regular(100))) == []
    assert diagnose.find_gaps(stream("m", "s", regular(10) + [500.0])) == []           # too few records
    assert diagnose.find_gaps(stream("m", "event", regular(30, period_ms=1000.0) + [60000.0])) == []   # 1 Hz: no gap rule


def test_a_late_cycle_is_between_the_late_limit_and_a_hole():
    times = regular(30) + [290 + 20.0] + regular(30, start=310 + 10.0)                 # one 20 ms period among 10 ms ones
    [late] = diagnose.find_late_cycles(stream("arm_controller", "ctl_state", times))
    assert late.kind == "late"
    assert diagnose.find_late_cycles(stream("a", "ctl_state", regular(60))) == []


def test_the_hardware_read_cycle_is_checked_for_late_cycles_like_the_controller_cycle():
    times = regular(30) + [290 + 20.0] + regular(30, start=310 + 10.0)
    found = diagnose.collect_anomalies([stream("marvin_hardware", "hw_read", times)])
    assert [a.kind for a in found] == ["late"] and found[0].stream == "marvin_hardware/hw_read"


def test_a_jump_in_a_commanded_joint_is_found_but_not_across_a_state_change():
    positions = [(0.0, 0.0)] * 5 + [(0.0, 0.5)] * 5            # joint 1 jumps 0.5 rad at index 5
    [jump] = diagnose.find_jumps(stream("arm_controller", "ctl_command", regular(10), aux=3, values=positions))
    assert jump.kind == "jump" and "joint 1" in jump.detail and jump.wall_ns == T0 + 50 * MS
    states = [2] * 5 + [3] * 5
    assert diagnose.find_jumps(stream("a", "ctl_command", regular(10), aux=states, values=positions)) == []
    assert diagnose.find_jumps(stream("a", "ctl_command", regular(10), aux=3, values=positions), step_limit=1.0) == []


def test_a_module_that_stopped_writing_while_the_others_went_on():
    running = stream("arm_controller", "ctl_state", regular(500))        # 5 s
    dead = stream("xr_bridge", "xr_frame_in", regular(100))              # 1 s
    [stopped] = diagnose.find_stopped([running, dead])
    assert stopped.kind == "stopped" and stopped.stream == "xr_bridge"
    assert diagnose.find_stopped([running]) == []                          # nothing to compare with
    assert diagnose.find_stopped([running, stream("x", "s", regular(450))]) == []


def test_everything_stopping_together_shows_against_the_moment_of_collection():
    all_dead = [stream("arm_controller", "ctl_state", regular(100)), stream("body_joint_controller", "ctl_state", regular(100))]
    assert diagnose.find_stopped(all_dead) == []                            # nothing to compare with each other
    found = diagnose.find_stopped(all_dead, now_ns=T0 + 60_000 * MS)         # collected a minute after the last record
    assert sorted(a.stream for a in found) == ["arm_controller", "body_joint_controller"]
    assert all(a.kind == "silent" and "59." in a.detail for a in found)
    assert diagnose.find_stopped(all_dead, now_ns=T0 + 3_000 * MS) == []     # 2 s of silence is normal
    both = diagnose.find_stopped([stream("a", "s", regular(500)), stream("b", "s", regular(100))], now_ns=T0 + 60_000 * MS)
    assert sorted((a.stream, a.kind) for a in both) == [("a", "silent"), ("b", "stopped")]      # a module is reported once


def test_a_ring_left_over_from_an_older_run_is_not_a_module_that_stopped():
    running = stream("arm_controller", "ctl_state", regular(500), start_ms=0.0)
    this_run = stream("xr_bridge", "xr_frame_in", regular(100), start_ms=5.0)
    older_run = stream("whole_body_controller", "ctl_state", regular(100), start_ms=-600_000.0)   # started 10 min earlier
    kept, left_out = diagnose.current_run([running, this_run, older_run])
    assert left_out == ["whole_body_controller"] and len(kept) == 2
    assert [a.stream for a in diagnose.find_stopped(kept)] == ["xr_bridge"]
    assert "whole_body_controller" not in [a.stream for a in diagnose.collect_anomalies([running, this_run, older_run])]


def test_hands_may_step_their_command_and_are_not_checked_for_jumps():
    positions = [(0.0, 0.0)] * 5 + [(0.0, 1.4)] * 5
    hand = stream("left_hand_controller", "ctl_command", regular(10), aux=3, values=positions)
    arm = stream("arm_controller", "ctl_command", regular(10), aux=3, values=positions)
    found = diagnose.collect_anomalies([hand, arm])
    assert [a.stream for a in found] == ["arm_controller/ctl_command"]


def test_hop_delay_is_the_time_since_the_latest_earlier_record():
    source = stream("a", "s", [0.0, 10.0, 20.0])
    sink = stream("b", "t", [4.0, 13.0, 29.0])
    assert diagnose.hop_delays_ms(source, sink) == [4.0, 3.0, 9.0]
    sided = stream("b", "t", [4.0, 13.0], aux=[0, 1])
    assert diagnose.hop_delays_ms(source, sided, sink_aux=1) == [3.0]
    assert diagnose.hop_delays_ms(stream("a", "s", [50.0]), sink) == []   # nothing earlier


def test_the_earliest_anomaly_comes_first_in_the_report(tmp_path):
    ring_dir = tmp_path / "rings"
    ring_dir.mkdir()
    # arm_controller: 1 kHz records (make_ring steps 1 ms), a state stream with a jump in its commands at record 30
    records = []
    for i in range(60):
        records.append((302, 3, [0.0, 0.0]))
        records.append((303, 3, [0.0, 0.0 if i < 30 else 0.6]))
    make_ring(ring_dir / "arm_controller.ring", records, capacity=256, module="arm_controller")
    make_ring(ring_dir / "xr_bridge.ring", [(101, 7, [0.0] * 26)] * 25, capacity=64, module="xr_bridge")
    text = diagnose.report(str(tmp_path))
    assert "arm_controller/ctl_command" in text and "jump" in text
    assert "xr_bridge/xr_frame_in" in text
    assert "anomalies, earliest first" in text and " jump " in text.split("anomalies")[1]


def test_a_directory_without_rings_says_so_and_the_cli_works(tmp_path, capsys):
    assert "no readable rings" in diagnose.report(str(tmp_path))
    make_ring(tmp_path / "m.ring", [(101, 7, [0.0] * 26)] * 30, capacity=64, module="m")
    out = tmp_path / "timeline.csv"
    assert diagnose.main([str(tmp_path), "--timeline", str(out)]) == 0
    assert "records written" in capsys.readouterr().out
    rows = list(csv.reader(open(out, encoding="utf-8")))
    assert rows[0][:4] == ["wall_time", "module", "type", "aux"] and len(rows) == 31 and rows[1][2] == "xr_frame_in"


def test_an_incident_directory_uses_its_rings_subdirectory(tmp_path):
    (tmp_path / "rings").mkdir()
    make_ring(tmp_path / "rings" / "m.ring", [(101, 7, [0.0] * 26)] * 30, capacity=64, module="m")
    assert "m/xr_frame_in" in diagnose.report(str(tmp_path))


# ---- the evidence bundle ------------------------------------------------------------------------------------------
NOW = datetime(2026, 11, 13, 10, 30, 0)
STOP = next(f.code for f in error_code.load() if f.is_stop)
MONITOR = next(f.code for f in error_code.load() if not f.is_stop)


def line(minutes_ago, text, severity="INFO", node="n"):
    moment = NOW - timedelta(minutes=minutes_ago)
    return f"[{severity}] [{moment:%Y-%m-%d %H:%M:%S}.123] [{node}]: {text}"


def make_logs(tmp_path, units):
    logs = tmp_path / "logs"
    for unit, lines in units.items():
        (logs / unit).mkdir(parents=True)
        (logs / unit / "20260101_000000.log").write_text("\n".join(lines) + "\n")
        os.symlink("20260101_000000.log", logs / unit / "latest.log")
    return logs


def test_only_the_lines_of_the_window_are_kept_and_continuations_follow_their_line():
    lines = [line(30, "too old"), line(5, "inside"), "  Traceback continues", line(0.5, "newest"), line(-5, "future")]
    kept, timed = diagnose.window_lines(lines, NOW - timedelta(minutes=10), NOW)
    assert timed and kept == [lines[1], lines[2], lines[3]]


def test_a_log_without_times_gives_its_last_lines():
    lines = [f"raw {i}" for i in range(1000)]
    kept, timed = diagnose.window_lines(lines, NOW - timedelta(minutes=10), NOW)
    assert not timed and kept == lines[-diagnose.UNTIMED_TAIL_LINES:]


def test_faults_are_found_per_unit_with_count_times_and_stop_class_first():
    found = diagnose.faults_in({
        "control": [line(8, f"[{MONITOR}] slow"), line(7, f"[{STOP}] stopped"), line(6, f"[{STOP}] stopped again")],
        "xr": [line(9, "[XX-999] not in the registry")]})
    assert [(f.code, f.unit, f.count) for f in found] == [(STOP, "control", 2), (MONITOR, "control", 1)]
    assert found[0].first.endswith(".123") and found[0].first < found[0].last
    text = "\n".join(diagnose.fault_section(found))
    assert f"{STOP} [S" in text and "stopped" in text
    assert diagnose.fault_section([]) == ["fault codes in the window: none"]


def test_a_fault_comes_with_the_errors_and_warnings_just_before_it_because_the_cause_is_usually_there():
    # On the robot: "[ERROR] Motor 1 emergency frame errors: undervoltage" came first, then "[HW-006] motor error".
    cause = line(7, "Motor 1 emergency frame errors: undervoltage (SEVERE)", severity="ERROR", node="juxie_body")
    found = diagnose.faults_in({"control": [
        line(30, "an old error long before", severity="ERROR"),
        line(8, "a plain info line", severity="INFO"),
        cause,
        line(7, f"[{STOP}] motor error ,return", severity="ERROR", node="juxie_body"),
        line(6, f"[{STOP}] motor error ,return", severity="ERROR", node="juxie_body")]})

    assert found[0].before == [cause.strip()]  # only the error right before the first time; not the old one, not info
    text = "\n".join(diagnose.fault_section(found))
    assert "undervoltage" in text and text.index("undervoltage") < text.index("motor error")


def test_a_fault_with_nothing_wrong_just_before_it_has_no_context():
    found = diagnose.faults_in({"control": [line(8, "all fine"), line(7, f"[{STOP}] stopped")]})

    assert found[0].before == [] and "before" not in "\n".join(diagnose.fault_section(found))


def test_health_lines_are_summed_up_per_component_for_cycles_and_for_rates():
    health = diagnose.health_in({"control": [
        line(5, "[HEALTH] arm_controller window=10.0s cycles=1000 period_avg_ms=10.00 period_max_ms=12.00 late=0 "
                "duration_avg_us=100 duration_max_us=300"),
        line(4, "[HEALTH] arm_controller window=10.0s cycles=990 period_avg_ms=10.10 period_max_ms=31.00 late=2 "
                "duration_avg_us=110 duration_max_us=900", severity="WARN"),
        line(3, "[HEALTH] xr_bridge window=10.0s count=2000 rate_hz=200.0 gap_max_ms=9.1 connected=1"),
        line(2, "[HEALTH] xr_bridge window=10.0s count=900 rate_hz=90.0 gap_max_ms=250.0")]})
    text = "\n".join(diagnose.health_section(health))
    assert "arm_controller" in text and "late 2" in text and "period max 31.00 ms" in text and "1 warning window" in text
    assert "rate 90.0..200.0 Hz" in text and "longest gap 250.0 ms" in text
    assert diagnose.health_section([]) == ["health in the window: no [HEALTH] lines"]


def test_gather_writes_the_window_logs_rings_events_and_a_report(tmp_path):
    logs = make_logs(tmp_path, {"control": [line(30, "ancient"), line(4, f"[{STOP}] stop now", severity="ERROR")],
                                "pc_service": ["raw line one", "raw line two"]})
    (logs / "events.log").write_text('{"kind": "start"}\n')
    rings = tmp_path / "rings"
    rings.mkdir()
    make_ring(rings / "arm_controller.ring", [(302, 2, [0.0, 0.1])] * 30, capacity=64, module="arm_controller")
    out = tmp_path / "out"
    report = diagnose.gather(str(logs), str(out), minutes=10, reason="the arm stopped", ring_dir=str(rings), now=NOW)
    assert "the arm stopped" in report and f"{STOP} [S" in report and "no times in the log of: pc_service" in report
    assert "arm_controller/ctl_state" in report
    assert "ancient" not in (out / "logs" / "control.log").read_text()
    assert "stop now" in (out / "logs" / "control.log").read_text()
    assert (out / "logs" / "pc_service.log").read_text() == "raw line one\nraw line two\n"
    assert (out / "events.log").read_text().strip() == '{"kind": "start"}'
    assert (out / "rings" / "arm_controller.ring").exists() and (out / "report.txt").read_text() == report


def test_gather_with_nothing_there_still_reports(tmp_path):
    report = diagnose.gather(str(tmp_path / "logs"), str(tmp_path / "out"), ring_dir=str(tmp_path / "none"), now=NOW)
    assert "no unit logs" in report and "no readable rings" in report
