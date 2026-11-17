"""Evidence: the window of the logs, the fault codes and the health in it, and the report that ties it together."""
import os
from datetime import datetime, timedelta

from robot_log_tools import evidence, fault_codes
from test_flight_decoder import make_ring

NOW = datetime(2026, 11, 13, 10, 30, 0)
STOP = next(f.code for f in fault_codes.load() if f.is_stop)
MONITOR = next(f.code for f in fault_codes.load() if not f.is_stop)


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
    kept, timed = evidence.window_lines(lines, NOW - timedelta(minutes=10), NOW)
    assert timed and kept == [lines[1], lines[2], lines[3]]


def test_a_log_without_times_gives_its_last_lines():
    lines = [f"raw {i}" for i in range(1000)]
    kept, timed = evidence.window_lines(lines, NOW - timedelta(minutes=10), NOW)
    assert not timed and kept == lines[-evidence.UNTIMED_TAIL_LINES:]


def test_faults_are_found_per_unit_with_count_times_and_stop_class_first():
    found = evidence.faults_in({
        "control": [line(8, f"[{MONITOR}] slow"), line(7, f"[{STOP}] stopped"), line(6, f"[{STOP}] stopped again")],
        "xr": [line(9, "[XX-999] not in the registry")]})
    assert [(f.code, f.unit, f.count) for f in found] == [(STOP, "control", 2), (MONITOR, "control", 1)]
    assert found[0].first.endswith(".123") and found[0].first < found[0].last
    text = "\n".join(evidence.fault_section(found))
    assert f"{STOP} [S" in text and "stopped" in text
    assert evidence.fault_section([]) == ["fault codes in the window: none"]


def test_a_fault_comes_with_the_errors_and_warnings_just_before_it_because_the_cause_is_usually_there():
    # On the robot: "[ERROR] Motor 1 emergency frame errors: undervoltage" came first, then "[HW-006] motor error".
    cause = line(7, "Motor 1 emergency frame errors: undervoltage (SEVERE)", severity="ERROR", node="juxie_body")
    found = evidence.faults_in({"control": [
        line(30, "an old error long before", severity="ERROR"),
        line(8, "a plain info line", severity="INFO"),
        cause,
        line(7, f"[{STOP}] motor error ,return", severity="ERROR", node="juxie_body"),
        line(6, f"[{STOP}] motor error ,return", severity="ERROR", node="juxie_body")]})

    assert found[0].before == [cause.strip()]  # only the error right before the first time; not the old one, not info
    text = "\n".join(evidence.fault_section(found))
    assert "undervoltage" in text and text.index("undervoltage") < text.index("motor error")


def test_a_fault_with_nothing_wrong_just_before_it_has_no_context():
    found = evidence.faults_in({"control": [line(8, "all fine"), line(7, f"[{STOP}] stopped")]})

    assert found[0].before == [] and "before" not in "\n".join(evidence.fault_section(found))


def test_health_lines_are_summed_up_per_component_for_cycles_and_for_rates():
    health = evidence.health_in({"control": [
        line(5, "[HEALTH] arm_controller window=10.0s cycles=1000 period_avg_ms=10.00 period_max_ms=12.00 late=0 "
                "duration_avg_us=100 duration_max_us=300"),
        line(4, "[HEALTH] arm_controller window=10.0s cycles=990 period_avg_ms=10.10 period_max_ms=31.00 late=2 "
                "duration_avg_us=110 duration_max_us=900", severity="WARN"),
        line(3, "[HEALTH] xr_bridge window=10.0s count=2000 rate_hz=200.0 gap_max_ms=9.1 connected=1"),
        line(2, "[HEALTH] xr_bridge window=10.0s count=900 rate_hz=90.0 gap_max_ms=250.0")]})
    text = "\n".join(evidence.health_section(health))
    assert "arm_controller" in text and "late 2" in text and "period max 31.00 ms" in text and "1 warning window" in text
    assert "rate 90.0..200.0 Hz" in text and "longest gap 250.0 ms" in text
    assert evidence.health_section([]) == ["health in the window: no [HEALTH] lines"]


def test_gather_writes_the_window_logs_rings_events_and_a_report(tmp_path):
    logs = make_logs(tmp_path, {"control": [line(30, "ancient"), line(4, f"[{STOP}] stop now", severity="ERROR")],
                                "pc_service": ["raw line one", "raw line two"]})
    (logs / "events.log").write_text('{"kind": "start"}\n')
    rings = tmp_path / "rings"
    rings.mkdir()
    make_ring(rings / "arm_controller.ring", [(302, 2, [0.0, 0.1])] * 30, capacity=64, module="arm_controller")
    out = tmp_path / "out"
    report = evidence.gather(str(logs), str(out), minutes=10, reason="the arm stopped", ring_dir=str(rings), now=NOW)
    assert "the arm stopped" in report and f"{STOP} [S" in report and "no times in the log of: pc_service" in report
    assert "arm_controller/ctl_state" in report
    assert "ancient" not in (out / "logs" / "control.log").read_text()
    assert "stop now" in (out / "logs" / "control.log").read_text()
    assert (out / "logs" / "pc_service.log").read_text() == "raw line one\nraw line two\n"
    assert (out / "events.log").read_text().strip() == '{"kind": "start"}'
    assert (out / "rings" / "arm_controller.ring").exists() and (out / "report.txt").read_text() == report


def test_gather_with_nothing_there_still_reports(tmp_path):
    report = evidence.gather(str(tmp_path / "logs"), str(tmp_path / "out"), ring_dir=str(tmp_path / "none"), now=NOW)
    assert "no unit logs" in report and "no readable rings" in report
