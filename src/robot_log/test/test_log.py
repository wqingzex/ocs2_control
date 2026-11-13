"""The Python log form: same lines as the C++ nodes, fault codes checked, mode followed."""
import io
import logging
import re

import pytest

from robot_log_tools import fault_codes, log

LINE = re.compile(r"^\[(DEBUG|INFO|WARN|ERROR|FATAL)\] \[\d{4}-\d\d-\d\d \d\d:\d\d:\d\d\.\d{3}\] \[([^\]]+)\]: (.*)$")


@pytest.fixture
def logger(monkeypatch):
    monkeypatch.delenv(log.LOG_MODE_ENV, raising=False)
    stream = io.StringIO()
    name = f"t_{id(stream)}"
    yield log.get_logger(name, stream), stream, name
    logging.getLogger(name).handlers.clear()


def lines(stream):
    return stream.getvalue().splitlines()


def test_a_line_has_severity_time_name_and_message(logger):
    lg, stream, name = logger
    lg.info("hello %s", "world")
    lg.warning("careful")
    lg.error("bad")
    lg.critical("worse")
    parsed = [LINE.match(l).groups() for l in lines(stream)]
    assert parsed == [("INFO", name, "hello world"), ("WARN", name, "careful"), ("ERROR", name, "bad"),
                      ("FATAL", name, "worse")]


def test_normal_mode_hides_debug_and_debug_mode_shows_it(logger, monkeypatch):
    lg, stream, name = logger
    lg.debug("detail")
    assert lines(stream) == []
    monkeypatch.setenv(log.LOG_MODE_ENV, "debug")
    lg = log.get_logger(name, stream)
    lg.debug("detail")
    assert LINE.match(lines(stream)[0]).group(1) == "DEBUG"
    assert log.log_mode() == "debug"
    monkeypatch.setenv(log.LOG_MODE_ENV, "bogus")
    assert log.log_mode() == "normal"


def test_asking_twice_does_not_duplicate_lines(logger):
    lg, stream, name = logger
    log.get_logger(name, stream).info("once")
    assert len(lines(stream)) == 1


def test_a_fault_line_carries_its_registered_code_and_the_evidence_report_recognises_it(logger):
    from robot_log_tools import evidence
    lg, stream, name = logger
    code = next(f.code for f in fault_codes.load() if f.is_stop)
    log.fault(lg, code, "something %d", 7)
    [line] = lines(stream)
    assert LINE.match(line).group(1) == "WARN" and f"[{code}] something 7" in line and "unregistered" not in line
    [seen] = evidence.faults_in({name: [line]})
    assert (seen.code, seen.count) == (code, 1) and seen.first != ""


def test_an_unregistered_code_is_logged_with_a_note_not_raised(logger):
    lg, stream, _ = logger
    log.fault(lg, "XX-999", "typo")
    assert "[XX-999] typo (unregistered fault code)" in stream.getvalue()


def test_a_message_with_a_percent_sign_and_no_arguments_is_safe(logger):
    lg, stream, _ = logger
    log.fault(lg, fault_codes.load()[0].code, "disk 90% full")
    assert "disk 90% full" in stream.getvalue()


def test_the_level_of_a_fault_can_be_raised(logger):
    lg, stream, _ = logger
    log.fault(lg, fault_codes.load()[0].code, "stop", level=logging.ERROR)
    assert LINE.match(lines(stream)[0]).group(1) == "ERROR"


def test_an_exception_is_appended(logger):
    lg, stream, _ = logger
    try:
        raise ValueError("boom")
    except ValueError:
        lg.exception("failed")
    assert "ValueError: boom" in stream.getvalue()


# ---- health -------------------------------------------------------------------------------------------------------
def test_rate_stats_count_rate_and_the_longest_gap_and_the_gap_across_windows():
    stats = log.RateStats()
    for t in (1000.0, 1000.010, 1000.020, 1000.095, 1000.100):          # a 75 ms hole
        stats.record(t)
    snap = stats.take(2.0)
    assert snap.count == 5 and abs(snap.rate_hz - 2.5) < 1e-9 and abs(snap.gap_max_ms - 75.0) < 1e-6
    stats.record(1000.400)                                               # 300 ms after the last of the previous window
    assert abs(stats.take(1.0).gap_max_ms - 300.0) < 1e-6
    assert log.RateStats().take(10.0) == log.RateSnapshot()              # nothing: zeros, no division by zero


def test_the_health_line_is_the_one_the_cpp_side_writes():
    # the same literals as test_health.cpp (HealthLine.RateLineHasTheAgreedKeysAndOptionalExtras)
    snap = log.RateSnapshot(count=2000, rate_hz=200.0, gap_max_ms=9.1)
    assert log.format_rate_line("xr_bridge", 10.0, snap) == "[HEALTH] xr_bridge window=10.0s count=2000 rate_hz=200.0 gap_max_ms=9.1"
    assert log.format_rate_line("xr_bridge", 10.0, snap, "connected=1 parse_fail=0").endswith("gap_max_ms=9.1 connected=1 parse_fail=0")


def test_the_window_is_ten_seconds_in_normal_and_one_in_debug(monkeypatch):
    monkeypatch.delenv(log.LOG_MODE_ENV, raising=False)
    assert log.health_window_s() == 10.0
    monkeypatch.setenv(log.LOG_MODE_ENV, "debug")
    assert log.health_window_s() == 1.0


def test_the_reporter_writes_info_while_healthy_and_a_coded_warning_on_a_gap(logger):
    lg, stream, _ = logger
    code = next(f.code for f in fault_codes.load() if not f.is_stop)
    stats = log.RateStats()
    reporter = log.HealthReporter(lg, "my_unit", stats, extra=lambda: "connected=1", gap_warn_ms=100.0, code=code, window_s=10.0)
    for t in (1.0, 1.01, 1.02):
        stats.record(t)
    healthy = reporter.report_once()
    assert healthy.startswith("[HEALTH] my_unit window=10.0s count=3") and healthy.endswith("connected=1")
    for t in (1.5, 1.8):                                                 # a 480 ms gap since 1.02
        stats.record(t)
    reporter.report_once()
    out = lines(stream)
    assert out[0].startswith("[INFO]") and out[1].startswith("[WARN]") and f"[{code}] [HEALTH] my_unit" in out[1]


def test_the_reporter_thread_reports_once_per_window_and_stops(logger):
    import time
    lg, stream, _ = logger
    stats = log.RateStats()
    reporter = log.HealthReporter(lg, "t", stats, window_s=0.05).start()
    stats.record()
    time.sleep(0.3)
    reporter.stop()
    n = len(lines(stream))
    assert n >= 2
    time.sleep(0.15)
    assert len(lines(stream)) == n                                       # stopped: no more lines
