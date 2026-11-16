"""Gathers what is needed to find out what went wrong, after it went wrong: the logs of the minutes before, what they
say about faults and health, and what each module received and sent (the flight recorder rings).

    gather(log_dir, out_dir, minutes=10)       writes into out_dir:
        report.txt        the window, the fault codes seen in it, the health of each component, the analysis of the rings
        rings/            copies of the flight recorder rings (and the ones of the run before a restart)
        logs/<unit>.log   the lines of each unit log in the window
        events.log        the end of the event timeline

Nothing runs all the time: the rings live in shared memory and survive a crashed module, the logs are on disk, so this
can be done after the fact, by hand (./scripts/start.sh diag). The unit logs carry a time on every line
([2026-11-13 10:00:00.123]); a log without times (the PC Service) is taken as its last lines.
"""
from __future__ import annotations

import os
import re
import statistics
from collections import defaultdict
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from typing import Dict, List, Optional, Tuple

from robot_log_tools import analyze, fault_codes, flight

DEFAULT_MINUTES = 10
MAX_READ_BYTES = 32 * 1024 * 1024     # the end of a log that is read at most
UNTIMED_TAIL_LINES = 300              # a log without times: this many last lines
EVENTS_TAIL_LINES = 200
MAX_FAULT_ROWS = 40
_TIME = re.compile(r"\[(\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)\.(\d{3})\]")
_CODE = re.compile(r"\[([A-Z]{2,4}-\d{3})\]")
_HEALTH = re.compile(r"\[HEALTH\] (\S+) window=([\d.]+)s (.*)$")
_PAIR = re.compile(r"(\w+)=(-?[\d.]+)")


def _read_tail(path: str, limit: int = MAX_READ_BYTES) -> List[str]:
    try:
        with open(path, "rb") as handle:
            size = os.fstat(handle.fileno()).st_size
            handle.seek(max(0, size - limit))
            data = handle.read().decode("utf-8", errors="replace")
    except OSError:
        return []
    lines = data.splitlines()
    return lines[1:] if size > limit and lines else lines     # the first line is cut


def line_time(line: str) -> Optional[datetime]:
    match = _TIME.search(line)
    if not match:
        return None
    try:
        return datetime.strptime(match.group(1), "%Y-%m-%d %H:%M:%S") + timedelta(milliseconds=int(match.group(2)))
    except ValueError:
        return None


def window_lines(lines: List[str], since: datetime, until: datetime) -> Tuple[List[str], bool]:
    """(the lines from `since` to `until`, whether the log has times at all). A line without a time (a stack trace
    line, a continuation) belongs to the line before it. A log with no times at all gives its last lines."""
    kept: List[str] = []
    timed = False
    inside = False
    for line in lines:
        moment = line_time(line)
        if moment is not None:
            timed = True
            inside = since <= moment <= until
        if inside:
            kept.append(line)
    if not timed:
        return lines[-UNTIMED_TAIL_LINES:], False
    return kept, True


def unit_log_paths(log_dir: str) -> Dict[str, str]:
    """unit name -> the log of its latest run."""
    paths: Dict[str, str] = {}
    try:
        names = sorted(os.listdir(log_dir))
    except OSError:
        return paths
    for name in names:
        latest = os.path.join(log_dir, name, "latest.log")
        if os.path.lexists(latest):
            paths[name] = os.path.realpath(latest)
    return paths


def known_faults() -> Dict[str, fault_codes.Fault]:
    try:
        return {f.code: f for f in fault_codes.load()}
    except (OSError, fault_codes.FaultRegistryError):
        return {}


# ---- faults in the window -----------------------------------------------------------------------------------------
@dataclass
class FaultSeen:
    code: str
    unit: str
    count: int = 0
    first: str = ""
    last: str = ""
    sample: str = ""


def faults_in(lines_by_unit: Dict[str, List[str]],
              known: Optional[Dict[str, fault_codes.Fault]] = None) -> List[FaultSeen]:
    """Every fault code that appears in the lines, per unit, stop-class first, then by the time it first appeared."""
    known = known_faults() if known is None else known
    seen: Dict[Tuple[str, str], FaultSeen] = {}
    for unit, lines in lines_by_unit.items():
        for line in lines:
            for code in dict.fromkeys(_CODE.findall(line)):
                if code not in known:
                    continue
                item = seen.setdefault((code, unit), FaultSeen(code, unit, sample=line.strip()[:200]))
                match = _TIME.search(line)
                stamp = f"{match.group(1)}.{match.group(2)}" if match else ""
                item.count += 1
                item.first = item.first or stamp
                item.last = stamp or item.last
    return sorted(seen.values(), key=lambda f: (not known[f.code].is_stop, f.first, f.code))


def fault_section(found: List[FaultSeen], known: Optional[Dict[str, fault_codes.Fault]] = None) -> List[str]:
    known = known_faults() if known is None else known
    if not found:
        return ["fault codes in the window: none"]
    lines = ["fault codes in the window (stop-class first; what each means: docs/故障码.md):"]
    for item in found[:MAX_FAULT_ROWS]:
        cls = known[item.code].fault_class if item.code in known else "?"
        lines.append(f"  {item.code} [{cls}] {item.unit:<14} x{item.count:<5} first {item.first or '-'}  "
                     f"last {item.last or '-'}")
        lines.append(f"      {item.sample}")
    if len(found) > MAX_FAULT_ROWS:
        lines.append(f"  ... {len(found) - MAX_FAULT_ROWS} more")
    return lines


# ---- health in the window -----------------------------------------------------------------------------------------
@dataclass
class HealthSeen:
    unit: str
    name: str
    windows: int = 0
    values: Dict[str, List[float]] = field(default_factory=lambda: defaultdict(list))
    warned: int = 0


def health_in(lines_by_unit: Dict[str, List[str]]) -> List[HealthSeen]:
    """The [HEALTH] lines of the window, one entry per component: cycle lines and rate lines are both understood."""
    seen: Dict[Tuple[str, str], HealthSeen] = {}
    for unit, lines in lines_by_unit.items():
        for line in lines:
            match = _HEALTH.search(line)
            if not match:
                continue
            item = seen.setdefault((unit, match.group(1)), HealthSeen(unit, match.group(1)))
            item.windows += 1
            if "[WARN]" in line:
                item.warned += 1
            for key, value in _PAIR.findall(match.group(3)):
                item.values[key].append(float(value))
    return sorted(seen.values(), key=lambda h: (h.unit, h.name))


def health_section(found: List[HealthSeen]) -> List[str]:
    if not found:
        return ["health in the window: no [HEALTH] lines"]
    lines = ["health in the window (per component; a health line is one window of 10 s, 1 s in debug mode):"]
    for h in found:
        v = h.values
        if "cycles" in v:
            text = (f"cycles/window avg {statistics.mean(v['cycles']):.0f}, late {sum(v.get('late', [0])):.0f}, "
                    f"period max {max(v.get('period_max_ms', [0])):.2f} ms, "
                    f"duration max {max(v.get('duration_max_us', [0])):.0f} us")
        elif "rate_hz" in v:
            text = (f"rate {min(v['rate_hz']):.1f}..{max(v['rate_hz']):.1f} Hz, "
                    f"longest gap {max(v.get('gap_max_ms', [0])):.1f} ms")
        else:
            text = ", ".join(f"{k} {statistics.mean(x):.1f}" for k, x in sorted(v.items()))
        flag = f"   <- {h.warned} warning window(s)" if h.warned else ""
        lines.append(f"  {h.name:<24} {h.windows:>4} windows  {text}{flag}")
    return lines


# ---- everything ---------------------------------------------------------------------------------------------------
def gather(log_dir: str, out_dir: str, minutes: float = DEFAULT_MINUTES, reason: str = "",
           ring_dir: Optional[str] = None, now: Optional[datetime] = None) -> str:
    """Write the evidence of the last `minutes` into out_dir (made if needed); returns the report."""
    now = now or datetime.now()
    since = now - timedelta(minutes=minutes)
    os.makedirs(os.path.join(out_dir, "logs"), exist_ok=True)

    # The rings first: they keep being written, and the oldest records are the ones lost next.
    flight.snapshot(ring_dir or flight.ring_dir(), os.path.join(out_dir, "rings"))

    lines_by_unit: Dict[str, List[str]] = {}
    untimed: List[str] = []
    for unit, path in unit_log_paths(log_dir).items():
        lines, timed = window_lines(_read_tail(path), since, now)
        lines_by_unit[unit] = lines
        if not timed:
            untimed.append(unit)
        with open(os.path.join(out_dir, "logs", unit + ".log"), "w", encoding="utf-8") as handle:
            handle.write("\n".join(lines) + ("\n" if lines else ""))
    with open(os.path.join(out_dir, "events.log"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(_read_tail(os.path.join(log_dir, "events.log"))[-EVENTS_TAIL_LINES:]) + "\n")

    known = known_faults()
    report: List[str] = [
        f"collected {now.isoformat(timespec='seconds')}" + (f" - {reason}" if reason else ""),
        f"window: the last {minutes:g} minutes ({since:%H:%M:%S} to {now:%H:%M:%S}); lines kept: "
        + (", ".join(f"{u} {len(l)}" for u, l in lines_by_unit.items()) or "no unit logs"),
    ]
    if untimed:
        report.append(f"no times in the log of: {', '.join(untimed)} (its last {UNTIMED_TAIL_LINES} lines are kept)")
    report.append("")
    report += fault_section(faults_in(lines_by_unit, known), known)
    report.append("")
    report += health_section(health_in(lines_by_unit))
    report.append("")
    report.append("flight recorder rings (what each module received and sent):")
    try:
        report.append(analyze.report(out_dir, now_ns=int(now.timestamp() * 1e9)).rstrip("\n"))
    except (OSError, ValueError, flight.FlightError) as exc:      # the evidence is kept even if the analysis fails
        report.append(f"analysis failed: {exc}")
    text = "\n".join(report) + "\n"
    with open(os.path.join(out_dir, "report.txt"), "w", encoding="utf-8") as handle:
        handle.write(text)
    return text
