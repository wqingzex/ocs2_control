"""Reads the rings of an incident (or of a live directory) together and says which hop went wrong first.

    python3 -m robot_log_tools.analyze logs/incidents/<id>          report to the terminal
    python3 -m robot_log_tools.analyze DIR --timeline merged.csv    also write every record of every module in time order
    python3 -m robot_log_tools.analyze DIR --step-limit 0.3         the largest change of a commanded joint per cycle, rad

DIR is an incident directory (its rings/ is used) or a directory holding *.ring files. All modules run on one host, so
the wall clock of the records lines them up. The report has, per module and record type: how many records, the rate and
the longest silence; then the delay across each hop of the chain; then the anomalies in time order, the earliest first.

What it can tell: a module that stopped writing (rings left by an older run are told apart by when they were made), a hole in a stream, a late control cycle, a commanded joint that jumped,
the delay between hops. What it cannot: why. It points at the first hop that went wrong; the unit logs say what happened.
"""
from __future__ import annotations

import argparse
import bisect
import csv
import os
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

from robot_log_tools import flight

MIN_RECORDS = 20             # fewer than this is not a stream worth judging
MIN_RATE_HZ = 5.0            # slower streams (events) have no meaningful gap
GAP_FACTOR = 4.0             # a silence this many times the usual period is a hole ...
MIN_GAP_MS = 50.0            # ... if it is also at least this long
LATE_FACTOR = 1.5            # a control cycle this many times the usual period is late (the controller's own rule)
STOPPED_S = 1.0              # a module whose last record is this much older than the newest one has stopped writing
SILENT_S = 5.0               # no record at all this long before the moment of collection
STEP_LIMIT_RAD = 0.2         # a commanded joint that moves more than this in one cycle is a jump
LATE_CHECKED = ("ctl_state", "hw_read")   # one record per cycle: the start of a late cycle shows in the gaps
NO_JUMP_CHECK = ("hand", "gripper")   # these open and close in one step on purpose
SAME_RUN_S = 15.0            # rings made within this time of the newest ring belong to the same run (a run makes them within seconds)


@dataclass
class Stream:
    """The records of one type written by one module, oldest first."""
    module: str
    name: str
    type_id: int
    wall_ns: List[int]
    steady_ns: List[int]
    aux: List[int]
    values: List[tuple]
    start_wall_ns: int = 0       # when the module's ring was made: a ring from an older run starts much earlier

    @property
    def label(self) -> str:
        return f"{self.module}/{self.name}"

    @property
    def is_previous_run(self) -> bool:
        return self.module.endswith(".prev")


@dataclass(frozen=True)
class Anomaly:
    wall_ns: int
    stream: str
    kind: str          # stopped | gap | late | jump
    detail: str


def _type_names(types: Optional[Dict[int, dict]]) -> Dict[int, str]:
    table = flight.load_types() if types is None else types
    return {tid: entry["name"] for tid, entry in table.items()}


def find_ring_files(path: str, include_prev: bool = False) -> List[str]:
    rings = os.path.join(path, "rings")
    directory = rings if os.path.isdir(rings) else path
    return flight.ring_files(directory, include_prev=include_prev)


def load_streams(path: str, include_prev: bool = False, types: Optional[Dict[int, dict]] = None) -> List[Stream]:
    """One stream per (module, record type) of every readable ring under `path`."""
    names = _type_names(types)
    streams: List[Stream] = []
    for ring_path in find_ring_files(path, include_prev):
        try:
            ring = flight.read_ring(ring_path)
        except (OSError, flight.FlightError):
            continue
        module = ring.module + (".prev" if ring_path.endswith(".prev") else "")
        by_type: Dict[int, Stream] = {}
        for record in ring.records:
            stream = by_type.get(record.type)
            if stream is None:
                stream = by_type[record.type] = Stream(module, names.get(record.type, f"type_{record.type}"),
                                                       record.type, [], [], [], [], ring.start_wall_ns)
            stream.wall_ns.append(record.wall_ns)
            stream.steady_ns.append(record.steady_ns)
            stream.aux.append(record.aux)
            stream.values.append(record.values)
        streams.extend(by_type.values())
    return sorted(streams, key=lambda s: (s.module, s.type_id))


# ---- one stream ----------------------------------------------------------------------------------------------------
def intervals_ms(stream: Stream) -> List[float]:
    steady = stream.steady_ns
    return [(b - a) / 1e6 for a, b in zip(steady, steady[1:])]


def summary(stream: Stream) -> Dict[str, float]:
    gaps = intervals_ms(stream)
    span_s = (stream.steady_ns[-1] - stream.steady_ns[0]) / 1e9 if len(stream.steady_ns) > 1 else 0.0
    return {"count": len(stream.wall_ns), "span_s": span_s,
            "rate_hz": (len(gaps) / span_s) if span_s > 0 else 0.0,
            "median_ms": statistics.median(gaps) if gaps else 0.0,
            "max_gap_ms": max(gaps) if gaps else 0.0}


def find_gaps(stream: Stream) -> List[Anomaly]:
    """Silences much longer than the stream's usual period."""
    if len(stream.wall_ns) < MIN_RECORDS:
        return []
    gaps = intervals_ms(stream)
    median = statistics.median(gaps)
    if median <= 0 or 1000.0 / median < MIN_RATE_HZ:
        return []
    limit = max(GAP_FACTOR * median, MIN_GAP_MS)
    return [Anomaly(stream.wall_ns[i + 1], stream.label, "gap",
                    f"{gap:.0f} ms without a record (usual period {median:.1f} ms)")
            for i, gap in enumerate(gaps) if gap > limit]


def find_late_cycles(stream: Stream) -> List[Anomaly]:
    """Control cycles that started later than LATE_FACTOR periods after the previous one (one record per cycle).
    A silence long enough to be a hole is reported by find_gaps instead."""
    if len(stream.wall_ns) < MIN_RECORDS:
        return []
    gaps = intervals_ms(stream)
    median = statistics.median(gaps)
    if median <= 0:
        return []
    hole = max(GAP_FACTOR * median, MIN_GAP_MS)
    return [Anomaly(stream.wall_ns[i + 1], stream.label, "late",
                    f"cycle started {gap:.1f} ms after the previous (usual {median:.1f} ms)")
            for i, gap in enumerate(gaps) if LATE_FACTOR * median < gap <= hole]


def find_jumps(command: Stream, step_limit: float = STEP_LIMIT_RAD) -> List[Anomaly]:
    """A commanded joint that moves more than `step_limit` between two cycles in the same state."""
    found: List[Anomaly] = []
    for i in range(1, len(command.values)):
        if command.aux[i] != command.aux[i - 1]:
            continue          # the state changed: the target legitimately differs
        steps = [abs(b - a) for a, b in zip(command.values[i - 1], command.values[i])]
        if not steps:
            continue
        worst = max(range(len(steps)), key=steps.__getitem__)
        if steps[worst] > step_limit:
            found.append(Anomaly(command.wall_ns[i], command.label, "jump",
                                 f"joint {worst} commanded {steps[worst]:.3f} rad further in one cycle"))
    return found


def current_run(streams: Sequence[Stream]) -> Tuple[List[Stream], List[str]]:
    """The streams of the newest run, and the modules left out. A run makes its rings within seconds of each other;
    a ring in shared memory that started much earlier belongs to an older run (another robot mode, a module that is
    no longer started) and would only add false findings."""
    current = [s for s in streams if not s.is_previous_run and s.wall_ns]
    if not current:
        return [], []
    run_start = max(s.start_wall_ns for s in current) - int(SAME_RUN_S * 1e9)
    kept = [s for s in current if s.start_wall_ns >= run_start]
    left_out = sorted({s.module for s in current if s.start_wall_ns < run_start})
    return kept, left_out


def find_stopped(streams: Sequence[Stream], now_ns: Optional[int] = None) -> List[Anomaly]:
    """A module whose newest record is much older than the newest record of the others: it stopped writing
    (crashed, hung or was killed) while the rest went on. Pass the streams of one run (see current_run).
    With the moment of collection (now_ns), a module that wrote nothing for SILENT_S before it is also reported, as
    "silent": that is how a killed process shows when everything in it stopped together (no module left to compare
    with). A module that only had nothing to record looks the same, so the text says both."""
    newest_by_module: Dict[str, int] = {}
    for stream in streams:
        if not stream.is_previous_run and stream.wall_ns:
            newest_by_module[stream.module] = max(newest_by_module.get(stream.module, 0), stream.wall_ns[-1])
    found: List[Anomaly] = []
    if len(newest_by_module) >= 2:
        newest = max(newest_by_module.values())
        found += [Anomaly(wall, module, "stopped",
                          f"last record {(newest - wall) / 1e9:.1f} s before the newest of any module")
                  for module, wall in newest_by_module.items() if (newest - wall) / 1e9 > STOPPED_S]
    if now_ns is not None:
        already = {a.stream for a in found}
        found += [Anomaly(wall, module, "silent",
                          f"no record for {(now_ns - wall) / 1e9:.1f} s before the collection "
                          f"(the process stopped, or it had nothing to record)")
                  for module, wall in newest_by_module.items()
                  if (now_ns - wall) / 1e9 > SILENT_S and module not in already]
    return found


# ---- hops ------------------------------------------------------------------------------------------------------------
def hop_delays_ms(source: Stream, sink: Stream, source_aux: Optional[int] = None,
                  sink_aux: Optional[int] = None) -> List[float]:
    """For every record of `sink`, the time since the latest earlier record of `source` (wall clock)."""
    times = [t for t, a in zip(source.wall_ns, source.aux) if source_aux is None or a == source_aux]
    delays: List[float] = []
    for t, a in zip(sink.wall_ns, sink.aux):
        if sink_aux is not None and a != sink_aux:
            continue
        index = bisect.bisect_right(times, t) - 1
        if index >= 0:
            delays.append((t - times[index]) / 1e6)
    return delays


def percentile(values: Sequence[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(fraction * len(ordered)))] if ordered else 0.0


# (source stream, sink stream, title, match the hand/arm side of the two)
HOPS = (("xr_bridge/xr_frame_in", "xr_bridge/xr_publish", "headset frame received -> published", False),
        ("xr_bridge/xr_publish", "teleop_command/tel_xr_in", "published by the bridge -> seen by teleop_command", False),
        ("teleop_command/tel_xr_in", "teleop_command/tel_target_out", "XR pose in -> target out (same hand)", True))


def hop_table(streams: Sequence[Stream]) -> List[Tuple[str, List[float]]]:
    """Delays across the hops of the chain that the rings can show."""
    by_label = {s.label: s for s in streams if not s.is_previous_run}
    hops: List[Tuple[str, List[float]]] = []
    for source_label, sink_label, title, per_side in HOPS:
        source, sink = by_label.get(source_label), by_label.get(sink_label)
        if source is None or sink is None:
            continue
        if per_side:
            delays = [d for side in (0, 1) for d in hop_delays_ms(source, sink, side, side)]
        else:
            delays = hop_delays_ms(source, sink)
        if delays:
            hops.append((title, delays))
    return hops


# ---- the report ----------------------------------------------------------------------------------------------------
def collect_anomalies(streams: Sequence[Stream], step_limit: float = STEP_LIMIT_RAD,
                      now_ns: Optional[int] = None) -> List[Anomaly]:
    streams, _ = current_run(streams)
    found: List[Anomaly] = find_stopped(streams, now_ns)
    for stream in streams:
        if stream.is_previous_run:
            continue
        found += find_gaps(stream)
        if stream.name in LATE_CHECKED:
            found += find_late_cycles(stream)
        if stream.name == "ctl_command" and not any(word in stream.module for word in NO_JUMP_CHECK):
            found += find_jumps(stream, step_limit)
    return sorted(found, key=lambda a: a.wall_ns)


def fsm_changes(streams: Sequence[Stream]) -> List[Tuple[int, str, tuple]]:
    return sorted((wall, s.module, v) for s in streams if s.name == "ctl_fsm" for wall, v in zip(s.wall_ns, s.values))


def report(path: str, step_limit: float = STEP_LIMIT_RAD, max_anomalies: int = 30, now_ns: Optional[int] = None) -> str:
    streams, older = current_run(load_streams(path))
    if not streams:
        return f"no readable rings under {path}\n"
    lines: List[str] = []
    first, last = min(s.wall_ns[0] for s in streams), max(s.wall_ns[-1] for s in streams)
    lines.append(f"{len({s.module for s in streams})} modules, records from {flight.iso(first)} to {flight.iso(last)} "
                 f"({(last - first) / 1e9:.1f} s)")
    if older:
        lines.append(f"left out, rings of an older run: {', '.join(older)}")
    lines.append("")
    lines.append("streams (records, rate, usual period, longest silence):")
    for s in streams:
        m = summary(s)
        lines.append(f"  {s.label:<42} {m['count']:>7.0f}  {m['rate_hz']:>7.1f} Hz  {m['median_ms']:>7.1f} ms  "
                     f"{m['max_gap_ms']:>8.1f} ms")
    hops = hop_table(streams)
    if hops:
        lines.append("")
        lines.append("hop delays (median / 95% / max, ms):")
        for title, delays in hops:
            lines.append(f"  {title:<48} {statistics.median(delays):>8.1f} {percentile(delays, 0.95):>8.1f} {max(delays):>8.1f}")
    changes = fsm_changes(streams)
    if changes:
        lines.append("")
        lines.append("state machine changes (from -> to, fsm_command):")
        for wall, module, v in changes[-10:]:
            lines.append(f"  {flight.iso(wall)}  {module}: {int(v[0])} -> {int(v[1])}  command {int(v[2])}")
    anomalies = collect_anomalies(streams, step_limit, now_ns)
    lines.append("")
    if not anomalies:
        lines.append("anomalies: none found in these records")
    else:
        lines.append(f"anomalies, earliest first ({len(anomalies)} in all; the first is where to start looking):")
        for a in anomalies[:max_anomalies]:
            lines.append(f"  {flight.iso(a.wall_ns)}  {a.stream:<36} {a.kind:<8} {a.detail}")
        if len(anomalies) > max_anomalies:
            lines.append(f"  ... {len(anomalies) - max_anomalies} more")
    return "\n".join(lines) + "\n"


def write_timeline(path: str, out_path: str) -> int:
    """Every record of every module in wall clock order, one CSV."""
    streams, _ = current_run(load_streams(path))
    rows = sorted(((wall, s.module, s.name, aux, v) for s in streams
                   for wall, aux, v in zip(s.wall_ns, s.aux, s.values)), key=lambda r: r[0])
    width = max((len(r[4]) for r in rows), default=0)
    with open(out_path, "w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["wall_time", "module", "type", "aux"] + [f"v{i}" for i in range(width)])
        for wall, module, name, aux, values in rows:
            writer.writerow([flight.iso(wall), module, name, aux] + list(values))
    return len(rows)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(prog="analyze", description=__doc__.splitlines()[0])
    parser.add_argument("path", help="an incident directory, or a directory of *.ring files")
    parser.add_argument("--now", action="store_true", help="the moment of the report is now: report modules that went silent")
    parser.add_argument("--timeline", help="also write all records in time order to this CSV")
    parser.add_argument("--step-limit", type=float, default=STEP_LIMIT_RAD,
                        help="commanded joint change per cycle counted as a jump, rad (default %(default)s)")
    args = parser.parse_args(argv)
    try:
        sys.stdout.write(report(args.path, args.step_limit, now_ns=time.time_ns() if args.now else None))
        if args.timeline:
            print(f"{write_timeline(args.path, args.timeline)} records written to {args.timeline}")
    except (OSError, flight.FlightError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
