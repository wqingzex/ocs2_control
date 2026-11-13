#!/usr/bin/env python3
"""Measure how much a running system logs and what it costs, in a fixed time window.

    measure_log_load.py --log /tmp/launch.log --seconds 60 --label idle [--json out.json]

Reads the growth of one log file (a `ros2 launch` console log) and, in the same window, samples CPU and memory of
the processes whose command line matches --match. Per log node it reports lines per second, bytes per minute and the
share of each level. Standard library only; meant to be run before and after logging changes (docs/日志.md).
"""
from __future__ import annotations

import argparse
import collections
import json
import os
import re
import sys
import time
from typing import Dict, List

# "[ros2_control_node-2] [INFO] [2026-11-13 03:55:05.612] [WholeBodyInterface]: message"
# (or, with rcutils' default format, a seconds-since-epoch number in the time field)
LINE = re.compile(r"^\[(?P<proc>[^\]]+)\] \[(?P<level>DEBUG|INFO|WARN|ERROR|FATAL)\] \[[^\]]+\] \[(?P<node>[^\]]+)\]")
CLK_TCK = os.sysconf("SC_CLK_TCK")
PAGE_KB = os.sysconf("SC_PAGE_SIZE") // 1024


def matching_pids(patterns: List[str]) -> List[int]:
    found = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as handle:
                cmd = handle.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if any(pattern in cmd for pattern in patterns) and "measure_log_load" not in cmd:
            found.append(int(entry))
    return sorted(found)


def cpu_ticks(pid: int) -> int:
    try:
        with open(f"/proc/{pid}/stat") as handle:
            fields = handle.read().rsplit(")", 1)[1].split()
        return int(fields[11]) + int(fields[12])  # utime + stime
    except (OSError, IndexError, ValueError):
        return 0


def rss_kb(pid: int) -> int:
    try:
        with open(f"/proc/{pid}/statm") as handle:
            return int(handle.read().split()[1]) * PAGE_KB
    except (OSError, IndexError, ValueError):
        return 0


def command_name(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            parts = handle.read().split(b"\0")
        first = parts[0].decode(errors="replace")
        if os.path.basename(first).startswith("python") and len(parts) > 1:
            return os.path.basename(parts[1].decode(errors="replace"))
        return os.path.basename(first)
    except OSError:
        return str(pid)


def measure(log_path: str, seconds: float, patterns: List[str]) -> Dict:
    pids = matching_pids(patterns)
    start_ticks = {pid: cpu_ticks(pid) for pid in pids}
    start_size = os.path.getsize(log_path) if os.path.exists(log_path) else 0
    start_time = time.monotonic()
    peak_rss = {pid: rss_kb(pid) for pid in pids}
    while time.monotonic() - start_time < seconds:
        time.sleep(1.0)
        for pid in pids:
            peak_rss[pid] = max(peak_rss[pid], rss_kb(pid))
    elapsed = time.monotonic() - start_time

    lines = 0
    per_node: Dict[str, collections.Counter] = collections.defaultdict(collections.Counter)
    bytes_per_node: Dict[str, int] = collections.defaultdict(int)
    unleveled = 0
    unleveled_bytes = 0
    with open(log_path, "rb") as handle:
        handle.seek(start_size)
        for raw in handle:
            text = raw.decode(errors="replace")
            lines += 1
            match = LINE.match(text)
            if match:
                per_node[match["node"]][match["level"]] += 1
                bytes_per_node[match["node"]] += len(raw)
            else:
                unleveled += 1
                unleveled_bytes += len(raw)
    total_bytes = sum(bytes_per_node.values()) + unleveled_bytes

    processes = []
    for pid in pids:
        used = (cpu_ticks(pid) - start_ticks[pid]) / CLK_TCK
        processes.append({"pid": pid, "name": command_name(pid), "cpu_percent": round(100.0 * used / elapsed, 2),
                          "rss_mb": round(peak_rss[pid] / 1024.0, 1)})
    nodes_report = []
    for node, levels in sorted(per_node.items(), key=lambda item: -sum(item[1].values())):
        count = sum(levels.values())
        nodes_report.append({"node": node, "lines_per_s": round(count / elapsed, 3),
                             "bytes_per_min": int(bytes_per_node[node] * 60.0 / elapsed),
                             "levels": dict(levels)})
    return {"seconds": round(elapsed, 1), "lines": lines, "lines_per_s": round(lines / elapsed, 2),
            "bytes_per_min": int(total_bytes * 60.0 / elapsed), "unleveled_lines": unleveled,
            "nodes": nodes_report, "processes": processes}


def print_report(label: str, report: Dict) -> None:
    print(f"== {label}: {report['seconds']} s window, {report['lines']} lines "
          f"({report['lines_per_s']}/s, {report['bytes_per_min'] / 1024:.1f} KB/min), "
          f"{report['unleveled_lines']} lines without a level prefix")
    for entry in report["nodes"][:12]:
        levels = " ".join(f"{k}={v}" for k, v in sorted(entry["levels"].items()))
        print(f"   {entry['node']:<34s} {entry['lines_per_s']:>7.2f}/s {entry['bytes_per_min'] / 1024:>8.1f} KB/min  {levels}")
    for entry in report["processes"]:
        print(f"   proc {entry['name']:<28s} pid={entry['pid']:<7d} cpu={entry['cpu_percent']:>6.2f}%  rss={entry['rss_mb']:>7.1f} MB")


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--log", required=True, help="console log of the running ros2 launch")
    parser.add_argument("--seconds", type=float, default=60.0)
    parser.add_argument("--label", default="window")
    parser.add_argument("--match", action="append",
                        help="substring of the command line of a process to sample (repeatable); default: the "
                             "controller manager, robot_state_publisher, teleop_command and the XR bridge")
    parser.add_argument("--json", help="write the report here")
    args = parser.parse_args(argv)
    patterns = args.match or ["ros2_control_node", "robot_state_publisher", "teleop_command_node", "xr_bridge_node"]
    report = measure(args.log, args.seconds, patterns)
    report["label"] = args.label
    print_report(args.label, report)
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2, ensure_ascii=False)
    return 0


if __name__ == "__main__":
    sys.exit(main())
