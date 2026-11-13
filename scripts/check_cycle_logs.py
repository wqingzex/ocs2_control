#!/usr/bin/env python3
"""List log statements that may run every control cycle or for every message without a rate limit.

    scripts/check_cycle_logs.py [--root DIR] [--all]

A heuristic aid for docs/日志.md, not a gate: it looks inside functions that are called per cycle or per message
(update, read, write, run, *Callback, ...) for RCLCPP_* calls without _THROTTLE / _ONCE, and prints them with the
function name. Most hits are events (a button press, a state change) that are fine; the ones to fix are conditions
that can stay true cycle after cycle. Functions are found by brace matching, so templates or macros can fool it.
"""
from __future__ import annotations

import argparse
import collections
import os
import re
import subprocess
import sys

HOT = re.compile(r"^(update|read|write|run|onCycleBegin|onCycleEnd|[A-Za-z]*Callback|[A-Za-z_]*_callback|"
                 r"calNextPoint|getNextJointPos|getNextPoint|stepLineArm|processFrame|handleFrame)$")
LOG = re.compile(r"RCLCPP_(DEBUG|INFO|WARN|ERROR|FATAL)(_STREAM)?(_THROTTLE|_ONCE|_SKIPFIRST[A-Z_]*|_EXPRESSION)?\(")
FUNCTION = re.compile(r"(?<![\w:>])((?:[A-Za-z_]\w*::)*([A-Za-z_]\w*))\s*\([^;{}]*\)\s*(?:const)?\s*(?:override)?\s*"
                      r"(?:noexcept)?\s*\{")
SKIP_PARTS = ("third_party", "/external/", "/examples/", "/test/")


def source_files(root: str):
    listed = subprocess.run(["git", "ls-files", "-z", "src"], capture_output=True, text=True, cwd=root).stdout
    for rel in listed.split("\0"):
        if rel.endswith((".cpp", ".h", ".hpp")) and not any(part in rel for part in SKIP_PARTS):
            yield rel


def functions(text: str):
    for match in FUNCTION.finditer(text):
        name = match.group(2)
        if name in ("if", "for", "while", "switch", "catch"):
            continue
        depth, index = 0, match.end() - 1
        while index < len(text):
            if text[index] == "{":
                depth += 1
            elif text[index] == "}":
                depth -= 1
                if depth == 0:
                    break
            index += 1
        yield name, match.start(), index


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--root", default=os.getcwd(), help="repository root (default: cwd)")
    parser.add_argument("--all", action="store_true", help="also list DEBUG statements (hidden unless debug is on)")
    args = parser.parse_args(argv)
    hits = collections.defaultdict(list)
    for rel in source_files(args.root):
        try:
            with open(os.path.join(args.root, rel), encoding="utf-8", errors="replace") as handle:
                text = handle.read()
        except OSError:
            continue
        for name, start, end in functions(text):
            if not HOT.match(name):
                continue
            for log in LOG.finditer(text[start:end]):
                if log.group(3) or (log.group(1) == "DEBUG" and not args.all):
                    continue
                hits[rel].append((text.count("\n", 0, start + log.start()) + 1, name, log.group(1)))
    total = sum(len(items) for items in hits.values())
    print(f"{total} unthrottled log statements in per-cycle / per-message functions, {len(hits)} files")
    for rel, items in sorted(hits.items(), key=lambda entry: -len(entry[1])):
        for line, name, level in items:
            print(f"{rel}:{line}  {name}()  {level}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
