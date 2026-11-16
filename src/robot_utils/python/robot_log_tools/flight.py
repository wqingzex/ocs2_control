"""Reads the rings of the flight recorder (robot_log/flight.hpp) and turns them into something a person can look at.

    python3 -m robot_log_tools.flight list [--dir D]                      the rings and how much each holds
    python3 -m robot_log_tools.flight csv RING.ring [-o OUT.csv]          one ring as CSV
    python3 -m robot_log_tools.flight snapshot DEST [--dir D]             copy every ring (and .prev) to DEST

A ring is a file in shared memory that every module writes to as it receives and sends data. It outlives a crashed
module, so after an incident the files can be copied and read even if the process that wrote them is gone.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import os
import shutil
import struct
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml

from robot_log_tools import config_path

DEFAULT_DIR = "/dev/shm/robot_flight"
MAGIC = b"RFLT0001"
# magic, version, record_size, capacity, header_size, module, start_wall_ns, start_steady_ns, pid, reserved
HEADER = struct.Struct("<8sIIII32sQQII")
WRITE_INDEX_OFFSET = 80
HEADER_SIZE = 128
RECORD_SIZE = 256
SLOT = struct.Struct("<QQQHHI28d")            # seq, steady_ns, wall_ns, type, count, aux, v[28]
SKIP_OLDEST = 64                              # a copy taken while a writer runs can mix bytes in the oldest slots


class FlightError(ValueError):
    """The file is not a ring (or not one this reader understands)."""


@dataclass(frozen=True)
class Record:
    index: int
    steady_ns: int
    wall_ns: int
    type: int
    aux: int
    values: tuple


@dataclass
class Ring:
    module: str
    pid: int
    start_wall_ns: int
    start_steady_ns: int
    capacity: int
    written: int
    records: List[Record]


def ring_dir() -> str:
    return os.environ.get("ROBOT_FLIGHT_DIR") or DEFAULT_DIR


def read_ring(path: str, skip_oldest: int = SKIP_OLDEST) -> Ring:
    """The valid records of a ring, oldest first. A ring that has wrapped loses its `skip_oldest` oldest records:
    those are the slots a writer was about to reuse, and a copy made while it runs can mix old and new bytes there."""
    with open(path, "rb") as handle:
        data = handle.read()
    if len(data) < HEADER_SIZE:
        raise FlightError(f"{path}: too short to be a ring")
    magic, version, record_size, capacity, header_size, module, wall, steady, pid, _ = HEADER.unpack_from(data, 0)
    if magic != MAGIC or version != 1 or record_size != RECORD_SIZE or header_size != HEADER_SIZE or capacity < 2:
        raise FlightError(f"{path}: not a flight recorder ring (magic {magic!r}, version {version})")
    if len(data) < HEADER_SIZE + capacity * RECORD_SIZE:
        raise FlightError(f"{path}: the file is shorter than its header says")
    (written,) = struct.unpack_from("<Q", data, WRITE_INDEX_OFFSET)
    first = max(0, written - capacity)
    if written > capacity:
        first += min(skip_oldest, capacity // 4)   # never more than a quarter of a small ring
    records: List[Record] = []
    for index in range(first, written):
        offset = HEADER_SIZE + (index % capacity) * RECORD_SIZE
        seq, steady_ns, wall_ns, rtype, count, aux, *values = SLOT.unpack_from(data, offset)
        if seq != index + 1:
            continue          # being written, or overwritten since
        records.append(Record(index, steady_ns, wall_ns, rtype, aux, tuple(values[:min(count, 28)])))
    return Ring(module.split(b"\0", 1)[0].decode("ascii", "replace"), pid, wall, steady, capacity, written, records)


def default_types_path() -> str:
    """config/flight_types.yaml of robot_log."""
    return config_path("flight_types.yaml")


def load_types(path: Optional[str] = None) -> Dict[int, dict]:
    with open(path or default_types_path(), encoding="utf-8") as handle:
        entries = yaml.safe_load(handle)
    types: Dict[int, dict] = {}
    for entry in entries:
        if entry["id"] in types:
            raise FlightError(f"duplicate record type id {entry['id']}")
        if ("fields" in entry) == ("array" in entry):
            raise FlightError(f"record type {entry['id']}: give either fields or array")
        types[entry["id"]] = entry
    return types


def column_names(entry: Optional[dict], count: int) -> List[str]:
    if entry is None:
        return [f"v{i}" for i in range(count)]
    if "fields" in entry:
        names = list(entry["fields"])
        return names + [f"v{i}" for i in range(len(names), count)]
    return [f"{entry['array']}{i}" for i in range(count)]


def iso(wall_ns: int) -> str:
    return datetime.datetime.fromtimestamp(wall_ns / 1e9).isoformat(timespec="milliseconds")


def write_csv(ring: Ring, out, types: Optional[Dict[int, dict]] = None) -> int:
    """One row per record: time, type, aux, then the values (v0, v1, ...). Names of the values per type are in
    `column_names`. Returns the rows written."""
    types = load_types() if types is None else types
    width = max((len(r.values) for r in ring.records), default=0)
    writer = csv.writer(out)
    writer.writerow(["module", "index", "wall_time", "wall_ns", "steady_ms", "type", "aux"] + [f"v{i}" for i in range(width)])
    for record in ring.records:
        entry = types.get(record.type)
        name = entry["name"] if entry else f"type_{record.type}"
        writer.writerow([ring.module, record.index, iso(record.wall_ns), record.wall_ns,
                         f"{record.steady_ns / 1e6:.3f}", name, record.aux] + list(record.values))
    return len(ring.records)


def ring_files(directory: str, include_prev: bool = True) -> List[str]:
    try:
        names = sorted(os.listdir(directory))
    except OSError:
        return []
    return [os.path.join(directory, n) for n in names
            if n.endswith(".ring") or (include_prev and n.endswith(".ring.prev"))]


def snapshot(directory: str, dest: str) -> List[str]:
    """Copy every ring (and every .ring.prev, the one before a restart) to `dest`. Safe while the writers run."""
    copied = []
    os.makedirs(dest, exist_ok=True)
    for path in ring_files(directory):
        target = os.path.join(dest, os.path.basename(path))
        try:
            shutil.copyfile(path, target)
            copied.append(target)
        except OSError:
            pass                      # vanished between the listing and the copy
    return copied


def _describe(path: str) -> str:
    try:
        ring = read_ring(path)
    except (OSError, FlightError) as exc:
        return f"{os.path.basename(path)}: unreadable ({exc})"
    if ring.records:
        span = (ring.records[-1].steady_ns - ring.records[0].steady_ns) / 1e9
        return (f"{os.path.basename(path)}: module {ring.module}, pid {ring.pid}, {len(ring.records)} records "
                f"over {span:.1f} s, {ring.written} written in all")
    return f"{os.path.basename(path)}: module {ring.module}, pid {ring.pid}, empty"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)
    listing = sub.add_parser("list")
    listing.add_argument("--dir", default=None)
    one = sub.add_parser("csv")
    one.add_argument("ring")
    one.add_argument("-o", "--output")
    snap = sub.add_parser("snapshot")
    snap.add_argument("dest")
    snap.add_argument("--dir", default=None)
    args = parser.parse_args(argv)
    try:
        if args.command == "list":
            files = ring_files(args.dir or ring_dir())
            for path in files:
                print(_describe(path))
            if not files:
                print("no rings found")
        elif args.command == "csv":
            ring = read_ring(args.ring)
            if args.output:
                with open(args.output, "w", encoding="utf-8", newline="") as handle:
                    write_csv(ring, handle)
            else:
                write_csv(ring, sys.stdout)
        else:
            copied = snapshot(args.dir or ring_dir(), args.dest)
            print(f"{len(copied)} ring file(s) copied to {args.dest}")
    except (OSError, FlightError, yaml.YAMLError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
