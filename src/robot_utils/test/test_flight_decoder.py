"""The flight recorder decoder: layout, wrapping, CSV, the record type table against the C++ constants."""
import io
import os
import re
import struct

import pytest

from robot_log_tools import flight

CAPACITY = 8


def make_ring(path, records, capacity=CAPACITY, module="unit", written=None):
    """Writes a ring file the way flight.hpp does. records: list of (type, aux, values), record i in slot i % capacity."""
    data = bytearray(flight.HEADER_SIZE + capacity * flight.RECORD_SIZE)
    flight.HEADER.pack_into(data, 0, flight.MAGIC, 1, flight.RECORD_SIZE, capacity, flight.HEADER_SIZE,
                            module.encode(), 1_700_000_000_000_000_000, 5_000_000, 4242, 0)
    for i, (rtype, aux, values) in enumerate(records):
        padded = list(values) + [0.0] * (28 - len(values))
        flight.SLOT.pack_into(data, flight.HEADER_SIZE + (i % capacity) * flight.RECORD_SIZE,
                              i + 1, 1_000_000 * i, 1_700_000_000_000_000_000 + 1_000_000 * i,
                              rtype, len(values), aux, *padded)
    struct.pack_into("<Q", data, flight.WRITE_INDEX_OFFSET, len(records) if written is None else written)
    path.write_bytes(bytes(data))
    return path


def test_layout_constants_match_the_slot_struct():
    assert flight.SLOT.size == flight.RECORD_SIZE
    assert flight.HEADER.size <= flight.WRITE_INDEX_OFFSET


def test_reads_records_in_order_with_header_fields(tmp_path):
    path = make_ring(tmp_path / "unit.ring", [(7, 11, [1.0, 2.0]), (8, 0, []), (9, 3, [-4.5])])
    ring = flight.read_ring(str(path))
    assert (ring.module, ring.pid, ring.written) == ("unit", 4242, 3)
    assert [r.type for r in ring.records] == [7, 8, 9]
    assert ring.records[0].values == (1.0, 2.0) and ring.records[0].aux == 11
    assert ring.records[1].values == ()


def test_a_wrapped_ring_keeps_the_newest_and_skips_the_oldest(tmp_path):
    records = [(1, 0, [float(i)]) for i in range(20)]
    path = tmp_path / "w.ring"
    # records 12..19 occupy the slots; write them the way the C++ writer would have left them
    make_ring(path, records)
    ring = flight.read_ring(str(path), skip_oldest=0)
    assert [r.values[0] for r in ring.records] == [float(i) for i in range(12, 20)]
    skipped = flight.read_ring(str(path), skip_oldest=2)
    assert [r.values[0] for r in skipped.records] == [float(i) for i in range(14, 20)]


def test_a_slot_that_is_being_written_is_left_out(tmp_path):
    path = make_ring(tmp_path / "b.ring", [(1, 0, [1.0]), (1, 0, [2.0]), (1, 0, [3.0])])
    data = bytearray(path.read_bytes())
    struct.pack_into("<Q", data, flight.HEADER_SIZE + 1 * flight.RECORD_SIZE, 0)   # seq cleared: mid-write
    path.write_bytes(bytes(data))
    assert [r.values[0] for r in flight.read_ring(str(path)).records] == [1.0, 3.0]


def test_garbage_and_short_files_are_refused(tmp_path):
    (tmp_path / "junk.ring").write_bytes(b"x" * 8192)
    (tmp_path / "short.ring").write_bytes(b"x" * 10)
    for name in ("junk.ring", "short.ring"):
        with pytest.raises(flight.FlightError):
            flight.read_ring(str(tmp_path / name))


def test_csv_has_one_row_per_record_and_names_the_type(tmp_path):
    path = make_ring(tmp_path / "c.ring", [(201, 1, [0.1, 0.2, 0.3, 1, 0, 0, 0]), (999, 0, [5.0])])
    out = io.StringIO()
    assert flight.write_csv(flight.read_ring(str(path)), out) == 2
    lines = out.getvalue().strip().splitlines()
    assert lines[0].startswith("module,index,wall_time,wall_ns,steady_ms,type,aux,v0")
    assert ",tel_xr_in,1,0.1," in lines[1]
    assert ",type_999,0,5.0" in lines[2]


def test_column_names_follow_the_type_table():
    types = flight.load_types()
    assert flight.column_names(types[201], 7) == ["x", "y", "z", "qw", "qx", "qy", "qz"]
    assert flight.column_names(types[302], 3) == ["position0", "position1", "position2"]
    assert flight.column_names(None, 2) == ["v0", "v1"]


def test_snapshot_copies_rings_and_prev_but_nothing_else(tmp_path):
    src, dest = tmp_path / "src", tmp_path / "dest"
    src.mkdir()
    make_ring(src / "a.ring", [(1, 0, [1.0])])
    make_ring(src / "a.ring.prev", [(1, 0, [0.0])])
    (src / "notes.txt").write_text("x")
    copied = flight.snapshot(str(src), str(dest))
    assert sorted(os.path.basename(p) for p in copied) == ["a.ring", "a.ring.prev"]
    assert flight.snapshot(str(tmp_path / "missing"), str(tmp_path / "d2")) == []


def test_cli_list_and_csv(tmp_path, capsys):
    make_ring(tmp_path / "m.ring", [(1, 0, [1.0])], module="m")
    assert flight.main(["list", "--dir", str(tmp_path)]) == 0
    assert "module m" in capsys.readouterr().out
    assert flight.main(["csv", str(tmp_path / "nope.ring")]) == 1


def test_type_table_agrees_with_the_cpp_constants():
    types = flight.load_types()
    here = os.path.dirname(flight.default_types_path())
    header = os.path.join(here, "..", "include", "robot_log", "flight_types.hpp")
    constants = {int(v) for v in re.findall(r"constexpr std::uint16_t k\w+\s*=\s*(\d+);", open(header).read())}
    assert constants == set(types), "flight_types.hpp and flight_types.yaml list different record types"
    for entry in types.values():
        assert entry["id"] // 100 in (1, 2, 3, 4), "the hundreds digit is the chain segment"
