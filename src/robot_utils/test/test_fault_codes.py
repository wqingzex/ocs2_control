"""The fault code registry: well formed, anchored in the code, and the document is generated from it."""
import os
from pathlib import Path

import pytest

from robot_log_tools import fault_codes
from robot_log_tools.fault_codes import FaultRegistryError

REPO_ROOT = Path(__file__).resolve().parents[3]
DOC = REPO_ROOT / "docs" / "故障码.md"


def entry(**changes):
    base = {"code": "HW-001", "class": "S1", "title": "t", "behavior": "b", "check": "c",
            "source": {"file": "a.cpp", "match": "text"}}
    base.update(changes)
    return {key: value for key, value in base.items() if value is not None}


# --- the registry that ships ---------------------------------------------------------------------------------
def test_the_registry_loads_and_has_every_class_and_segment():
    faults = fault_codes.load()
    assert {fault.fault_class for fault in faults} == set(fault_codes.CLASSES)
    assert {fault.segment for fault in faults} == set(fault_codes.SEGMENTS)
    assert len(faults) == len({fault.code for fault in faults})


def test_every_source_anchor_still_exists_in_the_code():
    if not (REPO_ROOT / "src").is_dir():
        pytest.skip("not running from a source checkout")
    assert fault_codes.check_sources(fault_codes.load(), str(REPO_ROOT)) == []


def test_the_document_is_what_the_registry_renders():
    if not DOC.is_file():
        pytest.skip("not running from a source checkout")
    expected = fault_codes.render_markdown(fault_codes.load())
    assert DOC.read_text(encoding="utf-8") == expected, \
        "docs/故障码.md is stale: python3 -m robot_log_tools.fault_codes --markdown > docs/故障码.md"


def test_stop_faults_say_what_the_system_does():
    for fault in fault_codes.load():
        if fault.is_stop:
            assert fault.behavior and fault.check, fault.code


# --- parsing ------------------------------------------------------------------------------------------------
def test_a_valid_entry_parses():
    [fault] = fault_codes.parse([entry()])
    assert (fault.code, fault.fault_class, fault.segment, fault.is_stop) == ("HW-001", "S1", "HW", True)


@pytest.mark.parametrize("bad, message", [
    (entry(code="HW1"), "code must look like"),
    (entry(code="ZZ-001"), "code must look like"),
    (entry(**{"class": "S3"}), "class must be one of"),
    (entry(title=""), "'title' is missing"),
    (entry(check=None), "'check' is missing"),
    (entry(source=None), "must say where its log comes from"),
    (entry(source={"file": "a.cpp"}), "source needs a file and a match"),
    (entry(owner="x"), "unknown field"),
])
def test_a_malformed_entry_is_rejected_with_its_code(bad, message):
    with pytest.raises(FaultRegistryError) as caught:
        fault_codes.parse([bad])
    assert message in str(caught.value)


def test_an_entry_without_a_source_needs_an_external_note():
    [fault] = fault_codes.parse([entry(source=None, external="ros2_control")])
    assert fault.source_file is None and fault.external == "ros2_control"


def test_duplicate_codes_are_rejected():
    with pytest.raises(FaultRegistryError, match="duplicate"):
        fault_codes.parse([entry(), entry()])


def test_the_registry_must_be_a_list():
    with pytest.raises(FaultRegistryError, match="list"):
        fault_codes.parse({"code": "HW-001"})


# --- anchors ------------------------------------------------------------------------------------------------
def test_a_reworded_log_message_is_reported(tmp_path):
    (tmp_path / "a.cpp").write_text('RCLCPP_ERROR(logger, "[HW-001] something else entirely");', encoding="utf-8")
    faults = fault_codes.parse([entry()])
    problems = fault_codes.check_sources(faults, str(tmp_path))
    assert len(problems) == 1 and "HW-001" in problems[0] and "no longer contains" in problems[0]


def test_a_missing_source_file_is_reported(tmp_path):
    problems = fault_codes.check_sources(fault_codes.parse([entry()]), str(tmp_path))
    assert problems and "not found" in problems[0]


def test_a_present_message_passes(tmp_path):
    (tmp_path / "a.cpp").write_text('RCLCPP_ERROR(logger, "[HW-001] the text is here");', encoding="utf-8")
    assert fault_codes.check_sources(fault_codes.parse([entry()]), str(tmp_path)) == []


def test_a_message_without_its_code_prefix_is_reported(tmp_path):
    (tmp_path / "a.cpp").write_text('RCLCPP_ERROR(logger, "the text is here");', encoding="utf-8")
    problems = fault_codes.check_sources(fault_codes.parse([entry()]), str(tmp_path))
    assert len(problems) == 1 and "does not tag" in problems[0] and "[HW-001]" in problems[0]


def test_an_entry_can_say_why_it_is_untagged(tmp_path):
    (tmp_path / "a.cpp").write_text('RCLCPP_ERROR(logger, "the text is here");', encoding="utf-8")
    faults = fault_codes.parse([entry(untagged="the text comes from another project")])
    assert fault_codes.check_sources(faults, str(tmp_path)) == []


def test_main_check_fails_with_exit_code_1_on_a_stale_anchor(tmp_path, capsys):
    registry = tmp_path / "codes.yaml"
    registry.write_text("- code: HW-001\n  class: S1\n  title: t\n  behavior: b\n  check: c\n"
                        "  source: {file: a.cpp, match: gone}\n", encoding="utf-8")
    assert fault_codes.main(["--file", str(registry), "--check", "--repo-root", str(tmp_path)]) == 1
    assert "HW-001" in capsys.readouterr().err


def test_the_markdown_lists_every_code_once(capsys):
    assert fault_codes.main(["--markdown"]) == 0
    text = capsys.readouterr().out
    for fault in fault_codes.load():
        assert text.count(f"### {fault.code}") == 1


# ---- where the tables are found ---------------------------------------------------------------------------------

def test_tables_are_found_in_the_source_tree():
    from robot_log_tools import config_path
    assert os.path.isfile(config_path("fault_codes.yaml"))
    assert os.path.isfile(config_path("flight_types.yaml"))


def test_a_table_in_neither_place_is_an_error_naming_both(monkeypatch, tmp_path):
    packages = pytest.importorskip("ament_index_python.packages")
    monkeypatch.setattr(packages, "get_package_share_directory", lambda name: str(tmp_path))
    from robot_log_tools import config_path
    with pytest.raises(FileNotFoundError) as raised:
        config_path("no_such_table.yaml")
    assert "source tree" in str(raised.value) and str(tmp_path) in str(raised.value)


def test_an_installed_table_is_found_through_the_package_share_directory(monkeypatch, tmp_path):
    packages = pytest.importorskip("ament_index_python.packages")
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "only_installed.yaml").write_text("x: 1\n")
    seen = []
    monkeypatch.setattr(packages, "get_package_share_directory", lambda name: seen.append(name) or str(tmp_path))
    from robot_log_tools import config_path
    assert config_path("only_installed.yaml") == str(tmp_path / "config" / "only_installed.yaml")
    assert seen == ["robot_utils"]  # the package that installs config/ (it was robot_log before the merge)
