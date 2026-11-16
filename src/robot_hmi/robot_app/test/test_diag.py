"""Diagnostic bundle and session info (no ROS, no hardware)."""
import os
import tarfile

import sys
from pathlib import Path

import pytest

from robot_app import diag, events


@pytest.fixture(autouse=True)
def no_live_rings(tmp_path, monkeypatch):
    """A bundle holds the flight recorder rings of the machine it runs on; tests must not pick up real ones."""
    monkeypatch.setenv("ROBOT_FLIGHT_DIR", str(tmp_path / "no_rings"))


def make_workspace(tmp_path):
    ws = tmp_path / "ws"
    (ws / "logs" / "control").mkdir(parents=True)
    (ws / "logs" / "control" / "20260101_000000.log").write_text("[ERROR] solver diverged\n")
    os.symlink("20260101_000000.log", ws / "logs" / "control" / "latest.log")
    (ws / "logs" / "events.log").write_text('{"kind": "start", "unit": "control"}\n')
    (ws / "run").mkdir()
    (ws / "run" / "control.json").write_text('{"pid": 1}')
    (ws / "config").mkdir()
    (ws / "config" / "console.token").write_text("SECRET-TOKEN\n")
    config = ws / "config" / "robot.local.yaml"
    config.write_text("robot: {hardware: real}\n")
    return ws, config


def members(path):
    with tarfile.open(path) as tar:
        return {m.name.split("/", 1)[1]: tar.extractfile(m).read().decode() for m in tar.getmembers() if m.isfile()}


def test_the_bundle_holds_logs_timeline_state_and_config_but_never_the_token(tmp_path):
    ws, config = make_workspace(tmp_path)
    files = members(diag.make_bundle(str(ws), str(config), ros=lambda: "NODES", machine=lambda: ""))
    assert "solver diverged" in files["logs/control.log"]
    assert '"kind": "start"' in files["events.log"] and files["run/control.json"] == '{"pid": 1}'
    assert files["config/robot.local.yaml"].startswith("robot:") and "NODES" in files["info.txt"]
    assert not any("console.token" in name or "SECRET-TOKEN" in text for name, text in files.items())


def test_the_report_sits_beside_the_bundle_and_is_inside_it(tmp_path):
    ws, config = make_workspace(tmp_path)
    path = diag.make_bundle(str(ws), str(config), minutes=5, reason="arm stopped", ros=lambda: "", machine=lambda: "M")
    files = members(path)
    report = open(diag.report_path(path), encoding="utf-8").read()
    assert files["report.txt"] == report and "arm stopped" in report and "last 5 minutes" in report
    assert "reason: arm stopped" in files["info.txt"] and "M" in files["info.txt"]
    assert not any(name.startswith(".work_") for name in os.listdir(ws / "logs" / "diag"))      # no leftovers


def test_the_rings_as_they_are_now_are_in_the_bundle(tmp_path, monkeypatch):
    helpers = Path(__file__).resolve().parents[3] / "robot_utils" / "test"
    assert helpers.is_dir(), f"the ring helpers of the robot_utils tests are not at {helpers}"
    sys.path.insert(0, str(helpers))
    from test_flight_decoder import make_ring
    ws, config = make_workspace(tmp_path)
    rings = tmp_path / "rings"
    rings.mkdir()
    make_ring(rings / "arm_controller.ring", [(302, 2, [0.0])] * 30, capacity=64, module="arm_controller")
    monkeypatch.setenv("ROBOT_FLIGHT_DIR", str(rings))
    path = diag.make_bundle(str(ws), str(config), ros=lambda: "", machine=lambda: "")
    with tarfile.open(path) as tar:
        names = [m.name.split("/", 1)[1] for m in tar.getmembers()]
    assert "rings/arm_controller.ring" in names
    assert "arm_controller/ctl_state" in open(diag.report_path(path), encoding="utf-8").read()


def test_session_info_names_the_code_version_and_the_hardware_selection(tmp_path):
    cfg = {"robot": {"entry": "w2", "hardware": "real", "end_effector": "linkerhand_l6", "end_effector_bus": "usb_rs485", "other": 1}}
    info = events.session_info(str(tmp_path), cfg)  # not a git checkout: the version is unknown, not an error
    assert info["git"] == "unknown" and info["robot"] == {
        "entry": "w2", "hardware": "real", "end_effector": "linkerhand_l6", "end_effector_bus": "usb_rs485"}


def test_a_clock_that_is_not_synchronised_is_called_out_and_an_unknown_one_is_not():
    assert "NTPSynchronized=no" in diag.clock_warning(lambda cmd: "no\n")
    assert diag.clock_warning(lambda cmd: "yes") is None
    assert diag.clock_warning(lambda cmd: "(unavailable: no timedatectl)") is None      # a container


def test_the_diag_command_works_with_a_broken_config(tmp_path, capsys):
    from robot_app import units_cli
    ws, config = make_workspace(tmp_path)
    config.write_text("robot: {entry: not_an_entry}\ncameras: {enable: true}\n")      # fails strict validation
    code = units_cli.main(["--config", str(config), "--ws-root", str(ws), "diag", "--minutes", "5", "config", "is", "broken"])
    out = capsys.readouterr().out
    assert code == 0 and "config is broken" in out and "诊断包已生成" in out
    assert units_cli.main(["--config", str(config), "--ws-root", str(ws), "diag", "--minutes", "0"]) == 2
