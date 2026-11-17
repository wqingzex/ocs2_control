"""XR pairing: network choice, starting the units, waiting for the headset and saving its id (no ROS, no headset)."""
import io

import pytest
import yaml

from robot_config import config_edit
from robot_config.template import section_defaults
from robot_config.testing import full_config
from robot_config.device_setup import Io
from xr_setup import session as pairing

def config_text(**xr):
    """A complete config (split_body, cameras off) with the given xr keys, and a comment that must survive."""
    return "# 本机配置（注释必须保留）\n" + yaml.safe_dump(
        full_config("split_body", cameras={"enable": False}, xr=xr), sort_keys=False)


class ScriptedIo(Io):
    def __init__(self, answers):
        super().__init__(out=io.StringIO())
        self.answers = list(answers)
        self.questions = []

    def ask(self, question, default):
        self.questions.append((question, default))
        answer = self.answers.pop(0) if self.answers else ""
        return answer or default


@pytest.fixture
def ws_root(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "robot.local.yaml").write_text(config_text(), encoding="utf-8")
    return tmp_path


# --- XR step --------------------------------------------------------------------------------------------

def test_xr_step_reports_missing_pc_service(ws_root):
    path = ws_root / "config" / "robot.local.yaml"
    path.write_text(config_text(pc_service_script="/nonexistent/runService.sh"), encoding="utf-8")
    out = io.StringIO()
    assert not pairing.step_xr(str(path), str(ws_root), Io(assume_yes=True, out=out))
    assert "未安装 XRoboToolkit PC Service" in out.getvalue()


def test_xr_step_skips_only_when_user_declines(ws_root):
    path = ws_root / "config" / "robot.local.yaml"
    path.write_text(config_text(enable=False), encoding="utf-8")
    assert pairing.step_xr(str(path), str(ws_root), ScriptedIo(["n"]))


@pytest.fixture
def connected_xr(monkeypatch):
    events = []

    class FakeSupervisor:
        def __init__(self, *args, **kwargs):
            pass

        def is_running(self, name):
            return False

        def start(self, name):
            events.append(("start", name))

        def stop(self, name):
            events.append(("stop", name))

    class ConnectedProbe:
        xr_connected = True
        xr_device_id = "PICO-001"

        def close(self):
            events.append("probe_closed")

    monkeypatch.setattr(pairing, "Supervisor", FakeSupervisor)
    from xr_teleop.xr_network import NetworkAddress
    monkeypatch.setattr(pairing, "network_addresses", lambda mode="":
                        [NetworkAddress("eth0", "192.168.1.10", "ethernet")])
    return ConnectedProbe, events


def test_xr_step_can_enable_and_verify_pairing(ws_root, connected_xr):
    ConnectedProbe, events = connected_xr
    path = ws_root / "config/robot.local.yaml"
    path.write_text(config_text(enable=False, start_pc_service=False), encoding="utf-8")
    assert pairing.step_xr(str(path), str(ws_root), ScriptedIo(["y"]), probe_factory=ConnectedProbe)
    _, data = config_edit.load_rt(str(path))
    assert data["xr"]["enable"] is True
    assert data["xr"]["network_mode"] == "ethernet"
    assert data["xr"]["network_interface"] == "eth0"
    assert data["xr"]["device_id"] == "PICO-001"
    assert events == [("start", "xr"), "probe_closed", ("stop", "xr")]


def test_xr_default_wait_can_exceed_sixty_seconds(ws_root, connected_xr, monkeypatch):
    probe, _ = connected_xr
    probe.xr_connected = False
    clock = [0.0]
    monkeypatch.setattr(pairing.time, "monotonic", lambda: clock[0])

    def advance(_):
        clock[0] += 65.0
        if clock[0] >= 130.0:
            probe.xr_connected = True

    monkeypatch.setattr(pairing.time, "sleep", advance)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(config_text(start_pc_service=False), encoding="utf-8")
    io = ScriptedIo([])
    assert pairing.step_xr(str(path), str(ws_root), io, probe_factory=probe)
    assert clock[0] >= 130.0
    assert "不限时" in io.out.getvalue()


def test_cancelling_xr_wait_closes_probe_and_only_started_units(ws_root, connected_xr, monkeypatch):
    probe, events = connected_xr
    probe.xr_connected = False

    def cancel(_):
        raise KeyboardInterrupt

    monkeypatch.setattr(pairing.time, "sleep", cancel)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(config_text(start_pc_service=False), encoding="utf-8")
    with pytest.raises(KeyboardInterrupt):
        pairing.step_xr(str(path), str(ws_root), ScriptedIo([]), probe_factory=probe)
    assert events == [("start", "xr"), "probe_closed", ("stop", "xr")]


def test_cancelled_pairing_command_returns_130(ws_root, monkeypatch, capsys):
    def cancel(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(pairing, "step_xr", cancel)
    assert pairing.main(["--config", str(ws_root / "config/robot.local.yaml"),
                            "--ws-root", str(ws_root), "xr"]) == 130
    assert "已取消初始化" in capsys.readouterr().out


def test_xr_does_not_pair_without_device_identity(ws_root, connected_xr, monkeypatch):
    probe, _ = connected_xr
    probe.xr_device_id = ""
    monkeypatch.setattr(pairing.time, "sleep", lambda _: None)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(config_text(start_pc_service=False), encoding="utf-8")
    assert not pairing.step_xr(str(path), str(ws_root), ScriptedIo([]), timeout_s=0.01, probe_factory=probe)
    _, raw = config_edit.load_rt(str(path))
    assert not raw["xr"].get("device_id")


def test_failed_rebinding_preserves_previous_headset(ws_root, connected_xr, monkeypatch):
    probe, _ = connected_xr
    probe.xr_connected = False
    monkeypatch.setattr(pairing.time, "sleep", lambda _: None)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(config_text(start_pc_service=False, device_id="PICO-OLD"), encoding="utf-8")
    assert not pairing.step_xr(str(path), str(ws_root), ScriptedIo(["ethernet", "y"]),
                                 timeout_s=0.01, probe_factory=probe)
    _, raw = config_edit.load_rt(str(path))
    assert raw["xr"]["device_id"] == "PICO-OLD"


def test_xr_network_can_select_wifi(ws_root, monkeypatch):
    from xr_teleop.xr_network import NetworkAddress
    path = str(ws_root / "config/robot.local.yaml")
    xr = section_defaults("xr")
    modes = []

    def addresses(mode):
        modes.append(mode)
        return [NetworkAddress("wlan0", "192.168.1.11", "wifi")]

    monkeypatch.setattr(pairing, "network_addresses", addresses)
    assert pairing._xr_network(path, xr, ScriptedIo(["wifi"]))
    _, raw = config_edit.load_rt(path)
    assert modes == ["wifi"]
    assert raw["xr"]["network_mode"] == "wifi"
    assert raw["xr"]["network_interface"] == "wlan0"


def test_the_cable_to_the_arm_is_not_offered_and_the_only_other_adapter_is_taken(ws_root, monkeypatch):
    from xr_teleop.xr_network import NetworkAddress
    path = str(ws_root / "config/robot.local.yaml")
    monkeypatch.setattr(pairing, "network_addresses", lambda mode="": [
        NetworkAddress("enp4s0", "192.168.1.100", "ethernet", "192.168.1.0/24"),
        NetworkAddress("enx1", "192.168.10.101", "ethernet", "192.168.10.0/24")])
    io = ScriptedIo([])
    assert pairing._xr_network(path, section_defaults("xr"), io)
    assert "enp4s0" not in io.out.getvalue()
    assert not any("网卡编号" in question for question, _ in io.questions)  # one adapter left: no question
    _, raw = config_edit.load_rt(path)
    assert raw["xr"]["network_interface"] == "enx1"


def test_xr_network_missing_address_does_not_wait_silently(ws_root, monkeypatch):
    monkeypatch.setattr(pairing, "network_addresses", lambda mode="": [])
    out = ScriptedIo([])
    assert not pairing._xr_network(str(ws_root / "config/robot.local.yaml"), section_defaults("xr"), out)
    assert "没有找到可用的有线 IPv4 地址" in out.out.getvalue()


@pytest.mark.parametrize("raw, needed", [
    ({}, True),
    ({"xr": {"device_id": ""}}, True),
    ({"xr": {"device_id": "PICO-1"}}, False),
    ({"xr": {"enable": False}}, False),
])
def test_xr_pairing_is_needed_only_without_a_saved_headset_id(raw, needed):
    assert pairing.xr_status(raw)[0] is needed
