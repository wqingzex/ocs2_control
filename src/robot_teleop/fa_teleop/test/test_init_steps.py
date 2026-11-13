"""Tests for device parsing, config editing and the init steps (no ROS, no cameras)."""
import io

import pytest

from fa_teleop_runtime import init_steps
from w2_payload.config import edit as config_edit
from w2_payload.devices import cameras as devices

SCAN = """\
[INFO] [1.0] [list_device_node]: SDK file log enabled: false
[INFO] [1.1] [list_device_node]: name: Orbbec Gemini 305
[INFO] [1.1] [list_device_node]: pid: 0x0840
[INFO] [1.1] [list_device_node]: serial: CP0000000003
[INFO] [1.1] [list_device_node]: connection: USB2.0
[INFO] [1.1] [list_device_node]: usb port: 2-1.4
[INFO] [1.2] [list_device_node]: name: Orbbec Gemini 335Lg
[INFO] [1.2] [list_device_node]: pid: 0x0800
[INFO] [1.2] [list_device_node]: serial: CP0000000001
[INFO] [1.2] [list_device_node]: connection: GMSL2
[INFO] [1.3] [list_device_node]: name: Orbbec Gemini 305
[INFO] [1.3] [list_device_node]: serial: CP0000000002
[INFO] [1.3] [list_device_node]: connection: USB3.2
[INFO] [1.3] [list_device_node]: usb port: 2-1.3
"""

TELEOP = """\
# 本机遥操作配置（注释必须保留）
robot:
  entry: split_body
  hand: linkerhand_l6   # 末端
  hand_drive: usb

cameras:
  head:        {serial: ""}   # Gemini 335Lg
  left_wrist:  {serial: ""}   # Gemini 305
  right_wrist: {serial: ""}   # Gemini 305
"""

PROFILE = """\
# 平台配置（和上面同一个文件）
hardware:
  max_joint_speed: 100
  usb_left_port: /dev/ttyUSB0   # 左手
  usb_right_port: /dev/ttyUSB2
"""


class ScriptedIo(init_steps.Io):
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
    (tmp_path / "config" / "robot.local.yaml").write_text(TELEOP + "\n" + PROFILE, encoding="utf-8")
    return tmp_path


# --- VR step --------------------------------------------------------------------------------------------

def test_xr_step_reports_missing_pc_service(ws_root):
    path = ws_root / "config" / "robot.local.yaml"
    path.write_text(TELEOP + "xr:\n  pc_service_script: /nonexistent/runService.sh\n", encoding="utf-8")
    out = io.StringIO()
    assert not init_steps.step_xr(str(path), str(ws_root), init_steps.Io(assume_yes=True, out=out))
    assert "未安装 XRoboToolkit PC Service" in out.getvalue()


def test_xr_step_skips_only_when_user_declines(ws_root):
    path = ws_root / "config" / "robot.local.yaml"
    path.write_text(TELEOP + "xr:\n  enable: false\n", encoding="utf-8")
    assert init_steps.step_xr(str(path), str(ws_root), ScriptedIo(["n"]))


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

    monkeypatch.setattr(init_steps, "Supervisor", FakeSupervisor)
    from fa_teleop_runtime.xr_network import NetworkAddress
    monkeypatch.setattr(init_steps, "network_addresses", lambda mode="":
                        [NetworkAddress("eth0", "192.168.1.10", "ethernet")])
    return ConnectedProbe, events


def test_xr_step_can_enable_and_verify_pairing(ws_root, connected_xr):
    ConnectedProbe, events = connected_xr
    path = ws_root / "config/robot.local.yaml"
    path.write_text(TELEOP + "xr:\n  enable: false\n  start_pc_service: false\n", encoding="utf-8")
    assert init_steps.step_xr(str(path), str(ws_root), ScriptedIo(["y"]), probe_factory=ConnectedProbe)
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
    monkeypatch.setattr(init_steps.time, "monotonic", lambda: clock[0])

    def advance(_):
        clock[0] += 65.0
        if clock[0] >= 130.0:
            probe.xr_connected = True

    monkeypatch.setattr(init_steps.time, "sleep", advance)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(TELEOP + "xr:\n  start_pc_service: false\n", encoding="utf-8")
    io = ScriptedIo([])
    assert init_steps.step_xr(str(path), str(ws_root), io, probe_factory=probe)
    assert clock[0] >= 130.0
    assert "不限时" in io.out.getvalue()


def test_cancelling_xr_wait_closes_probe_and_only_started_units(ws_root, connected_xr, monkeypatch):
    probe, events = connected_xr
    probe.xr_connected = False

    def cancel(_):
        raise KeyboardInterrupt

    monkeypatch.setattr(init_steps.time, "sleep", cancel)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(TELEOP + "xr:\n  start_pc_service: false\n", encoding="utf-8")
    with pytest.raises(KeyboardInterrupt):
        init_steps.step_xr(str(path), str(ws_root), ScriptedIo([]), probe_factory=probe)
    assert events == [("start", "xr"), "probe_closed", ("stop", "xr")]


def test_cancelled_pairing_command_returns_130(ws_root, monkeypatch, capsys):
    def cancel(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(init_steps, "step_xr", cancel)
    assert init_steps.main(["--config", str(ws_root / "config/robot.local.yaml"),
                            "--ws-root", str(ws_root), "xr"]) == 130
    assert "已取消初始化" in capsys.readouterr().out


def test_xr_does_not_pair_without_device_identity(ws_root, connected_xr, monkeypatch):
    probe, _ = connected_xr
    probe.xr_device_id = ""
    monkeypatch.setattr(init_steps.time, "sleep", lambda _: None)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(TELEOP + "xr:\n  start_pc_service: false\n", encoding="utf-8")
    assert not init_steps.step_xr(str(path), str(ws_root), ScriptedIo([]), timeout_s=0.01, probe_factory=probe)
    _, raw = config_edit.load_rt(str(path))
    assert not raw["xr"].get("device_id")


def test_failed_rebinding_preserves_previous_headset(ws_root, connected_xr, monkeypatch):
    probe, _ = connected_xr
    probe.xr_connected = False
    monkeypatch.setattr(init_steps.time, "sleep", lambda _: None)
    path = ws_root / "config/robot.local.yaml"
    path.write_text(TELEOP + "xr:\n  start_pc_service: false\n  device_id: PICO-OLD\n", encoding="utf-8")
    assert not init_steps.step_xr(str(path), str(ws_root), ScriptedIo(["ethernet", "y"]),
                                 timeout_s=0.01, probe_factory=probe)
    _, raw = config_edit.load_rt(str(path))
    assert raw["xr"]["device_id"] == "PICO-OLD"


def test_xr_network_can_select_wifi(ws_root, monkeypatch):
    from fa_teleop_runtime.xr_network import NetworkAddress
    from fa_teleop_runtime.config import DEFAULTS
    path = str(ws_root / "config/robot.local.yaml")
    xr = dict(DEFAULTS["xr"])
    modes = []

    def addresses(mode):
        modes.append(mode)
        return [NetworkAddress("wlan0", "192.168.1.11", "wifi")]

    monkeypatch.setattr(init_steps, "network_addresses", addresses)
    assert init_steps._xr_network(path, xr, ScriptedIo(["wifi"]))
    _, raw = config_edit.load_rt(path)
    assert modes == ["wifi"]
    assert raw["xr"]["network_mode"] == "wifi"
    assert raw["xr"]["network_interface"] == "wlan0"


def test_xr_network_missing_address_does_not_wait_silently(ws_root, monkeypatch):
    from fa_teleop_runtime.config import DEFAULTS
    monkeypatch.setattr(init_steps, "network_addresses", lambda mode="": [])
    out = ScriptedIo([])
    assert not init_steps._xr_network(str(ws_root / "config/robot.local.yaml"), dict(DEFAULTS["xr"]), out)
    assert "没有找到可用的有线 IPv4 地址" in out.out.getvalue()


@pytest.mark.parametrize("raw, needed", [
    ({}, True),
    ({"xr": {"device_id": ""}}, True),
    ({"xr": {"device_id": "PICO-1"}}, False),
    ({"xr": {"enable": False}}, False),
])
def test_xr_pairing_is_needed_only_without_a_saved_headset_id(raw, needed):
    assert init_steps.xr_status(raw)[0] is needed
