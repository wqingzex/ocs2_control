"""Tests for device parsing, config editing and the init steps (no ROS, no cameras)."""
import io

import pytest

from w2_payload.config import edit as config_edit
from w2_payload.devices import cameras as devices
from w2_payload.setup import steps as init_steps

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


# --- devices ------------------------------------------------------------------------------------

def test_scan_output_is_parsed_including_gmsl_without_usb_port():
    devs = devices.parse_list_devices(SCAN)
    assert [d.serial for d in devs] == ["CP0000000003", "CP0000000001", "CP0000000002"]
    assert devs[1].model == "Orbbec Gemini 335Lg" and devs[1].connection == "GMSL2" and devs[1].usb_port == ""
    assert devs[0].usb_port == "2-1.4"


def test_roles_are_suggested_by_model():
    roles = devices.suggest_roles(devices.parse_list_devices(SCAN))
    assert roles["head"].serial == "CP0000000001"
    assert {roles["left_wrist"].serial, roles["right_wrist"].serial} == {"CP0000000002", "CP0000000003"}


def test_existing_assignment_is_kept():
    roles = devices.suggest_roles(devices.parse_list_devices(SCAN),
                                  {"left_wrist": "CP0000000003", "right_wrist": "CP0000000002"})
    assert roles["left_wrist"].serial == "CP0000000003" and roles["right_wrist"].serial == "CP0000000002"


# --- config editing -------------------------------------------------------------------------------

def test_camera_edit_keeps_comments_and_other_fields(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    config_edit.set_cameras(path, {"head": {"serial": "CP1", "usb_port": "2-1"}, "left_wrist": {"enable": False}})
    text = (ws_root / "config" / "robot.local.yaml").read_text(encoding="utf-8")
    assert "# 本机遥操作配置（注释必须保留）" in text and "# 末端" in text and "# Gemini 335Lg" in text
    assert config_edit.camera_serials(path) == {"head": "CP1", "left_wrist": "", "right_wrist": ""}
    _, data = config_edit.load_rt(path)
    assert data["cameras"]["left_wrist"]["enable"] is False
    assert data["robot"]["entry"] == "split_body"


def test_unknown_role_is_rejected(ws_root):
    with pytest.raises(ValueError, match="未知相机角色"):
        config_edit.set_cameras(str(ws_root / "config" / "robot.local.yaml"), {"tail": {"serial": "x"}})


# --- camera step ------------------------------------------------------------------------------------

def test_camera_step_cannot_confirm_first_setup_unattended(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    ok = init_steps.step_cameras(path, str(ws_root), init_steps.Io(assume_yes=True, out=io.StringIO()),
                                 scanner=lambda: devices.parse_list_devices(SCAN))
    assert not ok
    serials = config_edit.camera_serials(path)
    assert not any(serials.values())


def test_camera_step_opens_visual_confirmation_and_discards_cancelled_draft(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    original = (ws_root / 'config/robot.local.yaml').read_text()
    seen = []
    def window(session, io):
        seen.extend(session.devices)
        assert session.kind == 'cameras'
        session.cancel()
        return False
    assert not init_steps.step_cameras(path, str(ws_root), ScriptedIo([]),
                                      scanner=lambda: devices.parse_list_devices(SCAN), runner=window)
    assert len(seen) == 3 and (ws_root / 'config/robot.local.yaml').read_text() == original


def test_camera_step_releases_own_resources_when_window_fails(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    closed = []
    def window(session, io):
        class Preview:
            def close(self): closed.append(True)
        session.previews = Preview()
        raise RuntimeError('window failed')
    with pytest.raises(RuntimeError, match='window failed'):
        init_steps.step_cameras(path, str(ws_root), ScriptedIo([]),
                               scanner=lambda: devices.parse_list_devices(SCAN), runner=window)
    assert closed == [True]
    assert config_edit.camera_serials(path)["head"] == ""


def test_unattended_camera_step_does_not_save_when_a_camera_is_missing(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    two = devices.parse_list_devices(SCAN)[:2]
    out = io.StringIO()
    assert not init_steps.step_cameras(path, str(ws_root), init_steps.Io(assume_yes=True, out=out),
                                       scanner=lambda: two)
    assert config_edit.camera_serials(path)["head"] == ""  # nothing written
    assert "2" in out.getvalue() and "未保存" in out.getvalue()


def test_camera_step_reports_no_devices(ws_root):
    out = io.StringIO()
    assert not init_steps.step_cameras(str(ws_root / "config" / "robot.local.yaml"), str(ws_root),
                                       init_steps.Io(assume_yes=True, out=out), scanner=lambda: [])
    assert "[ERROR] 没有发现相机" in out.getvalue()
    assert "初始化已停止" in out.getvalue()
    assert "[WARN]" not in out.getvalue()


def test_camera_scan_reports_process_failure(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(devices, "_scan_usb_metadata", lambda: [])
    monkeypatch.setattr(devices.subprocess, "run", lambda *args, **kwargs:
                        SimpleNamespace(returncode=1, stdout="", stderr="USB access denied"))
    with pytest.raises(RuntimeError, match="USB access denied"):
        devices.scan()


def test_camera_scan_reports_sdk_error_even_if_tool_returns_zero(monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr(devices, "_scan_usb_metadata", lambda: [])
    monkeypatch.setattr(devices.subprocess, "run", lambda *args, **kwargs:
                        SimpleNamespace(returncode=0, stdout="", stderr="[ERROR] uvc_open failed: -6"))
    with pytest.raises(RuntimeError, match="uvc_open"):
        devices.scan()


def test_usb_discovery_does_not_open_camera_or_use_sdk_and_ignores_unmapped_devices(tmp_path, monkeypatch):
    sysfs = tmp_path / "sys"
    devfs = tmp_path / "dev"
    sysfs.mkdir()
    for port, serial, number, exposed in (("2-2", "CP001", 2, True), ("2-7", "CP002", 8, False)):
        entry = sysfs / port
        entry.mkdir()
        for name, value in {"idVendor": "2bc5", "product": "Orbbec Gemini 335Lg", "serial": serial,
                            "busnum": "2", "devnum": str(number), "speed": "5000"}.items():
            (entry / name).write_text(value)
        if exposed:
            (devfs / "002").mkdir(parents=True, exist_ok=True)
            (devfs / "002" / f"{number:03}").touch()
    monkeypatch.setattr(devices, "USB_SYSFS", sysfs)
    monkeypatch.setattr(devices, "USB_DEVFS", devfs)
    monkeypatch.setattr(devices.subprocess, "run", lambda *args, **kwargs:
                        pytest.fail("USB discovery must not open the SDK"))
    assert devices.scan() == [devices.Device("Orbbec Gemini 335Lg", "CP001", "USB3.0", "2-2")]


def test_camera_failure_prints_single_step_retry_command(ws_root, monkeypatch, capsys):
    monkeypatch.setattr(init_steps, "step_cameras", lambda *args, **kwargs: False)
    code = init_steps.main(["--config", str(ws_root / "config/robot.local.yaml"),
                            "--ws-root", str(ws_root), "cameras"])
    assert code == 1
    assert "./scripts/setup.sh cam" in capsys.readouterr().out


def test_camera_setup_does_not_enable_disabled_cameras_when_cancelled(ws_root):
    path = str(ws_root / "config/robot.local.yaml")
    config_edit.set_fields(path, "cameras", {"enable": False})
    answers = ScriptedIo(["y"])
    assert not init_steps.step_cameras(path, str(ws_root), answers,
                                      scanner=lambda: devices.parse_list_devices(SCAN), runner=lambda *args: False)
    _, data = config_edit.load_rt(path)
    assert data["cameras"]["enable"] is False


def test_disabled_cameras_require_explicit_skip(ws_root):
    path = str(ws_root / "config/robot.local.yaml")
    config_edit.set_fields(path, "cameras", {"enable": False})
    assert init_steps.step_cameras(path, str(ws_root), ScriptedIo(["n"]),
                                  scanner=lambda: pytest.fail("user chose not to configure cameras"))


# --- hand step ----------------------------------------------------------------------------------------

def test_hand_step_uses_window_and_preserves_cancelled_config(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    answers = ScriptedIo([])
    seen = []
    def window(session, io):
        seen.append(session.draft)
        return False
    assert not init_steps.step_hand(path, answers, ports=lambda: ["/dev/ttyUSB1", "/dev/ttyUSB3"], runner=window)
    _, prof = config_edit.load_rt(path)
    assert prof["hardware"]["usb_left_port"] == "/dev/ttyUSB0" and prof["hardware"]["usb_right_port"] == "/dev/ttyUSB2"
    assert seen[0]['left_port'] == '/dev/ttyUSB1' and seen[0]['right_port'] == '/dev/ttyUSB3'
    text = (ws_root / "config" / "robot.local.yaml").read_text(encoding="utf-8")
    assert "# 左手" in text and "# 本机遥操作配置" in text  # one file: both parts keep their comments


def test_hand_unattended_first_confirmation_is_rejected(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    assert not init_steps.step_hand(path, init_steps.Io(assume_yes=True, out=io.StringIO()), ports=lambda: [])


def test_legacy_485_configuration_is_normalized_in_native_window(ws_root):
    path = str(ws_root / "config/robot.local.yaml")
    config_edit.set_fields(path, "robot", {"hand_drive": "marvin"})
    seen = []
    def window(session, io):
        seen.append(session.draft['drive'])
        return False
    assert not init_steps.step_hand(path, ScriptedIo([]), ports=lambda: [], runner=window)
    assert seen == ['485']


def test_hand_window_can_be_cancelled_without_inventing_missing_ports(ws_root):
    seen = []
    def window(session, io):
        seen.append(session.draft['left_port'])
        return False
    assert not init_steps.step_hand(str(ws_root / "config" / "robot.local.yaml"), ScriptedIo([]),
                                    ports=lambda: [], runner=window)
    assert seen == ['']
