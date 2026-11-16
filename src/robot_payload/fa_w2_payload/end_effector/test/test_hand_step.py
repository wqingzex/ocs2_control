"""The hand setup step (no ROS, no hands)."""
import io

import pytest

from end_effector import hand_setup
from fa_w2_payload import config_edit
from fa_w2_payload.device_setup import Io

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
  end_effector: linkerhand_l6   # 末端
  end_effector_bus: usb_rs485

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
    (tmp_path / "config" / "robot.local.yaml").write_text(TELEOP + "\n" + PROFILE, encoding="utf-8")
    return tmp_path


def test_hand_step_uses_window_and_preserves_cancelled_config(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    answers = ScriptedIo([])
    seen = []
    def window(session, io):
        seen.append(session.draft)
        return False
    assert not hand_setup.step_hand(path, str(ws_root), answers, ports=lambda: ["/dev/ttyUSB1", "/dev/ttyUSB3"], runner=window)
    _, prof = config_edit.load_rt(path)
    assert prof["hardware"]["usb_left_port"] == "/dev/ttyUSB0" and prof["hardware"]["usb_right_port"] == "/dev/ttyUSB2"
    assert seen[0]['left_port'] == '/dev/ttyUSB1' and seen[0]['right_port'] == '/dev/ttyUSB3'
    text = (ws_root / "config" / "robot.local.yaml").read_text(encoding="utf-8")
    assert "# 左手" in text and "# 本机遥操作配置" in text  # one file: both parts keep their comments


def test_hand_unattended_first_confirmation_is_rejected(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    assert not hand_setup.step_hand(path, str(ws_root), hand_setup.Io(assume_yes=True, out=io.StringIO()), ports=lambda: [])


def test_arm_rs485_is_the_485_wiring_in_the_native_window(ws_root):
    path = str(ws_root / "config/robot.local.yaml")
    config_edit.set_fields(path, "robot", {"end_effector_bus": "arm_rs485"})
    seen = []
    def window(session, io):
        seen.append(session.draft['drive'])
        return False
    assert not hand_setup.step_hand(path, str(ws_root), ScriptedIo([]), ports=lambda: [], runner=window)
    assert seen == ['485']


def test_hand_window_can_be_cancelled_without_inventing_missing_ports(ws_root):
    seen = []
    def window(session, io):
        seen.append(session.draft['left_port'])
        return False
    assert not hand_setup.step_hand(str(ws_root / "config" / "robot.local.yaml"), str(ws_root), ScriptedIo([]),
                                    ports=lambda: [], runner=window)
    assert seen == ['']
