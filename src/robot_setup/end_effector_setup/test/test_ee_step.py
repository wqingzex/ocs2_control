"""The hand setup step (no ROS, no hands)."""
import io

import pytest
import yaml

from end_effector_setup import session as ee_setup
from robot_config import config_file
from robot_config.testing import full_config
from robot_config.setup_steps import Io

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


CONFIG = "# 本机配置（注释必须保留）\n" + yaml.safe_dump(full_config(
    "split_body", cameras={"enable": False},
    robot={"end_effector": {"type": "linkerhand_l6", "interface": {
        "bus": "usb_rs485", "left": {"port": "/dev/ttyUSB0"}, "right": {"port": "/dev/ttyUSB2"}}}}), sort_keys=False)


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
    (tmp_path / "config" / "robot.local.yaml").write_text(CONFIG, encoding="utf-8")
    return tmp_path


def test_hand_step_uses_window_and_preserves_cancelled_config(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    answers = ScriptedIo([])
    seen = []
    def window(session, io):
        seen.append(session.draft)
        return False
    assert not ee_setup.step_ee(path, str(ws_root), answers, ports=lambda: ["/dev/ttyUSB1", "/dev/ttyUSB3"], runner=window)
    _, prof = config_file.load_rt(path)
    interface = prof["robot"]["end_effector"]["interface"]
    assert interface["left"]["port"] == "/dev/ttyUSB0" and interface["right"]["port"] == "/dev/ttyUSB2"
    assert seen[0]['left_port'] == '/dev/ttyUSB1' and seen[0]['right_port'] == '/dev/ttyUSB3'
    text = (ws_root / "config" / "robot.local.yaml").read_text(encoding="utf-8")
    assert "# 本机配置（注释必须保留）" in text  # the setup keeps the comments of the file


def test_hand_unattended_first_confirmation_is_rejected(ws_root):
    path = str(ws_root / "config" / "robot.local.yaml")
    assert not ee_setup.step_ee(path, str(ws_root), ee_setup.Io(assume_yes=True, out=io.StringIO()), ports=lambda: [])


def test_arm_rs485_is_the_485_wiring_in_the_native_window(ws_root):
    path = str(ws_root / "config/robot.local.yaml")
    config_file.set_fields(path, "robot.end_effector.interface", {"bus": "arm_rs485"})
    seen = []
    def window(session, io):
        seen.append(session.draft['drive'])
        return False
    assert not ee_setup.step_ee(path, str(ws_root), ScriptedIo([]), ports=lambda: [], runner=window)
    assert seen == ['485']


def test_hand_window_can_be_cancelled_without_inventing_missing_ports(ws_root):
    seen = []
    def window(session, io):
        seen.append(session.draft['left_port'])
        return False
    assert not ee_setup.step_ee(str(ws_root / "config" / "robot.local.yaml"), str(ws_root), ScriptedIo([]),
                                    ports=lambda: [], runner=window)
    assert seen == ['']
