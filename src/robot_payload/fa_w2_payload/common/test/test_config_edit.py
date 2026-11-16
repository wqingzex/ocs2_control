"""Config editing keeps comments and unrelated fields."""
import pytest

from fa_w2_payload import config_edit

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


@pytest.fixture
def ws_root(tmp_path):
    (tmp_path / "config").mkdir()
    (tmp_path / "config" / "robot.local.yaml").write_text(TELEOP + "\n" + PROFILE, encoding="utf-8")
    return tmp_path


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
