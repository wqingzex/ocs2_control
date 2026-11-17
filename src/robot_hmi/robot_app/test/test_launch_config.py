"""Tests for robot_app: config validation, controller args, camera specs, check CLI."""
import os

import pytest
import yaml

from robot_app import ConfigError, validate
from launch_helpers.devices.camera import camera_specs, wrist_eye
from robot_app.check import main as check_main
from robot_app.robot import controller_launch

SERIALS = {"head": {"serial": "CP111"}, "left_wrist": {"serial": "CP222"}, "right_wrist": {"serial": "CP333"}}
HERE = os.path.dirname(os.path.abspath(__file__))


def cams_with_serials():
    return {k: dict(v) for k, v in SERIALS.items()}


def eef(type=None, bus=None):
    """A robot section naming the end effector and its bus (None = the default)."""
    section = {}
    if type:
        section["type"] = type
    if bus:
        section["interface"] = {"bus": bus}
    return {"end_effector": section}


def make_cfg(**sections):
    base = {"cameras": cams_with_serials()}
    for key, value in sections.items():
        base.setdefault(key, {}).update(value)
    return validate(base)


# --- validation -------------------------------------------------------------------------------

def test_defaults_are_valid_when_cameras_are_disabled():
    merged = validate({"cameras": {"enable": False}})
    assert merged["robot"]["end_effector"]["type"] == "linkerhand_l6"
    assert merged["xr"]["frame_timeout_ms"] == 300


def test_unknown_field_is_rejected_with_its_path():
    with pytest.raises(ConfigError, match="robot.hnad"):
        validate({"robot": {"hnad": "xhand1"}})


@pytest.mark.parametrize("field", ["control_scheme", "button_profile"])
def test_removed_key_scheme_fields_are_rejected(field):
    # There is one key table now: the old scheme choice is an unknown field like any other.
    with pytest.raises(ConfigError, match=f"xr.{field}"):
        validate({"xr": {field: "simple"}, "cameras": {"enable": False}})


def test_load_lenient_disables_cameras_without_serials(tmp_path):
    from robot_app.config import load_lenient
    path = tmp_path / "t.yaml"
    path.write_text("robot: {end_effector: {type: xhand1}}\n", encoding="utf-8")  # camera serials not set yet
    with pytest.raises(ConfigError):
        validate(yaml.safe_load(path.read_text()))
    merged = load_lenient(str(path))
    assert merged["cameras"]["enable"] is False and merged["robot"]["end_effector"]["type"] == "xhand1"


def test_camera_assignment_values_are_checked():
    from robot_config.config import validate_camera_assignments
    validate_camera_assignments({"head": {"serial": "CP1", "rotation": 90, "usb_port": "2-1.3", "enable": True},
                                 "left_wrist": {"enable": False}})
    for bad in ({"rotation": 45}, {"serial": "a b"}, {"enable": "yes"}, {"usb_port": "x y"}, {"rotation": True}):
        with pytest.raises(ConfigError):
            validate_camera_assignments({"head": bad})


def test_joystick_and_navigation_defaults_and_validation():
    merged = validate({"cameras": {"enable": False}})
    assert merged["joystick"]["device"] == "" and merged["navigation"]["map_only"] is False
    assert validate({"cameras": {"enable": False}, "joystick": {"device": "/dev/input/js1"}})
    for bad in ("/dev/input/event3", "js1", "/dev/input/js1; rm"):
        with pytest.raises(ConfigError, match="joystick.device"):
            validate({"cameras": {"enable": False}, "joystick": {"device": bad}})
    with pytest.raises(ConfigError, match="navigation.map"):
        validate({"cameras": {"enable": False}, "navigation": {"map": "../etc"}})


def test_the_control_section_shares_the_file_and_is_passed_through():
    raw = {"cameras": {"enable": False}, "control": {"arm_mode": "position", "patch": {"x_controller": {}}}}
    merged = validate(raw)
    assert merged["robot"]["entry"] == "whole_body" and "control" not in merged  # read by the ROS launch, not here
    with pytest.raises(ConfigError, match="control"):
        validate({"cameras": {"enable": False}, "control": "fast"})  # must be a mapping


@pytest.mark.parametrize("section", ["platform", "hardware", "defaults"])
def test_the_former_sections_are_unknown_fields(section):
    with pytest.raises(ConfigError, match=f"unknown field '{section}'"):
        validate({"cameras": {"enable": False}, section: {}})


@pytest.mark.parametrize("field, value", [
    ("end_effector", "xhand1"), ("end_effector_bus", "usb_rs485"), ("hand", "xhand1"), ("profile", "x.yaml"),
    ("rviz", True),
])
def test_the_former_robot_fields_are_errors(field, value):
    with pytest.raises(ConfigError):
        validate({"cameras": {"enable": False}, "robot": {field: value}})


def test_the_arm_needs_a_robot_model_for_the_entry():
    with pytest.raises(ConfigError, match="robot.arm.type"):
        make_cfg(robot={"arm": {"type": "rokae"}})


def test_arm_firmware_and_control_values_are_checked():
    with pytest.raises(ConfigError, match="firmware"):
        make_cfg(robot={"arm": {"interface": {"firmware": 340}}})
    with pytest.raises(ConfigError, match="control.arm_mode"):
        validate({"cameras": {"enable": False}, "control": {"arm_mode": "fast"}})


def test_xr_defaults_to_ethernet_and_accepts_wifi_binding():
    assert make_cfg()["xr"]["network_mode"] == "ethernet"
    cfg = make_cfg(xr={"network_mode": "wifi", "network_interface": "wlan0", "device_id": "PICO-001"})
    assert cfg["xr"]["device_id"] == "PICO-001"
    with pytest.raises(ConfigError, match="network_mode"):
        make_cfg(xr={"network_mode": "bluetooth"})


def test_xhand_on_the_arm_rs485_is_rejected():
    with pytest.raises(ConfigError, match="xhand1 only supports usb_rs485"):
        make_cfg(robot=eef("xhand1", "arm_rs485"))


@pytest.mark.parametrize("bus", ["usb", "485", "marvin"])
def test_old_bus_values_are_rejected(bus):
    with pytest.raises(ConfigError, match="end_effector.interface.bus' must be one of"):
        make_cfg(robot=eef(bus=bus))


def test_rg75_is_an_end_effector_on_the_arm_rs485_only():
    _, args = controller_launch(make_cfg(robot=eef("jodell_rg75", "arm_rs485")),
                                "/ws/config/robot.local.yaml")
    assert args["end_effector"] == "jodell_rg75"  # the same names all the way down to the xacro
    assert args["end_effector_bus"] == "arm_rs485"
    with pytest.raises(ConfigError, match="jodell_rg75 is only wired for arm_rs485"):
        make_cfg(robot=eef("jodell_rg75", "usb_rs485"))


def test_enabled_camera_without_serial_is_rejected():
    with pytest.raises(ConfigError, match="cameras.head.serial"):
        validate({"cameras": {"left_wrist": {"serial": "a"}, "right_wrist": {"serial": "b"}}})


def test_disabled_camera_may_have_empty_serial():
    merged = validate({"cameras": {"head": {"enable": False}, "left_wrist": {"serial": "a"},
                                   "right_wrist": {"serial": "b"}}})
    assert [s.name for s in camera_specs(merged)] == ["camera_left_wrist", "camera_right_wrist"]


@pytest.mark.parametrize("field,value,match", [
    ("rotation", 45, "rotation"),
    ("serial", "bad serial", "invalid characters"),
])
def test_camera_field_values_are_checked(field, value, match):
    cams = cams_with_serials()
    cams["head"][field] = value
    with pytest.raises(ConfigError, match=match):
        validate({"cameras": cams})


def test_wrong_types_are_rejected():
    with pytest.raises(ConfigError, match="xr.enable"):
        make_cfg(xr={"enable": "yes"})
    with pytest.raises(ConfigError, match="cameras.fps"):
        make_cfg(cameras={"fps": 25})


# --- controller launch arguments ----------------------------------------------------------------

def test_usb_rs485_hand_uses_real_hardware_and_usb_drive():
    launch_file, args = controller_launch(make_cfg(robot=eef(bus="usb_rs485")), "/ws/config/robot.local.yaml")
    assert launch_file == "whole_body.launch.py"
    assert args["robot"] == "fa_w2"
    assert args["hardware"] == "real"  # real_usb is gone: the bus is passed per side
    assert args["end_effector"] == "linkerhand_l6"
    assert args["end_effector_bus"] == "usb_rs485"
    assert "use_profile_eef" not in args
    assert args["robot_profile"] == "/ws/config/robot.local.yaml"  # the config file is the profile too
    assert args["launch_mode"] == "control_only"  # RViz is its own on-demand unit


def test_arm_rs485_hand_uses_real_hardware():
    _, args = controller_launch(make_cfg(robot=eef(bus="arm_rs485")), "/ws/config/robot.local.yaml")
    assert args["hardware"] == "real"
    assert args["end_effector_bus"] == "arm_rs485"


def test_usb_can_hand_uses_real_hardware_and_usb_can_drive():
    _, args = controller_launch(make_cfg(robot=eef(bus="usb_can")), "/ws/config/robot.local.yaml")
    assert args["hardware"] == "real"
    assert args["end_effector"] == "linkerhand_l6"
    assert args["end_effector_bus"] == "usb_can"


def test_usb_can_is_only_for_l6():
    with pytest.raises(ConfigError, match="usb_can is only for linkerhand_l6"):
        make_cfg(robot=eef("jodell_rg75", "usb_can"))


def test_wuji_uses_real_hardware_and_ethernet_drive():
    _, args = controller_launch(make_cfg(robot=eef("wuji_hand2", "ethernet")),
                                "/ws/config/robot.local.yaml")
    assert args["hardware"] == "real"
    assert args["end_effector"] == "wuji_hand2"
    assert args["end_effector_bus"] == "ethernet"


@pytest.mark.parametrize("hand,drive", [("wuji_hand2", "usb_rs485"), ("linkerhand_l6", "ethernet")])
def test_ethernet_and_wuji_go_together(hand, drive):
    with pytest.raises(ConfigError, match="wuji_hand2 needs ethernet"):
        make_cfg(robot=eef(hand, drive))


def test_mock_hardware_has_no_drive_args():
    _, args = controller_launch(make_cfg(robot={"hardware": "mock_components"}), "/ws/config/robot.local.yaml")
    assert args["hardware"] == "mock_components"
    assert "end_effector_bus" not in args  # simulation: no wiring


def test_no_hand_and_dual_entry():
    launch_file, args = controller_launch(make_cfg(robot={**eef("none"), "entry": "dual_arm"}), "/ws/config/robot.local.yaml")
    assert launch_file == "dual_arm.launch.py"
    assert args["robot"] == "m6_ccs"
    assert args["end_effector"] == "none"
    assert args["hardware"] == "real"


# --- cameras ----------------------------------------------------------------------------------

@pytest.mark.parametrize("rotation,eye", [(0, "right"), (180, "right"), (90, "left"), (270, "left")])
def test_wrist_eye_follows_rotation(rotation, eye):
    assert wrist_eye(rotation) == eye


def test_wrist_spec_remaps_used_eye_and_writes_rotation():
    cams = cams_with_serials()
    cams["left_wrist"]["rotation"] = 90
    spec = next(s for s in camera_specs(validate({"cameras": cams})) if s.name == "camera_left_wrist")
    assert spec.launch_file == "gemini305.launch.py"
    assert ("/camera_left_wrist/left_color/image_raw/compressed",
            "/camera_left_wrist/color/image_raw/compressed") in spec.remaps
    assert spec.config_yaml["left_color_rotation"] == 90
    assert spec.config_yaml["enable_left_color"] is True
    assert spec.config_yaml["right_color_fps"] == 5
    assert spec.args["serial_number"] == "CP222"
    assert spec.args["color_exposure"] == "50"
    assert "usb_port" not in spec.args


def test_head_spec_uses_330_series_with_auto_exposure():
    spec = next(s for s in camera_specs(make_cfg(cameras={"fps": 60})) if s.name == "camera_head")
    assert spec.launch_file == "gemini_330_series.launch.py"
    assert spec.args["color_fps"] == "60"
    assert spec.args["enable_color_auto_exposure"] == "true"
    assert spec.args["color_format"] == "YUYV"
    assert spec.remaps == []


# --- check CLI ----------------------------------------------------------------------------------

def test_check_cli_reports_errors_and_success(tmp_path, capsys):
    bad = tmp_path / "bad.yaml"
    bad.write_text("robot: {end_effector: {type: xhand1, interface: {bus: arm_rs485}}}\ncameras: {enable: false}\n")
    assert check_main([str(bad), str(tmp_path)]) == 2
    assert "xhand1" in capsys.readouterr().err

    good = tmp_path / "good.yaml"
    good.write_text("cameras: {enable: false}\nxr: {start_pc_service: false}\n")
    assert check_main([str(good), str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "whole_body.launch.py" in out and "hardware:=real " in out


@pytest.mark.parametrize("robot,expected", [
    ("{end_effector: {type: linkerhand_l6, interface: {bus: usb_can}}}", "USB-CAN"),
    ("{end_effector: {type: wuji_hand2, interface: {bus: ethernet}}}", "以太网"),
    ("{end_effector: {type: linkerhand_l6, interface: {bus: usb_rs485}}}", "尚未确认"),
])
def test_check_reports_hand_confirmation_by_wiring(tmp_path, capsys, robot, expected):
    cfg = tmp_path / "robot.yaml"
    cfg.write_text(f"robot: {robot}\ncameras: {{enable: false}}\nxr: {{start_pc_service: false}}\n")
    assert check_main([str(cfg), str(tmp_path)]) == 0
    line = next(l for l in capsys.readouterr().out.splitlines() if "左右手实物" in l)
    assert expected in line


def test_package_template_matches_schema():
    with open(os.path.join(HERE, "..", "config", "teleop.yaml"), encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    raw["cameras"]["enable"] = False  # serials are intentionally empty in the template
    assert validate(raw)["robot"]["entry"] == "whole_body"


def test_repo_template_matches_schema_and_names_the_expected_arguments():
    """config/robot.template.yaml is what a new machine starts from: it must validate and reach the launch."""
    from launch_helpers.robot.profile import normalize_robot_profile
    with open(os.path.join(HERE, "..", "..", "..", "..", "config", "robot.template.yaml"), encoding="utf-8") as handle:
        raw = yaml.safe_load(handle)
    checked = {**raw, "cameras": {"enable": False}}  # serials are intentionally empty in the template
    assert validate(checked)["robot"]["end_effector"]["type"] == "linkerhand_l6"
    profile = normalize_robot_profile(raw)
    assert profile["xacro"] == {"base": "linkhou_s2", "base_joints_movable": "false"}
    assert profile["eef"] == {"end_effector": "linkerhand_l6", "bus": "usb_rs485"}
    assert profile["hardware"]["body_can_interface"] == "can0" and profile["hardware"]["arm_driver"] == "marvin"
    assert profile["hardware"]["max_joint_speed"] == "100" and profile["hardware"]["device_ip"] == "192.168.1.190"
    assert "body_joint_controller" in profile["control"]["patch"]
