"""Tests for robot_app: config validation, controller args, camera specs, check CLI."""
import re

import pytest
import yaml

from robot_app import ConfigError, validate
from launch_helpers.devices.camera import camera_specs, wrist_eye
from robot_app.check import main as check_main
from robot_app.robot import controller_launch
from robot_config.testing import full_config

SERIALS = {"head": {"serial": "CP111"}, "left_wrist": {"serial": "CP222"}, "right_wrist": {"serial": "CP333"}}
CAMERAS_OFF = {"enable": False}


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


def make_cfg(entry="whole_body", **sections):
    """A validated complete config (template values, serials for the cameras) with the given sections merged in."""
    cameras = {**cams_with_serials(), **sections.pop("cameras", {})}
    return validate(full_config(entry, cameras=cameras, **sections))


# --- validation -------------------------------------------------------------------------------

def test_defaults_are_valid_when_cameras_are_disabled():
    merged = validate(full_config(cameras=CAMERAS_OFF))
    assert merged["robot"]["end_effector"]["type"] == "linkerhand_l6"
    assert merged["xr"]["frame_timeout_ms"] == 300


def test_unknown_field_is_rejected_with_its_path():
    with pytest.raises(ConfigError, match="robot.hnad"):
        validate(full_config(cameras=CAMERAS_OFF, robot={"hnad": "xhand1"}))


def test_load_lenient_disables_cameras_without_serials(tmp_path):
    from robot_app.config import load_lenient
    path = tmp_path / "t.yaml"
    path.write_text(yaml.safe_dump(full_config(robot={"end_effector": {"type": "xhand1"}})), encoding="utf-8")  # camera serials not set yet
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


def test_camera_assignment_exposure_and_gain_are_both_auto_or_both_fixed():
    from robot_config.config import validate_camera_assignments
    validate_camera_assignments({"head": {"exposure": -1, "gain": -1}, "left_wrist": {"exposure": 50, "gain": 0}})
    for bad in ({"exposure": -1, "gain": 16}, {"exposure": 50, "gain": -1}, {"exposure": 0, "gain": 16},
                {"exposure": 50, "gain": -2}, {"exposure": True, "gain": 16}, {"exposure": 1.5, "gain": 16},
                {"exposure": 50}, {"gain": 16}):
        with pytest.raises(ConfigError):
            validate_camera_assignments({"head": bad})


def test_joystick_defaults_and_validation():
    merged = validate(full_config(cameras=CAMERAS_OFF))
    assert merged["joystick"]["device"] == ""
    assert validate(full_config(cameras=CAMERAS_OFF, joystick={"device": "/dev/input/js1"}))
    for bad in ("/dev/input/event3", "js1", "/dev/input/js1; rm"):
        with pytest.raises(ConfigError, match="joystick.device"):
            validate(full_config(cameras=CAMERAS_OFF, joystick={"device": bad}))


def test_the_control_section_shares_the_file_and_is_passed_through():
    merged = validate(full_config(cameras=CAMERAS_OFF, control={"arm_mode": "position", "patch": {"x_controller": {}}}))
    assert merged["robot"]["entry"] == "whole_body" and "control" not in merged  # read by the ROS launch, not here
    with pytest.raises(ConfigError, match="control"):
        validate(full_config(cameras=CAMERAS_OFF, control="fast"))  # must be a mapping


def head_with(**fields):
    """The three cameras with serials, the head changed by `fields`."""
    cameras = cams_with_serials()
    cameras["head"].update(fields)
    return cameras


# (case, builds a config the validation must refuse, what the message must say)
REJECTED = [
    ("an arm type without a robot model for the entry", lambda: make_cfg(robot={"arm": {"type": "rokae"}}), "robot.arm.type"),
    ("an arm firmware that does not exist", lambda: make_cfg(robot={"arm": {"interface": {"firmware": 340}}}), "firmware"),
    ("an arm mode that does not exist", lambda: validate(full_config(cameras=CAMERAS_OFF, control={"arm_mode": "fast"})), "control.arm_mode"),
    ("xhand1 on the arm rs485", lambda: make_cfg(robot=eef("xhand1", "arm_rs485")), "xhand1 only supports usb_rs485"),
    ("rg75 on usb_rs485", lambda: make_cfg(robot=eef("jodell_rg75", "usb_rs485")), "jodell_rg75 is only wired for arm_rs485"),
    ("usb_can for a gripper", lambda: make_cfg(robot=eef("jodell_rg75", "usb_can")), "usb_can is only for linkerhand_l6"),
    ("wuji_hand2 on usb_rs485", lambda: make_cfg(robot=eef("wuji_hand2", "usb_rs485")), "wuji_hand2 needs ethernet"),
    ("ethernet for linkerhand_l6", lambda: make_cfg(robot=eef("linkerhand_l6", "ethernet")), "wuji_hand2 needs ethernet"),
    ("an enabled camera without a serial",
     lambda: validate(full_config(cameras={"left_wrist": {"serial": "a"}, "right_wrist": {"serial": "b"}})), "cameras.head.serial"),
    ("a camera rotation of 45", lambda: validate(full_config(cameras=head_with(rotation=45))), "rotation"),
    ("a camera serial with a space", lambda: validate(full_config(cameras=head_with(serial="bad serial"))), "invalid characters"),
    ("xr.enable that is not a boolean", lambda: make_cfg(xr={"enable": "yes"}), "xr.enable"),
    ("cameras.fps that is not 30 or 60", lambda: make_cfg(cameras={"fps": 25}), "cameras.fps"),
]


def test_values_that_do_not_fit_are_rejected_with_a_message_that_names_the_field():
    for case, build, match in REJECTED:
        with pytest.raises(ConfigError) as info:
            build()
        assert re.search(match, str(info.value)), f"{case}: {info.value}"


def test_xr_defaults_to_ethernet_and_accepts_wifi_binding():
    assert make_cfg()["xr"]["network_mode"] == "ethernet"
    cfg = make_cfg(xr={"network_mode": "wifi", "network_interface": "wlan0", "device_id": "PICO-001"})
    assert cfg["xr"]["device_id"] == "PICO-001"
    with pytest.raises(ConfigError, match="network_mode"):
        make_cfg(xr={"network_mode": "bluetooth"})


def test_disabled_camera_may_have_empty_serial():
    merged = validate(full_config(cameras={"head": {"enable": False}, "left_wrist": {"serial": "a"},
                                           "right_wrist": {"serial": "b"}}))
    assert [s.name for s in camera_specs(merged)] == ["camera_left_wrist", "camera_right_wrist"]


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


# (end effector, its bus): every bus but usb_rs485 (tested above, with the rest of the arguments)
HAND_BUSES = [("linkerhand_l6", "arm_rs485"), ("linkerhand_l6", "usb_can"), ("wuji_hand2", "ethernet"), ("jodell_rg75", "arm_rs485")]


def test_a_hand_on_any_bus_uses_real_hardware_and_passes_its_names_down_to_the_xacro():
    for device, bus in HAND_BUSES:
        _, args = controller_launch(make_cfg(robot=eef(device, bus)), "/ws/config/robot.local.yaml")
        assert (args["hardware"], args["end_effector"], args["end_effector_bus"]) == ("real", device, bus), (device, bus)


def test_mock_hardware_has_no_drive_args():
    _, args = controller_launch(make_cfg(robot={"hardware": "mock_components"}), "/ws/config/robot.local.yaml")
    assert args["hardware"] == "mock_components"
    assert "end_effector_bus" not in args  # simulation: no wiring


def test_no_hand_and_dual_entry():
    launch_file, args = controller_launch(make_cfg("dual_arm", robot=eef("none")), "/ws/config/robot.local.yaml")
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
    spec = next(s for s in camera_specs(validate(full_config(cameras=cams))) if s.name == "camera_left_wrist")
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
    bad.write_text(yaml.safe_dump(full_config(cameras=CAMERAS_OFF, robot=eef("xhand1", "arm_rs485"))))
    assert check_main([str(bad), str(tmp_path)]) == 2
    assert "xhand1" in capsys.readouterr().err

    good = tmp_path / "good.yaml"
    good.write_text(yaml.safe_dump(full_config(cameras=CAMERAS_OFF, xr={"start_pc_service": False})))
    assert check_main([str(good), str(tmp_path)]) == 0
    out = capsys.readouterr().out
    assert "whole_body.launch.py" in out and "hardware:=real " in out


def test_an_incomplete_config_is_reported_with_how_to_complete_it(tmp_path, capsys):
    path = tmp_path / "short.yaml"
    path.write_text("robot: {entry: whole_body, hardware: real}\n")
    assert check_main([str(path), str(tmp_path)]) == 2
    assert "setup.sh config" in capsys.readouterr().err


@pytest.mark.parametrize("end_effector,bus,expected", [
    ("linkerhand_l6", "usb_can", "USB-CAN"),
    ("wuji_hand2", "ethernet", "以太网"),
    ("linkerhand_l6", "usb_rs485", "尚未确认"),
])
def test_check_reports_hand_confirmation_by_wiring(tmp_path, capsys, end_effector, bus, expected):
    cfg = tmp_path / "robot.yaml"
    cfg.write_text(yaml.safe_dump(full_config(cameras=CAMERAS_OFF, xr={"start_pc_service": False}, robot=eef(end_effector, bus))))
    assert check_main([str(cfg), str(tmp_path)]) == 0
    line = next(l for l in capsys.readouterr().out.splitlines() if "左右手实物" in l)
    assert expected in line
