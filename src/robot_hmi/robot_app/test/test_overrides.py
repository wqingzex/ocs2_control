"""--set overrides, ./scripts/start.sh viz and `up --with` (all without ROS)."""
import copy

import pytest
import yaml

from robot_app import config
from robot_config.testing import full_config
from robot_app.config import ConfigError
from robot_app.overrides import apply_overrides, parse_assignment
from robot_app.robot import viz_launch
from robot_app.units import build_units
from robot_app.units_cli import up_names

CAMERAS_OFF = {"enable": False}


@pytest.mark.parametrize("text, expected", [
    ("robot.hardware=isaac", (["robot", "hardware"], "isaac")),
    ("xr.enable=false", (["xr", "enable"], False)),
    ("cameras.fps=60", (["cameras", "fps"], 60)),
    ("cameras.head.enable=false", (["cameras", "head", "enable"], False)),
    ("robot.end_effector.interface.bus=usb_can", (["robot", "end_effector", "interface", "bus"], "usb_can")),
])
def test_assignment_values_are_read_as_yaml(text, expected):
    assert parse_assignment(text) == expected


@pytest.mark.parametrize("text", ["robot", "robot=1", "=x", "robot.hardware", "ro bot.hardware=1", "a..b=1"])
def test_malformed_assignments_are_rejected(text):
    with pytest.raises(ConfigError, match="--set expects"):
        parse_assignment(text)


@pytest.mark.parametrize("text", [
    "cameras.head.serial=ABC", "cameras.left_wrist.usb_port=2-1", "cameras.right_wrist.rotation=90",
    "xr.device_id=PICO", "control.limits.max_joint_speed=1", "robot.end_effector.interface.left.port=/dev/serial/by-id/usb-LEFT",
    "robot.end_effector.interface.right.port=/dev/serial/by-id/usb-RIGHT",
])
def test_device_mappings_and_profile_sections_cannot_be_overridden(text):
    with pytest.raises(ConfigError, match="cannot change"):
        parse_assignment(text)


def test_apply_returns_a_copy_and_creates_missing_sections():
    raw = {"robot": {"hardware": "real"}}
    original = copy.deepcopy(raw)
    result = apply_overrides(raw, ["robot.hardware=isaac", "cameras.head.enable=false"])
    assert raw == original
    assert result == {"robot": {"hardware": "isaac"}, "cameras": {"head": {"enable": False}}}


def test_a_scalar_in_the_way_is_an_error():
    with pytest.raises(ConfigError, match="not a section"):
        apply_overrides({"robot": 3}, ["robot.hardware=isaac"])


def _write(tmp_path, cameras_off=True):
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(full_config(cameras=CAMERAS_OFF if cameras_off else {})))
    return str(path)


def test_load_applies_overrides_and_leaves_the_file_alone(tmp_path):
    path = _write(tmp_path)
    cfg = config.load(path, ["robot.hardware=mock_components", "robot.entry=whole_body"])
    assert cfg["robot"]["hardware"] == "mock_components" and cfg["robot"]["entry"] == "whole_body"
    assert "mock_components" not in (tmp_path / "robot.yaml").read_text()


def test_overridden_values_are_validated_with_the_field_name(tmp_path):
    with pytest.raises(ConfigError, match="robot.hardware"):
        config.load(_write(tmp_path), ["robot.hardware=moon"])
    with pytest.raises(ConfigError, match="robot.entry"):
        config.load(_write(tmp_path), ["robot.entry=tiny"])


def test_load_lenient_keeps_overrides_when_cameras_are_not_set_up(tmp_path):
    path = _write(tmp_path, cameras_off=False)  # cameras enabled but no serials: strict load fails
    assert config.load_lenient(path, ["robot.hardware=isaac"])["robot"]["hardware"] == "isaac"


@pytest.mark.parametrize("entry, expected", [
    ("split_body", ("robot_visualize", "humanoid.launch.py", {"robot": "fa_w2"})),
    ("whole_body", ("robot_visualize", "humanoid.launch.py", {"robot": "fa_w2"})),
    ("dual_arm", ("robot_visualize", "manipulator.launch.py", {"robot": "m6_ccs", "arms": "dual"})),
])
def test_viz_launch_shows_the_model_only(entry, expected):
    assert viz_launch(config.validate(full_config(entry, cameras=CAMERAS_OFF))) == expected


def test_up_with_adds_on_demand_units_after_the_regular_ones():
    units = build_units(config.validate(full_config(cameras=CAMERAS_OFF, xr={"start_pc_service": False})), "/c.yaml", "/ws")
    assert up_names(units, [], []) is None
    assert up_names(units, ["control"], []) == ["control", "rviz"]  # naming the controller brings RViz
    assert up_names(units, ["control"], [], no_rviz=True) == ["control"]
    assert up_names(units, ["control"], ["rviz"]) == ["control", "rviz"]
    assert up_names(units, ["xr"], []) == ["xr"]
    names = up_names(units, [], ["rviz"])
    assert units.launch_order(names) == ["control", "xr", "rviz"]
