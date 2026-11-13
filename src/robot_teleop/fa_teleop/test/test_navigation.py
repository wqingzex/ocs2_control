"""Navigation launch helper: map choice and the ros2 launch command (no ROS needed)."""
import pytest

from fa_teleop_runtime import navigation
from fa_teleop_runtime.config import validate

MAPS = [{"id": "lab", "source": "fiveages_w2_description", "default": False},
        {"id": "hall", "source": "fiveages_w2_description", "default": True}]


def make_cfg(**navigation_fields):
    return validate({"cameras": {"enable": False}, "navigation": navigation_fields})


def test_default_map_is_used_when_none_is_named():
    cmd = navigation.launch_command(make_cfg(), MAPS, map_id=None, map_only=None)
    assert cmd[:4] == ("ros2", "launch", "robot_navigation", "navigation_isaac_gt.launch.py")
    assert "robot:=fiveages_w2" in cmd and "map:=hall" in cmd
    assert not any(a.startswith("nav2_profile") for a in cmd)


def test_cli_flags_override_the_config():
    cfg = make_cfg(map="lab", map_only=False, map_config_package="my_maps")
    cmd = navigation.launch_command(cfg, MAPS, map_id=None, map_only=None)
    assert "map:=lab" in cmd and "map_config_package:=my_maps" in cmd
    cmd = navigation.launch_command(cfg, MAPS, map_id="hall", map_only=True)
    assert "map:=hall" in cmd and "nav2_profile:=map_only" in cmd


def test_unknown_map_lists_the_choices():
    with pytest.raises(navigation.NavigationError, match="lab, hall"):
        navigation.launch_command(make_cfg(), MAPS, map_id="nope", map_only=None)


def test_no_maps_or_no_default_is_a_clear_error():
    with pytest.raises(navigation.NavigationError, match="没有找到地图"):
        navigation.launch_command(make_cfg(), [], map_id=None, map_only=None)
    no_default = [dict(m, default=False) for m in MAPS]
    with pytest.raises(navigation.NavigationError, match="--map"):
        navigation.launch_command(make_cfg(), no_default, map_id=None, map_only=None)


def test_joystick_devices_are_parsed_from_enumerate_output():
    text = "  0 : Xbox Controller : true : false : evdev\n  2 : Logitech F310 : true : false : evdev\nnoise\n"
    assert navigation.parse_joysticks(text) == [{"id": 0, "name": "Xbox Controller"},
                                                {"id": 2, "name": "Logitech F310"}]


def test_missing_nav2_is_reported_with_the_fix(monkeypatch):
    def lookup(name):
        raise navigation.PackageNotFoundError(name)
    monkeypatch.setattr(navigation, "_share_directory", lookup)
    message = navigation.missing_dependency()
    assert "nav2_bringup" in message and "ros-jazzy-nav2-bringup" in message
    monkeypatch.setattr(navigation, "_share_directory", lambda name: "/opt/ros/jazzy/share/" + name)
    assert navigation.missing_dependency() is None
