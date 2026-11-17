"""The units of a run: the controllers and RViz are built by robot_app, the devices' units come from their packages."""
import pytest

from robot_app import validate
from robot_app.units import build_units, display_env


def make_cfg(**sections):
    base = {"cameras": {k: dict(v) for k, v in SERIALS.items()}}
    for key, value in sections.items():
        base.setdefault(key, {}).update(value)
    return validate(base)


SERIALS = {"head": {"serial": "CP111"}, "left_wrist": {"serial": "CP222"}, "right_wrist": {"serial": "CP333"}}


def test_headset_id_is_passed_as_a_string_even_when_it_is_numeric():
    units = build_units(make_cfg(xr={"device_id": "123456"}), "/c.yaml", "/ws")
    assert 'device_id:="123456"' in units["xr"].cmd


def test_all_units_are_generated_from_the_config():
    units = build_units(make_cfg(), "/ws/config/robot.local.yaml", "/ws")
    assert units.names() == ["pc_service", "control", "rviz", "joystick", "xr",
                             "camera_left_wrist", "camera_right_wrist", "camera_head"]
    assert units["xr"].depends == ("pc_service",)
    assert "__node:=xr_target_node" in units["xr"].cmd
    assert "only:=camera_head" in units["camera_head"].cmd
    assert units["control"].cmd[:4] == ("ros2", "launch", "robot_bringup", "whole_body.launch.py")


def test_disabled_parts_produce_no_units():
    units = build_units(make_cfg(xr={"enable": False}, cameras={"enable": False}), "/c.yaml", "/ws")
    assert units.names() == ["control", "rviz", "joystick"]


def test_xr_without_managed_pc_service_has_no_dependency():
    units = build_units(make_cfg(xr={"start_pc_service": False}), "/c.yaml", "/ws")
    assert "pc_service" not in units
    assert units["xr"].depends == ()


def test_start_order_adds_dependencies_first_and_stop_order_dependents_first():
    units = build_units(make_cfg(), "/c.yaml", "/ws")
    assert units.start_order(["xr"]) == ["pc_service", "xr"]
    assert units.stop_order(["pc_service"]) == ["xr", "pc_service"]
    full = units.stop_order()
    assert full.index("xr") < full.index("pc_service")


def test_unknown_unit_names_the_choices():
    with pytest.raises(KeyError, match="可选"):
        build_units(make_cfg(), "/c.yaml", "/ws")["camera_tail"]


def test_joystick_is_an_on_demand_unit_that_needs_control():
    units = build_units(make_cfg(joystick={"device": "/dev/input/js2"}), "/c.yaml", "/ws")
    joystick = units["joystick"]
    assert joystick.on_demand and joystick.depends == ("control",)
    assert joystick.cmd[:4] == ("ros2", "launch", "robot_bringup", "joystick.launch.py")
    assert "joy_dev:=/dev/input/js2" in joystick.cmd
    auto = build_units(make_cfg(), "/c.yaml", "/ws")["joystick"]
    assert not any(a.startswith("joy_dev") for a in auto.cmd)
    assert "joystick" not in build_units(make_cfg(), "/c.yaml", "/ws").start_order()


def test_rviz_is_a_separate_on_demand_unit():
    units = build_units(make_cfg(), "/c.yaml", "/ws")
    control, rviz = units["control"].cmd, units["rviz"].cmd
    assert "launch_mode:=control_only" in control and "launch_mode:=rviz_only" in rviz
    # same robot / hand arguments, only the mode differs
    assert [a for a in control if not a.startswith("launch_mode")] == [a for a in rviz if not a.startswith("launch_mode")]
    assert units["rviz"].depends == ("control",) and units["rviz"].on_demand
    assert "rviz" not in units.start_order()          # up / "全部启动" leaves it out
    assert units.launch_order(["rviz"]) == ["rviz"]  # never starts the controllers by itself
    assert units.launch_order(["xr", "rviz"]) == ["pc_service", "xr", "rviz"]
    assert units.stop_order(["control"]) == ["joystick", "rviz", "control"]  # dependents stop first
    assert "rviz" in units.stop_order()               # "全部停止" still closes it


def test_display_env_keeps_an_existing_display_and_falls_back_to_local_x(monkeypatch, tmp_path):
    from robot_app import units as units_mod
    assert units_mod.display_env({"DISPLAY": ":1"}) == ()
    monkeypatch.setattr(units_mod.os.path, "exists", lambda p: p == "/tmp/.X11-unix/X0")
    (tmp_path / ".Xauthority").write_text("x")
    env = dict(units_mod.display_env({"HOME": str(tmp_path)}))
    assert env["DISPLAY"] == ":0" and env["XAUTHORITY"] == str(tmp_path / ".Xauthority")
