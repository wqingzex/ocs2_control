"""End effector launch arguments: robot.end_effector.type / interface.bus from the profile, launch overrides."""
from launch_helpers.robot.profile import (
    build_xacro_mappings,
    normalize_robot_profile,
    resolve_end_effector_buses,
    resolve_end_effectors,
)


def profile(**end_effector):
    return normalize_robot_profile({"robot": {"end_effector": end_effector}})


def test_profile_reads_the_robot_section():
    p = profile(type="linkerhand_l6", interface={"bus": "usb_rs485"})
    assert p["eef"] == {"end_effector": "linkerhand_l6", "bus": "usb_rs485"}
    assert resolve_end_effectors({}, p) == ("linkerhand_l6", "linkerhand_l6")
    assert resolve_end_effector_buses({}, p) == ("usb_rs485", "usb_rs485")


def test_robot_section_alone_is_a_profile():
    assert normalize_robot_profile({"robot": {"end_effector": {"type": "xhand1"}}})["eef"] == {"end_effector": "xhand1"}


def test_launch_overrides_profile_and_side_overrides_both():
    p = profile(type="linkerhand_l6", interface={"bus": "usb_rs485"})
    assert resolve_end_effectors({"end_effector": "xhand1"}, p) == ("xhand1", "xhand1")
    assert resolve_end_effectors({"end_effector": "xhand1", "right_end_effector": "jodell_rg75"}, p) == (
        "xhand1", "jodell_rg75")
    assert resolve_end_effector_buses({"left_end_effector_bus": "arm_rs485"}, p) == ("arm_rs485", "usb_rs485")


def test_none_means_no_end_effector():
    assert resolve_end_effectors({"end_effector": "none"}, None) == ("", "")
    assert resolve_end_effectors({}, profile(type="none")) == ("", "")


def test_xacro_mappings_on_the_real_robot(tmp_path):
    path = tmp_path / "robot.yaml"
    path.write_text("robot:\n  end_effector: {type: linkerhand_l6, interface: {bus: usb_rs485}}\n"
                    "  arm: {interface: {ip: 10.0.0.9}}\n")
    mappings = build_xacro_mappings("real", {"xacro_end_effector": "ignored"}, str(path))
    assert mappings["left_end_effector"] == mappings["right_end_effector"] == "linkerhand_l6"
    assert mappings["left_end_effector_bus"] == mappings["right_end_effector_bus"] == "usb_rs485"
    assert mappings["device_ip"] == "10.0.0.9"
    assert "end_effector" not in mappings  # resolved per side, never copied from an xacro_ override


def test_xacro_mappings_in_simulation_have_no_bus(tmp_path):
    path = tmp_path / "robot.yaml"
    path.write_text("robot: {end_effector: {type: jodell_rg75, interface: {bus: arm_rs485}}}\n")
    mappings = build_xacro_mappings("mock_components", {"arms": "left"}, str(path))
    assert mappings["left_end_effector"] == "jodell_rg75"
    assert "left_end_effector_bus" not in mappings
    assert mappings["arms"] == "left"
