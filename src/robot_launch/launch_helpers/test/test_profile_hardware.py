"""robot.local.yaml `hardware:` keys become xacro arguments, and only on real hardware."""
from launch_helpers.robot import profile


def _write(tmp_path, text):
    path = tmp_path / "robot.local.yaml"
    path.write_text(text, encoding="utf-8")
    return str(path)


PROFILE = """
platform:
  chassis: linkhou_s2
hardware:
  body_can_interface: can1
  max_joint_speed: 100
"""


def test_body_can_interface_reaches_the_xacro_on_real_hardware(tmp_path):
    mappings = profile.build_xacro_mappings("real", {}, _write(tmp_path, PROFILE))

    assert mappings["body_can_interface"] == "can1"
    assert mappings["max_joint_speed"] == "100"


def test_hardware_keys_are_ignored_in_simulation(tmp_path):
    mappings = profile.build_xacro_mappings("mock_components", {}, _write(tmp_path, PROFILE))

    assert "body_can_interface" not in mappings


def test_a_launch_argument_overrides_the_profile(tmp_path):
    mappings = profile.build_xacro_mappings(
        "real", {"hardware_body_can_interface": "can3"}, _write(tmp_path, PROFILE)
    )

    assert mappings["body_can_interface"] == "can3"
