"""spawners: which controller_manager entries a launch file spawns by name matching.

The MPC controllers (arm and whole-body plugin) are started by the main spawner of each launch file. A name such as
whole_body_controller contains "body", so it must never be picked up by the "body"/"head"/"hand" matching.
"""
import pytest

from launch_helpers.control import spawners

ARM = {"type": "arm_controller/ArmController"}
WHOLE_BODY = {"type": "whole_body_controller/WholeBodyController"}
JOINT = {"type": "joint_controller/JointController"}
HAND = {"type": "end_effector_controller/HandController"}
GRIPPER = {"type": "end_effector_controller/GripperController"}
CONTROLLER_MANAGER = {
    "arm_controller": ARM,
    "whole_body_controller": WHOLE_BODY,
    "body_joint_controller": JOINT,
    "head_joint_controller": JOINT,
    "left_hand_controller": HAND,
    "right_hand_controller": HAND,
    "left_gripper_controller": GRIPPER,
    "joint_state_broadcaster": {"type": "joint_state_broadcaster/JointStateBroadcaster"},
}
CONFIG = {"controller_manager": {"ros__parameters": CONTROLLER_MANAGER}}


def _detected(patterns):
    found = spawners.detect_controllers("robot", "", patterns, ros2_control_config=CONFIG)
    return sorted(controller["name"] for controller in found)


@pytest.mark.parametrize("patterns, expected", [
    (["body", "head"], ["body_joint_controller", "head_joint_controller"]),
    (["head"], ["head_joint_controller"]),
    (["hand", "gripper"], ["left_gripper_controller", "left_hand_controller", "right_hand_controller"]),
])
def test_name_matching_finds_the_joint_controllers_only(patterns, expected):
    assert _detected(patterns) == expected


def test_mpc_controllers_are_never_matched_by_name():
    every_part = ["arm", "whole", "body", "head", "hand", "gripper", "controller"]

    assert not {"arm_controller", "whole_body_controller"} & set(_detected(every_part))


def test_a_plain_string_entry_of_the_mpc_type_is_also_skipped():
    config = {
        "controller_manager": {"ros__parameters": {"whole_body_controller": "whole_body_controller/WholeBodyController"}},
    }

    assert spawners.detect_controllers("robot", "", ["body"], ros2_control_config=config) == []


def test_the_joint_plugin_is_a_normal_joint_controller():
    config = {"controller_manager": {"ros__parameters": {"body_joint_controller": JOINT}}}

    found = spawners.detect_controllers("robot", "", ["body"], ros2_control_config=config)

    assert [controller["name"] for controller in found] == ["body_joint_controller"]


# ---- FT sensor broadcasters: spawned only for a <sensor> the URDF declares ----------------------------------------

FT_CONFIG = {
    "controller_manager": {"ros__parameters": {
        "left_ft_broadcaster": {"type": "force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster"},
        "right_ft_broadcaster": {"type": "force_torque_sensor_broadcaster/ForceTorqueSensorBroadcaster"},
    }},
    "left_ft_broadcaster": {"ros__parameters": {"sensor_name": "left_ft_sensor", "frame_id": "left_eef"}},
    "right_ft_broadcaster": {"ros__parameters": {"sensor_name": "right_ft_sensor", "frame_id": "right_eef"}},
}
URDF_LEFT_FT = """<robot name="r">
  <joint name="left_gripper_joint" type="fixed"/>
  <ros2_control name="left_ft" type="sensor"><sensor name="left_ft_sensor"/></ros2_control>
</robot>"""


def test_ft_broadcaster_is_spawned_for_a_declared_sensor_only():
    found = spawners.detect_controllers("robot", "", ["ft_broadcaster"], robot_description=URDF_LEFT_FT,
                                        ros2_control_config=FT_CONFIG)
    assert [c["name"] for c in found] == ["left_ft_broadcaster"]


def test_ft_broadcaster_is_not_spawned_without_a_robot_description():
    assert spawners.detect_controllers("robot", "", ["ft_broadcaster"], ros2_control_config=FT_CONFIG) == []


def test_ft_broadcaster_detection_fails_loudly_on_a_broken_robot_description():
    import xml.etree.ElementTree as ET
    with pytest.raises(ET.ParseError):
        spawners.detect_controllers("robot", "", ["ft_broadcaster"], robot_description="<robot><unclosed",
                                    ros2_control_config=FT_CONFIG)
