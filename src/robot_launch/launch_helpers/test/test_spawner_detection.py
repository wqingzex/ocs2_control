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
