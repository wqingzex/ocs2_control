"""compose: the gripper joint is read from `joints: [x]` (GripperController) and from the legacy `joint: x`."""
from launch_helpers.control import compose


def test_joint_name_comes_from_joints_list():
    params = {"joints": ["left_gripper_joint"], "end_effector": "gripper"}
    assert compose._gripper_joint_name(params, "fallback") == "left_gripper_joint"


def test_the_joints_list_gives_the_joint_name_and_the_old_joint_parameter_is_not_read_any_more():
    assert compose._gripper_joint_name({"joints": ["gripper_joint"]}, "fallback") == "gripper_joint"
    assert compose._gripper_joint_name({"joint": "gripper_joint"}, "fallback") == "fallback"


def test_joint_name_uses_the_default_when_nothing_is_configured():
    assert compose._gripper_joint_name({}, "fallback") == "fallback"
    assert compose._gripper_joint_name({"joints": []}, "fallback") == "fallback"


def test_single_arm_compose_adds_the_gripper_joint_of_the_joints_list():
    base = {
        "gripper_controller": {"ros__parameters": {"joints": ["gripper_joint"]}},
        "joint_state_broadcaster": {"ros__parameters": {"joints": ["joint1", "joint2"]}},
    }
    composed = {"left_gripper_controller": {}, "controller_manager": {"ros__parameters": {}}}

    result = compose.normalize_single_arm_compose(base, composed)

    assert result["joint_state_broadcaster"]["ros__parameters"]["joints"][-1] == "gripper_joint"
