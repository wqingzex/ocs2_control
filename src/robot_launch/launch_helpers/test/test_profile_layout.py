"""robot.local.yaml (robot: base / body / arm / end_effector, control:) -> the xacro arguments and drivers it names.

The table of names is the contract with the robot descriptions: the xacro argument names did not change."""
from launch_helpers.robot.profile import normalize_robot_profile

FULL = {
    "robot": {
        "variant": "oem",
        "base": {"type": "linkhou_s2_v2", "joints_movable": False},
        "body": {"type": "juxie", "interface": {"can": "can7", "stats_period_s": 10}},
        "arm": {"type": "marvin", "interface": {"ip": "10.0.0.2", "firmware": 343}},
        "end_effector": {
            "type": "linkerhand_l6",
            "interface": {"bus": "usb_can",
                          "left": {"port": "/dev/serial/by-id/usb-L", "can": "can3", "channel": 2},
                          "right": {"port": "/dev/serial/by-id/usb-R", "can": "can4", "channel": 3}},
            "dyn_param": {"left": [1, 0, 0, 0.1, 0, 0, 0, 0, 0, 0], "right": []},
        },
    },
    "control": {"arm_mode": "joint_impedance", "limits": {"max_joint_speed": 80, "max_joint_acceleration": 60},
                "patch": {"body_joint_controller": {"ros__parameters": {"home_1": [0.0]}}}},
}


def test_every_field_reaches_its_xacro_argument():
    p = normalize_robot_profile(FULL)

    assert p["xacro"] == {"base": "linkhou_s2_v2", "base_joints_movable": "false", "variant": "oem"}
    assert p["eef"] == {"end_effector": "linkerhand_l6", "bus": "usb_can"}
    assert p["hardware"] == {
        "arm_driver": "marvin", "device_ip": "10.0.0.2",
        "body_can_interface": "can7", "body_can_stats_period_s": "10",
        "arm_ctrl_mode": "joint_impedance", "max_joint_speed": "80", "max_joint_acceleration": "60",
        "usb_left_port": "/dev/serial/by-id/usb-L", "left_ee_can_interface": "can3", "left_ee_channel": "2",
        "usb_right_port": "/dev/serial/by-id/usb-R", "right_ee_can_interface": "can4", "right_ee_channel": "3",
        "left_dyn_param": "1,0,0,0.1,0,0,0,0,0,0",
    }
    assert p["control"] == {"patch": FULL["control"]["patch"]}


def test_what_is_not_written_stays_out_so_the_description_default_applies():
    p = normalize_robot_profile({"robot": {"end_effector": {"type": "xhand1"}}})

    assert p["xacro"] == {} and p["hardware"] == {} and p["control"] == {}


def test_the_read_write_rate_of_each_hand_reaches_its_xacro_argument():
    p = normalize_robot_profile({"robot": {"end_effector": {"type": "xhand1",
                                                            "interface": {"left": {"rw_rate": 50}, "right": {"rw_rate": 20}}}}})

    assert p["hardware"] == {"left_ee_rw_rate": "50", "right_ee_rw_rate": "20"}


def test_a_hand_rate_that_is_not_written_leaves_the_controller_manager_rate_in_charge():
    p = normalize_robot_profile({"robot": {"end_effector": {"type": "xhand1",
                                                            "interface": {"left": {"rw_rate": None}, "right": {}}}}})

    assert "left_ee_rw_rate" not in p["hardware"] and "right_ee_rw_rate" not in p["hardware"]


def test_an_empty_value_is_not_written_either():
    p = normalize_robot_profile({"robot": {"body": {"interface": {"can": ""}},
                                          "end_effector": {"interface": {"left": {"port": "", "channel": None}}}}})

    assert p["hardware"] == {}


def test_the_former_layout_is_not_read():
    old = {"platform": {"chassis": "linkhou_s2", "arm_ctrl_mode": "position"},
           "hardware": {"body_can_interface": "can1", "device_ip": "10.0.0.9"}, "defaults": {"left_dyn_param": [1] * 10}}

    p = normalize_robot_profile(old)

    assert p["xacro"] == {} and p["hardware"] == {}


def test_the_load_override_is_passed_as_the_driver_reads_it():
    p = normalize_robot_profile({"robot": {"end_effector": {"dyn_param": {"right": [1.5] + [0.0] * 9}}}})

    assert p["hardware"] == {"right_dyn_param": "1.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0"}


def test_the_template_reaches_the_xacro_with_the_values_the_descriptions_use_anyway():
    """The template's values are the descriptions' own defaults, so a config made from it changes nothing."""
    from robot_config.config import template_for
    whole = normalize_robot_profile(template_for("whole_body"))
    assert whole["xacro"] == {"base": "linkhou_s2", "base_joints_movable": "false"}
    assert whole["hardware"] == {
        "arm_driver": "marvin", "device_ip": "192.168.1.190", "body_can_interface": "can0", "body_can_stats_period_s": "0",
        "arm_ctrl_mode": "position", "max_joint_speed": "100", "max_joint_acceleration": "100",
        "left_ee_can_interface": "can1", "right_ee_can_interface": "can2"}
    dual = normalize_robot_profile(template_for("dual_arm"))
    assert dual["xacro"] == {}  # no base on the dual arm
    assert dual["hardware"]["max_joint_speed"] == "50" and dual["hardware"]["max_joint_acceleration"] == "30"
    assert dual["hardware"]["left_ee_can_interface"] == "can0" and "body_can_interface" not in dual["hardware"]
