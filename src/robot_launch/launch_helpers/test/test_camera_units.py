"""The camera units declare a ready text, so the supervisor starts them one after another."""
from launch_helpers.devices.camera import camera_units
from robot_config.config import validate


def test_camera_units_wait_for_the_driver_to_finish_initializing():
    units = camera_units(validate({"cameras": {"head": {"serial": "CP111"}, "left_wrist": {"serial": "CP222"}, "right_wrist": {"serial": "CP333"}}}), "/c.yaml")
    for name in ("camera_head", "camera_left_wrist", "camera_right_wrist"):
        assert units[name].ready_log == "Initialize device cost" and units[name].ready_timeout_s > 0
