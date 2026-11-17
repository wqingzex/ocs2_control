"""The camera units declare a ready text, so the supervisor starts them one after another."""
from launch_helpers.devices.camera import camera_units
from robot_config.config import validate
from robot_config.testing import full_config


def test_camera_units_wait_for_the_driver_to_finish_initializing():
    cameras = {"head": {"serial": "CP111"}, "left_wrist": {"serial": "CP222"}, "right_wrist": {"serial": "CP333"}}
    units = camera_units(validate(full_config(cameras=cameras)), "/c.yaml")
    for name in ("camera_head", "camera_left_wrist", "camera_right_wrist"):
        assert units[name].ready_log == "Initialize device cost" and units[name].ready_timeout_s > 0


# ---- imaging parameters belong to each camera ---------------------------------------------------------
from launch_helpers.devices.camera import camera_specs

SERIALS = {"head": {"serial": "CP111"}, "left_wrist": {"serial": "CP222"}, "right_wrist": {"serial": "CP333"}}


def args_of(cameras):
    return {spec.name: spec.args for spec in camera_specs(validate(full_config(cameras=cameras)))}


def test_the_template_gives_the_head_auto_exposure_and_the_wrists_fixed_exposure():
    args = args_of(SERIALS)
    assert args["camera_head"]["enable_color_auto_exposure"] == "true"
    assert args["camera_head"]["color_exposure"] == "-1" and args["camera_head"]["color_gain"] == "-1"
    for wrist in ("camera_left_wrist", "camera_right_wrist"):
        assert args[wrist]["enable_color_auto_exposure"] == "false"
        assert args[wrist]["color_exposure"] == "50" and args[wrist]["color_gain"] == "16"
        assert args[wrist]["color_white_balance"] == "5600" and args[wrist]["color_ae_max_exposure"] == "120"


def test_each_camera_uses_its_own_imaging_values():
    cameras = {**SERIALS,
               "left_wrist": {"serial": "CP222", "exposure": 70, "gain": 8, "white_balance": 4600},
               "head": {"serial": "CP111", "ae_max_exposure": 90}}
    args = args_of(cameras)
    assert args["camera_left_wrist"]["color_exposure"] == "70" and args["camera_left_wrist"]["color_white_balance"] == "4600"
    assert args["camera_right_wrist"]["color_exposure"] == "50" and args["camera_right_wrist"]["color_white_balance"] == "5600"
    assert args["camera_head"]["color_ae_max_exposure"] == "90" and args["camera_left_wrist"]["color_ae_max_exposure"] == "120"


def test_a_wrist_can_use_auto_exposure_and_the_head_a_fixed_one():
    args = args_of({**SERIALS, "left_wrist": {"serial": "CP222", "exposure": -1, "gain": -1},
                    "head": {"serial": "CP111", "exposure": 30, "gain": 10}})
    assert args["camera_left_wrist"]["enable_color_auto_exposure"] == "true"
    assert args["camera_head"]["enable_color_auto_exposure"] == "false" and args["camera_head"]["color_gain"] == "10"
