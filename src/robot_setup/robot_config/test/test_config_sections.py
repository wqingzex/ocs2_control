"""The config must have every key of its entry's template, no more, and other packages add checks via Extension."""
import copy

import pytest

from robot_config.config import ConfigError, Extension, expect, load, load_lenient, validate
from robot_config.testing import full_config

CAMERAS_OFF = {"enable": False}


def config(**sections):
    return full_config(cameras=CAMERAS_OFF, **sections)


def test_a_complete_config_is_valid_and_comes_back_without_control_and_setup():
    cfg = validate(config())
    assert "control" not in cfg and "setup" not in cfg and "robot" in cfg and "xr" in cfg


@pytest.mark.parametrize("path, expected", [
    (("robot", "arm", "interface", "ip"), "missing field 'robot.arm.interface.ip'"),
    (("robot", "base"), "missing field 'robot.base'"),
    (("xr",), "missing field 'xr'"),
    (("control", "limits", "max_joint_speed"), "missing field 'control.limits.max_joint_speed'"),
    (("setup",), "missing field 'setup'"),
    (("cameras", "head", "rotation"), "missing field 'cameras.head.rotation'"),
])
def test_a_missing_key_is_an_error_that_says_how_to_add_it(path, expected):
    cfg = config()
    node = cfg
    for key in path[:-1]:
        node = node[key]
    del node[path[-1]]
    with pytest.raises(ConfigError, match=expected) as raised:
        validate(cfg)
    assert "setup.sh config" in str(raised.value)


@pytest.mark.parametrize("section, expected", [
    ({"robot": {"colour": "red"}}, "unknown field 'robot.colour'"),
    ({"platform": {}}, "unknown field 'platform'"),
    ({"hardware": {"can": 1}}, "unknown field 'hardware'"),
    ({"robot": {"arm": {"interface": {"port": 1}}}}, "unknown field 'robot.arm.interface.port'"),
    ({"control": {"foo": 1}}, "unknown field 'control.foo'"),
])
def test_a_key_that_is_not_in_the_template_is_an_error(section, expected):
    with pytest.raises(ConfigError, match=expected):
        validate(config(**section))


def test_no_default_is_filled_in():
    cfg = config()
    del cfg["joystick"]["device"]
    with pytest.raises(ConfigError, match="joystick.device"):
        validate(cfg)


def test_the_entry_picks_the_template():
    assert validate(full_config("dual_arm", cameras=CAMERAS_OFF))["robot"]["entry"] == "dual_arm"
    cfg = config()
    cfg["robot"]["entry"] = "nonsense"
    with pytest.raises(ConfigError, match="robot.entry"):
        validate(cfg)
    del cfg["robot"]["entry"]
    with pytest.raises(ConfigError, match="missing field 'robot.entry'"):
        validate(cfg)


def test_control_patch_and_setup_are_free_form():
    cfg = config(control={"patch": {"any_controller": {"ros__parameters": {"x": 1}}}}, setup={"cameras": {"confirmed": True}})
    validate(cfg)
    cfg["control"]["patch"] = []
    with pytest.raises(ConfigError, match="control.patch"):
        validate(cfg)


def test_extension_adds_checks_of_its_own_sections():
    def check(cfg):
        expect(cfg["joystick"]["device"] == "", "'joystick.device' must stay empty here")

    ext = Extension(checks=(check,))
    validate(config(), ext)
    with pytest.raises(ConfigError, match="joystick.device"):
        validate(config(joystick={"device": "/dev/input/js1"}), ext)
    assert len(Extension(checks=(check,)).merge(Extension(checks=(check,))).checks) == 2


def test_load_passes_the_extension_through(tmp_path):
    import yaml
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(config()))
    ext = Extension(checks=(lambda cfg: expect(False, "checked"),))
    with pytest.raises(ConfigError, match="checked"):
        load(str(path), ext)


def test_load_lenient_turns_the_cameras_off_when_they_are_what_fails(tmp_path):
    import yaml
    cfg = full_config()  # cameras on, no serials
    path = tmp_path / "robot.yaml"
    path.write_text(yaml.safe_dump(cfg))
    with pytest.raises(ConfigError, match="cameras.head.serial"):
        load(str(path))
    lenient = load_lenient(str(path))
    assert lenient["cameras"]["enable"] is False and lenient["cameras"]["fps"] == 30  # the other camera keys stay


def test_validate_does_not_change_its_input():
    cfg = config()
    before = copy.deepcopy(cfg)
    validate(cfg)
    assert cfg == before


@pytest.mark.parametrize("fields, expected", [
    ({"white_balance": 0}, "cameras.head.white_balance"),
    ({"ae_max_exposure": "120"}, "cameras.head.ae_max_exposure"),
    ({"exposure": 0, "gain": 0}, "cameras.head.exposure"),
    ({"exposure": 50, "gain": -2}, "cameras.head.gain"),
    ({"exposure": 50.5, "gain": 16}, "cameras.head.exposure"),
    ({"exposure": 50}, "both be -1"),   # head gain stays -1 (auto)
    ({"gain": 16}, "both be -1"),       # head exposure stays -1 (auto)
])
def test_the_imaging_values_of_a_camera_are_checked(fields, expected):
    cameras = {"enable": False, "head": fields}
    with pytest.raises(ConfigError, match=expected):
        validate(full_config(cameras=cameras))


def test_exposure_and_gain_may_be_both_auto_or_both_fixed():
    for fields in ({"exposure": -1, "gain": -1}, {"exposure": 50, "gain": 16}, {"exposure": 1, "gain": 0}):
        validate(full_config(cameras={"enable": False, "head": fields}))


def test_the_old_global_imaging_keys_are_unknown_fields():
    with pytest.raises(ConfigError, match="unknown field 'cameras.wrist_exposure'"):
        validate(full_config(cameras={"enable": False, "wrist_exposure": 50}))
