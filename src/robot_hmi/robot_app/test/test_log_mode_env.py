"""log.mode of the config reaches every process as ROBOT_LOG_MODE; nothing switches it while the system runs."""
import pytest

from robot_app import config as teleop_config
from robot_app.units import LOG_MODE_ENV, build_units

CAMERAS_OFF = "cameras: {enable: false}\n"  # a strict load wants camera serials; these tests are not about cameras


def load(tmp_path, text):
    path = tmp_path / "robot.yaml"
    path.write_text(CAMERAS_OFF + text, encoding="utf-8")
    return str(path), teleop_config.load(str(path))


def test_the_mode_defaults_to_normal(tmp_path):
    _, cfg = load(tmp_path, "robot: {entry: split_body}\n")
    assert cfg["log"]["mode"] == "normal"


@pytest.mark.parametrize("mode", ["normal", "debug"])
def test_the_config_takes_normal_and_debug(tmp_path, mode):
    _, cfg = load(tmp_path, f"robot: {{entry: split_body}}\nlog: {{mode: {mode}}}\n")
    assert cfg["log"]["mode"] == mode


@pytest.mark.parametrize("bad", ["loud", "Debug", "debug:control", ""])
def test_the_config_rejects_any_other_value(tmp_path, bad):
    with pytest.raises(teleop_config.ConfigError, match="log.mode"):
        load(tmp_path, f"robot: {{entry: split_body}}\nlog: {{mode: '{bad}'}}\n")


@pytest.mark.parametrize("mode", ["normal", "debug"])
def test_every_unit_gets_the_mode_in_its_environment(tmp_path, mode):
    path, cfg = load(tmp_path, f"robot: {{entry: split_body}}\nlog: {{mode: {mode}}}\n")
    units = build_units(cfg, path, str(tmp_path))
    assert units.names()
    for name in units.names():
        assert dict(units[name].env)[LOG_MODE_ENV] == mode, name


def test_the_mode_adds_no_unit_and_no_runtime_switch(tmp_path):
    path, cfg = load(tmp_path, "robot: {entry: split_body}\nlog: {mode: debug}\n")
    names = build_units(cfg, path, str(tmp_path)).names()
    assert "log_mode" not in names
    assert not hasattr(teleop_config, "parse_mode")
