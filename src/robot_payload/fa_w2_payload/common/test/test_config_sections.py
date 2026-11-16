"""Section ownership: fa_w2_payload checks robot and cameras; other packages add their sections via Extension."""
import pytest

from fa_w2_payload.config import ConfigError, Extension, expect, load, validate

CAMERAS_OFF = {"cameras": {"enable": False}}


def test_foreign_sections_are_ignored_and_not_returned():
    cfg = validate({**CAMERAS_OFF, "xr": {"anything": 1}, "navigation": "not even a mapping"})
    assert set(cfg) == {"robot", "cameras"}


def test_own_sections_are_still_strict_without_extension():
    with pytest.raises(ConfigError, match="unknown field 'robot.colour'"):
        validate({**CAMERAS_OFF, "robot": {"colour": "red"}})


def test_profile_sections_pass_through_untouched():
    assert set(validate({**CAMERAS_OFF, "hardware": {"max_joint_speed": 3}})) == {"robot", "cameras"}


def test_extension_adds_defaults_checks_and_renames():
    def check(cfg):
        expect(cfg["extra"]["level"] > 0, "'extra.level' must be > 0")

    ext = Extension(defaults={"extra": {"level": 1}}, renamed={"extra": {"old": "level"}}, checks=(check,))
    assert validate(CAMERAS_OFF, ext)["extra"] == {"level": 1}
    assert validate({**CAMERAS_OFF, "extra": {"old": 5}}, ext)["extra"] == {"level": 5}
    with pytest.raises(ConfigError, match="extra.level"):
        validate({**CAMERAS_OFF, "extra": {"level": 0}}, ext)


def test_extension_makes_unknown_top_level_sections_an_error():
    ext = Extension(defaults={"extra": {"level": 1}})
    with pytest.raises(ConfigError, match="unknown field 'camera'"):
        validate({**CAMERAS_OFF, "camera": {}}, ext)


def test_load_passes_the_extension_through(tmp_path):
    path = tmp_path / "robot.yaml"
    path.write_text("cameras: {enable: false}\nextra: {level: 4}\n")
    assert load(str(path), Extension(defaults={"extra": {"level": 1}}))["extra"]["level"] == 4
