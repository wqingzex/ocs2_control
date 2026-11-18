"""The template is the schema: complete for every entry, the same keys for all of them."""

import pytest

from robot_config import config
from robot_config.config import validate
from robot_config.testing import full_config


def keys(node, prefix=""):
    """Every key path of a config; control.patch and setup are free-form and end the path."""
    found = set()
    for key, value in node.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict) and path not in config.FREEFORM:
            found |= keys(value, path)
        else:
            found.add(path)
    return found


def test_there_is_one_overlay_per_entry():
    assert config.entries() == ("dual_arm", "split_body", "whole_body")


@pytest.mark.parametrize("entry", config.entries())
def test_the_template_of_every_entry_is_valid_with_the_cameras_off(entry):
    cfg = full_config(entry, cameras={"enable": False})
    assert validate(cfg)["robot"]["entry"] == entry


def test_every_entry_has_the_same_keys():
    reference = keys(config.template_for("whole_body"))
    for entry in config.entries():
        assert keys(config.template_for(entry)) == reference, entry


def test_an_overlay_only_changes_values():
    base, dual = config.template_for("whole_body"), config.template_for("dual_arm")
    assert dual["control"]["limits"] == {"max_joint_speed": 50, "max_joint_acceleration": 30}
    assert base["control"]["limits"] == {"max_joint_speed": 100, "max_joint_acceleration": 100}
    assert dual["robot"]["base"] == {"type": "", "joints_movable": None}
    assert dual["xr"] == base["xr"]


def test_an_unknown_entry_has_no_template():
    with pytest.raises(ValueError, match="no template"):
        config.template_for("nonsense")


def test_section_defaults_are_the_same_for_every_entry():
    assert config.section_defaults("cameras")["fps"] == 30
    assert all(config.template_for(e)["cameras"] == config.section_defaults("cameras") for e in config.entries())
