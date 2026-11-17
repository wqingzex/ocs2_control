"""The template is the schema: complete for every entry, the same keys for all of them."""
import io

import pytest
import yaml

from robot_config import template
from robot_config.config import validate
from robot_config.testing import full_config


def keys(node, prefix=""):
    """Every key path of a config; control.patch and setup are free-form and end the path."""
    found = set()
    for key, value in node.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict) and path not in template.FREEFORM:
            found |= keys(value, path)
        else:
            found.add(path)
    return found


def test_there_is_one_overlay_per_entry():
    assert template.entries() == ("dual_arm", "split_body", "whole_body")


@pytest.mark.parametrize("entry", template.entries())
def test_the_template_of_every_entry_is_valid_with_the_cameras_off(entry):
    cfg = full_config(entry, cameras={"enable": False})
    assert validate(cfg)["robot"]["entry"] == entry


def test_every_entry_has_the_same_keys():
    reference = keys(template.template_for("whole_body"))
    for entry in template.entries():
        assert keys(template.template_for(entry)) == reference, entry


def test_an_overlay_only_changes_values():
    base, dual = template.template_for("whole_body"), template.template_for("dual_arm")
    assert dual["control"]["limits"] == {"max_joint_speed": 50, "max_joint_acceleration": 30}
    assert base["control"]["limits"] == {"max_joint_speed": 100, "max_joint_acceleration": 100}
    assert dual["robot"]["base"] == {"type": "", "joints_movable": None}
    assert dual["xr"] == base["xr"]


def test_the_text_of_a_new_file_is_the_template_with_its_comments():
    text = template.template_text("dual_arm")
    assert yaml.safe_load(io.StringIO(text)) == template.template_for("dual_arm")
    assert "[必改]" in text and text.startswith("#")


def test_an_unknown_entry_has_no_template():
    with pytest.raises(ValueError, match="no template"):
        template.template_for("nonsense")


def test_section_defaults_are_the_same_for_every_entry():
    assert template.section_defaults("cameras")["fps"] == 30
    assert all(template.template_for(e)["cameras"] == template.section_defaults("cameras") for e in template.entries())
