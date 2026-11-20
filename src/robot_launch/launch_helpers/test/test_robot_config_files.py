"""load_robot_config: which file a robot package or a hand package gets as its default."""
import os

import pytest

from launch_helpers.robot import config


@pytest.fixture
def package(tmp_path, monkeypatch):
    config_dir = tmp_path / "config" / "ros2_control"
    config_dir.mkdir(parents=True)
    monkeypatch.setattr(config, "get_robot_package_path", lambda name: str(tmp_path))
    config.clear_config_cache()
    yield config_dir
    config.clear_config_cache()


def load(robot_type=""):
    return config.load_robot_config("robot", "ros2_control", robot_type)


def test_a_robot_package_with_only_common_yaml_uses_it_as_the_default(package):
    (package / "common.yaml").write_text("arm_controller: {ros__parameters: {home_1: [0.0]}}\n")

    cfg, path, meta = load()

    assert cfg["arm_controller"]["ros__parameters"]["home_1"] == [0.0]
    assert os.path.basename(path) == "common.yaml" and not meta.type_yaml_found


def test_a_hand_package_without_common_yaml_uses_ros2_controllers_yaml(package):
    (package / "ros2_controllers.yaml").write_text("hand_controller: {ros__parameters: {joints: [a]}}\n")

    cfg, path, _ = load()

    assert cfg["hand_controller"]["ros__parameters"]["joints"] == ["a"]
    assert os.path.basename(path) == "ros2_controllers.yaml"


def test_a_type_file_is_merged_over_common_yaml(package):
    (package / "common.yaml").write_text("arm_controller: {ros__parameters: {a: 1, b: 1}}\n")
    (package / "left.yaml").write_text("arm_controller: {ros__parameters: {b: 2}}\n")

    cfg, path, meta = load("left")

    assert cfg["arm_controller"]["ros__parameters"] == {"a": 1, "b": 2}
    assert os.path.basename(path) == "left.yaml" and meta.type_yaml_found


def test_a_package_without_any_default_file_raises(package):
    with pytest.raises(FileNotFoundError):
        load()
