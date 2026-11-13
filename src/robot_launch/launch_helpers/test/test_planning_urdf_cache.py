"""Planning URDF cache: the file name follows the generated content, so edits are never served stale."""
import os
import stat

import pytest

from launch_helpers.robot import urdf

URDF_A = '<robot name="a"><link name="base"/></robot>'
URDF_B = '<robot name="a"><link name="base"/><link name="tool"/></robot>'


@pytest.fixture(autouse=True)
def cache_root(tmp_path, monkeypatch):
    monkeypatch.setenv("OCS2_PLANNING_URDF_DIR", str(tmp_path))
    return tmp_path


def test_same_content_returns_same_path(cache_root):
    first = urdf.write_planning_urdf_cache("w2", URDF_A)
    second = urdf.write_planning_urdf_cache("w2", URDF_A)

    assert first == second
    assert os.path.isfile(first)


def test_changed_content_returns_new_path_and_keeps_old_file(cache_root):
    old = urdf.write_planning_urdf_cache("w2", URDF_A)
    new = urdf.write_planning_urdf_cache("w2", URDF_B)

    assert old != new
    assert open(new, encoding="utf-8").read() == URDF_B
    assert open(old, encoding="utf-8").read() == URDF_A


def test_write_leaves_no_temp_files(cache_root):
    path = urdf.write_planning_urdf_cache("w2", URDF_A)

    leftovers = [name for name in os.listdir(os.path.dirname(path)) if name.endswith(".tmp")]
    assert leftovers == []


def test_empty_content_writes_nothing(cache_root):
    assert urdf.write_planning_urdf_cache("w2", "") == ""


def test_robots_use_separate_directories(cache_root):
    w2 = urdf.write_planning_urdf_cache("w2", URDF_A)
    m6 = urdf.write_planning_urdf_cache("m6_ccs", URDF_A)

    assert os.path.dirname(w2) != os.path.dirname(m6)


def _stub_generation(monkeypatch, tmp_path, urdf_sequence):
    """Point build_planning_urdf_launch_params at a scripted URDF generator."""
    outputs = iter(urdf_sequence)
    monkeypatch.setattr(urdf, "get_planning_robot_description", lambda *a, **k: next(outputs))


def test_edited_xacro_with_unchanged_parameters_gets_a_fresh_file(cache_root, monkeypatch):
    _stub_generation(monkeypatch, cache_root, [URDF_A, URDF_B])

    before = urdf.build_planning_urdf_launch_params("w2", {}, planning_scope="full")
    after = urdf.build_planning_urdf_launch_params("w2", {}, planning_scope="full")

    assert before["planning_urdf_path"] != after["planning_urdf_path"]
    assert open(after["planning_urdf_path"], encoding="utf-8").read() == URDF_B


def test_unchanged_xacro_keeps_the_path_stable(cache_root, monkeypatch):
    _stub_generation(monkeypatch, cache_root, [URDF_A, URDF_A])

    before = urdf.build_planning_urdf_launch_params("w2", {}, planning_scope="full")
    after = urdf.build_planning_urdf_launch_params("w2", {}, planning_scope="full")

    assert before == after
    assert before["planning_urdf_variant"] == "xacro"


def test_generation_failure_returns_empty_params(cache_root, monkeypatch):
    _stub_generation(monkeypatch, cache_root, [None])

    assert urdf.build_planning_urdf_launch_params("w2", {}, planning_scope="full") == {}


def test_cache_directory_is_private_to_the_current_user(cache_root):
    path = urdf.write_planning_urdf_cache("w2", URDF_A)

    info = os.stat(os.path.dirname(path))
    assert info.st_uid == os.getuid()
    assert stat.S_IMODE(info.st_mode) & 0o077 == 0


def test_unsafe_robot_names_are_rejected(cache_root):
    for name in ("../escape", "a/b", "", "w2 x", ".."):
        with pytest.raises(ValueError):
            urdf.write_planning_urdf_cache(name, URDF_A)


def test_a_tampered_cache_file_is_not_trusted(cache_root):
    path = urdf.write_planning_urdf_cache("w2", URDF_A)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("<robot name='planted'/>")

    again = urdf.write_planning_urdf_cache("w2", URDF_A)

    assert again == path
    assert open(path, encoding="utf-8").read() == URDF_A


def test_a_directory_writable_by_others_is_refused(cache_root):
    path = urdf.write_planning_urdf_cache("w2", URDF_A)
    os.chmod(os.path.dirname(path), 0o777)

    with pytest.raises(PermissionError):
        urdf.write_planning_urdf_cache("w2", URDF_B)


def test_without_an_override_the_xdg_cache_home_is_used(cache_root, monkeypatch):
    monkeypatch.delenv("OCS2_PLANNING_URDF_DIR", raising=False)
    monkeypatch.setenv("XDG_CACHE_HOME", str(cache_root / "xdg"))

    path = urdf.write_planning_urdf_cache("w2", URDF_A)

    assert path.startswith(str(cache_root / "xdg" / "ocs2_ros2" / "w2" / "planning_urdf"))


def test_without_xdg_the_home_cache_is_used(cache_root, monkeypatch):
    monkeypatch.delenv("OCS2_PLANNING_URDF_DIR", raising=False)
    monkeypatch.delenv("XDG_CACHE_HOME", raising=False)
    monkeypatch.setenv("HOME", str(cache_root / "home"))

    path = urdf.write_planning_urdf_cache("w2", URDF_A)

    assert path.startswith(str(cache_root / "home" / ".ros" / "ocs2_cache" / "w2" / "planning_urdf"))


def test_the_last_resort_root_is_a_per_user_directory(cache_root, monkeypatch):
    for name in ("OCS2_PLANNING_URDF_DIR", "XDG_CACHE_HOME", "HOME"):
        monkeypatch.delenv(name, raising=False)

    assert urdf._cache_root() == f"/tmp/ocs2_ros2-{os.getuid()}"
