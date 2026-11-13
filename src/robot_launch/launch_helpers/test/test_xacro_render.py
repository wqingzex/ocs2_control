"""urdf: one place that finds a robot's xacro and turns it into URDF, failing soft with None."""
import pytest

from launch_helpers.robot import urdf


class _FakeDoc:
    def __init__(self, xml):
        self._xml = xml

    def toxml(self):
        return self._xml


@pytest.fixture
def fake_robot(tmp_path, monkeypatch):
    """A robot package with xacro/robot.xacro only (no ros2_control entry point)."""
    (tmp_path / "xacro").mkdir()
    (tmp_path / "xacro" / "robot.xacro").write_text("<robot/>", encoding="utf-8")
    monkeypatch.setattr(urdf, "get_robot_package_path", lambda name: str(tmp_path))
    return tmp_path


def test_resolves_the_requested_entry_point(fake_robot):
    path = urdf.resolve_robot_xacro("w2", urdf.PLANNING_XACRO)

    assert path == str(fake_robot / "xacro" / "robot.xacro")


def test_missing_entry_point_resolves_to_none(fake_robot):
    assert urdf.resolve_robot_xacro("w2", urdf.CONTROL_XACRO) is None


def test_unknown_robot_package_resolves_to_none(monkeypatch):
    monkeypatch.setattr(urdf, "get_robot_package_path", lambda name: None)

    assert urdf.resolve_robot_xacro("nope", urdf.PLANNING_XACRO) is None


def test_render_passes_mappings_and_returns_xml(fake_robot, monkeypatch):
    seen = {}

    def fake_process(path, mappings=None):
        seen["path"], seen["mappings"] = path, mappings
        return _FakeDoc("<robot name='w2'/>")

    monkeypatch.setattr(urdf.xacro, "process_file", fake_process)

    xml = urdf.render_robot_xacro("w2", urdf.PLANNING_XACRO, {"collider": "simple"})

    assert xml == "<robot name='w2'/>"
    assert seen["mappings"] == {"collider": "simple"}
    assert seen["path"].endswith("xacro/robot.xacro")


def test_render_returns_none_when_xacro_raises(fake_robot, monkeypatch):
    def boom(path, mappings=None):
        raise RuntimeError("undefined macro")

    monkeypatch.setattr(urdf.xacro, "process_file", boom)

    assert urdf.render_robot_xacro("w2", urdf.PLANNING_XACRO, {}) is None


def test_render_returns_none_without_calling_xacro_when_file_is_missing(fake_robot, monkeypatch):
    def must_not_run(path, mappings=None):
        raise AssertionError("xacro must not run for a missing entry point")

    monkeypatch.setattr(urdf.xacro, "process_file", must_not_run)

    assert urdf.render_robot_xacro("w2", urdf.CONTROL_XACRO, {}) is None
