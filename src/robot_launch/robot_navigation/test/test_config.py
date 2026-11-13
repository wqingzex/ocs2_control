"""Map lookup: the default is the first map found, and an unresolvable map fails loudly."""
import pytest

from robot_navigation import config


def _map(root, name):
    folder = root / "config" / "maps" / name
    folder.mkdir(parents=True)
    (folder / "map.yaml").write_text("image: map.png\n", encoding="utf-8")
    return folder / "map.yaml"


@pytest.fixture
def shares(tmp_path, monkeypatch):
    """Two packages on the search chain: the robot's own description and a component package."""
    robot = tmp_path / "robot_description"
    component = tmp_path / "component"
    mapping = {"demo_description": str(robot), "component": str(component)}
    monkeypatch.setattr(config, "get_package_share", lambda pkg, ws_dir=None: mapping.get(pkg))
    config._nav_config_cache.clear()
    return robot, component


def test_default_map_is_the_first_one_found(shares):
    robot, component = shares
    _map(component, "siemens")
    _map(component, "wind")

    maps = config.list_navigation_maps(robot_name="demo", map_config_package="component")

    assert [m["id"] for m in maps] == ["siemens", "wind"]
    assert [m["default"] for m in maps] == [True, False]


def test_empty_map_argument_resolves_to_the_default_map(shares):
    _, component = shares
    expected = _map(component, "siemens")

    resolved = config.resolve_map_yaml("", robot_name="demo", map_config_package="component")

    assert resolved == str(expected)


def test_map_is_resolved_by_folder_name(shares):
    _, component = shares
    _map(component, "siemens")
    wanted = _map(component, "wind")

    resolved = config.resolve_map_yaml("wind", robot_name="demo", map_config_package="component")

    assert resolved == str(wanted)


def test_unknown_map_raises_instead_of_falling_back(shares):
    _, component = shares
    _map(component, "siemens")

    with pytest.raises(FileNotFoundError, match="nope"):
        config.resolve_map_yaml("nope", robot_name="demo", map_config_package="component")


def test_no_maps_anywhere_raises_with_the_searched_packages(shares):
    with pytest.raises(FileNotFoundError, match="component"):
        config.default_map_yaml(robot_name="demo", map_config_package="component")


def test_robot_package_maps_are_found_without_an_explicit_package(shares):
    robot, _ = shares
    expected = _map(robot, "lab")

    assert config.default_map_yaml(robot_name="demo") == str(expected)


def test_nav2_params_fall_back_to_the_package_default(shares, monkeypatch, tmp_path):
    nav = tmp_path / "nav_pkg"
    (nav / "config" / "nav2").mkdir(parents=True)
    default = nav / "config" / "nav2" / config.NAV2_DEFAULT_FILENAME
    default.write_text("{}\n", encoding="utf-8")
    monkeypatch.setattr(
        config, "get_package_share", lambda pkg, ws_dir=None: str(nav) if pkg == "robot_navigation" else None
    )

    assert config.resolve_nav2_params(robot_name="demo") == str(default)


def test_a_package_that_is_not_installed_is_not_an_error(monkeypatch):
    def missing(name):
        raise config.PackageNotFoundError(name)

    monkeypatch.setattr(config, "get_package_share_directory", missing)

    assert config.get_package_share("nope") is None


def test_unexpected_errors_are_not_swallowed(monkeypatch):
    def broken(name):
        raise RuntimeError("ament index is broken")

    monkeypatch.setattr(config, "get_package_share_directory", broken)

    with pytest.raises(RuntimeError, match="ament index"):
        config.get_package_share("anything")
