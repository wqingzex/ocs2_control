"""Package boundaries: robot_config knows no application package, and its layers only point downwards."""
import ast
from pathlib import Path

import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "robot_config"
APPLICATION_NAMES = ("robot_app", "joystick_teleop", "keyboard_teleop", "target_generator", "arm_controller", "robot_startup", "launch_helpers", "robot_visualize", "robot_navigation")
# module -> the robot_config modules it may import. Dependencies only point down:
#   setup_steps -> config_file -> config (the template, the validation, the device mappings).
# The device packages (camera, end_effector) sit above all of these; robot_config never imports them.
ALLOWED = {
    "config": set(),
    "testing": {"config"},
    "config_file": {"config"},
    "setup_steps": {"config", "config_file"},
}


def _sources():
    return sorted(PACKAGE_DIR.rglob("*.py"))


def _imports(path):
    """Absolute module names imported by a file (relative imports resolved)."""
    package = ["robot_config"] + list(path.relative_to(PACKAGE_DIR).parts[:-1])
    names = []
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            names += [alias.name for alias in node.names]
        elif isinstance(node, ast.ImportFrom):
            base = package[:len(package) - (node.level - 1)] if node.level else []
            module = ".".join(base + ([node.module] if node.module else []))
            names += [module] + [f"{module}.{alias.name}" for alias in node.names]
    return names


@pytest.mark.parametrize("name", APPLICATION_NAMES)
def test_shared_package_never_mentions_application_packages(name):
    offenders = [str(path.relative_to(PACKAGE_DIR)) for path in _sources() if name in path.read_text(encoding="utf-8")]
    assert not offenders, f"robot_config must not know '{name}': {offenders}"


def test_shared_package_never_imports_the_device_packages():
    """camera and end_effector depend on robot_config, so it must not import them anywhere, not even inside a
    function (it did, to load the fingerprint registrations: a cycle package.xml did not show)."""
    for path in _sources():
        for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
            modules = [a.name for a in node.names] if isinstance(node, ast.Import) else \
                [node.module or ""] if isinstance(node, ast.ImportFrom) and node.level == 0 else []
            assert not any(m.split(".")[0] in ("camera", "end_effector") for m in modules), \
                f"{path.name} imports a device package"


def test_every_module_has_a_dependency_rule():
    assert {path.stem for path in PACKAGE_DIR.glob("*.py") if path.stem != "__init__"} == set(ALLOWED)


@pytest.mark.parametrize("module", sorted(ALLOWED))
def test_modules_only_import_the_modules_below_them(module):
    for name in _imports(PACKAGE_DIR / f"{module}.py"):
        parts = name.split(".")
        if len(parts) > 1 and parts[0] == "robot_config" and parts[1] in ALLOWED and parts[1] != module:
            assert parts[1] in ALLOWED[module], f"{module} imports {name}"
