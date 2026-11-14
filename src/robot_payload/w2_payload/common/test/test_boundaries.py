"""Package boundaries: w2_payload knows no application package, and its layers only point downwards."""
import ast
from pathlib import Path

import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "w2_payload"
APPLICATION_NAMES = ("teleop_app", "joystick_teleop", "keyboard_teleop", "teleop_command", "arm_controller", "robot_bringup", "launch_helpers", "robot_visualize", "robot_navigation")
# module -> the w2_payload modules it may import. Dependencies only point down:
#   load_identify / config_status -> device_setup -> config_edit -> config.
# The device packages (w2_camera, w2_hand) sit above all of these; w2_payload never imports them at module level.
ALLOWED = {
    "config": set(),
    "config_edit": {"config"},
    "device_setup": {"config_edit"},
    "config_status": {"config", "config_edit"},
    "load_identify": {"config_edit", "config_status", "device_setup"},
}


def _sources():
    return sorted(PACKAGE_DIR.rglob("*.py"))


def _imports(path):
    """Absolute module names imported by a file (relative imports resolved)."""
    package = ["w2_payload"] + list(path.relative_to(PACKAGE_DIR).parts[:-1])
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
    assert not offenders, f"w2_payload must not know '{name}': {offenders}"


def test_shared_package_imports_the_device_packages_only_inside_functions():
    """w2_camera and w2_hand sit above w2_payload; the status code loads them on demand to register fingerprints."""
    for path in _sources():
        for node in ast.parse(path.read_text(encoding="utf-8")).body:
            modules = [a.name for a in node.names] if isinstance(node, ast.Import) else \
                [node.module or ""] if isinstance(node, ast.ImportFrom) else []
            assert not any(m.split(".")[0] in ("w2_camera", "w2_hand") for m in modules), \
                f"{path.name} imports a device package at module level"


def test_every_module_has_a_dependency_rule():
    assert {path.stem for path in PACKAGE_DIR.glob("*.py") if path.stem != "__init__"} == set(ALLOWED)


@pytest.mark.parametrize("module", sorted(ALLOWED))
def test_modules_only_import_the_modules_below_them(module):
    for name in _imports(PACKAGE_DIR / f"{module}.py"):
        parts = name.split(".")
        if len(parts) > 1 and parts[0] == "w2_payload" and parts[1] in ALLOWED and parts[1] != module:
            assert parts[1] in ALLOWED[module], f"{module} imports {name}"
