"""Package boundaries: w2_payload knows no application package, and its layers only point downwards."""
import ast
from pathlib import Path

import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1] / "w2_payload"
APPLICATION_NAMES = ("fa_teleop", "arms_teleop", "teleop_command", "arm_controller", "robot_bringup", "launch_helpers", "robot_visualize", "robot_navigation")
# layer -> w2_payload layers it may import (config/__init__.py is the base; edit.py is the editing helper)
LAYER_RULES = {"runtime": set(), "config": set()}


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


def test_launch_files_do_not_mention_application_packages():
    launch_dir = PACKAGE_DIR.parent / "launch"
    text = "".join(path.read_text(encoding="utf-8") for path in launch_dir.glob("*.py"))
    assert not any(name in text for name in APPLICATION_NAMES)


@pytest.mark.parametrize("layer", sorted(LAYER_RULES))
def test_low_layers_do_not_import_higher_layers(layer):
    layers = {"runtime", "config", "devices", "setup"}
    forbidden = layers - {layer} - LAYER_RULES[layer]
    for path in (PACKAGE_DIR / layer).rglob("*.py"):
        if layer == "config" and path.name == "edit.py":
            continue  # the editing helper sits on top of the schema and may read devices and setup state
        for module in _imports(path):
            parts = module.split(".")
            assert not (len(parts) > 1 and parts[0] == "w2_payload" and parts[1] in forbidden), \
                f"{path.relative_to(PACKAGE_DIR)} imports {module}"
