"""robot_runtime and robot_log_tools stay independent of each other, the modules inside each only reach the ones below
them, and the names retired by renames do not come back."""
import ast
import re
from pathlib import Path

import pytest

PYTHON_DIR = Path(__file__).resolve().parents[1] / "python"
REPO_ROOT = Path(__file__).resolve().parents[3]

# module -> the modules of the same package it may import ("__init__" is the package itself, e.g. config_path)
ALLOWED = {
    "robot_runtime": {
        "__init__": set(),
        "units": set(),
        "processes": set(),
        "supervisor": {"units", "processes"},
    },
    "robot_log_tools": {
        "__init__": set(),
        "logger": {"error_code"},
        "error_code": {"__init__"},
        "blackbox": {"__init__"},
        "diagnose": {"blackbox", "error_code"},
    },
}


def _imports(path, package):
    """(modules of `package` imported, other workspace packages imported) by one source file, nested imports included."""
    siblings, others = set(), set()
    modules = {p.stem for p in (PYTHON_DIR / package).glob("*.py")}
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.ImportFrom):
            root = (node.module or "").split(".")[0]
            if node.level or root == package:
                parts = (node.module or "").split(".")
                tail = parts if node.level else parts[1:]
                if tail and tail[0]:
                    siblings.add(tail[0])
                else:
                    siblings.update(a.name if a.name in modules else "__init__" for a in node.names)
            elif root in ALLOWED:
                others.add(root)
        elif isinstance(node, ast.Import):
            for alias in node.names:
                parts = alias.name.split(".")
                if parts[0] == package:
                    siblings.add(parts[1] if len(parts) > 1 else "__init__")
                elif parts[0] in ALLOWED:
                    others.add(parts[0])
    return siblings, others


@pytest.mark.parametrize("package", sorted(ALLOWED))
def test_the_modules_of_a_package_are_the_ones_we_list(package):
    on_disk = {p.stem for p in (PYTHON_DIR / package).glob("*.py")}
    assert on_disk == set(ALLOWED[package]), f"{package}: update ALLOWED in test_boundaries.py"


@pytest.mark.parametrize("package", sorted(ALLOWED))
def test_a_module_imports_only_what_it_is_allowed_to(package):
    for module, allowed in ALLOWED[package].items():
        siblings, others = _imports(PYTHON_DIR / package / f"{module}.py", package)
        assert not others, f"{package}/{module}.py reaches into another package: {sorted(others)}"
        assert siblings - {module} <= allowed, (
            f"{package}/{module}.py imports {sorted(siblings - {module} - allowed)}, allowed: {sorted(allowed)}")


RETIRED = (
    r"robot_log_tools\.(flight|analyze|evidence|fault_codes|events|log)\b",
    r"robot_log_tools/(flight|analyze|evidence|fault_codes|events|log)\.py",
    r"robot_runtime\.events\b",
    r"FlightRecorder", r"robot_log/flight", r"robot_log::flight\b", r"ROBOT_FLIGHT_DIR", r"robot_flight\b",
    r"flight_types", r"kFlight[A-Z]", r"Flight(Header|Slot|Record|Error)\b", r"RFLT0001",
)
SUFFIXES = {".py", ".cpp", ".hpp", ".h", ".yaml", ".yml", ".md", ".txt", ".xml", ".sh", ".xacro"}
SKIP_DIRS = {"__pycache__", ".pytest_cache", "third_party", "install", "build", "log", ".git"}


def test_retired_names_do_not_come_back():
    if not (REPO_ROOT / "src").is_dir():
        pytest.skip("not a source checkout")
    pattern = re.compile("|".join(RETIRED))
    found = []
    for root in (REPO_ROOT / "src", REPO_ROOT / "docs", REPO_ROOT / "scripts"):
        for path in root.rglob("*") if root.is_dir() else []:
            if path.suffix not in SUFFIXES or SKIP_DIRS & set(path.relative_to(REPO_ROOT).parts) or path == Path(__file__):
                continue
            for number, line in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
                if pattern.search(line):
                    found.append(f"{path.relative_to(REPO_ROOT)}:{number}: {line.strip()[:90]}")
    assert not found, "retired names (renamed to logger / error_code / blackbox / diagnose):\n" + "\n".join(found[:40])
