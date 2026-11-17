"""robot_app reaches other workspace packages only through packages it declares in package.xml."""
import ast
import re
from pathlib import Path

import pytest

PACKAGE_DIR = Path(__file__).resolve().parents[1]
SRC_ROOT = PACKAGE_DIR.parents[1]
RETIRED_NAMES = ("w2_bringup", "w2_devices", "robot_bringup")


def _sources():
    return sorted(path for folder in ("robot_app", "launch") for path in (PACKAGE_DIR / folder).rglob("*.py"))


def _declared():
    text = (PACKAGE_DIR / "package.xml").read_text(encoding="utf-8")
    return set(re.findall(r"<(?:depend|exec_depend|build_depend)>([^<]+)<", text))


def _workspace_packages():
    """Names of the workspace packages at any depth (a directory holding package.xml); third_party is not ours."""
    manifests = (m for m in SRC_ROOT.rglob("package.xml") if "third_party" not in m.relative_to(SRC_ROOT).parts)
    return {manifest.parent.name for manifest in manifests} - {PACKAGE_DIR.name}


def _imported_roots(path):
    roots = set()
    for node in ast.walk(ast.parse(path.read_text(encoding="utf-8"))):
        if isinstance(node, ast.Import):
            roots |= {alias.name.split(".")[0] for alias in node.names}
        elif isinstance(node, ast.ImportFrom) and node.level == 0 and node.module:
            roots.add(node.module.split(".")[0])
    return roots


@pytest.mark.skipif(SRC_ROOT.name != "src", reason="needs the source tree layout")
def test_imported_workspace_packages_are_declared_in_package_xml():
    workspace = _workspace_packages()
    for path in _sources():
        undeclared = (_imported_roots(path) & workspace) - _declared()
        assert not undeclared, f"{path.name} imports {sorted(undeclared)} without declaring them in package.xml"


@pytest.mark.parametrize("name", RETIRED_NAMES)
def test_retired_package_names_are_gone(name):
    offenders = [path.name for path in _sources() if name in path.read_text(encoding="utf-8")]
    assert not offenders, offenders


def test_shared_code_is_not_re_exported_under_robot_app():
    shims = [p.name for p in (PACKAGE_DIR / "robot_app").glob("*.py")
             if "sys.modules[__name__]" in p.read_text(encoding="utf-8")]
    assert not shims, f"alias modules hide the dependency on robot_config: {shims}"
