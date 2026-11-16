"""check_layers.py on small workspaces built in a temporary directory: each rule fails when it is broken."""
import os
import subprocess
import sys
from pathlib import Path

SCRIPT = Path(__file__).resolve().parent / "check_layers.py"


def package(src: Path, group: str, name: str, deps=(), test_deps=(), files=None):
    directory = src / group / name
    directory.mkdir(parents=True)
    tags = "".join(f"<depend>{d}</depend>" for d in deps) + "".join(f"<test_depend>{d}</test_depend>" for d in test_deps)
    (directory / "package.xml").write_text(f"<package><name>{name}</name>{tags}</package>")
    for relative, text in (files or {}).items():
        target = directory / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(text)


def run(src: Path):
    result = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True,
                            env={**os.environ, "CHECK_LAYERS_SRC": str(src)})
    return result.returncode, result.stdout


def test_a_workspace_that_only_points_down_passes(tmp_path):
    package(tmp_path, "robot_utils", "robot_utils", files={"python/robot_runtime/__init__.py": ""})
    package(tmp_path, "robot_control", "ctl", deps=["robot_utils"])
    package(tmp_path, "robot_hmi", "app", deps=["ctl", "robot_utils"],
            files={"app/__init__.py": "", "app/main.py": "from robot_runtime import Unit\n"})
    code, out = run(tmp_path)
    assert code == 0 and "layering: OK" in out, out


def test_a_cycle_between_packages_fails(tmp_path):
    package(tmp_path, "robot_control", "a", deps=["b"])
    package(tmp_path, "robot_control", "b", deps=["a"])
    code, out = run(tmp_path)
    assert code == 1 and "package cycle: a -> b -> a" in out, out


def test_a_dependency_that_points_up_fails(tmp_path):
    package(tmp_path, "robot_control", "ctl", deps=["app"])
    package(tmp_path, "robot_hmi", "app")
    code, out = run(tmp_path)
    assert code == 1 and "ctl [robot_control] depends on app [robot_hmi]" in out, out


def test_a_cycle_between_directory_groups_fails_even_without_a_package_cycle(tmp_path):
    # launch -> hmi and hmi -> launch through different packages: no package cycle, but the groups form one.
    package(tmp_path, "robot_launch", "bringup", deps=["node"])
    package(tmp_path, "robot_hmi", "node")
    package(tmp_path, "robot_hmi", "app", deps=["bringup"])
    code, out = run(tmp_path)
    assert code == 1 and "cycle between directory groups" in out, out
    assert "bringup [robot_launch] depends on node [robot_hmi]" in out, out


def test_a_package_used_in_code_but_not_declared_fails(tmp_path):
    package(tmp_path, "robot_utils", "robot_utils", files={"python/robot_runtime/__init__.py": ""})
    package(tmp_path, "robot_hmi", "app", files={"app/__init__.py": "", "app/main.py": "import robot_runtime\n"})
    code, out = run(tmp_path)
    assert code == 1 and "app uses robot_utils without declaring it" in out, out


def test_a_cpp_include_of_an_undeclared_package_fails(tmp_path):
    package(tmp_path, "robot_utils", "robot_utils", files={"include/robot_log/flight.hpp": ""})
    package(tmp_path, "robot_control", "ctl", files={"src/a.cpp": '#include <robot_log/flight.hpp>\n'})
    code, out = run(tmp_path)
    assert code == 1 and "ctl uses robot_utils without declaring it" in out, out


def test_a_test_only_use_needs_test_depend_or_depend(tmp_path):
    package(tmp_path, "robot_utils", "robot_utils", files={"python/robot_runtime/__init__.py": ""})
    package(tmp_path, "robot_hmi", "app", test_deps=["robot_utils"],
            files={"app/__init__.py": "", "test/test_x.py": "from robot_runtime import Unit\n"})
    code, out = run(tmp_path)
    assert code == 0, out
    package(tmp_path, "robot_hmi", "app2", files={"app2/__init__.py": "", "test/test_x.py": "import robot_runtime\n"})
    code, out = run(tmp_path)
    assert code == 1 and "app2 uses robot_utils (tests) without declaring it" in out, out


def test_the_real_workspace_passes():
    result = subprocess.run([sys.executable, str(SCRIPT)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout
