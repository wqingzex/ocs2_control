#!/usr/bin/env python3
"""Workspace layering check: package cycles, dependencies that point up, and packages used but not declared.

    python3 scripts/check_layers.py        (exit 0: fine, 1: a rule is broken; only the standard library is used)

Rules (src/, third_party excluded; test_depend does not count as a dependency):
  1. No cycle between packages, and no cycle between the directory groups (robot_control, robot_launch, ...).
  2. A group depends only on its own group or on a lower layer: LAYERS below, lowest first.
  3. What the code uses (Python import, C++ #include of a workspace package) is declared in package.xml. Without
     it the build only works through some other package's dependencies, and a hidden cycle can sit behind it.
Known exceptions are listed in KNOWN_UNDECLARED with the reason; a new one fails the check.
"""
import os
import re
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

SRC = Path(os.environ.get("CHECK_LAYERS_SRC") or Path(__file__).resolve().parents[1] / "src")  # env: for the tests
SKIP_DIRS = {"third_party", "build", "install", "__pycache__", ".git", ".pytest_cache"}
# lowest layer first; a group depends only on its own group or on a lower layer
LAYERS = [["robot_utils"], ["robot_hardware", "robot_description"], ["robot_control"], ["robot_launch", "robot_payload"],
          ["robot_hmi"]]
LAYER_OF = {group: number for number, groups in enumerate(LAYERS) for group in groups}
# (user, used): why it is allowed. Empty: add a line with the reason before letting a package use another without
# declaring it (a declaration it cannot make without a cycle is a design problem, not an exception).
KNOWN_UNDECLARED = {}
RUNTIME_TAGS = ("depend", "build_depend", "buildtool_depend", "build_export_depend", "exec_depend")
IMPORT = re.compile(r"^\s*(?:from|import)\s+([A-Za-z_]\w*)((?:\s*,\s*[A-Za-z_][\w.]*)*)", re.M)
INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([A-Za-z_]\w*)/', re.M)


def load_packages():
    packages = {}
    for directory, subdirs, files in os.walk(SRC):
        subdirs[:] = [d for d in subdirs if d not in SKIP_DIRS]
        if "package.xml" in files:
            root = ET.parse(os.path.join(directory, "package.xml")).getroot()
            relative = Path(directory).relative_to(SRC)
            packages[root.findtext("name")] = {
                "dir": Path(directory), "group": relative.parts[0],
                "runtime": {e.text for tag in RUNTIME_TAGS for e in root.findall(tag)},
                "test": {e.text for e in root.findall("test_depend")}}
    return packages


def find_cycle(graph):
    """One cycle of the directed graph as a list of nodes, or None."""
    state, path = {}, []

    def visit(node):
        state[node] = 1
        path.append(node)
        for nxt in sorted(graph.get(node, ())):
            if state.get(nxt) == 1:
                return path[path.index(nxt):] + [nxt]
            if nxt not in state:
                cycle = visit(nxt)
                if cycle:
                    return cycle
        path.pop()
        state[node] = 2
        return None

    for start in sorted(graph):
        if start not in state:
            cycle = visit(start)
            if cycle:
                return cycle
    return None


def python_modules_and_include_namespaces(packages):
    modules, namespaces = {}, {}
    for name, info in packages.items():
        for base in (info["dir"], info["dir"] / "python"):
            for sub in base.iterdir() if base.is_dir() else ():
                if (sub / "__init__.py").is_file():
                    modules[sub.name] = name
        include = info["dir"] / "include"
        for sub in include.iterdir() if include.is_dir() else ():
            namespaces[sub.name] = name
    return modules, namespaces


def used_packages(packages):
    """package -> {(other package, is_test): first file}: what the code of each package actually uses."""
    modules, namespaces = python_modules_and_include_namespaces(packages)
    used = defaultdict(dict)
    for name, info in packages.items():
        for directory, subdirs, files in os.walk(info["dir"]):
            subdirs[:] = [d for d in subdirs if d not in SKIP_DIRS and not Path(directory, d, "package.xml").is_file()]
            for file in files:
                if not file.endswith((".py", ".h", ".hpp", ".cpp")):
                    continue
                path = Path(directory, file)
                text = path.read_text(encoding="utf-8", errors="ignore")
                is_test = "test" in path.relative_to(info["dir"]).parts or file.startswith("test_") or file == "conftest.py"
                if file.endswith(".py"):
                    names = []
                    for first, rest in IMPORT.findall(text):
                        names += [modules.get(first)]
                        names += [modules.get(m.strip().split(".")[0]) for m in rest.split(",") if m.strip()]
                else:
                    names = [namespaces.get(m) for m in INCLUDE.findall(text)]
                for other in filter(None, names):
                    if other != name:
                        used[name].setdefault((other, is_test), str(path.relative_to(SRC)))
    return used


def main():
    packages = load_packages()
    problems = []

    edges = {n: {d for d in p["runtime"] if d in packages and d != n} for n, p in packages.items()}
    cycle = find_cycle(edges)
    if cycle:
        problems.append("package cycle: " + " -> ".join(cycle))

    group_edges = defaultdict(set)
    for name, deps in edges.items():
        for dep in deps:
            if packages[name]["group"] != packages[dep]["group"]:
                group_edges[packages[name]["group"]].add(packages[dep]["group"])
    cycle = find_cycle(group_edges)
    if cycle:
        problems.append("cycle between directory groups: " + " -> ".join(cycle))

    for name, deps in sorted(edges.items()):
        for dep in sorted(deps):
            user, used = packages[name]["group"], packages[dep]["group"]
            if user != used and LAYER_OF.get(user, 99) <= LAYER_OF.get(used, 99):
                problems.append(f"{name} [{user}] depends on {dep} [{used}], which is not a lower layer")

    for name, uses in sorted(used_packages(packages).items()):
        for (other, is_test), where in sorted(uses.items()):
            declared = other in packages[name]["runtime"] or (is_test and other in packages[name]["test"])
            if not declared and (name, other) not in KNOWN_UNDECLARED:
                problems.append(f"{name} uses {other}{' (tests)' if is_test else ''} without declaring it "
                                f"in package.xml (e.g. {where})")

    print(f"{len(packages)} packages, {sum(len(d) for d in edges.values())} dependencies")
    for problem in problems:
        print("  FAIL", problem)
    print("layering: OK" if not problems else f"layering: {len(problems)} problem(s)")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
