"""The robot.local.yaml template: one complete base file plus a small overlay for every entry.

The template is the schema. Every key in it must be in the config file (nothing is filled in at load time), and a key that
is not in it is an error. `./scripts/setup.sh config` writes a new file from it and adds the keys a file lacks.
The values are what the robot descriptions use when nothing is written; a null means "not set"."""
from __future__ import annotations

import copy
import io
from pathlib import Path
from typing import Any, Dict, Tuple

import yaml

TEMPLATE_DIR = Path(__file__).resolve().parent / "templates"
BASE = TEMPLATE_DIR / "base.yaml"
# Contents that are not part of the schema: ros2_control overrides written by hand, and the confirmation records.
FREEFORM = ("control.patch", "setup")


def entries() -> Tuple[str, ...]:
    """The robot.entry values: one overlay file per entry."""
    return tuple(sorted(path.stem for path in TEMPLATE_DIR.glob("*.yaml") if path != BASE))


def _overlay_path(entry: str) -> Path:
    if entry not in entries():
        raise ValueError(f"no template for robot.entry {entry!r} (one of {list(entries())})")
    return TEMPLATE_DIR / f"{entry}.yaml"


def _merge(target: Dict[str, Any], overlay: Dict[str, Any]) -> None:
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _merge(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def template_for(entry: str) -> Dict[str, Any]:
    """The complete config of an entry with every key at its template value."""
    with open(BASE, encoding="utf-8") as handle:
        schema = yaml.safe_load(handle)
    with open(_overlay_path(entry), encoding="utf-8") as handle:
        _merge(schema, yaml.safe_load(handle) or {})
    return schema


def section_defaults(section: str) -> Dict[str, Any]:
    """One section's template values; the same for every entry (cameras, xr, ...)."""
    return template_for(entries()[0])[section]


def _yaml():
    from ruamel.yaml import YAML
    yaml_rt = YAML()
    yaml_rt.preserve_quotes = True
    yaml_rt.width = 4096
    return yaml_rt


def template_document(entry: str):
    """The template of an entry as a round-trip document (comments kept), for writing and for adding missing keys."""
    yaml_rt = _yaml()
    with open(BASE, encoding="utf-8") as handle:
        document = yaml_rt.load(handle)
    with open(_overlay_path(entry), encoding="utf-8") as handle:
        _merge(document, yaml_rt.load(handle) or {})
    return document


def template_text(entry: str) -> str:
    """The text of a new robot.local.yaml for an entry."""
    buffer = io.StringIO()
    _yaml().dump(template_document(entry), buffer)
    return buffer.getvalue()
