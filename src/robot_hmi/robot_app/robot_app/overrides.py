"""Command line overrides: --set section.field=value changes the config for one run; the file stays as it is.

Device mappings (camera serials, ports, rotation, the XR headset id) and the robot profile sections are
edited by the setup steps or in the file, never from the command line."""
from __future__ import annotations

import copy
import re
from typing import Any, Dict, Iterable, List, Tuple

import yaml

from fa_w2_payload.config import PROFILE_SECTIONS, ConfigError

DENIED_FIELDS = re.compile(r"^(cameras\.(head|left_wrist|right_wrist)\.(serial|usb_port|rotation)|xr\.device_id)$")


def parse_assignment(text: str) -> Tuple[List[str], Any]:
    """'robot.hardware=isaac' -> (['robot', 'hardware'], 'isaac'); the value is read as YAML (true, 30, text)."""
    key, sep, value = text.partition("=")
    parts = key.strip().split(".")
    if not sep or len(parts) < 2 or not all(re.fullmatch(r"[A-Za-z0-9_]+", part) for part in parts):
        raise ConfigError(f"--set expects section.field=value, got '{text}'")
    if parts[0] in PROFILE_SECTIONS:
        raise ConfigError(f"--set cannot change '{key}': the robot profile is edited in the config file")
    if DENIED_FIELDS.match(".".join(parts)):
        raise ConfigError(f"--set cannot change '{key}': device mappings are set up with ./scripts/setup.sh")
    try:
        return parts, yaml.safe_load(value)
    except yaml.YAMLError as exc:
        raise ConfigError(f"--set '{text}': invalid value: {exc}") from exc


def apply_overrides(raw: Dict[str, Any], assignments: Iterable[str]) -> Dict[str, Any]:
    """A copy of the raw config mapping with the assignments applied in order."""
    result = copy.deepcopy(raw)
    for text in assignments:
        parts, value = parse_assignment(text)
        node = result
        for part in parts[:-1]:
            child = node.get(part)
            if child is None:
                child = node[part] = {}
            if not isinstance(child, dict):
                raise ConfigError(f"--set '{text}': '{part}' is not a section")
            node = child
        node[parts[-1]] = value
    return result
