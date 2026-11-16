"""Load and validate the shared part of config/robot.local.yaml (sections robot and cameras).

Other packages add the sections they own through an Extension. Unknown keys and bad values
raise ConfigError naming the field."""
from __future__ import annotations

import copy
import re
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, Tuple

import yaml

ROTATIONS = (0, 90, 180, 270)
ROLES = ("head", "left_wrist", "right_wrist")  # camera positions
SERIAL_RE = re.compile(r"^[A-Za-z0-9._:-]*$")
END_EFFECTORS = ("none", "linkerhand_l6", "xhand1", "wuji_hand2", "jodell_rg75")  # hands and the gripper
# usb_rs485: USB-RS485 adapter on the PC; arm_rs485: the RS485 port at the arm's flange; usb_can: USB-CAN (L6);
# ethernet: Wuji Hand 2
END_EFFECTOR_BUSES = ("usb_rs485", "arm_rs485", "usb_can", "ethernet")
HARDWARE = ("real", "mock_components", "isaac")

DEFAULTS: Dict[str, Any] = {
    "robot": {
        "entry": "whole_body",  # which controller launch to start; the owner of the entries checks the value
        "end_effector": "linkerhand_l6",
        "end_effector_bus": "usb_rs485",
        "hardware": "real",
    },
    "cameras": {
        "enable": True,
        "fps": 30,
        "white_balance": 5600,
        "ae_max_exposure": 120,
        "wrist_exposure": 50,
        "wrist_gain": 16,
        "head": {"enable": True, "serial": "", "usb_port": "", "rotation": 0},
        "left_wrist": {"enable": True, "serial": "", "usb_port": "", "rotation": 0},
        "right_wrist": {"enable": True, "serial": "", "usb_port": "", "rotation": 0},
    },
}


class ConfigError(ValueError):
    """Invalid robot configuration (message names the offending field)."""


@dataclass(frozen=True)
class Extension:
    """Sections another package owns: their defaults, renamed fields and checks of the merged config."""
    defaults: Dict[str, Any] = field(default_factory=dict)
    renamed: Dict[str, Dict[str, str]] = field(default_factory=dict)
    checks: Tuple[Callable[[Dict[str, Any]], None], ...] = ()

    def merge(self, *others: "Extension") -> "Extension":
        """This extension plus others: every package that owns a section contributes its own."""
        defaults, renamed, checks = dict(self.defaults), {k: dict(v) for k, v in self.renamed.items()}, list(self.checks)
        for other in others:
            overlap = set(defaults) & set(other.defaults)
            if overlap:
                raise ValueError(f"config sections defined twice: {sorted(overlap)}")
            defaults.update(other.defaults)
            for section, names in other.renamed.items():
                renamed.setdefault(section, {}).update(names)
            checks.extend(other.checks)
        return Extension(defaults, renamed, tuple(checks))


def _merge(defaults: Dict[str, Any], user: Dict[str, Any], path: str) -> Dict[str, Any]:
    merged = copy.deepcopy(defaults)
    for key, value in user.items():
        name = f"{path}.{key}" if path else key
        if key not in defaults:
            raise ConfigError(f"unknown field '{name}'")
        if isinstance(defaults[key], dict):
            if not isinstance(value, dict):
                raise ConfigError(f"'{name}' must be a mapping")
            merged[key] = _merge(defaults[key], value, name)
        else:
            merged[key] = value
    return merged


# Fields older configs may still carry; accepted and ignored.
# robot.rviz: RViz is now its own on-demand unit (start.sh up rviz / console), not part of control.
RETIRED = {"robot": ("rviz", "profile")}  # profile: the config file itself is the robot profile now
# Sections the robot launch files read from the same file (the robot profile): kept as is.
PROFILE_SECTIONS = ("platform", "defaults", "hardware", "control", "setup")


# Fields that were renamed without keeping the old name: an old config fails with what to write instead.
MOVED = {
    "robot": {
        "hand": "'robot.hand' is now 'robot.end_effector' (same values, plus jodell_rg75)",
        "hand_drive": "'robot.hand_drive' is now 'robot.end_effector_bus': usb -> usb_rs485, \"485\" -> arm_rs485, "
                      "usb_can and ethernet unchanged",
    },
}


def _check_moved(cfg: Dict[str, Any]) -> None:
    for section, fields in MOVED.items():
        if isinstance(cfg.get(section), dict):
            for key, hint in fields.items():
                expect(key not in cfg[section], hint)


def _drop_retired(cfg: Dict[str, Any], renamed: Dict[str, Dict[str, str]]) -> Dict[str, Any]:
    cleaned = dict(cfg)
    for section, names in renamed.items():
        if isinstance(cleaned.get(section), dict):
            cleaned[section] = {names.get(k, k): v for k, v in cleaned[section].items()}
    for section, keys in RETIRED.items():
        if isinstance(cleaned.get(section), dict):
            cleaned[section] = {k: v for k, v in cleaned[section].items() if k not in keys}
    return cleaned


def expect(cond: bool, message: str) -> None:
    if not cond:
        raise ConfigError(message)


def _check_type(cfg: Dict[str, Any], defaults: Dict[str, Any], path: str) -> None:
    for key, default in defaults.items():
        name = f"{path}.{key}" if path else key
        value = cfg[key]
        if isinstance(default, dict):
            _check_type(value, default, name)
        elif isinstance(default, bool):
            expect(isinstance(value, bool), f"'{name}' must be true/false, got {value!r}")
        elif isinstance(default, (int, float)):
            expect(isinstance(value, (int, float)) and not isinstance(value, bool),
                   f"'{name}' must be a number, got {value!r}")
        elif isinstance(default, str):
            expect(isinstance(value, str), f"'{name}' must be a string, got {value!r}")


def check_end_effector(end_effector: str, bus: str, field_prefix: str = "robot.") -> None:
    """Which end effector can be on which bus (also used by the setup window before it saves)."""
    where = f"'{field_prefix}end_effector_bus'"
    expect(end_effector in END_EFFECTORS,
           f"'{field_prefix}end_effector' must be one of {list(END_EFFECTORS)}, got {end_effector!r}")
    expect(bus in END_EFFECTOR_BUSES, f"{where} must be one of {list(END_EFFECTOR_BUSES)}, got {bus!r}")
    if end_effector == "none":
        return
    expect(not (end_effector == "xhand1" and bus != "usb_rs485"), f"{where}: xhand1 only supports usb_rs485")
    expect(not (bus == "usb_can" and end_effector != "linkerhand_l6"), f"{where}: usb_can is only for linkerhand_l6")
    expect((end_effector == "wuji_hand2") == (bus == "ethernet"),
           f"{where}: wuji_hand2 needs ethernet, and ethernet is only for wuji_hand2")
    expect(not (end_effector == "jodell_rg75" and bus != "arm_rs485"),
           f"{where}: jodell_rg75 is only wired for arm_rs485 (the RS485 port at the arm)")


def _check_robot(robot: Dict[str, Any]) -> None:
    expect(robot["hardware"] in HARDWARE,
           f"'robot.hardware' must be one of {list(HARDWARE)}, got {robot['hardware']!r}")
    check_end_effector(robot["end_effector"], robot["end_effector_bus"])


def _check_cameras(cams: Dict[str, Any]) -> None:
    expect(cams["fps"] in (30, 60), f"'cameras.fps' must be 30 or 60, got {cams['fps']!r}")
    for name in ROLES:
        cam = cams[name]
        expect(cam["rotation"] in ROTATIONS,
               f"'cameras.{name}.rotation' must be one of {list(ROTATIONS)}, got {cam['rotation']!r}")
        for key in ("serial", "usb_port"):
            expect(bool(SERIAL_RE.match(cam[key])),
                   f"'cameras.{name}.{key}' has invalid characters: {cam[key]!r}")
        if cams["enable"] and cam["enable"]:
            expect(cam["serial"] != "",
                   f"'cameras.{name}.serial' is empty: set it (ros2 run orbbec_camera list_devices_node) "
                   f"or set 'cameras.{name}.enable: false'")


def validate(cfg: Dict[str, Any], ext: Extension | None = None) -> Dict[str, Any]:
    """Merge with defaults and validate; returns the complete config.

    Without ext only the sections of this package are checked and returned; the sections of other
    packages are ignored. With ext every top-level section must be known."""
    cfg = {} if cfg is None else cfg
    expect(isinstance(cfg, dict), "config root must be a mapping")
    ext = ext or Extension()
    _check_moved(cfg)
    cfg = _drop_retired(cfg, ext.renamed)
    for name in PROFILE_SECTIONS:
        expect(cfg.get(name) is None or isinstance(cfg[name], dict), f"'{name}' must be a mapping")
    owned = {**DEFAULTS, **ext.defaults}
    skipped = set(PROFILE_SECTIONS) if ext.defaults else set(PROFILE_SECTIONS) | (set(cfg) - set(owned))
    merged = _merge(owned, {k: v for k, v in cfg.items() if k not in skipped}, "")
    _check_type(merged, owned, "")
    _check_robot(merged["robot"])
    _check_cameras(merged["cameras"])
    for check in ext.checks:
        check(merged)
    return merged


def validate_camera_assignments(assignments: Dict[str, Dict[str, Any]]) -> None:
    """Check role -> {enable, serial, usb_port, rotation} before it is written to the config."""
    for role, values in assignments.items():
        where = f"cameras.{role}"
        if "enable" in values:
            expect(isinstance(values["enable"], bool), f"'{where}.enable' must be true/false")
        if "rotation" in values:
            rotation = values["rotation"]
            expect(not isinstance(rotation, bool) and rotation in ROTATIONS,
                   f"'{where}.rotation' must be one of {list(ROTATIONS)}, got {rotation!r}")
        for key in ("serial", "usb_port"):
            if key in values and values[key] is not None:
                expect(isinstance(values[key], str) and bool(SERIAL_RE.match(values[key])),
                       f"'{where}.{key}' has invalid characters: {values[key]!r}")


Edit = Callable[[Dict[str, Any]], Dict[str, Any]]  # changes the raw YAML mapping before it is validated


def read_yaml(path: str) -> Any:
    """Parse a YAML file; a missing or broken file raises ConfigError."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return yaml.safe_load(handle)
    except FileNotFoundError as exc:
        raise ConfigError(f"config file not found: {path}") from exc
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {path}: {exc}") from exc


def load(path: str, ext: Extension | None = None, edit: Edit | None = None) -> Dict[str, Any]:
    """Read and validate a robot YAML file (edit: applied to the raw mapping first, e.g. command line overrides)."""
    raw = read_yaml(path)
    if edit is not None:
        raw = edit(dict(raw or {}))
    return validate(raw, ext)


def load_lenient(path: str, ext: Extension | None = None, edit: Edit | None = None) -> Dict[str, Any]:
    """Like load(), but cameras are disabled when they are what fails (serials not set up yet)."""
    try:
        return load(path, ext, edit)
    except ConfigError:
        raw = dict(read_yaml(path) or {})
        if edit is not None:
            raw = edit(raw)
        raw["cameras"] = {"enable": False}
        return validate(raw, ext)
