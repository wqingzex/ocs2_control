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
BASE_TYPES = ("linkhou_s2", "linkhou_s2_v2")  # the mobile base
BODY_TYPES = ("juxie",)  # waist and head motors (CAN FD)
# Arm drivers the robot descriptions can build (xacro arg arm_driver); marvin is the only one with a robot model.
ARM_TYPES = ("marvin",)
MARVIN_FIRMWARES = (338, 343)  # Marvin SDK versions, must match the firmware of the arm controller
DEFAULT_ARM_IP = "192.168.1.190"
ARM_MODES = ("position", "joint_impedance")
DYN_LENGTH = 10  # load override: m, mx, my, mz, ixx, ixy, ixz, iyy, iyz, izz
SIDES = ("left", "right")
# Optional values default to None / "": unset means the robot description's own default applies.

DEFAULTS: Dict[str, Any] = {
    "robot": {
        "entry": "whole_body",  # which controller launch to start; the owner of the entries checks the value
        "hardware": "real",  # run mode: real | mock_components | isaac
        "variant": "",
        # Every part: what it is (type), then how it is connected (interface).
        "base": {"type": "", "joints_movable": None},
        "body": {"type": "juxie", "interface": {"can": "", "stats_period_s": None}},
        "arm": {"type": "marvin", "interface": {"ip": "", "firmware": None}},
        "end_effector": {
            "type": "linkerhand_l6",
            "interface": {
                "bus": "usb_rs485",
                "left": {"port": "", "can": "", "channel": None},
                "right": {"port": "", "can": "", "channel": None},
            },
            "dyn_param": {"left": [], "right": []},  # load override per side (10 numbers); [] = by end effector type
        },
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


# Sections the robot launch files read from the same file (the robot profile): checked by _check_control only.
PROFILE_SECTIONS = ("control", "setup")
CONTROL_KEYS = ("arm_mode", "limits", "patch")
LIMIT_KEYS = ("max_joint_speed", "max_joint_acceleration")


def _rename_fields(cfg: Dict[str, Any], renamed: Dict[str, Dict[str, str]]) -> Dict[str, Any]:
    cleaned = dict(cfg)
    for section, names in renamed.items():
        if isinstance(cleaned.get(section), dict):
            cleaned[section] = {names.get(k, k): v for k, v in cleaned[section].items()}
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


def check_end_effector(end_effector: str, bus: str, where_type: str = "robot.end_effector.type",
                       where_bus: str = "robot.end_effector.interface.bus") -> None:
    """Which end effector can be on which bus (also used by the setup window before it saves)."""
    where = f"'{where_bus}'"
    expect(end_effector in END_EFFECTORS,
           f"'{where_type}' must be one of {list(END_EFFECTORS)}, got {end_effector!r}")
    expect(bus in END_EFFECTOR_BUSES, f"{where} must be one of {list(END_EFFECTOR_BUSES)}, got {bus!r}")
    if end_effector == "none":
        return
    expect(not (end_effector == "xhand1" and bus != "usb_rs485"), f"{where}: xhand1 only supports usb_rs485")
    expect(not (bus == "usb_can" and end_effector != "linkerhand_l6"), f"{where}: usb_can is only for linkerhand_l6")
    expect((end_effector == "wuji_hand2") == (bus == "ethernet"),
           f"{where}: wuji_hand2 needs ethernet, and ethernet is only for wuji_hand2")
    expect(not (end_effector == "jodell_rg75" and bus != "arm_rs485"),
           f"{where}: jodell_rg75 is only wired for arm_rs485 (the RS485 port at the arm)")


def _is_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def _check_robot(robot: Dict[str, Any]) -> None:
    import ipaddress
    expect(robot["hardware"] in HARDWARE,
           f"'robot.hardware' must be one of {list(HARDWARE)}, got {robot['hardware']!r}")
    base, body, arm, eef = robot["base"], robot["body"], robot["arm"], robot["end_effector"]
    expect(base["type"] in ("", *BASE_TYPES), f"'robot.base.type' must be one of {list(BASE_TYPES)}, got {base['type']!r}")
    expect(base["joints_movable"] is None or isinstance(base["joints_movable"], bool),
           f"'robot.base.joints_movable' must be true/false, got {base['joints_movable']!r}")
    expect(body["type"] in BODY_TYPES, f"'robot.body.type' must be one of {list(BODY_TYPES)}, got {body['type']!r}")
    stats = body["interface"]["stats_period_s"]
    expect(stats is None or (_is_number(stats) and stats >= 0),
           f"'robot.body.interface.stats_period_s' must be a number >= 0, got {stats!r}")
    expect(arm["type"] in ARM_TYPES, f"'robot.arm.type' must be one of {list(ARM_TYPES)}, got {arm['type']!r}")
    ip = arm["interface"]["ip"]
    if ip:
        try:
            ipaddress.IPv4Address(ip)
        except ValueError as exc:
            raise ConfigError(f"'robot.arm.interface.ip' must be an IPv4 address, got {ip!r}") from exc
    firmware = arm["interface"]["firmware"]
    expect(firmware is None or firmware in MARVIN_FIRMWARES,
           f"'robot.arm.interface.firmware' must be one of {list(MARVIN_FIRMWARES)}, got {firmware!r}")
    check_end_effector(eef["type"], eef["interface"]["bus"])
    for side in SIDES:
        channel = eef["interface"][side]["channel"]
        expect(channel is None or channel in (1, 2, 3) and not isinstance(channel, bool),
               f"'robot.end_effector.interface.{side}.channel' must be 1, 2 or 3, got {channel!r}")
        load = eef["dyn_param"][side]
        expect(isinstance(load, list) and (not load or (len(load) == DYN_LENGTH and all(map(_is_number, load)))),
               f"'robot.end_effector.dyn_param.{side}' must be [] or {DYN_LENGTH} numbers, got {load!r}")


def _check_control(control: Any) -> None:
    expect(control is None or isinstance(control, dict), "'control' must be a mapping")
    control = control or {}
    for key in control:
        expect(key in CONTROL_KEYS, f"unknown field 'control.{key}' (one of {list(CONTROL_KEYS)})")
    expect(control.get("arm_mode", "position") in ARM_MODES,
           f"'control.arm_mode' must be one of {list(ARM_MODES)}, got {control.get('arm_mode')!r}")
    limits = control.get("limits") or {}
    expect(isinstance(limits, dict), "'control.limits' must be a mapping")
    for key, value in limits.items():
        expect(key in LIMIT_KEYS, f"unknown field 'control.limits.{key}' (one of {list(LIMIT_KEYS)})")
        expect(_is_number(value) and value > 0, f"'control.limits.{key}' must be a number > 0, got {value!r}")
    expect(control.get("patch") is None or isinstance(control["patch"], dict), "'control.patch' must be a mapping")


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
    cfg = _rename_fields(cfg, ext.renamed)
    for name in PROFILE_SECTIONS:
        expect(cfg.get(name) is None or isinstance(cfg[name], dict), f"'{name}' must be a mapping")
    owned = {**DEFAULTS, **ext.defaults}
    skipped = set(PROFILE_SECTIONS) if ext.defaults else set(PROFILE_SECTIONS) | (set(cfg) - set(owned))
    merged = _merge(owned, {k: v for k, v in cfg.items() if k not in skipped}, "")
    _check_type(merged, owned, "")
    _check_robot(merged["robot"])
    _check_control(cfg.get("control"))
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
