"""Load, validate and describe config/robot.local.yaml.

The template (templates/) is the schema: the file must have every key of the template for its robot.entry, no more, and good
values; nothing is filled in at load time. `./scripts/setup.sh config` writes a new file from the template and adds the keys
a file lacks. Other packages add checks of the sections they own through an Extension. A missing or unknown key or a bad
value raises ConfigError naming the field.

Three parts: the template (what keys exist and their values), the validation (ConfigError, load, validate), and the device
mappings (which fields make up the camera and hand mapping a setup step confirms; the fingerprint values must stay what
they were, test_mappings: a changed value makes every operator confirm again)."""
from __future__ import annotations

import copy
import io
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, Tuple

import yaml

# ---- the template ---------------------------------------------------------------------------------------

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


# ---- validation ------------------------------------------------------------------------------------------

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

class ConfigError(ValueError):
    """Invalid robot configuration (message names the offending field)."""


@dataclass(frozen=True)
class Extension:
    """Checks another package makes of the sections it owns (the values; which keys exist is the template's business)."""
    checks: Tuple[Callable[[Dict[str, Any]], None], ...] = ()

    def merge(self, *others: "Extension") -> "Extension":
        """This extension plus others: every package that owns a section contributes its own checks."""
        checks = list(self.checks)
        for other in others:
            checks.extend(other.checks)
        return Extension(tuple(checks))


# Sections the robot launch files read from the same file (the robot profile): checked by _check_control only.
PROFILE_SECTIONS = ("control", "setup")
CONTROL_KEYS = ("arm_mode", "limits", "patch")
LIMIT_KEYS = ("max_joint_speed", "max_joint_acceleration")


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


def _check_imaging(name: str, cam: Dict[str, Any]) -> None:
    """white_balance and ae_max_exposure are positive numbers; exposure and gain are both -1 (auto) or both fixed values."""
    for key in ("white_balance", "ae_max_exposure"):
        expect(_is_number(cam[key]) and cam[key] > 0, f"'cameras.{name}.{key}' must be a number > 0, got {cam[key]!r}")
    for key, floor in (("exposure", 1), ("gain", 0)):
        value = cam[key]
        expect(value == -1 or (isinstance(value, int) and not isinstance(value, bool) and value >= floor),
               f"'cameras.{name}.{key}' must be -1 (auto) or an integer >= {floor}, got {value!r}")
    expect((cam["exposure"] == -1) == (cam["gain"] == -1),
           f"'cameras.{name}.exposure' and 'gain' must both be -1 (auto exposure) or both be fixed values, "
           f"got exposure={cam['exposure']!r}, gain={cam['gain']!r}")


def _check_cameras(cams: Dict[str, Any]) -> None:
    expect(cams["fps"] in (30, 60), f"'cameras.fps' must be 30 or 60, got {cams['fps']!r}")
    for name in ROLES:
        cam = cams[name]
        expect(cam["rotation"] in ROTATIONS,
               f"'cameras.{name}.rotation' must be one of {list(ROTATIONS)}, got {cam['rotation']!r}")
        for key in ("serial", "usb_port"):
            expect(bool(SERIAL_RE.match(cam[key])),
                   f"'cameras.{name}.{key}' has invalid characters: {cam[key]!r}")
        _check_imaging(name, cam)
        if cams["enable"] and cam["enable"]:
            expect(cam["serial"] != "",
                   f"'cameras.{name}.serial' is empty: set it (ros2 run orbbec_camera list_devices_node) "
                   f"or set 'cameras.{name}.enable: false'")


def _check_shape(schema: Dict[str, Any], cfg: Dict[str, Any], path: str) -> None:
    """Every key of the schema is in cfg and cfg has no other key; scalars have the type of the template's value."""
    for key, node in schema.items():
        name = f"{path}.{key}" if path else key
        expect(key in cfg, f"missing field '{name}' (run ./scripts/setup.sh config to add it)")
        value = cfg[key]
        if name in FREEFORM:
            expect(isinstance(value, dict), f"'{name}' must be a mapping")
        elif isinstance(node, dict):
            expect(isinstance(value, dict), f"'{name}' must be a mapping")
            _check_shape(node, value, name)
    for key in cfg:
        expect(key in schema, f"unknown field '{path + '.' if path else ''}{key}'")


def validate(cfg: Dict[str, Any], ext: Extension | None = None) -> Dict[str, Any]:
    """Check a config against the template of its robot.entry; returns it without the control and setup sections."""
    expect(isinstance(cfg, dict), "config root must be a mapping")
    expect(isinstance(cfg.get("robot"), dict) and "entry" in cfg["robot"],
           "missing field 'robot.entry' (run ./scripts/setup.sh config to create the file)")
    entry = cfg["robot"]["entry"]
    expect(entry in entries(), f"'robot.entry' must be one of {list(entries())}, got {entry!r}")
    schema = template_for(entry)
    _check_shape(schema, cfg, "")
    checked = {key: value for key, value in cfg.items() if key not in PROFILE_SECTIONS}
    _check_type(checked, {key: value for key, value in schema.items() if key not in PROFILE_SECTIONS}, "")
    _check_robot(checked["robot"])
    _check_control(cfg["control"])
    _check_cameras(checked["cameras"])
    for check in (ext or Extension()).checks:
        check(checked)
    return copy.deepcopy(checked)


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
        raw["cameras"] = {**(raw.get("cameras") or {}), "enable": False}
        return validate(raw, ext)


# ---- device mappings (what a setup step confirms) --------------------------------------------------------

# The hand setup and its worker (end_effector_setup.discovery) speak of the wiring as usb / 485 / usb_can / ethernet; the config
# file says robot.end_effector.interface.bus. Converted at the file boundary only, which also keeps the fingerprint of an
# unchanged wiring the same as before the rename.
BUS_TO_DRIVE = {"usb_rs485": "usb", "arm_rs485": "485", "usb_can": "usb_can", "ethernet": "ethernet"}
DRIVE_TO_BUS = {drive: bus for bus, drive in BUS_TO_DRIVE.items()}
HAND_DRIVES = tuple(DRIVE_TO_BUS)


def _mapping(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def ee_type(robot: Dict[str, Any]) -> str:
    """robot.end_effector.type with its default."""
    return _mapping(robot.get("end_effector")).get("type") or "linkerhand_l6"


def ee_bus(robot: Dict[str, Any]) -> str:
    """robot.end_effector.interface.bus with its default."""
    return _mapping(_mapping(robot.get("end_effector")).get("interface")).get("bus") or "usb_rs485"


def ee_drive(robot: Dict[str, Any]) -> str:
    """The bus as the setup code names the wiring."""
    bus = ee_bus(robot)
    return BUS_TO_DRIVE.get(bus, bus)


def ee_side(robot: Dict[str, Any], side: str) -> Dict[str, Any]:
    """robot.end_effector.interface.<side>: port, can, channel (only what is written)."""
    return _mapping(_mapping(_mapping(robot.get("end_effector")).get("interface")).get(side))


def arm_ip(robot: Dict[str, Any]) -> str:
    """robot.arm.interface.ip with its default (the Marvin controller, also reached by the arm's RS485 port)."""
    return _mapping(_mapping(robot.get("arm")).get("interface")).get("ip") or DEFAULT_ARM_IP


def camera_mapping(raw: Dict[str, Any]) -> Dict[str, Any]:
    """The cameras section with the defaults filled in: a change to any camera field invalidates the confirmation."""
    value = section_defaults("cameras")
    for key, item in (raw.get("cameras") or {}).items():
        if isinstance(item, dict) and key in value:
            value[key].update(item)
        else:
            value[key] = item
    return value


def ee_mapping(raw: Dict[str, Any]) -> Dict[str, Any]:
    """The hand, its wiring and the ports or channels that wiring uses."""
    robot = raw.get("robot") or {}
    value = {"hand": ee_type(robot), "drive": ee_drive(robot)}
    if value["hand"] != "none":
        left, right = ee_side(robot, "left"), ee_side(robot, "right")
        if value["drive"] == "usb":
            keys = {"usb_left_port": left.get("port") or "", "usb_right_port": right.get("port") or ""}
        elif value["drive"] == "usb_can":
            keys = {"left_ee_can_interface": left.get("can") or "can1", "right_ee_can_interface": right.get("can") or "can2"}
        elif value["drive"] == "ethernet":
            keys = {}  # Wuji: the driver scans and matches each hand by hand_side
        else:
            keys = {"device_ip": arm_ip(robot), "left_ee_channel": left.get("channel") or 1,
                    "right_ee_channel": right.get("channel") or 1}
        value.update(keys)
    return value


# setup step -> the fields its confirmation covers
FIELDS = {"cameras": camera_mapping, "ee": ee_mapping}
