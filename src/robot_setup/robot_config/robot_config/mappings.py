"""Which fields of config/robot.local.yaml make up a device mapping.

A setup step (cameras, ee) is confirmed for the mapping the operator reviewed: config_edit stores a fingerprint of
these fields, and the step asks again as soon as one of them changes. The fields are configuration knowledge, so they
live here with the config; the device packages (camera, end_effector) use them and the shared package never imports
them.

Fingerprint values must stay what they were: a changed value makes every operator confirm again (test_mappings)."""
from __future__ import annotations

import copy
from typing import Any, Dict

from .config import DEFAULT_ARM_IP
from .template import section_defaults

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
