"""Which fields of config/robot.local.yaml make up a device mapping.

A setup step (cameras, hand) is confirmed for the mapping the operator reviewed: config_edit stores a fingerprint of
these fields, and the step asks again as soon as one of them changes. The fields are configuration knowledge, so they
live here with the config; the device packages (camera, end_effector) use them and the shared package never imports
them.

Fingerprint values must stay what they were: a changed value makes every operator confirm again (test_mappings)."""
from __future__ import annotations

import copy
from typing import Any, Dict

from .config import DEFAULTS

# The hand setup and its worker (end_effector.hand) speak of the wiring as usb / 485 / usb_can / ethernet; the config
# file says robot.end_effector_bus. Converted at the file boundary only, which also keeps the fingerprint of an
# unchanged wiring the same as before the rename.
BUS_TO_DRIVE = {"usb_rs485": "usb", "arm_rs485": "485", "usb_can": "usb_can", "ethernet": "ethernet"}
DRIVE_TO_BUS = {drive: bus for bus, drive in BUS_TO_DRIVE.items()}
HAND_DRIVES = tuple(DRIVE_TO_BUS)


def hand_name(robot: Dict[str, Any]) -> str:
    """robot.end_effector with its default."""
    return robot.get("end_effector", "linkerhand_l6")


def hand_drive(robot: Dict[str, Any]) -> str:
    """robot.end_effector_bus (with its default) as the setup code names the wiring."""
    bus = robot.get("end_effector_bus", "usb_rs485")
    return BUS_TO_DRIVE.get(bus, bus)


def camera_mapping(raw: Dict[str, Any]) -> Dict[str, Any]:
    """The cameras section with the defaults filled in: a change to any camera field invalidates the confirmation."""
    value = copy.deepcopy(DEFAULTS["cameras"])
    for key, item in (raw.get("cameras") or {}).items():
        if isinstance(item, dict) and key in value:
            value[key].update(item)
        else:
            value[key] = item
    return value


def hand_mapping(raw: Dict[str, Any]) -> Dict[str, Any]:
    """The hand, its wiring and the ports or channels that wiring uses."""
    robot, hardware = raw.get("robot") or {}, raw.get("hardware") or {}
    value = {"hand": hand_name(robot), "drive": hand_drive(robot)}
    if value["hand"] != "none":
        if value["drive"] == "usb":
            keys = {"usb_left_port": "", "usb_right_port": ""}
        elif value["drive"] == "usb_can":
            keys = {"left_ee_can_interface": "can1", "right_ee_can_interface": "can2"}
        elif value["drive"] == "ethernet":
            keys = {}  # Wuji: the driver scans and matches each hand by hand_side
        else:
            keys = {"device_ip": "192.168.1.190", "left_ee_channel": 1, "right_ee_channel": 1}
        value.update({key: hardware.get(key, default) for key, default in keys.items()})
    return value


# setup step -> the fields its confirmation covers
FIELDS = {"cameras": camera_mapping, "hand": hand_mapping}
