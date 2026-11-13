"""Confirmation receipts belong to the device mapping the operator reviewed."""
from __future__ import annotations

import copy
import hashlib
import json
from datetime import datetime, timezone

from ..config import DEFAULTS, normalize_hand_drive


def fingerprint(raw, section):
    if section == "cameras":
        value = copy.deepcopy(DEFAULTS["cameras"])
        for key, item in (raw.get("cameras") or {}).items():
            if isinstance(item, dict) and key in value:
                value[key].update(item)
            else:
                value[key] = item
    else:
        robot, hw = raw.get("robot") or {}, raw.get("hardware") or {}
        value = {"hand": robot.get("hand", "linkerhand_l6"),
                 "drive": normalize_hand_drive(robot.get("hand_drive", "usb"))}
        if value["hand"] != "none":
            keys = {"usb_left_port": "/dev/ttyUSB0", "usb_right_port": "/dev/ttyUSB2"} \
                if value["drive"] == "usb" else {"device_ip": "192.168.1.190", "left_ee_channel": 1, "right_ee_channel": 1}
            value.update({key: hw.get(key, default) for key, default in keys.items()})
    return hashlib.sha256(json.dumps(value, sort_keys=True, ensure_ascii=False).encode()).hexdigest()


def confirmed(raw, section):
    setup = raw.get('setup')
    receipt = setup.get(section) if isinstance(setup, dict) else None
    if not isinstance(receipt, dict):
        return False
    return receipt.get("fingerprint") == fingerprint(raw, section) and receipt.get("confirmed") is True


def receipt(raw, section):
    return {"confirmed": True, "fingerprint": fingerprint(raw, section),
            "confirmed_at": datetime.now(timezone.utc).isoformat(timespec="seconds")}
