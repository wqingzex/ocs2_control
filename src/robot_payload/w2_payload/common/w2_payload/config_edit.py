"""Edit config/robot.local.yaml in place keeping comments (ruamel.yaml round-trip), and the confirmation receipts.

Used by setup.sh and the web console; writes are atomic (temp file + rename). A receipt belongs to the device
mapping the operator reviewed: when the mapping changes, the receipt no longer matches and the step is asked again."""
from __future__ import annotations

import hashlib
import json
import os
import tempfile
from datetime import datetime, timezone
from typing import Any, Callable, Dict, Optional

from .config import ROLES, validate_camera_assignments


def _yaml():
    try:
        from ruamel.yaml import YAML
    except ImportError as exc:
        raise RuntimeError("缺少 ruamel.yaml：sudo apt install python3-ruamel.yaml（或 rosdep install）") from exc
    yaml = YAML()  # round-trip: comments, key order and flow style are preserved
    yaml.preserve_quotes = True
    yaml.width = 4096
    return yaml


def load_rt(path: str):
    yaml = _yaml()
    with open(path, encoding="utf-8") as handle:
        data = yaml.load(handle)
    return yaml, ({} if data is None else data)


def save_rt(path: str, yaml, data) -> None:
    directory = os.path.dirname(os.path.abspath(path))
    fd, tmp = tempfile.mkstemp(prefix=".tmp_", suffix=".yaml", dir=directory)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as handle:
            yaml.dump(data, handle)
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def _section(data, key: str):
    from ruamel.yaml.comments import CommentedMap
    if key not in data or data[key] is None:
        data[key] = CommentedMap()
    return data[key]


def set_cameras(path: str, assignments: Dict[str, Dict[str, Any]], *, enabled: Optional[bool] = None) -> None:
    """assignments: role -> {serial, usb_port?, rotation?, enable?}; other fields are left untouched."""
    unknown = set(assignments) - set(ROLES)
    if unknown:
        raise ValueError(f"未知相机角色: {sorted(unknown)}")
    validate_camera_assignments(assignments)  # never write values start.sh would then refuse
    yaml, data = load_rt(path)
    cameras = _section(data, "cameras")
    if enabled is not None:
        cameras["enable"] = enabled
    for role, values in assignments.items():
        entry = _section(cameras, role)
        for key in ("enable", "serial", "usb_port", "rotation"):
            if key in values and values[key] is not None:
                entry[key] = values[key]
    save_rt(path, yaml, data)


def set_fields(path: str, section: str, values: Dict[str, Any]) -> None:
    """Set scalar fields of one top-level section (e.g. robot.hand, robot.hand_drive)."""
    yaml, data = load_rt(path)
    target = _section(data, section)
    for key, value in values.items():
        target[key] = value
    save_rt(path, yaml, data)


def set_float_list(path: str, section: str, key: str, values) -> None:
    """A list of numbers in flow style ([a, b, c]), the way such lists are written by hand."""
    from ruamel.yaml.comments import CommentedSeq
    yaml, data = load_rt(path)
    seq = CommentedSeq([float(v) for v in values])
    seq.fa.set_flow_style()
    _section(data, section)[key] = seq
    save_rt(path, yaml, data)


def set_usb_ports(profile_path: str, left: Optional[str], right: Optional[str]) -> None:
    """hardware.usb_left_port / usb_right_port (serial ports of USB hands)."""
    values = {}
    if left:
        values["usb_left_port"] = left
    if right:
        values["usb_right_port"] = right
    if values:
        set_fields(profile_path, "hardware", values)


def camera_serials(path: str) -> Dict[str, str]:
    """role -> serial currently written in the config ('' if unset)."""
    _, data = load_rt(path)
    cameras = data.get("cameras") or {}
    return {role: str((cameras.get(role) or {}).get("serial", "") or "") for role in ROLES}


def save_setup(path: str, section: str, changes: Dict[str, Dict], original: str) -> None:
    """Commit one reviewed step, preserving comments and rejecting concurrent edits."""
    import fcntl
    with open(path + ".setup.lock", "a", encoding="utf-8") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        with open(path, encoding="utf-8") as handle:
            if handle.read() != original:
                raise ValueError("配置文件已被其他程序修改，请取消本次配置并重新进入")
        yaml, data = load_rt(path)
        for name, values in changes.items():
            target = _section(data, name)
            for key, value in values.items():
                if isinstance(value, dict):
                    _section(target, key).update(value)
                else:
                    target[key] = value
        _section(data, "setup")[section] = receipt(data, section)
        save_rt(path, yaml, data)


# ---- confirmation receipts ----
#
# A receipt belongs to the device mapping the operator reviewed: it stores a fingerprint of the fields that
# mapping consists of. Each device package says which fields those are (register_fingerprint); a step is
# confirmed only while its fingerprint still matches, so a changed serial, port or rotation asks again.

FINGERPRINTS: Dict[str, Callable[[Dict[str, Any]], Any]] = {}  # section -> fields that make up its mapping


def register_fingerprint(section: str, fields: Callable[[Dict[str, Any]], Any]) -> None:
    FINGERPRINTS[section] = fields


def fingerprint(raw, section):
    """Hash of the mapping fields of a section. The device package of the section must have been imported."""
    if section not in FINGERPRINTS:
        raise KeyError(f"no fingerprint registered for '{section}': import its device package first (w2_camera, w2_hand)")
    value = FINGERPRINTS[section](raw)
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
