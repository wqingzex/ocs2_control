"""./scripts/setup.sh config: write robot.local.yaml from the template of an entry, or add the keys it lacks.

    python3 -m robot_config.sync --config C [--entry E] [--yes] [--status]

Nothing is filled in when the file is loaded, so a file made before a key existed has to get the key here. Values and
comments already in the file stay; the new keys come from the template (the same entry the file names)."""
from __future__ import annotations

import argparse
import copy
import os
import shutil
import sys
from typing import Any, List, Optional, Tuple

from . import config_edit
from .device_setup import Io
from .config import FREEFORM, entries, template_document, template_text

DEFAULT_ENTRY = "whole_body"


def _paths(node: Any, prefix: str = "") -> List[str]:
    """Every key path of a config; control.patch and setup are free-form and end the path."""
    found: List[str] = []
    for key, value in node.items():
        path = f"{prefix}.{key}" if prefix else key
        if isinstance(value, dict) and path not in FREEFORM:
            found.extend(_paths(value, path))
        else:
            found.append(path)
    return found


def unknown_keys(local: Any, template: Any) -> List[str]:
    """Key paths of the file that the template does not have."""
    return sorted(set(_paths(local)) - set(_paths(template)))


def missing_keys(local: Any, template: Any) -> List[str]:
    """Key paths of the template that the file lacks."""
    present = set(_paths(local))
    return [path for path in _paths(template) if path not in present and not _has_parent(present, path)]


def _has_parent(present: set, path: str) -> bool:
    """A leaf of the template that sits below a leaf of the file (e.g. a list or a free-form mapping) is not missing."""
    parts = path.split(".")
    return any(".".join(parts[:n]) in present for n in range(1, len(parts)))


def add_missing(local: Any, template: Any, prefix: str = "") -> List[str]:
    """Add the template's keys the mapping lacks (values from the template); returns the paths added."""
    added: List[str] = []
    for key, value in template.items():
        path = f"{prefix}.{key}" if prefix else key
        if key not in local:
            local[key] = copy.deepcopy(value)
            added.append(path)
        elif isinstance(value, dict) and isinstance(local[key], dict) and path not in FREEFORM:
            added.extend(add_missing(local[key], value, path))
    return added


def _entry_of(raw: Any, requested: Optional[str], io: Io) -> str:
    entry = requested or ((raw.get("robot") or {}).get("entry") if raw else None)
    if not entry:
        entry = io.ask(f"控制入口 {' | '.join(entries())}", DEFAULT_ENTRY)
    if entry not in entries():
        raise ValueError(f"robot.entry 只能是 {list(entries())}，不是 {entry!r}")
    return entry


def status(path: str) -> Tuple[bool, str]:
    """(needs work, reason) without writing anything."""
    if not os.path.isfile(path):
        return True, "还没有本机配置"
    _, raw = config_edit.load_rt(path)
    entry = (raw.get("robot") or {}).get("entry")
    if entry not in entries():
        return True, f"robot.entry 缺失或不对（{entry!r}）"
    template = template_document(entry)
    missing, unknown = missing_keys(raw, template), unknown_keys(raw, template)
    if missing:
        return True, f"缺 {len(missing)} 个键（如 {missing[0]}）"
    if unknown:
        return True, f"有 {len(unknown)} 个多余的键（如 {unknown[0]}）"
    return False, f"配置完整（入口 {entry}）"


def run(path: str, requested_entry: Optional[str], io: Io) -> bool:
    if not os.path.isfile(path):
        entry = _entry_of(None, requested_entry, io)
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as handle:
            handle.write(template_text(entry))
        io.info(f"已按入口 {entry} 从模板生成 {path}；[必改] 的项请核对，设备相关的项由 ./scripts/setup.sh 后面的步骤写入")
        return True
    yaml_rt, raw = config_edit.load_rt(path)
    entry = _entry_of(raw, requested_entry, io)
    if (raw.get("robot") or {}).get("entry") not in (None, entry):
        io.error(f"配置里的 robot.entry 是 {raw['robot']['entry']!r}，不是 {entry!r}：改入口要手动改配置里的值，别的键按新入口的模板核对")
        return False
    template = template_document(entry)
    if "robot" not in raw:
        raw["robot"] = {}
    raw["robot"].setdefault("entry", entry)
    added = add_missing(raw, template)
    unknown = unknown_keys(raw, template)
    if added:
        shutil.copy2(path, path + ".bak")
        config_edit.save_rt(path, yaml_rt, raw)
        io.info(f"已补上 {len(added)} 个缺的键（原文件备份为 {path}.bak）：")
        for item in added:
            io.info(f"  + {item}")
    else:
        io.info("配置完整，没有要补的键")
    if unknown:
        io.error("下面这些键不在模板里，程序不会读它们，加载时会报错；删掉或改成模板里的写法：")
        for item in unknown:
            io.error(f"  - {item}")
        return False
    return True


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="setup.sh config")
    parser.add_argument("--config", required=True)
    parser.add_argument("--entry", choices=entries(), default=None)
    parser.add_argument("--yes", action="store_true")
    parser.add_argument("--status", action="store_true", help="only print whether the file needs work (exit 1 if so)")
    args = parser.parse_args(argv)
    if args.status:
        needed, reason = status(args.config)
        print(f"{'需配置' if needed else '已配置'}：{reason}")
        return 1 if needed else 0
    io = Io(assume_yes=args.yes)
    try:
        ok = run(args.config, args.entry, io)
    except KeyboardInterrupt:
        io.info("已取消")
        return 130
    except ValueError as exc:
        io.error(str(exc))
        ok = False
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
