"""Fault code registry: loading, checking and rendering config/fault_codes.yaml.

    python3 -m robot_log_tools.error_code --markdown > docs/故障码.md      (the document is generated, not written)
    python3 -m robot_log_tools.error_code --check [--repo-root DIR]        (exit 1 and list the problems)

The registry says, for every fault worth finding in the logs, which class it is (stop or keep monitoring), what the
system does today and what to check first. Each entry points at the log text in the source that produces it, so a
reworded message cannot leave the registry stale unnoticed.
"""
from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional

import yaml

from robot_log_tools import config_path

CLASSES = ("S1", "S2", "M")
SEGMENTS = {"XR": "头显和 PC Service", "TEL": "遥操作指令", "CTL": "控制器", "HW": "硬件", "SYS": "系统"}
CLASS_TITLES = {"S1": "S1 硬件级停机", "S2": "S2 控制级停机", "M": "M 监控"}
CODE_RE = re.compile(r"^(" + "|".join(SEGMENTS) + r")-\d{3}$")
REQUIRED = ("code", "class", "title", "behavior", "check")
KNOWN = set(REQUIRED) | {"source", "external", "escalation", "untagged"}


class FaultRegistryError(ValueError):
    """The registry file is malformed."""


@dataclass(frozen=True)
class Fault:
    code: str
    fault_class: str
    title: str
    behavior: str
    check: str
    source_file: Optional[str] = None
    source_match: Optional[str] = None
    external: Optional[str] = None
    escalation: Optional[str] = None
    untagged: Optional[str] = None   # why the log line does not carry the [CODE] prefix

    @property
    def segment(self) -> str:
        return self.code.split("-")[0]

    @property
    def is_stop(self) -> bool:
        return self.fault_class in ("S1", "S2")


def default_path() -> str:
    """config/fault_codes.yaml of robot_log."""
    return config_path("fault_codes.yaml")


def parse(entries: object) -> List[Fault]:
    """Entries as read from the YAML list; raises FaultRegistryError naming the entry that is wrong."""
    if not isinstance(entries, list):
        raise FaultRegistryError("fault_codes.yaml must be a list of entries")
    faults: List[Fault] = []
    for index, raw in enumerate(entries):
        where = f"entry {index + 1}" + (f" ({raw.get('code')})" if isinstance(raw, dict) and raw.get("code") else "")
        if not isinstance(raw, dict):
            raise FaultRegistryError(f"{where}: must be a mapping")
        for key in REQUIRED:
            if not isinstance(raw.get(key), str) or not raw[key].strip():
                raise FaultRegistryError(f"{where}: '{key}' is missing or empty")
        unknown = set(raw) - KNOWN
        if unknown:
            raise FaultRegistryError(f"{where}: unknown field(s) {sorted(unknown)}")
        if not CODE_RE.match(raw["code"]):
            raise FaultRegistryError(f"{where}: code must look like XR-001 (segments: {', '.join(SEGMENTS)})")
        if raw["class"] not in CLASSES:
            raise FaultRegistryError(f"{where}: class must be one of {CLASSES}, got {raw['class']!r}")
        source = raw.get("source")
        if source is not None:
            if not isinstance(source, dict) or set(source) != {"file", "match"} or \
                    not all(isinstance(value, str) and value for value in source.values()):
                raise FaultRegistryError(f"{where}: source needs a file and a match text")
        elif not isinstance(raw.get("external"), str) or not raw["external"].strip():
            raise FaultRegistryError(f"{where}: an entry without a source must say where its log comes from (external)")
        faults.append(Fault(
            code=raw["code"], fault_class=raw["class"], title=raw["title"].strip(), behavior=raw["behavior"].strip(),
            check=raw["check"].strip(), source_file=source["file"] if source else None,
            source_match=source["match"] if source else None, external=raw.get("external"),
            escalation=raw.get("escalation"), untagged=raw.get("untagged")))
    codes = [fault.code for fault in faults]
    duplicated = sorted({code for code in codes if codes.count(code) > 1})
    if duplicated:
        raise FaultRegistryError(f"duplicate code(s): {duplicated}")
    return faults


def load(path: Optional[str] = None) -> List[Fault]:
    with open(path or default_path(), encoding="utf-8") as handle:
        return parse(yaml.safe_load(handle))


def by_code(faults: List[Fault]) -> Dict[str, Fault]:
    return {fault.code: fault for fault in faults}


def check_sources(faults: List[Fault], repo_root: str) -> List[str]:
    """Problems with the source anchors: a file that is gone, or a log text that is no longer in it."""
    problems = []
    for fault in faults:
        if fault.source_file is None:
            continue
        path = os.path.join(repo_root, fault.source_file)
        try:
            with open(path, encoding="utf-8", errors="replace") as handle:
                text = handle.read()
        except OSError:
            problems.append(f"{fault.code}: source file not found: {fault.source_file}")
            continue
        if fault.source_match not in text:
            problems.append(f"{fault.code}: {fault.source_file} no longer contains {fault.source_match!r}")
        elif fault.untagged is None and f"[{fault.code}]" not in text:
            problems.append(f"{fault.code}: {fault.source_file} does not tag its log line with [{fault.code}]")
    return problems


def render_markdown(faults: List[Fault]) -> str:
    stops = [fault for fault in faults if fault.is_stop]
    lines = [
        "# 故障码",
        "",
        "<!-- 由 config/fault_codes.yaml 生成：python3 -m robot_log_tools.error_code --markdown > docs/故障码.md。"
        "不要直接改这个文件。 -->",
        "",
        "真机上遇到异常时，先用日志里的原文找到对应的故障码，再看“先查什么”。故障分两类：",
        "",
        "| 类别 | 含义 | 系统怎么处理 |",
        "|---|---|---|",
        "| **S1 硬件级停机** | 机器人已经停了，而且是断电、抱闸这一级 | 立即留证，操作员确认后重新使能 |",
        "| **S2 控制级停机** | 控制器切到 HOLD；或者关键组件起不来，机器人不会动 | 立即留证，操作员确认后恢复 |",
        "| **M 监控** | 机器人还在运行，但有劣化迹象 | 计数、看趋势，超阈值才留证；持续恶化时才升级 |",
        "",
        f"现在登记了 {len(faults)} 个故障码，其中停机类 {len(stops)} 个、监控类 {len(faults) - len(stops)} 个。"
        "“系统现在怎么做”写的是代码里实际发生的事；“升级候选”是还没有实现的想法，要逐条确认后才会改行为。",
        "",
    ]
    for segment, title in SEGMENTS.items():
        group = [fault for fault in faults if fault.segment == segment]
        if not group:
            continue
        lines += [f"## {segment}：{title}", ""]
        for fault in group:
            lines += [f"### {fault.code}　{fault.title}", "",
                      f"- **类别**：{CLASS_TITLES[fault.fault_class]}",
                      f"- **系统现在怎么做**：{fault.behavior}",
                      f"- **先查什么**：{fault.check}"]
            if fault.escalation:
                lines.append(f"- **升级候选（未实现）**：{fault.escalation}")
            if fault.source_file:
                tag = f"日志行以 `[{fault.code}]` 开头" if fault.untagged is None else f"没有编码前缀（{fault.untagged}）"
                lines.append(f"- **日志出处**：`{fault.source_file}`，原文含 `{fault.source_match}`；{tag}")
            if fault.external:
                lines.append(f"- **说明**：{fault.external}")
            lines.append("")
    return "\n".join(lines).rstrip("\n") + "\n"


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--file", help="registry file (default: config/fault_codes.yaml)")
    parser.add_argument("--markdown", action="store_true", help="print docs/故障码.md")
    parser.add_argument("--check", action="store_true", help="check that every source anchor still exists")
    parser.add_argument("--repo-root", default=os.getcwd(), help="repository root for --check (default: cwd)")
    args = parser.parse_args(argv)
    try:
        faults = load(args.file)
    except (FaultRegistryError, OSError, yaml.YAMLError) as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    if args.check:
        problems = check_sources(faults, args.repo_root)
        for problem in problems:
            print(f"[ERROR] {problem}", file=sys.stderr)
        if problems:
            return 1
        print(f"{len(faults)} fault codes, all source anchors found")
        return 0
    if args.markdown:
        sys.stdout.write(render_markdown(faults))
        return 0
    parser.print_help()
    return 0


if __name__ == "__main__":
    sys.exit(main())
