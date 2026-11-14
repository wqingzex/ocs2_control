"""Is the XR bridge really receiving the headset? `up xr` used to print "已启动" as soon as the process existed, while
the bridge may still be waiting: for the PC Service, or for a headset that is not connected to it.

The bridge log tells: "PC Service 已连接" once the SDK reaches the service, "接收头显追踪数据" at the first frame."""
from __future__ import annotations

import re
from typing import List, NamedTuple

READY, FAILED, PENDING = "ready", "failed", "pending"  # same values as teleop_app.control_ready

SERVICE_CONNECTED = "SDK 事件: PC Service 已连接"
HEADSET_SELECTED = re.compile(r"头显选择：(\S+)")
HEADSET_DATA = re.compile(r"接收头显追踪数据，设备 ID: (\S+)")
ERROR_LINE = re.compile(r"\[XR-00\d\][^\n]*")
MAX_ERRORS = 2


class Verdict(NamedTuple):
    state: str          # READY | FAILED | PENDING
    reasons: List[str]  # READY: the headset id; PENDING: what it waits for; FAILED: why the bridge is gone


def judge(log_text: str, alive: bool) -> Verdict:
    """READY at the first headset frame; FAILED when the bridge exited; otherwise PENDING with what is missing."""
    data = HEADSET_DATA.search(log_text)
    if data and alive:
        return Verdict(READY, [data.group(1)])
    errors = list(dict.fromkeys(m.group(0).strip() for m in ERROR_LINE.finditer(log_text)))[-MAX_ERRORS:]
    if not alive:
        return Verdict(FAILED, ["XR 桥进程已退出"] + errors)
    selected = HEADSET_SELECTED.search(log_text)
    wanted = selected.group(1) if selected else "头显"
    if SERVICE_CONNECTED not in log_text:
        return Verdict(PENDING, ["桥还没有连上 PC Service（127.0.0.1:60061）"] + errors)
    return Verdict(PENDING, [f"桥已连上 PC Service，但没有收到头显发来的追踪数据（等待 {wanted}）"])
