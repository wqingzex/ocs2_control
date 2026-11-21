"""Live status of what an operator watches on the console: controller, PC Service, headset, cameras.

A process that is alive is not a part that works (the controller launch outlives a controller node that aborted),
so the controller and the headset are judged from their logs and from what ROS reports, not from the process alone."""
from __future__ import annotations

import os
from typing import Any, Callable, Dict, List, Optional, Tuple

from launch_helpers.devices import xr as xr_ready

from ..control_ready import FAILED, READY, judge as judge_control

OK, WARN, BAD, OFF = "ok", "warn", "bad", "off"
LOG_HEAD_BYTES = 256 * 1024  # the spawners report in the first lines ...
LOG_TAIL_BYTES = 128 * 1024  # ... and a node that dies later shows at the end
CONTROL_STARTING_S = 120.0   # the real robot needs about 10 s; hands that do not answer make it much longer
MAX_DETAIL = 2

Item = Dict[str, Any]


def item(key: str, label: str, level: str, text: str, detail: str = "", unit: Optional[str] = None) -> Item:
    """unit: the console unit behind the part (the page offers start / restart for it); None for the FSM."""
    return {"key": key, "label": label, "level": level, "text": text, "detail": detail, "unit": unit}


class LogReader:
    """Head and tail of a unit's log, re-read only when the file changed (the state is polled every second)."""

    def __init__(self) -> None:
        self._cache: Dict[str, Tuple[Tuple[int, float], str]] = {}

    def __call__(self, path: Optional[str]) -> str:
        if not path:
            return ""
        try:
            stat = os.stat(path)
        except OSError:
            return ""
        stamp = (stat.st_size, stat.st_mtime)
        cached = self._cache.get(path)
        if cached and cached[0] == stamp:
            return cached[1]
        with open(path, "rb") as handle:
            head = handle.read(LOG_HEAD_BYTES)
            tail = b""
            if stat.st_size > LOG_HEAD_BYTES:
                handle.seek(max(LOG_HEAD_BYTES, stat.st_size - LOG_TAIL_BYTES))
                tail = handle.read()
        text = (head + b"\n" + tail).decode("utf-8", errors="replace")
        self._cache[path] = (stamp, text)
        return text


def _control(unit: Optional[Item], fsm: Optional[int], read_log: Callable[[Optional[str]], str],
             inactive: Optional[Dict[str, str]]) -> Item:
    if unit is None:
        return item("control", "运控", OFF, "未运行")
    if not unit["running"]:
        return item("control", "运控", OFF, "未运行", unit="control")
    verdict = judge_control(read_log(unit.get("log")), alive=True)
    if verdict.state == FAILED:
        return item("control", "运控", BAD, "异常", "\n".join(verdict.reasons[:MAX_DETAIL]), unit="control")
    if inactive == {} and fsm is not None:
        return item("control", "运控", OK, "就绪", unit="control")
    uptime = unit.get("uptime_s") or 0.0
    if inactive:
        detail = "，".join(f"{name}（{state}）" for name, state in inactive.items())
        if verdict.state == READY or uptime >= CONTROL_STARTING_S:
            return item("control", "运控", BAD, "控制器未激活", detail, unit="control")
        return item("control", "运控", WARN, "启动中", detail, unit="control")
    if inactive == {}:
        return item("control", "运控", WARN, "已激活，等待状态", "控制器已激活，还没有收到 /fsm_state", unit="control")
    if verdict.state == READY:
        return item("control", "运控", WARN, "状态未确认", "未收到控制器管理器的实时状态", unit="control")
    if uptime < CONTROL_STARTING_S:
        return item("control", "运控", WARN, "启动中", f"已启动 {uptime:.0f} 秒，等待控制器全部激活", unit="control")
    return item("control", "运控", BAD, "没有响应",
                f"启动 {uptime:.0f} 秒仍有控制器没有激活，也没有 /fsm_state：看运控日志", unit="control")


def _headset(unit: Optional[Item], connected: Optional[bool], read_log: Callable[[Optional[str]], str]) -> Item:
    if unit is None:
        return item("xr", "XR", OFF, "未运行")
    if not unit["running"]:
        return item("xr", "XR", OFF, "未运行", unit="xr")
    if connected:
        return item("xr", "XR", OK, "头显已连接", unit="xr")
    verdict = xr_ready.judge(read_log(unit.get("log")), alive=True)
    detail = "\n".join(verdict.reasons[:MAX_DETAIL]) or "没有收到头显发来的追踪数据"
    return item("xr", "XR", WARN, "等待头显", detail, unit="xr")


def _camera(unit: Item) -> Item:
    short = unit["title"].replace("相机", "").strip() or unit["name"]
    if not unit["running"]:
        return item(unit["name"], short, OFF, "未运行", unit=unit["name"])
    rates = [hz for hz in (unit.get("rates") or {}).values() if hz is not None]
    best = max(rates, default=0.0)
    if best > 0:
        return item(unit["name"], short, OK, f"{best:.0f} Hz", unit=unit["name"])
    return item(unit["name"], short, WARN, "无画面", "进程在运行但没有收到画面；腕部相机启动后要等约 50 秒", unit=unit["name"])


def summary(units: List[Item], fsm: Optional[int], xr_connected: Optional[bool],
            read_log: Callable[[Optional[str]], str], *, inactive: Optional[Dict[str, str]] = None) -> List[Item]:
    by_name = {u["name"]: u for u in units}
    control = _control(by_name.get("control"), fsm, read_log, inactive)
    pc = by_name.get("pc_service")
    rows = [
        control,
        item("pc_service", "PC Service", OK if pc and pc["running"] else OFF, "运行" if pc and pc["running"] else "未运行",
             unit="pc_service" if pc else None),
        _headset(by_name.get("xr"), xr_connected, read_log),
    ]
    rows.extend(_camera(u) for u in units if u.get("is_camera"))
    return rows
