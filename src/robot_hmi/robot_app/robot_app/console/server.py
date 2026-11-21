"""Web console backend (FastAPI): units, control mode, logs, camera setup and preview.

There is no login: whoever can reach the port can control the robot, so the console listens on this computer only
unless it is started with --host. Closing the console leaves units running."""
from __future__ import annotations

from dataclasses import replace
import os
import re
import threading
import time
from typing import Any, Callable, Dict, List, Optional, Tuple

from fastapi import Body, FastAPI, HTTPException
from fastapi.responses import FileResponse, StreamingResponse

from robot_config import config_file
from camera_setup import discovery as devices
from . import health
from ..diag import make_bundle
from ..config import ConfigError, load, load_lenient
from ..events import session_info
from ..robot import MOTION_CONTROLLERS
from ..ros_probe import BODY_COMMANDS, FSM_ACTIONS, FSM_COMMANDS, FSM_HOLD, SWITCH_POSE_PULSE_S, body_options, fsm_name
from robot_runtime.supervisor import Supervisor, UnitError
from ..units import build_units

DIAG_NAME = re.compile(r"^diag_\d{8}_\d{6}\.tar\.gz$")
PREVIEW_HZ = 10.0
PREVIEW_IDLE_S = 5.0  # end a preview stream after this long without frames; the page reconnects


class Console:
    """Application state; one instance per server."""

    def __init__(self, config_path: str, ws_root: str, probe_factory: Optional[Callable[[], Any]] = None,
                 scanner: Callable[[], List[devices.Device]] = devices.scan):
        self.config_path = os.path.abspath(config_path)
        self.ws_root = ws_root
        self.scanner = scanner
        self.access_urls: List[str] = []  # set by the entrypoint: what other computers can open
        self.lock = threading.RLock()  # serialises every state-changing action
        self.activity: Optional[Dict[str, Any]] = None  # what a running start / stop / restart is doing now
        self.inference_requested = False
        self.read_log = health.LogReader()
        self.supervisor: Optional[Supervisor] = None
        self.config_error: Optional[str] = None
        self.probe_error: Optional[str] = None
        self.probe = None
        if probe_factory is not None:
            try:
                self.probe = probe_factory()
            except Exception as exc:  # the console still manages processes without ROS introspection
                self.probe_error = f"ROS 探针不可用：{exc}"
        self.reload()

    # ---- config / units --------------------------------------------------------------------------
    def reload(self) -> None:
        with self.lock:
            try:
                cfg, error = load(self.config_path), None
            except ConfigError as exc:
                # Camera serials are often unset before setup: keep everything else usable.
                cfg, error = load_lenient(self.config_path), str(exc)
            units = build_units(cfg, self.config_path, self.ws_root)
            self.motion_controllers = MOTION_CONTROLLERS[cfg["robot"]["entry"]]
            self.required_controllers = (self.motion_controllers[0], "joint_state_broadcaster")
            if self.supervisor is None:
                self.supervisor = Supervisor(units, os.path.join(self.ws_root, "run"), os.path.join(self.ws_root, "logs"))
                self.supervisor.events.record("session", source="console", **session_info(self.ws_root, cfg))
            else:
                self.supervisor.units = units
            self.config_error = error
            if self.probe is not None:
                for unit in units.units.values():
                    if unit.group == "camera":
                        self.probe.watch_image(unit.topics[0])

    # ---- state -----------------------------------------------------------------------------------------
    def _inactive_controllers(self) -> Optional[Dict[str, str]]:
        states = self.probe.controller_states() if self.probe is not None else None
        if states is None:
            return None
        return {name: states.get(name, "未加载") for name in self.required_controllers if states.get(name) != "active"}

    def state(self) -> Dict[str, Any]:
        sup = self.supervisor
        if self.probe is not None:
            self.probe.watch([t for u in sup.units.units.values() for t in u.topics])
        units = []
        for status in sup.status_all():
            unit = sup.units[status.name]
            item = status.to_dict()
            item["rates"] = {t: (self.probe.rate(t) if self.probe else None) for t in unit.topics}
            item["is_camera"] = unit.group == "camera"
            units.append(item)
        running = {item["name"] for item in units if item["running"]}
        inactive = self._inactive_controllers()
        control_active = "control" in running and inactive == {}
        fsm = getattr(self.probe, "fsm_state", None) if control_active else None
        xr_connected = getattr(self.probe, "xr_connected", None)
        return {
            "units": units,
            "summary": health.summary(units, fsm, xr_connected, self.read_log, inactive=inactive),
            "activity": self.activity,
            "inference_active": self.inference_requested and set(self.inference_names()).issubset(running),
            "fsm": fsm,
            "fsm_name": fsm_name(fsm),
            # mode buttons the page may offer (none while the controllers are down)
            "fsm_actions": list(FSM_ACTIONS.get(fsm, ())),
            "xr_connected": xr_connected,
            "body": self.body_state(control_active),
            "access_urls": self.access_urls,
            "config_path": self.config_path,
            "config_error": self.config_error,
            "probe_error": self.probe_error,
            "time": time.time(),
        }

    # ---- actions -------------------------------------------------------------------------------------
    def _check_control_stop(self, names: List[str], confirm: bool) -> None:
        """Running motion controllers outside HOLD need confirmation, then go to HOLD before stopping."""
        sup = self.supervisor
        if "control" not in sup.units.stop_order(names) or not sup.is_running("control"):
            return
        states = self.probe.controller_states() if self.probe is not None else None
        # An emergency stop can deactivate motion controllers while the probe still has an old FSM value.
        if states is not None and all(states.get(name) in {"inactive", "unconfigured", "finalized"}
                                      for name in self.motion_controllers):
            return
        state = getattr(self.probe, "fsm_state", None)
        if state in (None, FSM_HOLD):
            return
        if not confirm:
            raise HTTPException(409, {"need_confirm": True, "fsm": fsm_name(state),
                                      "message": f"当前 FSM 为 {fsm_name(state)}，确认先切到 HOLD 再停止运控？"})
        if not self.probe.request_hold(5.0):
            raise HTTPException(409, {"hold_failed": True, "fsm": fsm_name(self.probe.fsm_state),
                                      "message": "5 秒内未回到 HOLD，未停止运控"})

    def fsm_command(self, action: str) -> Dict[str, Any]:
        """Switch the control mode like the RViz FSM panel does (only transitions the panel offers)."""
        if action not in FSM_COMMANDS:
            raise HTTPException(404, {"message": f"未知控制模式 '{action}'，可选: {', '.join(FSM_COMMANDS)}"})
        with self.lock:
            sup = self.supervisor
            if "control" not in sup.units or not sup.is_running("control"):
                raise HTTPException(400, {"message": "运控未运行"})
            if self.probe is None:
                raise HTTPException(503, {"message": "ROS 探针不可用，无法切换模式"})
            if self._inactive_controllers() != {}:
                raise HTTPException(409, {"message": "控制器未激活或实时状态不可用，无法切换模式"})
            state = self.probe.fsm_state
            allowed = FSM_ACTIONS.get(state, ())
            if action not in allowed:
                choices = "、".join(allowed) or "无（等待 FSM 状态）"
                raise HTTPException(409, {"message": f"当前 FSM 为 {fsm_name(state)}，不能切到 {action}（可选：{choices}）"})
            self.probe.send_fsm(FSM_COMMANDS[action])
            if action == "SWITCH_POSE":
                time.sleep(SWITCH_POSE_PULSE_S)
                self.probe.send_fsm(0)
            sup.events.record("fsm_command", action=action, **{"from": fsm_name(state)})
            return {"sent": action, "from": fsm_name(state)}

    def body_state(self, control_active: bool) -> Dict[str, Any]:
        """Body modes (自由/竖直/跟随…) the controller offers and the current one, like the RViz panel's combo box."""
        options = body_options(getattr(self.probe, "whole_body_capability", None)) if control_active else []
        return {"options": options, "current": getattr(self.probe, "body_state", None) if options else None}

    def body_command(self, action: str) -> Dict[str, Any]:
        if action not in BODY_COMMANDS:
            raise HTTPException(404, {"message": f"未知机体模式 '{action}'，可选: {', '.join(BODY_COMMANDS)}"})
        with self.lock:
            if self.probe is None:
                raise HTTPException(503, {"message": "ROS 探针不可用，无法切换机体模式"})
            control_active = self.supervisor.is_running("control") and self._inactive_controllers() == {}
            if action not in {o["action"] for o in self.body_state(control_active)["options"]}:
                raise HTTPException(409, {"message": f"当前控制器不支持机体模式 {action}（或运控未运行）"})
            self.probe.send_body_mode(action)
            self.supervisor.events.record("body_mode", action=action)
            return {"sent": action}

    def make_diag(self) -> str:
        """Pack logs, event timeline and config for a developer; built on demand, not under the console lock."""
        self.supervisor.events.record("diag", source="console")
        return make_bundle(self.ws_root, self.config_path)

    def _sequence(self, label: str, steps: List[Tuple[str, Callable[[], Any]]]) -> List[Any]:
        """Run the steps one after another and publish which one is running (state()["activity"]), so the page
        can show the progress of a start or stop that takes seconds."""
        results = []
        try:
            for index, (text, run_step) in enumerate(steps, start=1):
                self.activity = {"action": label, "step": text, "index": index, "total": len(steps),
                                 "since": time.time()}
                results.append(run_step())
        finally:
            self.activity = None
        return results

    def _start_steps(self, names: Optional[List[str]]) -> List[Tuple[str, Callable[[], Any]]]:
        sup = self.supervisor
        order = sup.units.launch_order(names)
        if "xr" in order or "pc_service" in order:
            # Pairing can change the headset and the LAN interface while the console is running.
            try:
                cfg = load_lenient(self.config_path)
            except (ConfigError, OSError) as exc:
                raise UnitError(f"无法读取最新 XR 配置：{exc}") from exc
            updated_units = build_units(cfg, self.config_path, self.ws_root).units
            if "xr" in order and "xr" not in updated_units:
                raise UnitError("本机配置未启用 XR")
            if "xr" in order:
                sup.units.units["xr"] = replace(sup.units["xr"], cmd=updated_units["xr"].cmd)
            if "pc_service" in order and "pc_service" in updated_units:
                sup.units.units["pc_service"] = replace(sup.units["pc_service"], driver=updated_units["pc_service"].driver)
        return [(f"启动{sup.units[n].title}", lambda n=n: sup.start(n).to_dict()) for n in order]

    def _stop_steps(self, names: Optional[List[str]]) -> List[Tuple[str, Callable[[], Any]]]:
        sup = self.supervisor
        return [(f"停止{sup.units[n].title}", lambda n=n: sup.stop(n).to_dict()) for n in sup.units.stop_order(names)]

    def inference_names(self) -> List[str]:
        """What inference needs from this robot: the controllers and the cameras (the policy itself runs elsewhere).
        No XR, no PC Service, no RViz."""
        units = self.supervisor.units
        return ["control"] + [n for n in units.names() if units[n].group == "camera"]

    def start(self, names: Optional[List[str]], *, inference: bool = False) -> List[Dict]:
        with self.lock:
            result = self._sequence("启动", self._start_steps(names))
            if inference or names is None:
                self.inference_requested = inference
            return result

    def stop(self, names: Optional[List[str]], confirm: bool) -> List[Dict]:
        with self.lock:
            self._check_control_stop(names or self.supervisor.units.names(), confirm)
            if self.inference_requested and set(self.inference_names()).intersection(self.supervisor.units.stop_order(names)):
                self.inference_requested = False
            return self._sequence("停止", self._stop_steps(names))

    def restart(self, name: str, confirm: bool) -> Dict:
        with self.lock:
            sup = self.supervisor
            self._check_control_stop([name], confirm)
            # Stopping a unit also stops what depends on it (RViz, the gamepad): bring back what was running.
            again = [n for n in sup.units.stop_order([name]) if n == name or sup.is_running(n)]
            steps = self._stop_steps([name]) + self._start_steps(again)
            self._sequence("重启", steps)
            return sup.status(name).to_dict()

    def scan_cameras(self) -> Dict[str, Any]:
        with self.lock:
            sup = self.supervisor
            stopped = []
            for name in sup.units.names():
                if sup.units[name].group == "camera" and sup.is_running(name):
                    sup.stop(name)
                    stopped.append(name)
            try:
                devs = self.scanner()
            except RuntimeError as exc:
                # The scan did not happen: leave the cameras as they were, not stopped.
                not_restored = []
                for name in stopped:
                    try:
                        sup.start(name)
                    except UnitError:
                        not_restored.append(sup.units[name].title)
                message = str(exc)
                if not_restored:
                    message += f"；以下相机未能恢复运行，请手动启动：{'、'.join(not_restored)}"
                raise HTTPException(500, {"message": message})
            current = config_file.camera_serials(self.config_path)
            suggestion = devices.suggest_roles(devs, current)
            _, raw = config_file.load_rt(self.config_path)
            cams = raw.get("cameras") or {}
            return {
                "devices": [d.__dict__ for d in devs],
                "current": current,
                "rotation": {r: int((cams.get(r) or {}).get("rotation", 0) or 0) for r in devices.ROLES},
                "suggestion": {r: (d.serial if d else None) for r, d in suggestion.items()},
                "stopped": stopped,
            }

    def assign_cameras(self, assignments: Dict[str, Dict[str, Any]]) -> Dict[str, Any]:
        serials = [a.get("serial") for a in assignments.values() if a.get("enable", True) and a.get("serial")]
        if len(serials) != len(set(serials)):
            raise HTTPException(400, {"message": "同一台相机被分配给了多个角色"})
        with self.lock:
            try:
                config_file.set_cameras(self.config_path, assignments)
            except (ValueError, RuntimeError) as exc:
                raise HTTPException(400, {"message": str(exc)})
            self.reload()
            return self.state()

    def frames(self, unit: str):
        """MJPEG parts for a camera unit; ends after PREVIEW_IDLE_S without new frames."""
        topic = self.supervisor.units[unit].topics[0]
        period = 1.0 / PREVIEW_HZ
        last, idle_since = None, time.monotonic()
        while time.monotonic() - idle_since < PREVIEW_IDLE_S:
            frame = self.probe.latest_image(topic)
            if frame and frame[0] != last:
                last, idle_since = frame[0], time.monotonic()
                mime = "image/png" if "png" in frame[1] else "image/jpeg"
                yield (b"--frame\r\n" + f"Content-Type: {mime}\r\nContent-Length: {len(frame[2])}\r\n\r\n".encode()
                       + frame[2] + b"\r\n")
            time.sleep(period)


def create_app(console: Console, static_dir: Optional[str]) -> FastAPI:
    app = FastAPI(title="robot_control_suite 控制台", docs_url=None, redoc_url=None, openapi_url=None)

    def unit_or_404(name: str) -> str:
        if name not in console.supervisor.units:
            raise HTTPException(404, {"message": f"未知模块 '{name}'"})
        return name

    def run(action: Callable[[], Any]):
        try:
            return action()
        except UnitError as exc:
            raise HTTPException(400, {"message": str(exc)})

    @app.get("/api/state")
    def get_state():
        return console.state()

    @app.post("/api/units/{name}/start")
    def start_unit(name: str):
        return run(lambda: console.start([unit_or_404(name)]))

    @app.post("/api/units/{name}/stop")
    def stop_unit(name: str, confirm: bool = Body(False, embed=True)):
        return run(lambda: console.stop([unit_or_404(name)], confirm))

    @app.post("/api/units/{name}/restart")
    def restart_unit(name: str, confirm: bool = Body(False, embed=True)):
        return run(lambda: console.restart(unit_or_404(name), confirm))

    @app.post("/api/inference/start")
    def start_inference():
        return run(lambda: console.start(console.inference_names(), inference=True))

    @app.post("/api/inference/stop")
    def stop_inference(confirm: bool = Body(False, embed=True)):
        return run(lambda: console.stop(console.inference_names(), confirm))

    @app.post("/api/all/start")
    def start_all():
        return run(lambda: console.start(None))

    @app.post("/api/all/stop")
    def stop_all(confirm: bool = Body(False, embed=True)):
        return run(lambda: console.stop(None, confirm))

    @app.post("/api/fsm/{action}")
    def fsm_command(action: str):
        return console.fsm_command(action)

    @app.post("/api/body/{action}")
    def body_command(action: str):
        return console.body_command(action)

    @app.post("/api/diag")
    def make_diag():
        return {"name": os.path.basename(console.make_diag())}

    @app.get("/api/diag/{name}")
    def download_diag(name: str):
        if not DIAG_NAME.fullmatch(name):
            raise HTTPException(404, {"message": "没有这个诊断包"})
        path = os.path.join(console.ws_root, "logs", "diag", name)
        if not os.path.isfile(path):
            raise HTTPException(404, {"message": "没有这个诊断包"})
        return FileResponse(path, filename=name, media_type="application/gzip")

    @app.get("/api/units/{name}/log")
    def unit_log(name: str, lines: int = 200):
        return {"lines": console.supervisor.log_tail(unit_or_404(name), max(1, min(lines, 2000)))}

    @app.post("/api/cameras/scan")
    def scan():
        return console.scan_cameras()

    @app.post("/api/cameras/assign")
    def assign(assignments: Dict[str, Dict[str, Any]] = Body(...)):
        return console.assign_cameras(assignments)

    @app.get("/api/cameras/{name}/stream")
    def stream(name: str):
        unit = unit_or_404(name)
        if console.supervisor.units[unit].group != "camera":
            raise HTTPException(400, {"message": f"{name} 不是相机"})
        if console.probe is None:
            raise HTTPException(503, {"message": "ROS 探针不可用，无法预览"})
        return StreamingResponse(console.frames(unit), media_type="multipart/x-mixed-replace; boundary=frame")

    if static_dir and os.path.isdir(static_dir):
        root = os.path.realpath(static_dir)

        # The page files change with the repo: browsers must revalidate (the ETag makes that cheap) instead of
        # showing an older copy they cached heuristically.
        revalidate = {"Cache-Control": "no-cache"}

        @app.get("/")
        def index():
            return FileResponse(os.path.join(root, "index.html"), headers=revalidate)

        @app.get("/static/{filename}")
        def static(filename: str):
            path = os.path.realpath(os.path.join(root, filename))
            if not path.startswith(root + os.sep) or not os.path.isfile(path):
                raise HTTPException(404)
            return FileResponse(path, headers=revalidate)
    return app
