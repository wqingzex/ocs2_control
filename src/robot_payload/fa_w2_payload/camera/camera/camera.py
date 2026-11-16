"""Orbbec cameras: discovery, launch specs, processes that hold them, and an image probe.

Discovery lists USB cameras without opening them (the SDK tool covers other connections).
Specs: head Gemini 335Lg, wrists Gemini 305, ported from camera_setup @04398b9c; same driver launch files,
parameters and topics (/camera_<name>/color/image_raw/compressed)."""
from __future__ import annotations

import os
import re
import signal
import stat
import subprocess
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import yaml

from fa_w2_payload.config import ROLES

# ---- discovery: one block per device "...]: name: Orbbec Gemini 335Lg", "serial: ...", "connection: ...", "usb port: ..." ----

USB_SYSFS = Path("/sys/bus/usb/devices")
USB_DEVFS = Path("/dev/bus/usb")
_LINE = re.compile(r"\]:\s*(name|serial|connection|usb port):\s*(.*?)\s*$")


@dataclass(frozen=True)
class Device:
    model: str
    serial: str
    connection: str = ""
    usb_port: str = ""

    @property
    def is_head_model(self) -> bool:  # Gemini 330 series (335Lg)
        return bool(re.search(r"33\d", self.model))

    @property
    def is_wrist_model(self) -> bool:  # Gemini 305 / 305g
        return "305" in self.model


def parse_list_devices(text: str) -> List[Device]:
    devices: List[Device] = []
    current: Dict[str, str] = {}

    def flush():
        if current.get("serial"):
            devices.append(Device(model=current.get("name", "?"), serial=current["serial"],
                                  connection=current.get("connection", ""), usb_port=current.get("usb port", "")))
        current.clear()

    for line in text.splitlines():
        match = _LINE.search(line)
        if not match:
            continue
        key, value = match.groups()
        if key == "name" and current:
            flush()
        current[key] = value
        if key == "usb port":
            flush()
    flush()
    return devices


def _usb_cameras():
    """(sysfs entry, device node) of every Orbbec USB device the kernel lists."""
    try:
        entries = sorted(USB_SYSFS.iterdir())
    except OSError:
        return
    for entry in entries:
        try:
            if (entry / "idVendor").read_text().strip().lower() != "2bc5":
                continue
            bus, number = int((entry / "busnum").read_text()), int((entry / "devnum").read_text())
        except (OSError, ValueError):
            continue
        yield entry, USB_DEVFS / f"{bus:03}" / f"{number:03}"


def _scan_usb_metadata() -> List[Device]:
    """Read sysfs metadata only, skipping devices whose node this environment cannot open."""
    devices = []
    for entry, node in _usb_cameras():
        if not os.access(node, os.R_OK | os.W_OK):
            continue
        try:
            model, serial = (entry / "product").read_text().strip(), (entry / "serial").read_text().strip()
            speed = float((entry / "speed").read_text())
        except (OSError, ValueError):
            continue
        if serial:
            devices.append(Device(model, serial, "USB3.0" if speed >= 5000 else "USB2.0", entry.name))
    return devices


def inaccessible_usb_devices():
    """(serial, node) of Orbbec cameras that are listed but whose device node is not accessible."""
    missing = []
    for entry, node in _usb_cameras():
        if not os.access(node, os.R_OK | os.W_OK):
            try:
                missing.append(((entry / "serial").read_text().strip(), str(node)))
            except OSError:
                pass
    return missing


def scan(timeout_s: float = 20.0) -> List[Device]:
    """Prefer USB metadata; the SDK tool opens cameras and can fail if they are in use."""
    usb_devices = _scan_usb_metadata()
    if usb_devices:
        return usb_devices
    try:
        result = subprocess.run(["ros2", "run", "orbbec_camera", "list_devices_node"],
                                capture_output=True, text=True, timeout=timeout_s, check=False)
    except FileNotFoundError as exc:
        raise RuntimeError("找不到 ros2 命令：请先 source ROS 工作区") from exc
    except subprocess.TimeoutExpired as exc:
        raise RuntimeError(f"扫描相机超时（{timeout_s:.0f}s）") from exc
    output = result.stdout + result.stderr
    if result.returncode != 0:
        raise RuntimeError(f"扫描相机失败（退出码 {result.returncode}）：{output.strip()[-1500:]}")
    devices = parse_list_devices(output)
    if not devices and "[ERROR]" in output:
        raise RuntimeError(f"相机枚举失败：{output.strip()[-1500:]}")
    return devices


def suggest_roles(devices: List[Device], current: Optional[Dict[str, str]] = None) -> Dict[str, Optional[Device]]:
    """Role -> device. Keeps serials already assigned in the config; otherwise 33x -> head and the
    305s -> wrists (left/right order by serial; the operator must confirm or swap)."""
    current = current or {}
    by_serial = {d.serial: d for d in devices}
    roles: Dict[str, Optional[Device]] = {r: by_serial.get(current.get(r, "")) for r in ROLES}
    used = {d.serial for d in roles.values() if d}
    free = [d for d in sorted(devices, key=lambda d: d.serial) if d.serial not in used]
    if roles["head"] is None:
        head = next((d for d in free if d.is_head_model), None)
        if head:
            roles["head"] = head
            free.remove(head)
    for role in ("left_wrist", "right_wrist"):
        if roles[role] is None:
            wrist = next((d for d in free if d.is_wrist_model), None) or (free[0] if free else None)
            if wrist:
                roles[role] = wrist
                free.remove(wrist)
    return roles


# ---- launch specs ----

WIDTH, HEIGHT = 640, 480


@dataclass
class CameraSpec:
    name: str                       # ROS camera_name, e.g. camera_left_wrist
    launch_file: str                # file in orbbec_camera/launch
    args: Dict[str, str]            # launch arguments
    remaps: List[Tuple[str, str]] = field(default_factory=list)
    config_yaml: Dict[str, Any] = field(default_factory=dict)  # written to config_file_path (wrist)


def wrist_eye(rotation: int) -> str:
    """Gemini 305 has two color eyes; camera_setup uses the right eye, the left one at 90/270°."""
    return "left" if rotation in (90, 270) else "right"


def _wrist_yaml(eye: str, rotation: int, fps: int) -> Dict[str, Any]:
    """Keys gemini305.launch.py does not take as launch args (camera_setup gemini305_*_wrist.yaml)."""
    other = "left" if eye == "right" else "right"
    return {
        "device_preset": "Dual Color Streams",
        "enable_color": False,
        "enable_depth": False,
        "enable_left_ir": False,
        "enable_right_ir": False,
        "enable_point_cloud": False,
        f"enable_{eye}_color": True,
        f"{eye}_color_width": WIDTH,
        f"{eye}_color_height": HEIGHT,
        f"{eye}_color_fps": fps,
        f"{eye}_color_format": "YUYV",
        f"{eye}_color_rotation": rotation,
        f"{eye}_color.image_raw.enable_pub_plugins": ["image_transport/compressed"],
        # The preset forces both eyes on; keep the unused one minimal so the USB 2 link is not saturated.
        f"enable_{other}_color": False,
        f"{other}_color_width": WIDTH,
        f"{other}_color_height": HEIGHT,
        f"{other}_color_fps": 5,
        f"{other}_color.image_raw.enable_pub_plugins": ["image_transport/compressed"],  # not raw, theora, zstd
    }


def _common_color_args(cams: Dict[str, Any]) -> Dict[str, str]:
    return {
        "color_white_balance": str(cams["white_balance"]),
        "color_ae_max_exposure": str(cams["ae_max_exposure"]),
        "color_powerline_freq": "50hz",
        "color_anti_flicker": "true",
        "publish_tf": "false",
        "diagnostic_period": "0.0",
    }


def _device_args(name: str, cam: Dict[str, Any]) -> Dict[str, str]:
    args = {"camera_name": name, "serial_number": cam["serial"]}
    if cam["usb_port"]:
        args["usb_port"] = cam["usb_port"]
    return args


def camera_specs(cfg: Dict[str, Any]) -> List[CameraSpec]:
    """Specs for every enabled camera in a validated config."""
    cams = cfg["cameras"]
    if not cams["enable"]:
        return []
    fps = int(cams["fps"])
    specs: List[CameraSpec] = []

    for side in ("left", "right"):
        cam = cams[f"{side}_wrist"]
        if not cam["enable"]:
            continue
        name = f"camera_{side}_wrist"
        eye = wrist_eye(int(cam["rotation"]))
        args = _device_args(name, cam)
        args.update(_common_color_args(cams))
        args.update({
            "enable_color_auto_exposure_priority": "false",
            "enable_color_auto_white_balance": "false",
            "enable_color_auto_exposure": "false",
            "color_exposure": str(cams["wrist_exposure"]),
            "color_gain": str(cams["wrist_gain"]),
            "ae_reference_stream": "color",
            "ae_strategy": "motion",
        })
        specs.append(CameraSpec(
            name=name,
            launch_file="gemini305.launch.py",
            args=args,
            # Publish the eye in use under <name>/color regardless of rotation.
            remaps=[
                (f"/{name}/{eye}_color/image_raw/compressed", f"/{name}/color/image_raw/compressed"),
                (f"/{name}/{eye}_color/camera_info", f"/{name}/color/camera_info"),
            ],
            config_yaml=_wrist_yaml(eye, int(cam["rotation"]), fps),
        ))

    head = cams["head"]
    if head["enable"]:
        name = "camera_head"
        args = _device_args(name, head)
        args.update(_common_color_args(cams))
        # Native 640x480 profile; the SDK rotates decoded frames, so the format must be YUYV (not MJPG).
        args.update({
            "enable_color": "true",
            "color_width": str(WIDTH),
            "color_height": str(HEIGHT),
            "color_format": "YUYV",
            "color_rotation": str(head["rotation"]),
            "color_fps": str(fps),
            "enable_color_auto_exposure_priority": "false",
            "enable_color_auto_exposure": "true",
            "enable_color_auto_white_balance": "false",
            "color_exposure": "-1",
            "color_gain": "-1",
            "color.image_raw.enable_pub_plugins": '["image_transport/compressed"]',
            "enable_depth": "false",
            "enable_left_ir": "false",
            "enable_right_ir": "false",
            "enable_point_cloud": "false",
        })
        specs.append(CameraSpec(name=name, launch_file="gemini_330_series.launch.py", args=args))
    return specs


def runtime_dir() -> str:
    """Private per-user directory for generated driver YAML (like camera_setup)."""
    base = os.environ.get("XDG_RUNTIME_DIR") or "/tmp"
    path = os.path.join(base, f"fa_w2_camera-{os.getuid()}")
    os.makedirs(path, mode=0o700, exist_ok=True)
    return path


# ---- processes that hold the cameras (stopped only after explicit user approval) ----

PROC = Path('/proc')


@dataclass(frozen=True)
class CameraProcess:
    pid: int
    start_ticks: int
    camera: str
    serial: str
    kind: str


def read_process(path):
    try:
        fields = (path / 'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        args = [arg.decode(errors='replace') for arg in (path / 'cmdline').read_bytes().split(b'\0') if arg]
        return args, int(fields[19])
    except (OSError, ValueError, IndexError):
        return None


def usb_device_nodes(devices):
    """Character device number -> serial of the given cameras."""
    serials = {device.serial for device in devices}
    found = {}
    for entry, node in _usb_cameras():
        try:
            serial, item = (entry / "serial").read_text().strip(), node.stat()
        except OSError:
            continue
        if serial in serials and stat.S_ISCHR(item.st_mode):
            found[item.st_rdev] = serial
    return found


def open_usb_serials(pid, nodes):
    found = set()
    try:
        for descriptor in (PROC / str(pid) / 'fd').iterdir():
            try:
                item = descriptor.stat()
                if stat.S_ISCHR(item.st_mode) and item.st_rdev in nodes:
                    found.add(nodes[item.st_rdev])
            except OSError:
                continue
    except OSError:
        pass
    return found


def find_camera_processes(devices, excluded_names=()):
    serials = {device.serial for device in devices}
    processes, cameras, launches = {}, {}, []
    for path in PROC.iterdir():
        if path.name.isdigit():
            current = read_process(path)
            if current:
                processes[int(path.name)] = current
    for pid, (args, ticks) in processes.items():
        if 'launch' not in args:
            continue
        index = args.index('launch')
        if len(args) <= index + 2:
            continue
        package, launch = args[index + 1:index + 3]
        values = dict(arg.split(':=', 1) for arg in args if ':=' in arg)
        matched = {}
        if package == 'orbbec_camera' and launch.endswith('.launch.py'):
            serial, name = values.get('serial_number'), values.get('camera_name')
            if serial in serials and name and name not in excluded_names:
                matched[name] = serial
        elif package == 'camera' and launch == 'cameras.launch.py':
            try:
                raw = yaml.safe_load(Path(values['config']).read_text()) or {}
                cams = raw.get('cameras') or {}
                for role in ROLES:
                    item = cams.get(role) or {}
                    name = values.get('name_prefix', '') + 'camera_' + role
                    if cams.get('enable', True) and item.get('enable', True) and item.get('serial') in serials and name not in excluded_names:
                        matched[name] = item['serial']
            except (OSError, KeyError, TypeError, yaml.YAMLError):
                continue
        if matched:
            cameras.update(matched)
            launches.append(CameraProcess(pid, ticks, ' / '.join(matched), ', '.join(matched.values()), 'launch'))
    drivers = []
    nodes = usb_device_nodes(devices)
    for pid, (args, ticks) in processes.items():
        if not args or Path(args[0]).name not in ('component_container', 'component_container_mt'):
            continue
        if '__node:=camera_container' not in args:
            continue
        namespace = next((arg.split(':=/', 1)[1] for arg in args if arg.startswith('__ns:=/')), '')
        if namespace in excluded_names:
            continue
        owned = [(name, serial) for name, serial in cameras.items() if '__ns:=/' + name in args]
        drivers += [CameraProcess(pid, ticks, name, serial, 'driver') for name, serial in owned]
        if not owned:
            # A killed launch process may leave a camera component in another
            # session. Match its actual open USB device, not the missing parent.
            held = open_usb_serials(pid, nodes)
            if held:
                drivers.append(CameraProcess(pid, ticks, namespace, ', '.join(sorted(held)), 'driver'))
    return [asdict(item) for item in launches + drivers]


def stop_camera_processes(approved, devices, excluded_names=()):
    current = find_camera_processes(devices, excluded_names)
    permitted = {(item['pid'], item['start_ticks']) for item in approved}
    if any((item['pid'], item['start_ticks']) not in permitted for item in current):
        raise ValueError('相机进程列表已变化，请重新确认要停止的进程')
    targets = [item for item in current if (item['pid'], item['start_ticks']) in permitted]

    def alive(item):
        process = read_process(PROC / str(item['pid']))
        return process is not None and process[1] == item['start_ticks']

    for sig, grace in ((signal.SIGINT, 3), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
        for item in targets:
            if alive(item):
                try:
                    os.kill(item['pid'], sig)
                except ProcessLookupError:
                    pass
                except PermissionError as exc:
                    raise ValueError(f"没有权限停止相机进程 {item['pid']}，请先自行停止原采集程序") from exc
        deadline = time.monotonic() + grace
        while any(alive(item) for item in targets) and time.monotonic() < deadline:
            time.sleep(.05)
        if not any(alive(item) for item in targets):
            return
    raise ValueError('旧相机进程未完全退出，请检查后重试')


def camera_error(message):
    lower = message.lower()
    if 'uvc_open failed' in lower and ('return code: -6' in lower or 'return code:-6' in lower):
        return '相机正被其他采集程序占用，请点击“应用配置并预览”停止旧采集并重试。容器看不到的宿主机进程需先在宿主机停止。'
    if 'permission denied' in lower or 'access denied' in lower:
        return '相机设备访问被拒绝，请检查 USB 权限及容器设备映射。'
    return message


# ---- image-only ROS subscriber, independent of teleoperation, FSM and XR ----

RATE_WINDOW_S = 2.0
# The driver logs this once the device is initialized; until then it holds the shared device lock, and a second
# driver that starts meanwhile gives up after 5 x 100 ms. Cameras are therefore started one after another.
DRIVER_READY_LOG = "Initialize device cost"
DRIVER_READY_TIMEOUT_S = 30.0


class ImageProbe:
    def __init__(self, name):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node(name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.images, self.subscriptions, self.arrivals = {}, {}, {}
        self.lock, self.stopped = threading.Lock(), threading.Event()
        self.thread = threading.Thread(target=self.spin, daemon=True)
        self.thread.start()

    def spin(self):
        while not self.stopped.is_set():
            self.executor.spin_once(timeout_sec=.1)

    def watch_image(self, topic):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage
        if topic not in self.subscriptions:
            self.subscriptions[topic] = self.node.create_subscription(CompressedImage, topic,
                lambda message: self.receive(topic, message), qos_profile_sensor_data)

    def receive(self, topic, message):
        now = time.monotonic()
        with self.lock:
            self.images[topic] = (now, message.format, bytes(message.data))
            arrivals = self.arrivals.setdefault(topic, deque())
            arrivals.append(now)
            while arrivals and now - arrivals[0] > RATE_WINDOW_S:
                arrivals.popleft()

    def latest_image(self, topic):
        with self.lock:
            return self.images.get(topic)

    def rate(self, topic):
        """Frames per second over the last RATE_WINDOW_S seconds (0.0 when none arrived in that time)."""
        now = time.monotonic()
        with self.lock:
            arrivals = self.arrivals.get(topic, ())
            count = sum(1 for stamp in arrivals if now - stamp <= RATE_WINDOW_S)
        return round(count / RATE_WINDOW_S, 1)

    def close(self):
        self.stopped.set()
        self.thread.join(timeout=2)
        self.executor.shutdown()
        self.node.destroy_node()
        self.context.shutdown()
