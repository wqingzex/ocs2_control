"""Orbbec cameras: discovery, processes that hold them, and an image probe (used by the camera setup).

Discovery lists USB cameras without opening them (the SDK tool covers other connections). How the cameras are
started is in launch_helpers.devices.camera."""
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

from robot_config.config import ROLES

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
        elif package == 'robot_startup' and launch == 'cameras.launch.py':
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
        self.images, self.subscriptions, self.arrivals, self.clients = {}, {}, {}, {}
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

    def set_value(self, service, kind, value, timeout=2.0):
        """Call one driver setting service (kind 'int': orbbec_camera_msgs SetInt32, 'bool': std_srvs SetBool) and wait
        for its answer. Returns (ok, message); never raises for a driver that is absent or refuses."""
        from orbbec_camera_msgs.srv import SetInt32
        from std_srvs.srv import SetBool
        service_type = SetBool if kind == 'bool' else SetInt32
        with self.lock:
            client = self.clients.get(service)
            if client is None:
                client = self.clients[service] = self.node.create_client(service_type, service)
        if not client.wait_for_service(timeout_sec=timeout):
            return False, '相机驱动没有响应'
        answered = threading.Event()
        request = service_type.Request()
        request.data = value
        future = client.call_async(request)
        future.add_done_callback(lambda _: answered.set())
        if not answered.wait(timeout):
            return False, '相机驱动没有应答'
        response = future.result()
        return bool(response.success), response.message

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
