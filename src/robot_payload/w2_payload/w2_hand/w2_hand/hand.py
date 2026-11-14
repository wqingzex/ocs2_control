"""Bounded, standalone hand probes. No controller manager, servo enable or arm commands.

Each request runs in a child process: native SDK state and open device descriptors
cannot survive the setup session. Only an explicit 'test' request (the open/close test) moves fingers.
"""
from __future__ import annotations

import contextlib
import ctypes
import fcntl
import glob
import ipaddress
import json
import math
import os
from pathlib import Path
import select
import socket
import struct
import sys
import termios
import time


SERIAL_BY_ID = '/dev/serial/by-id/'


def is_serial_number_path(path):
    """A hand's serial port is identified by the adapter's serial number: /dev/serial/by-id/<vendor>_<model>_<serial>.

    /dev/ttyUSBn is numbered in enumeration order and changes when a port is replugged or the machine restarts;
    /dev/serial/by-path depends on the physical socket. Neither says which adapter is on the line."""
    return str(path).startswith(SERIAL_BY_ID)


def serial_devices():
    """The serial adapters that can be told apart by serial number, one entry per character device."""
    found = {}
    for path in sorted(glob.glob(SERIAL_BY_ID + '*')):
        real = os.path.realpath(path)
        if real not in found and os.path.exists(path):
            found[real] = {"path": path, "device": real}
    return list(found.values())


def crc16(data, polynomial=0xA001):
    value = 0xFFFF if polynomial == 0xA001 else 0
    for byte in data:
        if polynomial == 0xA001:
            value ^= byte
            for _ in range(8):
                value = (value >> 1) ^ (polynomial if value & 1 else 0)
        else:
            value ^= byte << 8
            for _ in range(8):
                value = ((value << 1) ^ (polynomial if value & 0x8000 else 0)) & 0xFFFF
    return value


def with_crc(data, polynomial=0xA001):
    return data + struct.pack('<H', crc16(data, polynomial))


def frame_length(data, protocol):
    if protocol == 'xhand1':
        if len(data) >= 7:
            size = struct.unpack_from('<H', data, 5)[0] + 9
            if size > 4096:
                raise RuntimeError('XHand1 响应长度无效')
            return size
    elif len(data) >= 3:
        return 5 if data[1] & 0x80 else (8 if data[1] == 16 else data[2] + 5)
    return None


def ensure_serial_free(path):
    """Do not open a port already owned by another visible process."""
    target = os.stat(path)
    for process in Path('/proc').iterdir():
        if not process.name.isdigit() or int(process.name) == os.getpid():
            continue
        try:
            descriptors = (process / 'fd').iterdir()
            for descriptor in descriptors:
                try:
                    item = descriptor.stat()
                    if item.st_rdev == target.st_rdev and item.st_mode == target.st_mode:
                        raise RuntimeError(f'{path} 正被进程 {process.name} 占用，请先自行停止使用该设备的程序')
                except OSError:
                    continue
        except (OSError, PermissionError):
            continue


class SerialTransport:
    def __init__(self, path, baud):
        allowed = {os.path.realpath(item['path']) for item in serial_devices()}
        if os.path.realpath(path) not in allowed:
            raise ValueError('所选串口不可用；容器需映射串口设备')
        ensure_serial_free(path)
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            fcntl.flock(self.fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            fcntl.ioctl(self.fd, termios.TIOCEXCL)
            self.original = termios.tcgetattr(self.fd)
            settings = termios.tcgetattr(self.fd)
            settings[0] = settings[1] = settings[3] = 0
            settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            settings[4] = settings[5] = getattr(termios, f'B{baud}')
            settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        except BaseException:
            self.close()
            raise

    def exchange(self, request, protocol='modbus'):
        termios.tcflush(self.fd, termios.TCIFLUSH)
        deadline, response, pending = time.monotonic() + 0.7, bytearray(), memoryview(request)
        while pending and time.monotonic() < deadline:
            if select.select([], [self.fd], [], 0.05)[1]:
                try:
                    pending = pending[os.write(self.fd, pending):]
                except BlockingIOError:
                    continue
        if pending:
            raise RuntimeError('串口发送超时')
        while time.monotonic() < deadline:
            if select.select([self.fd], [], [], 0.05)[0]:
                response.extend(os.read(self.fd, 4096))
                size = frame_length(response, protocol)
                if size is not None and len(response) >= size:
                    return bytes(response[:size])
        raise RuntimeError('未收到手部反馈：检查串口、接线、供电与设备地址')

    def close(self):
        if getattr(self, 'fd', -1) >= 0:
            try:
                if hasattr(self, 'original'):
                    termios.tcsetattr(self.fd, termios.TCSANOW, self.original)
                fcntl.ioctl(self.fd, termios.TIOCNXCL)
            finally:
                os.close(self.fd)
                self.fd = -1


class Arm485Transport:
    """Only the SDK's dedicated 485 datagrams; never call OnSetSend/OnClearErr."""
    def __init__(self, ws_root, address, side, channel):
        address = ipaddress.IPv4Address(address)
        if address.is_loopback or address.is_multicast or address.is_unspecified or str(address) == '255.255.255.255':
            raise ValueError('机械臂控制器 IPv4 地址无效')
        if channel not in (1, 2, 3):
            raise ValueError('末端 485 通道必须是 1、2 或 3')
        # The vendor SDK falls back to shared memory if its fixed UDP port is busy.
        # Refuse that path instead of attaching to somebody else's controller.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as guard:
            try:
                guard.bind(('0.0.0.0', 4730))
            except OSError as exc:
                raise RuntimeError('机械臂 SDK 端口 4730 正被占用，未连接控制器；请先自行停止原运控程序') from exc
        paths = [Path(ws_root) / 'install/marvin_hardware/lib/libMarvinSDK.so',
                 Path(ws_root) / 'install/lib/libMarvinSDK.so']
        try:
            from ament_index_python.packages import get_package_prefix
            paths.insert(0, Path(get_package_prefix('marvin_hardware')) / 'lib/libMarvinSDK.so')
        except (ImportError, LookupError):
            pass
        library = next((p for p in paths if p.is_file()), None)
        if library is None:
            raise RuntimeError('未找到机械臂末端 SDK，请先运行 ./scripts/setup.sh build-workspace')
        self.sdk = ctypes.CDLL(str(library))
        self.sdk.OnLinkTo.argtypes, self.sdk.OnLinkTo.restype = [ctypes.c_ubyte] * 4, ctypes.c_bool
        self.sdk.OnRelease.restype = ctypes.c_bool
        self.channel = channel
        suffix = 'A' if side == 'left' else 'B'
        self.send, self.receive = getattr(self.sdk, 'OnSetChData' + suffix), getattr(self.sdk, 'OnGetChData' + suffix)
        self.send.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.c_long, ctypes.c_long]
        self.send.restype = ctypes.c_bool
        self.receive.argtypes = [ctypes.POINTER(ctypes.c_ubyte), ctypes.POINTER(ctypes.c_long)]
        self.receive.restype = ctypes.c_long
        if not self.sdk.OnLinkTo(*address.packed):
            self.close()
            raise RuntimeError('连接机械臂末端 485 失败')

    def exchange(self, request, protocol='modbus'):
        if protocol != 'modbus' or len(request) > 256:
            raise ValueError('末端 485 仅支持 L6 Modbus 请求')
        buffer = (ctypes.c_ubyte * 256)()
        channel = ctypes.c_long(self.channel)
        # Discard this client's stale receive queue, without clearing controller settings.
        for _ in range(8):
            if self.receive(buffer, ctypes.byref(channel)) <= 0:
                break
        buffer[:len(request)] = request
        if not self.send(buffer, len(request), self.channel):
            raise RuntimeError('末端 485 发送失败')
        deadline, response = time.monotonic() + 0.7, bytearray()
        while time.monotonic() < deadline:
            channel.value = self.channel
            count = self.receive(buffer, ctypes.byref(channel))
            if 0 < count <= 256 and channel.value == self.channel:
                response.extend(bytes(buffer[:count]))
                size = frame_length(response, protocol)
                if size is not None and len(response) >= size:
                    return bytes(response[:size])
            time.sleep(0.005)
        raise RuntimeError('末端 485 未收到反馈，请检查左右臂接线、通道与供电')

    def close(self):
        if getattr(self, 'sdk', None) is not None:
            self.sdk.OnRelease()
            self.sdk = None


class L6:
    def __init__(self, transport, side):
        self.transport, self.address = transport, 0x28 if side == 'left' else 0x27

    def exchange(self, function, payload):
        response = self.transport.exchange(with_crc(bytes([self.address, function]) + payload))
        if len(response) < 5 or crc16(response[:-2]) != struct.unpack('<H', response[-2:])[0]:
            raise RuntimeError('L6 响应 CRC 校验失败')
        if response[0] != self.address or response[1] != function:
            raise RuntimeError('L6 设备地址或响应功能码不匹配，请检查左右手及接线')
        return response

    def read(self, count=6):
        response = self.exchange(4, struct.pack('>HH', 0, count))
        if response[2] != count * 2 or len(response) != count * 2 + 5:
            raise RuntimeError('L6 反馈长度不匹配')
        result = list(struct.unpack('>' + 'H' * count, response[3:-2]))
        if any(value > 255 for value in result):
            raise RuntimeError('L6 反馈超出协议范围')
        return result

    def write(self, values, start=0):
        payload = struct.pack('>HHB', start, len(values), 2 * len(values))
        response = self.exchange(16, payload + struct.pack('>' + 'H' * len(values), *values))
        if response[2:6] != struct.pack('>HH', start, len(values)):
            raise RuntimeError('L6 写入回执不匹配')

    FINGERS = (2, 3, 4, 5)  # index, middle, ring, pinky; the thumb joints stay where they are
    HALF, OPEN, MIN_TRAVEL, STEPS, STEP_S = 130, 255, 20, 10, 0.1  # register units: 255 = open, 0 = closed

    def _ramp(self, start, end, dynamics):
        """Write evenly spaced positions from start to end (6 registers each) with the slow speed/torque set."""
        for step in range(1, self.STEPS + 1):
            self.write([round(a + (b - a) * step / self.STEPS) for a, b in zip(start, end)] + dynamics)
            time.sleep(self.STEP_S)

    def test(self):
        """Open/close test: the four fingers go half way to the other end and come back, slowly.

        Positions AND dynamics are read before anything is written, so unsupported firmware fails before motion.
        Whatever happens, the measured starting pose and the original speed/torque are written back."""
        initial = self.read(18)
        start, slow = initial[:6], [26] * 6 + [13] * 6
        target = list(start)
        for index in self.FINGERS:
            target[index] = self.HALF if start[index] > 190 else self.OPEN  # closing from open, otherwise opening
        reached = start
        try:
            self._ramp(start, target, slow)
            reached = self.read()
            self._ramp(target, start, slow)
        finally:
            try:
                self.write(start + slow)
                time.sleep(0.5)
                self.write(initial[6:], start=6)
            except (RuntimeError, OSError):
                pass  # the original error is the one worth reporting
        returned = self.read()
        if any(abs(reached[i] - start[i]) < self.MIN_TRAVEL for i in self.FINGERS):
            raise RuntimeError('测试指令已发送，但有手指未检测到位移；请检查实物反馈')
        return returned


class XHand:
    def __init__(self, transport):
        self.transport = transport

    def exchange(self, positions=None, active=()):
        positions = positions or [0.0] * 12
        payload = b''.join(struct.pack('<HhhhfHHHHHH', index, 100, 0, 0, positions[index],
                                      30 if index in active else 0,
                                      3 if index in active else 0, 0, 0, 0, 0)
                           for index in range(12))
        request = with_crc(b'\x55\xaa\xfe\x80\x02' + struct.pack('<H', len(payload)) + payload, 0x1021)
        response = self.transport.exchange(request, 'xhand1')
        if (len(response) < 297 or response[:5] != b'\x55\xaa\x80\xfe\x02'
                or crc16(response[:-2], 0x1021) != struct.unpack('<H', response[-2:])[0]
                or len(response) != struct.unpack_from('<H', response, 5)[0] + 9):
            raise RuntimeError('XHand1 响应地址、长度或 CRC 校验失败')
        result = {}
        for offset in range(7, 7 + 12 * 24, 24):
            index, value = response[offset], struct.unpack_from('<f', response, offset + 2)[0]
            if index >= 12 or index in result or not math.isfinite(value) or not -2 < value < 3:
                raise RuntimeError('XHand1 关节反馈无效')
            result[index] = value
        return [result[index] for index in range(12)]

    def read(self):
        # Same zero-torque, disabled-mode feedback request used by the hardware driver.
        return self.exchange()

    # First bend joint of index, middle, ring and pinky; 0 is open (the previous single finger test used joint 4).
    FINGERS, TRAVEL, MIN_TRAVEL, STEPS, STEP_S = (4, 6, 8, 10), 0.8, 0.2, 10, 0.05

    def _ramp(self, start, end):
        """Command the four fingers from start to end in STEPS slow steps; self.pose keeps the last feedback."""
        for step in range(1, self.STEPS + 1):
            command = list(end)
            for i in self.FINGERS:
                command[i] = start[i] + (end[i] - start[i]) * step / self.STEPS
            self.pose = self.exchange(command, self.FINGERS)
            time.sleep(self.STEP_S)

    def test(self):
        """Open/close test: the four fingers move 0.8 rad away from their current end and come back, slowly."""
        initial = self.pose = self.read()
        if any(not 0 <= initial[i] <= 1.7 for i in self.FINGERS):
            raise RuntimeError('XHand1 测试关节不在可测试范围内')
        target = list(initial)
        for i in self.FINGERS:
            target[i] += self.TRAVEL if initial[i] < 0.85 else -self.TRAVEL
        try:
            self._ramp(initial, target)
            reached = self.pose
        finally:
            try:
                self._ramp(self.pose, initial)
                self.read()  # disable test torque
            except (RuntimeError, OSError):
                pass
        result = self.read()
        if any(abs(reached[i] - initial[i]) < self.MIN_TRAVEL for i in self.FINGERS):
            raise RuntimeError('测试指令已发送，但有手指未检测到位移；请检查实物反馈')
        return result


def run(request):
    side, operation, hand, drive = (request[key] for key in ('side', 'operation', 'hand', 'drive'))
    if side not in ('left', 'right') or operation not in ('probe', 'test'):
        raise ValueError('无效的手部测试请求')
    if hand not in ('linkerhand_l6', 'xhand1') or drive not in ('usb', '485'):
        raise ValueError('不支持的手型或接线方式')
    if hand == 'xhand1' and drive != 'usb':
        raise ValueError('XHand1 仅支持 USB 接线')
    if drive == 'usb':
        transport = SerialTransport(request['port'], 115200 if hand == 'linkerhand_l6' else 3000000)
    else:
        transport = Arm485Transport(request['ws_root'], request['device_ip'], side, request['channel'])
    with contextlib.closing(transport):
        device = L6(transport, side) if hand == 'linkerhand_l6' else XHand(transport)
        positions = device.test() if operation == 'test' else device.read()
    return {'positions': positions, 'tested': operation == 'test',
            'message': '已完成开合测试，请观察是哪只手并确认左右' if operation == 'test' else '已收到有效设备反馈，请核对实物左右'}


def main():
    try:
        result = {'ok': True, **run(json.loads(sys.stdin.read()))}
    except (ValueError, RuntimeError, OSError) as exc:
        result = {'ok': False, 'message': str(exc)}
    # A marker keeps vendor SDK stdout messages separate from the JSON response.
    print('\nW2_HAND_RESULT=' + json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    sys.exit(main())
