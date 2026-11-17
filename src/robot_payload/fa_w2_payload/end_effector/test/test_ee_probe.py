"""Fake byte transports only: no test enumerates or opens real hand hardware."""
import struct

import pytest

from end_effector import ee_probe


class L6Transport:
    def __init__(self, fail=False):
        self.requests, self.fail = [], fail
        self.values = [100] * 18

    def exchange(self, request, protocol='modbus'):
        self.requests.append(request)
        address, function = request[:2]
        assert request[-2:] == struct.pack('<H', ee_probe.crc16(request[:-2]))
        if function == 4:
            count = struct.unpack_from('>H', request, 4)[0]
            result = bytes([address, function, count * 2]) + struct.pack('>' + 'H' * count, *self.values[:count])
        else:
            count = struct.unpack_from('>H', request, 4)[0]
            start = struct.unpack_from('>H', request, 2)[0]
            self.values[start:start + count] = struct.unpack('>' + 'H' * count, request[7:-2])
            result = request[:6]
        return ee_probe.with_crc(result)[:-1] + b'\xff' if self.fail else ee_probe.with_crc(result)


def test_l6_probe_only_reads_and_uses_the_correct_side_address():
    transport = L6Transport()
    assert len(ee_probe.L6(transport, 'left').read()) == 6
    ee_probe.L6(transport, 'right').read()
    assert [request[:2] for request in transport.requests] == [b'\x28\x04', b'\x27\x04']


def _sleepless(monkeypatch):
    monkeypatch.setattr(ee_probe.time, 'sleep', lambda _: None)


def _position_writes(transport):
    return [struct.unpack('>18H', request[7:-2]) for request in transport.requests
            if request[1] == 16 and struct.unpack_from('>H', request, 2)[0] == 0]


def test_l6_open_close_closes_four_fingers_halfway_slowly_and_restores_everything(monkeypatch):
    _sleepless(monkeypatch)
    transport = L6Transport()
    transport.values[:6] = [255, 255, 255, 255, 255, 255]  # fully open
    transport.values[6:12], transport.values[12:] = [90] * 6, [40] * 6
    initial = list(transport.values)
    ee_probe.L6(transport, 'left').test()
    ramp = _position_writes(transport)
    assert ramp[0][6:12] == (26,) * 6 and ramp[0][12:] == (13,) * 6  # slow speed and low torque while moving
    peak = max(ramp, key=lambda w: -w[2])
    assert peak[:2] == (255, 255) and peak[2:6] == (130,) * 4  # thumbs stay, fingers go to half
    assert all(abs(b[2] - a[2]) <= 13 for a, b in zip(ramp, ramp[1:11]))  # evenly spaced small steps
    assert transport.values == initial  # pose, speed and torque are back


def test_l6_open_close_opens_a_hand_that_starts_closed(monkeypatch):
    _sleepless(monkeypatch)
    transport = L6Transport()
    transport.values[:6] = [100] * 6
    ee_probe.L6(transport, 'right').test()
    assert max(w[2] for w in _position_writes(transport)) == 255
    assert transport.values[:6] == [100] * 6


def test_l6_open_close_restores_the_pose_when_a_write_fails_midway(monkeypatch):
    _sleepless(monkeypatch)
    transport = L6Transport()
    initial = list(transport.values)
    real, calls = transport.exchange, []

    def flaky(request, protocol='modbus'):
        if request[1] == 16 and struct.unpack_from('>H', request, 2)[0] == 0:
            calls.append(1)
            if len(calls) == 4:
                raise RuntimeError('串口发送超时')
        return real(request, protocol)

    transport.exchange = flaky
    with pytest.raises(RuntimeError, match='串口发送超时'):
        ee_probe.L6(transport, 'left').test()
    assert transport.values == initial


def test_l6_open_close_reports_fingers_that_did_not_move_and_still_restores(monkeypatch):
    _sleepless(monkeypatch)
    transport = L6Transport()
    initial = list(transport.values)
    real_read = ee_probe.L6.read
    # The hand ignores the commands: feedback for the six positions never changes.
    monkeypatch.setattr(ee_probe.L6, 'read', lambda self, count=6: [100] * count if count == 6 else real_read(self, count))
    with pytest.raises(RuntimeError, match='未检测到位移'):
        ee_probe.L6(transport, 'left').test()
    assert transport.values == initial


def test_l6_bad_feedback_prevents_motion():
    transport = L6Transport(fail=True)
    with pytest.raises(RuntimeError, match='CRC'):
        ee_probe.L6(transport, 'left').test()
    assert all(request[1] == 4 for request in transport.requests)


class XTransport:
    def __init__(self):
        self.requests, self.positions = [], [0.2] * 12

    def exchange(self, request, protocol):
        assert protocol == 'xhand1' and len(request) == 297
        assert request[:5] == b'\x55\xaa\xfe\x80\x02'
        assert ee_probe.crc16(request[:-2], 0x1021) == struct.unpack('<H', request[-2:])[0]
        self.requests.append(request)
        for index in range(12):
            record = request[7 + index * 24:7 + (index + 1) * 24]
            if struct.unpack_from('<H', record, 14)[0] == 3:
                self.positions[index] = struct.unpack_from('<f', record, 8)[0]
        records = b''.join(bytes([index, 0]) + struct.pack('<f', self.positions[index]) + bytes(18)
                           for index in range(12))
        return ee_probe.with_crc(b'\x55\xaa\x80\xfe\x02' + struct.pack('<H', len(records)) + records, 0x1021)


def test_xhand_probe_uses_disabled_zero_torque_frame():
    transport = XTransport()
    assert len(ee_probe.XHand(transport).read()) == 12
    for offset in range(7, 7 + 12 * 24, 24):
        assert struct.unpack_from('<HH', transport.requests[0], offset + 12) == (0, 0)


def test_xhand_open_close_enables_only_the_four_finger_joints_and_finishes_disabled(monkeypatch):
    monkeypatch.setattr(ee_probe.time, 'sleep', lambda _: None)
    transport = XTransport()
    ee_probe.XHand(transport).test()
    for request in transport.requests:
        for index in range(12):
            torque, mode = struct.unpack_from('<HH', request, 7 + index * 24 + 12)
            assert (index in ee_probe.XHand.FINGERS and torque <= 30 and mode in (0, 3)) or (torque == 0 and mode == 0)
    assert all(struct.unpack_from('<H', transport.requests[-1], 7 + index * 24 + 14)[0] == 0 for index in range(12))
    assert all(abs(value - 0.2) < 0.001 for value in transport.positions)


def test_485_refuses_busy_sdk_port_before_loading_or_connecting(monkeypatch):
    class BusySocket:
        def __enter__(self): return self
        def __exit__(self, *args): pass
        def bind(self, address): raise OSError('busy')

    monkeypatch.setattr(ee_probe.socket, 'socket', lambda *args: BusySocket())
    monkeypatch.setattr(ee_probe.ctypes, 'CDLL', lambda *args: pytest.fail('must not load SDK'))
    with pytest.raises(RuntimeError, match='端口 4730'):
        ee_probe.Arm485Transport('/workspace', '192.168.1.190', 'left', 1)
