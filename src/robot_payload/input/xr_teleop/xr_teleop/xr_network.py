"""Read IPv4 addresses from wired/Wi-Fi interfaces without an external ip command."""
from __future__ import annotations

import fcntl
import ipaddress
import socket
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import List

SYS_NET = Path("/sys/class/net")
DEFAULT_ARM_IP = "192.168.1.190"  # the Marvin controller (marvin_hardware's device_ip default)


@dataclass(frozen=True)
class NetworkAddress:
    interface: str
    address: str
    mode: str
    network: str = ""  # e.g. "192.168.1.0/24"; empty when the netmask could not be read


def _ipv4(interface: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        request = struct.pack("256s", interface.encode()[:15])
        response = fcntl.ioctl(sock.fileno(), 0x8915, request)  # SIOCGIFADDR, no network traffic
    return socket.inet_ntoa(response[20:24])


def _netmask(interface: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        request = struct.pack("256s", interface.encode()[:15])
        response = fcntl.ioctl(sock.fileno(), 0x891B, request)  # SIOCGIFNETMASK
    return socket.inet_ntoa(response[20:24])


def arm_ip(raw) -> str:
    """The arm controller's address from the config (hardware.device_ip), or the default."""
    return str((raw.get("hardware") or {}).get("device_ip") or DEFAULT_ARM_IP)


def without_arm_link(addresses: List["NetworkAddress"], arm: str) -> List["NetworkAddress"]:
    """Drop the adapter on the arm's own subnet: it is the cable to the arm, the headset is never on it."""
    try:
        target = ipaddress.ip_address(arm)
    except ValueError:
        return list(addresses)
    return [entry for entry in addresses if not (entry.network and target in ipaddress.ip_network(entry.network))]


def network_addresses(mode: str = "") -> List[NetworkAddress]:
    addresses = []
    for _, interface in socket.if_nameindex():
        if interface == "lo" or interface.startswith(("docker", "veth", "br-", "virbr")):
            continue
        path = SYS_NET / interface
        try:
            if (path / "type").read_text().strip() != "1":
                continue
            if (path / "operstate").exists() and (path / "operstate").read_text().strip() in (
                    "down", "notpresent", "lowerlayerdown", "dormant"):
                continue
            address = _ipv4(interface)
            if ipaddress.ip_address(address).is_loopback:
                continue
        except (OSError, ValueError):
            continue
        kind = "wifi" if (path / "wireless").exists() or interface.startswith(("wl", "wlan")) else "ethernet"
        try:
            network = str(ipaddress.ip_interface(f"{address}/{_netmask(interface)}").network)
        except (OSError, ValueError):
            network = ""
        if not mode or kind == mode:
            addresses.append(NetworkAddress(interface, address, kind, network))
    return sorted(addresses, key=lambda entry: (entry.mode != "ethernet", entry.interface))
