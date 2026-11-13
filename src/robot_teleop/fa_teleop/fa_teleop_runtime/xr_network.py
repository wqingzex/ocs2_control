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

@dataclass(frozen=True)
class NetworkAddress:
    interface: str
    address: str
    mode: str


def _ipv4(interface: str) -> str:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        request = struct.pack("256s", interface.encode()[:15])
        response = fcntl.ioctl(sock.fileno(), 0x8915, request)  # SIOCGIFADDR, no network traffic
    return socket.inet_ntoa(response[20:24])


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
        if not mode or kind == mode:
            addresses.append(NetworkAddress(interface, address, kind))
    return sorted(addresses, key=lambda entry: (entry.mode != "ethernet", entry.interface))
