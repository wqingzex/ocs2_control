"""Wired/Wi-Fi address selection must work without iproute2 or sending network traffic."""
import pytest

from xr_teleop import xr_network


@pytest.mark.parametrize("mode,interfaces", [
    ("", ["enp1s0", "wlp2s0"]), ("ethernet", ["enp1s0"]), ("wifi", ["wlp2s0"]),
])
def test_enumeration_prefers_wired_and_filters_network_type(tmp_path, monkeypatch, mode, interfaces):
    names = ["lo", "docker0", "br-123", "veth1", "enp1s0", "wlp2s0", "enp2s0"]
    for name in names:
        path = tmp_path / name
        path.mkdir()
        (path / "type").write_text("1")
    (tmp_path / "wlp2s0/wireless").mkdir()
    (tmp_path / "enp2s0/operstate").write_text("down")
    monkeypatch.setattr(xr_network, "SYS_NET", tmp_path)
    monkeypatch.setattr(xr_network.socket, "if_nameindex", lambda: list(enumerate(names)))

    def address(name):
        if name == "enp2s0":
            pytest.fail("inactive interfaces must not be offered")
        return {"enp1s0": "192.168.1.10", "wlp2s0": "192.168.2.10"}[name]

    monkeypatch.setattr(xr_network, "_ipv4", address)
    monkeypatch.setattr(xr_network, "_netmask", lambda name: "255.255.255.0")
    assert [entry.interface for entry in xr_network.network_addresses(mode)] == interfaces


def test_the_network_of_each_adapter_comes_from_its_netmask(tmp_path, monkeypatch):
    (tmp_path / "enp1s0").mkdir()
    (tmp_path / "enp1s0/type").write_text("1")
    monkeypatch.setattr(xr_network, "SYS_NET", tmp_path)
    monkeypatch.setattr(xr_network.socket, "if_nameindex", lambda: [(1, "enp1s0")])
    monkeypatch.setattr(xr_network, "_ipv4", lambda name: "192.168.10.101")
    monkeypatch.setattr(xr_network, "_netmask", lambda name: "255.255.255.0")
    assert xr_network.network_addresses()[0].network == "192.168.10.0/24"


def test_the_adapter_on_the_arms_subnet_is_not_offered():
    arm_link = xr_network.NetworkAddress("enp4s0", "192.168.1.100", "ethernet", "192.168.1.0/24")
    lan = xr_network.NetworkAddress("enx1", "192.168.10.101", "ethernet", "192.168.10.0/24")
    unknown = xr_network.NetworkAddress("eth9", "10.0.0.5", "ethernet")  # netmask unreadable: kept
    assert xr_network.without_arm_link([arm_link, lan, unknown], "192.168.1.190") == [lan, unknown]
    assert xr_network.without_arm_link([arm_link, lan], "not an address") == [arm_link, lan]


def test_arm_address_comes_from_the_config_or_the_default():
    assert xr_network.arm_ip({}) == xr_network.DEFAULT_ARM_IP == "192.168.1.190"
    assert xr_network.arm_ip({"hardware": {"device_ip": "10.1.1.2"}}) == "10.1.1.2"
