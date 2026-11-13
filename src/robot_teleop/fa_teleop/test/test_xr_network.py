"""Wired/Wi-Fi address selection must work without iproute2 or sending network traffic."""
import pytest

from fa_teleop_runtime import xr_network


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
    assert [entry.interface for entry in xr_network.network_addresses(mode)] == interfaces
