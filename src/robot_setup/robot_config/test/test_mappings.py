"""Which config fields make up a device mapping, and the fingerprints computed from them.

The hand fingerprints below were taken from the code before the field lists moved into this package; the camera ones were
renewed when white_balance, ae_max_exposure, exposure and gain moved into each camera (one confirmation of the cameras on
every machine). They must not change otherwise: a changed fingerprint makes every operator confirm the cameras / hands again."""
import subprocess
import sys

import pytest

from robot_config import config_edit, mappings

# case -> (cameras fingerprint, hand fingerprint)
GOLDEN = {
    "defaults": ("d6c2e06cfa3e40a3070e7c2a5b9a09e75d0e25683ff3bb741e08a316fd99a0d4",
                 "1928ab2a4fd5e64b537e42a4f44f80804983fc2fdc29c9839219f312491223f9"),
    "usb_xhand": ("d6c2e06cfa3e40a3070e7c2a5b9a09e75d0e25683ff3bb741e08a316fd99a0d4",
                  "908a13c12b98ec893854b4131db710ea767d1ab464e35d20efde35f06db4d264"),
    "usb_can": ("d6c2e06cfa3e40a3070e7c2a5b9a09e75d0e25683ff3bb741e08a316fd99a0d4",
                "79d79d838d96cc514acf6082b2f9ef5125a61c919838bfd505beecf39a5819b7"),
    "ethernet": ("d6c2e06cfa3e40a3070e7c2a5b9a09e75d0e25683ff3bb741e08a316fd99a0d4",
                 "ff9eb6adf2e4c9db39db31225491a85a8e54324e1911b0c6dd685b0251ffda70"),
    "arm_rs485": ("d6c2e06cfa3e40a3070e7c2a5b9a09e75d0e25683ff3bb741e08a316fd99a0d4",
                  "01b1b3f08728216cef1ad49d6a67a8fd9ad2e3032d7f67171a58f819d8f29d18"),
    "none": ("d6c2e06cfa3e40a3070e7c2a5b9a09e75d0e25683ff3bb741e08a316fd99a0d4",
             "e4fa57b6493dadb55edefacfa2b00414cdbff223bd8367e9aa7326655e6b0eba"),
    "cam_changed": ("13a6144735d9c719cbd6fb451786b216645278ff68b9a1be5f1a1e84a1e72d7b",
                    "1928ab2a4fd5e64b537e42a4f44f80804983fc2fdc29c9839219f312491223f9"),
}
CASES = {
    "defaults": {},
    "usb_xhand": {"robot": {"end_effector": {"type": "xhand1", "interface": {
        "bus": "usb_rs485", "left": {"port": "/dev/serial/by-id/usb-A"}, "right": {"port": "/dev/serial/by-id/usb-B"}}}}},
    "usb_can": {"robot": {"end_effector": {"type": "linkerhand_l6", "interface": {
        "bus": "usb_can", "left": {"can": "can3"}}}}},
    "ethernet": {"robot": {"end_effector": {"type": "wuji_hand2", "interface": {"bus": "ethernet"}}}},
    "arm_rs485": {"robot": {"end_effector": {"type": "jodell_rg75", "interface": {
        "bus": "arm_rs485", "left": {"channel": 2}}}, "arm": {"interface": {"ip": "10.0.0.7"}}}},
    "none": {"robot": {"end_effector": {"type": "none"}}},
    "cam_changed": {"cameras": {"enable": True, "left": {"serial": "CAM123", "rotation": 90}}},
}


@pytest.mark.parametrize("case", sorted(CASES))
def test_fingerprints_are_the_same_as_before_the_fields_moved(case):
    cameras, hand = GOLDEN[case]
    assert config_edit.fingerprint(CASES[case], "cameras") == cameras
    assert config_edit.fingerprint(CASES[case], "ee") == hand


def test_a_camera_field_that_changes_changes_the_camera_fingerprint_only():
    base, changed = CASES["defaults"], CASES["cam_changed"]
    assert config_edit.fingerprint(base, "cameras") != config_edit.fingerprint(changed, "cameras")
    assert config_edit.fingerprint(base, "ee") == config_edit.fingerprint(changed, "ee")


@pytest.mark.parametrize("bus, expected_keys", [
    ("usb_rs485", {"usb_left_port", "usb_right_port"}),
    ("usb_can", {"left_ee_can_interface", "right_ee_can_interface"}),
    ("ethernet", set()),  # Wuji: the driver scans and matches each hand by hand_side
    ("arm_rs485", {"device_ip", "left_ee_channel", "right_ee_channel"}),
])
def test_the_hand_mapping_holds_the_fields_its_wiring_uses(bus, expected_keys):
    value = mappings.ee_mapping({"robot": {"end_effector": {"type": "linkerhand_l6", "interface": {"bus": bus}}}})
    assert set(value) - {"hand", "drive"} == expected_keys


def test_no_end_effector_has_no_wiring_fields():
    assert mappings.ee_mapping({"robot": {"end_effector": {"type": "none"}}}) == {"hand": "none", "drive": "usb"}


def test_an_unknown_section_is_an_error_naming_the_known_ones():
    with pytest.raises(KeyError) as raised:
        config_edit.fingerprint({}, "dyn")
    assert "cameras" in str(raised.value) and "ee" in str(raised.value)


def test_status_and_check_work_without_importing_the_device_packages():
    """The shared package must not need camera / end_effector for fingerprints (they depend on it, not the reverse)."""
    code = (
        "import sys\n"
        "from robot_config import config_status\n"
        "raw = {'robot': {'end_effector': {'type': 'xhand1', 'interface': {'bus': 'usb_rs485'}}}}\n"
        "print(config_status.camera_status(raw).state, config_status.ee_status(raw).state)\n"
        "loaded = [m for m in ('camera', 'end_effector') if m in sys.modules]\n"
        "assert not loaded, loaded\n")
    result = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.split() == ["需配置", "需配置"]
