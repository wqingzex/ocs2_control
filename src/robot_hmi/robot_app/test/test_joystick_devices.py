"""Gamepad discovery: the enumerate output is parsed into id / name pairs."""
from robot_app import joystick_devices


def test_joystick_devices_are_parsed_from_enumerate_output():
    text = "  0 : Xbox Controller : true : false : evdev\n  2 : Logitech F310 : true : false : evdev\nnoise\n"
    assert joystick_devices.parse_joysticks(text) == [{"id": 0, "name": "Xbox Controller"},
                                                      {"id": 2, "name": "Logitech F310"}]
