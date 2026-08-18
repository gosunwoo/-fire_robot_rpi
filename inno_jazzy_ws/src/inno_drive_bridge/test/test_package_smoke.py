import pytest

from inno_drive_bridge.cmd_vel_mode_mux import (
    command_source_for_mode,
    inspection_stops_output,
)
from inno_drive_bridge.cmdvel_to_esp32_serial import CmdVelToEsp32Serial
from inno_drive_bridge.keyboard_cmdvel_demo import (
    waypoint_command_on_mode_select,
)


def test_drive_package_imports():
    import inno_drive_bridge.cmd_vel_mode_mux  # noqa: F401
    import inno_drive_bridge.cmdvel_to_esp32_serial  # noqa: F401


def test_mode_two_and_three_share_only_the_autonomous_input():
    assert command_source_for_mode(1) == 1
    assert command_source_for_mode(2) == 2
    assert command_source_for_mode(3) == 2
    with pytest.raises(ValueError, match='1, 2, or 3'):
        command_source_for_mode(4)


def test_inspection_hold_only_stops_mode_three():
    assert not inspection_stops_output(1, True)
    assert not inspection_stops_output(2, True)
    assert inspection_stops_output(3, True)
    assert not inspection_stops_output(3, False)


def test_repeated_identical_esp32_error_is_throttled(monkeypatch):
    bridge = object.__new__(CmdVelToEsp32Serial)
    bridge.repeat_error_interval = 5.0
    bridge._last_error_line = None
    bridge._last_error_time = float("-inf")
    statuses = []
    errors = []
    bridge._publish_status = statuses.append
    bridge.get_logger = lambda: type(
        "Logger", (), {"error": lambda _self, text: errors.append(text)}
    )()
    timestamps = iter((0.0, 1.0, 5.0))
    monkeypatch.setattr(
        "inno_drive_bridge.cmdvel_to_esp32_serial.time.monotonic",
        lambda: next(timestamps),
    )

    line = "ERR,ENCODER_NOT_READY,10,10"
    bridge._parse_line(line)
    bridge._parse_line(line)
    bridge._parse_line(line)

    assert statuses == [line, line]
    assert errors == [f"ESP32: {line}", f"ESP32: {line}"]


def test_mode_two_starts_continuous_queue_automatically():
    assert waypoint_command_on_mode_select(1) is None
    assert waypoint_command_on_mode_select(2) == 'GO'
    assert waypoint_command_on_mode_select(3) is None
    with pytest.raises(ValueError, match='1, 2, or 3'):
        waypoint_command_on_mode_select(0)
