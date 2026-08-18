import pytest


pytest.importorskip('rclpy')
pytest.importorskip('serial')

from inno_mmwave.c4001_node import (  # noqa: E402
    FILTER_PARAMETER_DEFAULTS,
    C4001Node,
    C4001TelemetryPipeline,
    filtered_to_telemetry,
    measurement_to_telemetry,
    sanitise_error_reason,
    signal_filter_config_from_parameters,
)
from inno_mmwave.c4001_protocol import C4001Measurement  # noqa: E402
from inno_mmwave.signal_filter import HOLDING, TRACKING  # noqa: E402


def test_measurement_to_telemetry_orders_ros_values():
    measurement = C4001Measurement(
        target_count=1,
        target_id=1,
        distance_m=2.75,
        speed_mps=-0.2,
        energy=4321,
        raw_frame='$DFDMD,1,1,2.75,-0.2,4321, , *',
    )
    assert measurement_to_telemetry(measurement) == (
        True,
        2.75,
        -0.2,
        4321,
    )


def test_no_target_converts_to_explicit_zero_values():
    measurement = C4001Measurement(
        target_count=0,
        target_id=None,
        distance_m=None,
        speed_mps=None,
        energy=None,
        raw_frame='$DFDMD,0, , , , , , *',
    )
    assert measurement_to_telemetry(measurement) == (False, 0.0, 0.0, 0)


def test_error_reason_is_one_line_and_bounded():
    assert sanitise_error_reason(' serial I/O\nfailed ') == 'SERIAL_I_O_FAILED'
    assert len(sanitise_error_reason('x' * 200)) == 64


def target_measurement(distance=2.0, speed=0.0, energy=100):
    return C4001Measurement(
        target_count=1,
        target_id=1,
        distance_m=distance,
        speed_mps=speed,
        energy=energy,
        raw_frame=f'$DFDMD,1,1,{distance},{speed},{energy}, , *',
    )


def no_target_measurement():
    return C4001Measurement(
        target_count=0,
        target_id=None,
        distance_m=None,
        speed_mps=None,
        energy=None,
        raw_frame='$DFDMD,0, , , , , , *',
    )


def default_pipeline():
    config = signal_filter_config_from_parameters(FILTER_PARAMETER_DEFAULTS)
    return C4001TelemetryPipeline(config, timestamp=0.0)


def acquire_pipeline_target(pipeline):
    result = None
    distances = (1.99, 2.01, 2.0, 2.02, 1.98, 2.01, 2.0)
    for index, distance in enumerate(distances):
        result = pipeline.handle_measurement(
            target_measurement(distance, speed=0.0, energy=100 + index),
            index * 0.1,
        )
    assert result is not None and result.filtered.presence
    return result


def test_pipeline_preserves_raw_frame_but_rejects_distance_jump():
    pipeline = default_pipeline()
    acquired = acquire_pipeline_target(pipeline)
    assert acquired.raw == (True, 2.0, 0.0, 106)
    assert acquired.filtered.tracking_state == TRACKING
    assert acquired.filtered.speed_mps == 0.0

    jumped = pipeline.handle_measurement(
        target_measurement(10.0, speed=-7.0, energy=999),
        timestamp=0.7,
    )
    assert jumped.raw == (True, 10.0, -7.0, 999)
    assert jumped.filtered.presence
    assert jumped.filtered.tracking_state == HOLDING
    assert jumped.filtered.distance_m == pytest.approx(2.0, abs=0.05)
    assert jumped.filtered.speed_mps == 0.0


def test_zero_target_frames_are_owned_by_filter_hold_and_expiry():
    pipeline = default_pipeline()
    acquire_pipeline_target(pipeline)

    briefly_missing = pipeline.handle_measurement(
        no_target_measurement(), timestamp=0.7
    )
    assert briefly_missing.raw == (False, 0.0, 0.0, 0)
    assert briefly_missing.filtered.presence
    assert briefly_missing.filtered.tracking_state == HOLDING

    expired = pipeline.handle_measurement(
        no_target_measurement(), timestamp=3.7
    )
    assert expired.raw == (False, 0.0, 0.0, 0)
    assert not expired.filtered.presence
    assert expired.filtered.distance_m == 0.0


def test_heartbeat_never_republishes_raw_and_expires_silent_track():
    pipeline = default_pipeline()
    acquire_pipeline_target(pipeline)

    held = pipeline.heartbeat(1.0)
    assert held.raw is None
    assert held.filtered.presence

    expired = pipeline.heartbeat(3.7)
    assert expired.raw is None
    assert not expired.filtered.presence


def test_pipeline_reset_clears_filter_without_fabricating_raw_frame():
    pipeline = default_pipeline()
    acquire_pipeline_target(pipeline)
    reset = pipeline.reset(0.7, 'serial_failure')
    assert reset.raw is None
    assert not reset.filtered.presence
    assert reset.filtered.reason == 'serial_failure'


class Recorder:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def publish(self, message):
        self.events.append((self.name, message.data))


def test_filtered_publish_uses_identical_legacy_aliases_and_order():
    pipeline = default_pipeline()
    target = acquire_pipeline_target(pipeline).filtered
    assert filtered_to_telemetry(target)[0]

    events = []
    fake_node = type('FakeNode', (), {})()
    for name in (
        'filtered_distance_publisher',
        'distance_publisher',
        'filtered_speed_publisher',
        'speed_publisher',
        'motion_activity_publisher',
        'filter_state_publisher',
        'filtered_presence_publisher',
        'presence_publisher',
    ):
        setattr(fake_node, name, Recorder(name, events))

    C4001Node._publish_filtered_telemetry(fake_node, target)
    values = dict(events)
    assert values['filtered_distance_publisher'] == values['distance_publisher']
    assert values['filtered_speed_publisher'] == values['speed_publisher']
    assert values['motion_activity_publisher'] == target.activity_percent
    assert values['filtered_presence_publisher'] == values['presence_publisher']
    first_presence = min(
        index for index, event in enumerate(events) if 'presence' in event[0]
    )
    assert all(
        index < first_presence
        for index, event in enumerate(events)
        if 'distance' in event[0] or 'speed' in event[0]
    )


def test_all_signal_filter_defaults_are_exposed_as_prefixed_parameters():
    config = signal_filter_config_from_parameters(FILTER_PARAMETER_DEFAULTS)
    assert config.min_distance_m == 1.2
    assert config.max_distance_m == 12.0
    assert config.motion_start_mps == 0.10


def test_stale_stream_closes_uart_and_schedules_restart():
    events = []
    fake_node = type('FakeNode', (), {})()
    fake_node._initialization_complete = True
    fake_node._last_frame_monotonic = 10.0
    fake_node._data_wait_started = None
    fake_node.stale_timeout_sec = 1.5
    fake_node.reconnect_interval_sec = 2.0
    fake_node._sensor_state = 'ONLINE'
    fake_node._reset_filter_and_publish = (
        lambda now, reason: events.append(('reset', now, reason))
    )
    fake_node._set_sensor_state = (
        lambda state: events.append(('state', state))
    )
    fake_node._close_serial = lambda: events.append(('close',))

    C4001Node._update_health(fake_node, 12.0)

    assert events == [
        ('reset', 12.0, 'sensor_stale'),
        ('state', 'OFFLINE'),
        ('close',),
    ]
    assert not fake_node._initialization_complete
    assert fake_node._last_frame_monotonic is None
    assert fake_node._data_wait_started is None
    assert fake_node._next_connect_monotonic == 14.0
