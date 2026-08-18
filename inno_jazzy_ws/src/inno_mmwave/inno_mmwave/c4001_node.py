"""ROS 2 UART driver for the DFRobot C4001 SEN0610 mmWave sensor."""

from dataclasses import dataclass, fields
import re
import time
from typing import Mapping, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import serial
from std_msgs.msg import Bool, Float32, String, UInt32

from .c4001_config import (
    C4001DesiredConfig,
    CommandTiming,
    build_official_configuration_plan,
)
from .c4001_protocol import (
    C4001Measurement,
    C4001StreamParser,
    encode_command,
)
from .signal_filter import (
    NO_TARGET,
    FilteredTarget,
    SignalFilterConfig,
    SingleTargetSignalFilter,
)


TelemetryValues = Tuple[bool, float, float, int]
FilteredTelemetryValues = Tuple[bool, float, float]


FILTER_PARAMETER_DEFAULTS = {
    f'filter_{item.name}': item.default
    for item in fields(SignalFilterConfig)
}


def measurement_to_telemetry(measurement: C4001Measurement) -> TelemetryValues:
    """Convert optional protocol fields to the stable ROS topic contract."""

    if not measurement.detected:
        return False, 0.0, 0.0, 0
    assert measurement.distance_m is not None
    assert measurement.speed_mps is not None
    assert measurement.energy is not None
    return (
        True,
        float(measurement.distance_m),
        float(measurement.speed_mps),
        int(measurement.energy),
    )


def filtered_to_telemetry(
    target: FilteredTarget,
) -> FilteredTelemetryValues:
    """Convert one robust target to the ROS value/alias contract."""

    return (
        bool(target.presence),
        float(target.distance_m),
        float(target.speed_mps),
    )


def signal_filter_config_from_parameters(
    values: Mapping[str, object],
) -> SignalFilterConfig:
    """Build and validate filter configuration from prefixed ROS values."""

    kwargs = {
        item.name: values[f'filter_{item.name}']
        for item in fields(SignalFilterConfig)
    }
    config = SignalFilterConfig(**kwargs)
    config.validate()
    return config


@dataclass(frozen=True)
class TelemetryPublication:
    """Values emitted for a real frame or a filtered-only heartbeat."""

    raw: Optional[TelemetryValues]
    filtered: FilteredTarget


class C4001TelemetryPipeline:
    """Own raw-to-filtered processing independently of ROS publishers."""

    def __init__(self, config: SignalFilterConfig, timestamp: float) -> None:
        self.filter = SingleTargetSignalFilter(config)
        self._last_raw_timestamp: Optional[float] = None
        self.latest_filtered = self._empty_target(timestamp, 'startup')

    @staticmethod
    def _empty_target(timestamp: float, reason: str) -> FilteredTarget:
        return FilteredTarget(
            timestamp=float(timestamp),
            presence=False,
            distance_m=0.0,
            speed_mps=0.0,
            sample_accepted=False,
            tracking_state=NO_TARGET,
            reason=reason,
        )

    def handle_measurement(
        self, measurement: C4001Measurement, timestamp: float
    ) -> TelemetryPublication:
        raw = measurement_to_telemetry(measurement)
        presence, distance_m, speed_mps, energy = raw
        self.latest_filtered = self.filter.update(
            timestamp=float(timestamp),
            detected=presence,
            distance_m=distance_m,
            speed_mps=speed_mps,
            energy=energy,
        )
        self._last_raw_timestamp = float(timestamp)
        return TelemetryPublication(raw=raw, filtered=self.latest_filtered)

    def heartbeat(self, timestamp: float) -> TelemetryPublication:
        """Return filtered state only; never fabricate/replay a raw frame."""

        now = float(timestamp)
        if (
            self.latest_filtered.presence
            and self._last_raw_timestamp is not None
            and now - self._last_raw_timestamp
            >= self.filter.config.presence_hold_sec
        ):
            self.latest_filtered = self.filter.advance(now)
        return TelemetryPublication(raw=None, filtered=self.latest_filtered)

    def reset(self, timestamp: float, reason: str) -> TelemetryPublication:
        self.filter.reset()
        self._last_raw_timestamp = None
        self.latest_filtered = self._empty_target(timestamp, reason)
        return TelemetryPublication(raw=None, filtered=self.latest_filtered)


def sanitise_error_reason(reason: str) -> str:
    """Create a one-line state suffix safe for ``ERROR:<reason>``."""

    cleaned = re.sub(r'[^A-Z0-9_]+', '_', str(reason).strip().upper()).strip('_')
    return cleaned[:64] or 'UNKNOWN'


class C4001Node(Node):
    """Configure SPEED_MODE + micro-motion and publish active UART reports."""

    def __init__(self) -> None:
        super().__init__('c4001_node')
        defaults = {
            'serial_port': '/dev/ttyAMA0',
            'baud_rate': 9600,
            'serial_timeout_sec': 0.0,
            'write_timeout_sec': 0.5,
            'serial_exclusive': True,
            'configure_on_start': True,
            'send_start_on_connect': True,
            'micro_motion_enabled': True,
            'min_range_m': 1.2,
            'max_range_m': 12.0,
            'detection_threshold': 10,
            'sensor_warmup_sec': 1.0,
            'command_interval_sec': 0.15,
            'stop_settle_sec': 1.0,
            'save_settle_sec': 0.80,
            'poll_rate_hz': 50.0,
            'stale_timeout_sec': 1.5,
            'reconnect_interval_sec': 2.0,
            'read_chunk_size': 512,
            'max_frame_bytes': 256,
            'publish_heartbeat_sec': 0.5,
        }
        defaults.update(FILTER_PARAMETER_DEFAULTS)
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.serial_timeout_sec = float(
            self.get_parameter('serial_timeout_sec').value
        )
        self.write_timeout_sec = float(
            self.get_parameter('write_timeout_sec').value
        )
        self.serial_exclusive = bool(
            self.get_parameter('serial_exclusive').value
        )
        self.configure_on_start = bool(
            self.get_parameter('configure_on_start').value
        )
        self.send_start_on_connect = bool(
            self.get_parameter('send_start_on_connect').value
        )
        self.micro_motion_enabled = bool(
            self.get_parameter('micro_motion_enabled').value
        )
        self.min_range_m = float(self.get_parameter('min_range_m').value)
        self.max_range_m = float(self.get_parameter('max_range_m').value)
        self.detection_threshold = int(
            self.get_parameter('detection_threshold').value
        )
        self.sensor_warmup_sec = float(
            self.get_parameter('sensor_warmup_sec').value
        )
        self.command_interval_sec = float(
            self.get_parameter('command_interval_sec').value
        )
        self.stop_settle_sec = float(
            self.get_parameter('stop_settle_sec').value
        )
        self.save_settle_sec = float(
            self.get_parameter('save_settle_sec').value
        )
        self.poll_rate_hz = float(self.get_parameter('poll_rate_hz').value)
        self.stale_timeout_sec = float(
            self.get_parameter('stale_timeout_sec').value
        )
        self.reconnect_interval_sec = float(
            self.get_parameter('reconnect_interval_sec').value
        )
        self.read_chunk_size = int(self.get_parameter('read_chunk_size').value)
        self.max_frame_bytes = int(self.get_parameter('max_frame_bytes').value)
        self.publish_heartbeat_sec = float(
            self.get_parameter('publish_heartbeat_sec').value
        )
        filter_values = {
            name: self.get_parameter(name).value
            for name in FILTER_PARAMETER_DEFAULTS
        }
        self.filter_config = signal_filter_config_from_parameters(
            filter_values
        )
        self._validate_parameters()

        desired_config = C4001DesiredConfig(
            min_range_m=self.min_range_m,
            max_range_m=self.max_range_m,
            threshold_factor=self.detection_threshold,
            micro_motion_enabled=self.micro_motion_enabled,
        )
        command_timing = CommandTiming(
            command_settle_sec=self.command_interval_sec,
            stop_settle_sec=self.stop_settle_sec,
            save_settle_sec=self.save_settle_sec,
            start_settle_sec=self.command_interval_sec,
        )
        # Preserve the official mode, range/threshold, and micro-motion
        # stop/save/start cycles instead of bundling all settings together.
        self._full_configuration = [
            step.command for step in build_official_configuration_plan(
                desired_config, command_timing
            ).steps
        ]

        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.raw_presence_publisher = self.create_publisher(
            Bool, '/mmwave/raw/presence', latched_qos
        )
        self.raw_distance_publisher = self.create_publisher(
            Float32, '/mmwave/raw/distance_m', latched_qos
        )
        self.raw_speed_publisher = self.create_publisher(
            Float32, '/mmwave/raw/speed_mps', latched_qos
        )
        self.raw_energy_publisher = self.create_publisher(
            UInt32, '/mmwave/raw/energy_raw', latched_qos
        )
        self.filtered_presence_publisher = self.create_publisher(
            Bool, '/mmwave/filtered_presence', latched_qos
        )
        self.filtered_distance_publisher = self.create_publisher(
            Float32, '/mmwave/filtered_distance_m', latched_qos
        )
        self.filtered_speed_publisher = self.create_publisher(
            Float32, '/mmwave/filtered_speed_mps', latched_qos
        )
        self.motion_activity_publisher = self.create_publisher(
            Float32, '/mmwave/motion_activity', latched_qos
        )
        self.filter_state_publisher = self.create_publisher(
            String, '/mmwave/filter_state', latched_qos
        )
        # Backward-compatible aliases now carry the robust values.
        self.presence_publisher = self.create_publisher(
            Bool, '/mmwave/presence', latched_qos
        )
        self.distance_publisher = self.create_publisher(
            Float32, '/mmwave/distance_m', latched_qos
        )
        self.speed_publisher = self.create_publisher(
            Float32, '/mmwave/speed_mps', latched_qos
        )
        self.energy_publisher = self.create_publisher(
            UInt32, '/mmwave/energy_raw', latched_qos
        )
        self.sensor_state_publisher = self.create_publisher(
            String, '/mmwave/sensor_state', latched_qos
        )

        self._serial: Optional[serial.Serial] = None
        self._parser = C4001StreamParser(self.max_frame_bytes)
        self._sensor_state = 'CONNECTING'
        initial_now = time.monotonic()
        self._pipeline = C4001TelemetryPipeline(
            self.filter_config, initial_now
        )
        self._last_publish_monotonic = 0.0
        self._last_frame_monotonic: Optional[float] = None
        self._data_wait_started: Optional[float] = None
        self._next_connect_monotonic = time.monotonic()
        self._initialization_complete = False
        self._has_configured_once = False
        self._active_sequence_is_full_configuration = False
        self._commands = []
        self._command_index = 0
        self._next_command_monotonic = 0.0

        self._publish_sensor_state()
        self._publish_filtered_telemetry(self._pipeline.latest_filtered)
        self.create_timer(1.0 / self.poll_rate_hz, self._poll)

    def _validate_parameters(self) -> None:
        if not self.serial_port:
            raise ValueError('serial_port must not be empty')
        if self.baud_rate <= 0:
            raise ValueError('baud_rate must be positive')
        if self.serial_timeout_sec < 0.0 or self.write_timeout_sec <= 0.0:
            raise ValueError('serial timeouts are invalid')
        non_negative = (
            self.sensor_warmup_sec,
            self.command_interval_sec,
            self.stop_settle_sec,
            self.save_settle_sec,
        )
        if any(value < 0.0 for value in non_negative):
            raise ValueError('sensor command timing must not be negative')
        if self.poll_rate_hz <= 0.0:
            raise ValueError('poll_rate_hz must be positive')
        if self.stale_timeout_sec <= 0.0:
            raise ValueError('stale_timeout_sec must be positive')
        if self.reconnect_interval_sec <= 0.0:
            raise ValueError('reconnect_interval_sec must be positive')
        if self.read_chunk_size <= 0:
            raise ValueError('read_chunk_size must be positive')
        if self.max_frame_bytes < 16:
            raise ValueError('max_frame_bytes is too small')
        if self.publish_heartbeat_sec <= 0.0:
            raise ValueError('publish_heartbeat_sec must be positive')

    def _publish_sensor_state(self) -> None:
        self.sensor_state_publisher.publish(String(data=self._sensor_state))

    def _set_sensor_state(self, state: str) -> None:
        if state == self._sensor_state:
            return
        self._sensor_state = state
        self._publish_sensor_state()
        if state == 'ONLINE':
            self.get_logger().info('C4001 sensor ONLINE')
        elif state == 'OFFLINE':
            self.get_logger().warning('C4001 stream is stale (OFFLINE)')

    def _publish_raw_telemetry(self, telemetry: TelemetryValues) -> None:
        present, distance_m, speed_mps, energy = telemetry
        # Raw diagnostics are emitted exactly once per physical UART frame.
        self.raw_distance_publisher.publish(Float32(data=distance_m))
        self.raw_speed_publisher.publish(Float32(data=speed_mps))
        self.raw_energy_publisher.publish(UInt32(data=energy))
        self.energy_publisher.publish(UInt32(data=energy))
        self.raw_presence_publisher.publish(Bool(data=present))

    def _publish_filtered_telemetry(self, target: FilteredTarget) -> None:
        present, distance_m, speed_mps = filtered_to_telemetry(target)
        # Measurements precede presence in both the robust and legacy
        # contracts, so edge-triggered consumers see a matching sample.
        self.filtered_distance_publisher.publish(Float32(data=distance_m))
        self.distance_publisher.publish(Float32(data=distance_m))
        self.filtered_speed_publisher.publish(Float32(data=speed_mps))
        self.speed_publisher.publish(Float32(data=speed_mps))
        self.motion_activity_publisher.publish(
            Float32(data=float(target.activity_percent))
        )
        self.filter_state_publisher.publish(
            String(data=target.tracking_state)
        )
        self.filtered_presence_publisher.publish(Bool(data=present))
        self.presence_publisher.publish(Bool(data=present))
        self._last_publish_monotonic = time.monotonic()

    def _reset_filter_and_publish(self, now: float, reason: str) -> None:
        publication = self._pipeline.reset(now, reason)
        self._publish_filtered_telemetry(publication.filtered)

    def _open_serial(self, now: float) -> None:
        self._set_sensor_state('CONNECTING')
        try:
            port = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.serial_timeout_sec,
                write_timeout=self.write_timeout_sec,
                exclusive=self.serial_exclusive,
            )
            port.reset_input_buffer()
        except (serial.SerialException, OSError, ValueError) as error:
            self.get_logger().error(
                f'Cannot open C4001 serial port {self.serial_port}: {error}'
            )
            self._set_sensor_state('ERROR:PORT_OPEN_FAILED')
            self._next_connect_monotonic = now + self.reconnect_interval_sec
            return

        self._serial = port
        self._parser.reset()
        self._reset_filter_and_publish(now, 'serial_open')
        self._initialization_complete = False
        self._last_frame_monotonic = None
        self._data_wait_started = None
        self._command_index = 0
        if self.configure_on_start and not self._has_configured_once:
            self._commands = list(self._full_configuration)
            self._active_sequence_is_full_configuration = True
        elif self.send_start_on_connect:
            self._commands = ['sensorStart']
            self._active_sequence_is_full_configuration = False
        else:
            self._commands = []
            self._active_sequence_is_full_configuration = False
        self._next_command_monotonic = now + self.sensor_warmup_sec
        self.get_logger().info(
            f'Opened C4001 UART {self.serial_port} at {self.baud_rate} baud'
        )

    def _finish_initialization(self, now: float) -> None:
        if self._active_sequence_is_full_configuration:
            self._has_configured_once = True
        self._initialization_complete = True
        self._last_frame_monotonic = None
        self._data_wait_started = now
        self._parser.reset()
        self._set_sensor_state('CONNECTING')
        self.get_logger().info(
            'C4001 SPEED_MODE initialization complete; waiting for $DFDMD frames'
        )

    def _drive_initialization(self, now: float) -> None:
        if self._initialization_complete or now < self._next_command_monotonic:
            return
        if self._command_index >= len(self._commands):
            self._finish_initialization(now)
            return
        assert self._serial is not None

        command = self._commands[self._command_index]
        if command == 'sensorStart':
            # Discard stopped-mode replies and any old frame before starting
            # the new speed-mode stream.
            self._serial.reset_input_buffer()
            self._parser.reset()
        self._serial.write(encode_command(command))
        self._serial.flush()
        self._command_index += 1

        delay = self.command_interval_sec
        if command == 'sensorStop':
            delay = max(delay, self.stop_settle_sec)
        elif command == 'saveConfig':
            delay = max(delay, self.save_settle_sec)
        self._next_command_monotonic = now + delay

        if self._command_index >= len(self._commands):
            self._finish_initialization(now)

    def _read_available(self, now: float) -> None:
        assert self._serial is not None
        reads_remaining = 8
        while reads_remaining > 0:
            available = int(self._serial.in_waiting)
            if available <= 0:
                return
            chunk = self._serial.read(min(available, self.read_chunk_size))
            if not chunk:
                return
            malformed_before = self._parser.malformed_frames
            measurements = self._parser.feed(chunk)
            if self._initialization_complete:
                for measurement in measurements:
                    self._handle_measurement(measurement, now)
                if self._parser.malformed_frames > malformed_before:
                    self.get_logger().debug('Ignored malformed C4001 UART frame')
            reads_remaining -= 1

    def _handle_measurement(
        self, measurement: C4001Measurement, now: float
    ) -> None:
        self._last_frame_monotonic = now
        self._set_sensor_state('ONLINE')
        publication = self._pipeline.handle_measurement(measurement, now)
        assert publication.raw is not None
        self._publish_raw_telemetry(publication.raw)
        self._publish_filtered_telemetry(publication.filtered)

    def _update_health(self, now: float) -> None:
        if not self._initialization_complete:
            self._set_sensor_state('CONNECTING')
            return
        reference = self._last_frame_monotonic
        if reference is None:
            reference = self._data_wait_started
        if reference is None or now - reference <= self.stale_timeout_sec:
            return
        if self._sensor_state == 'OFFLINE':
            return
        self._reset_filter_and_publish(now, 'sensor_stale')
        self._set_sensor_state('OFFLINE')
        # A stale but still-open tty does not raise SerialException. Close it
        # explicitly so the normal reconnect path reopens UART and sends
        # sensorStart again. This repeats for the lifetime of the mission.
        self._close_serial()
        self._initialization_complete = False
        self._last_frame_monotonic = None
        self._data_wait_started = None
        self._next_connect_monotonic = now + self.reconnect_interval_sec

    def _close_serial(self) -> None:
        port = self._serial
        self._serial = None
        if port is None:
            return
        try:
            port.close()
        except (serial.SerialException, OSError):
            pass

    def _handle_serial_failure(self, now: float, reason: str, error: Exception) -> None:
        self.get_logger().error(f'C4001 serial I/O failure: {error}')
        self._close_serial()
        self._initialization_complete = False
        self._set_sensor_state(f'ERROR:{sanitise_error_reason(reason)}')
        self._reset_filter_and_publish(now, 'serial_failure')
        self._next_connect_monotonic = now + self.reconnect_interval_sec

    def _maybe_publish_heartbeat(self, now: float) -> None:
        if now - self._last_publish_monotonic < self.publish_heartbeat_sec:
            return
        publication = self._pipeline.heartbeat(now)
        self._publish_sensor_state()
        self._publish_filtered_telemetry(publication.filtered)

    def _poll(self) -> None:
        now = time.monotonic()
        if self._serial is None:
            if now >= self._next_connect_monotonic:
                self._open_serial(now)
            self._maybe_publish_heartbeat(now)
            return
        try:
            self._drive_initialization(now)
            self._read_available(now)
            self._update_health(now)
        except (serial.SerialException, OSError) as error:
            self._handle_serial_failure(now, 'SERIAL_IO', error)
        self._maybe_publish_heartbeat(now)

    def destroy_node(self):
        self._close_serial()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = C4001Node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        if node is None:
            print(f'c4001_node: {error}', flush=True)
        else:
            node.get_logger().error(str(error))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
