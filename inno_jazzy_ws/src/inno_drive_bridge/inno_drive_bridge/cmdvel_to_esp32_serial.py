import math
import threading
import time

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Int32, Int64MultiArray, String


class CmdVelToEsp32Serial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_esp32_serial')
        defaults = {
            'serial_port': '/dev/ttyUSB0',
            'baudrate': 115200,
            'wheel_radius': 0.04,
            'wheel_separation': 0.30,
            'motor_full_steps_per_rev': 200,
            'microsteps': 8,
            'gear_ratio': 1.0,
            'max_steps_per_sec': 1600,
            'left_sign': 1,
            'right_sign': 1,
            'cmd_timeout_sec': 0.5,
            'repeat_error_interval_sec': 30.0,
            'ultrasonic_frame': 'ultrasonic_front',
            'ultrasonic_field_of_view_rad': 0.261799,
            'ultrasonic_min_range_m': 0.02,
            'ultrasonic_max_range_m': 4.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value
        )
        self.full_steps = int(
            self.get_parameter('motor_full_steps_per_rev').value
        )
        self.microsteps = int(self.get_parameter('microsteps').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.max_sps = int(self.get_parameter('max_steps_per_sec').value)
        self.left_sign = int(self.get_parameter('left_sign').value)
        self.right_sign = int(self.get_parameter('right_sign').value)
        self.cmd_timeout = float(self.get_parameter('cmd_timeout_sec').value)
        self.repeat_error_interval = float(
            self.get_parameter('repeat_error_interval_sec').value
        )
        self.ultrasonic_frame = str(
            self.get_parameter('ultrasonic_frame').value
        )
        self.ultrasonic_field_of_view = float(
            self.get_parameter('ultrasonic_field_of_view_rad').value
        )
        self.ultrasonic_min_range = float(
            self.get_parameter('ultrasonic_min_range_m').value
        )
        self.ultrasonic_max_range = float(
            self.get_parameter('ultrasonic_max_range_m').value
        )
        self._validate_parameters()

        self.status_publisher = self.create_publisher(
            String, '/esp32/status', 10
        )
        self.ticks_publisher = self.create_publisher(
            Int64MultiArray, '/wheel_ticks', 10
        )
        self.left_motor_publisher = self.create_publisher(
            Int32, '/motor/left_steps_per_sec', 10
        )
        self.right_motor_publisher = self.create_publisher(
            Int32, '/motor/right_steps_per_sec', 10
        )
        self.ultrasonic_publisher = self.create_publisher(
            Range, '/ultrasonic/front/range', 10
        )
        self.create_subscription(Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        self._serial_lock = threading.Lock()
        self._rx_buffer = bytearray()
        self._seq = 0
        self._last_cmd_time = time.monotonic()
        self._timed_out = False
        self._last_error_line = None
        self._last_error_time = float("-inf")

        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baudrate,
                timeout=0.0,
                write_timeout=0.2,
            )
        except (serial.SerialException, OSError) as error:
            raise RuntimeError(
                f'Cannot open ESP32 serial port {self.serial_port} at '
                f'{self.baudrate} baud: {error}. Check the device path and '
                'dialout permission.'
            ) from error

        self._send('STOP')
        self._publish_motor_targets(0, 0)
        self.create_timer(0.01, self._poll_serial)
        self.create_timer(0.05, self._watchdog)
        self.get_logger().info(
            f'ESP32 serial connected: {self.serial_port} @ '
            f'{self.baudrate} baud'
        )

    def _validate_parameters(self):
        if self.baudrate <= 0:
            raise ValueError('baudrate must be greater than zero')
        if self.wheel_radius <= 0.0 or self.wheel_separation <= 0.0:
            raise ValueError(
                'wheel_radius and wheel_separation must be greater than zero'
            )
        if (
            self.full_steps <= 0 or self.microsteps <= 0
            or self.gear_ratio <= 0.0
        ):
            raise ValueError(
                'motor step parameters and gear_ratio must be greater '
                'than zero'
            )
        if (
            self.max_sps <= 0 or self.cmd_timeout <= 0.0
            or self.repeat_error_interval <= 0.0
        ):
            raise ValueError(
                'max_steps_per_sec, cmd_timeout_sec, and '
                'repeat_error_interval_sec must be greater than zero'
            )
        if self.left_sign not in (-1, 1) or self.right_sign not in (-1, 1):
            raise ValueError('left_sign and right_sign must be either -1 or 1')
        if (
            not self.ultrasonic_frame
            or self.ultrasonic_field_of_view <= 0.0
            or self.ultrasonic_min_range <= 0.0
            or self.ultrasonic_max_range <= self.ultrasonic_min_range
        ):
            raise ValueError('ultrasonic frame, FOV, and range limits are invalid')

    def _cmd_vel_callback(self, message):
        self._last_cmd_time = time.monotonic()
        self._timed_out = False
        linear = float(message.linear.x)
        angular = float(message.angular.z)
        left_mps = linear - angular * self.wheel_separation * 0.5
        right_mps = linear + angular * self.wheel_separation * 0.5

        left_sps = self._meters_per_second_to_steps(left_mps, self.left_sign)
        right_sps = self._meters_per_second_to_steps(
            right_mps, self.right_sign
        )
        self._publish_motor_targets(left_sps, right_sps)
        if left_sps == 0 and right_sps == 0:
            self._send('STOP')
        else:
            self._send('M', left_sps, right_sps)

    def _meters_per_second_to_steps(self, speed, direction_sign):
        steps_per_rev = self.full_steps * self.microsteps * self.gear_ratio
        steps = round(
            speed * steps_per_rev / (2.0 * math.pi * self.wheel_radius)
        )
        return max(-self.max_sps, min(self.max_sps, steps * direction_sign))

    def _publish_motor_targets(self, left_sps, right_sps):
        """Mirror clamped UART motor targets onto separate ROS topics."""

        self.left_motor_publisher.publish(Int32(data=int(left_sps)))
        self.right_motor_publisher.publish(Int32(data=int(right_sps)))

    def _next_seq(self):
        self._seq = (self._seq + 1) % 2147483647
        return self._seq

    def _send(self, command, *fields):
        if not hasattr(self, 'serial') or not self.serial.is_open:
            return
        seq = self._next_seq()
        line = ','.join(
            [command, str(seq), *(str(field) for field in fields)]
        ) + '\n'
        try:
            with self._serial_lock:
                self.serial.write(line.encode('ascii'))
        except (
            serial.SerialException, serial.SerialTimeoutException, OSError
        ) as error:
            self.get_logger().error(f'Serial write failed: {error}')
            self._publish_status(f'ERR,serial_write,{error}')

    def _watchdog(self):
        if (
            not self._timed_out
            and time.monotonic() - self._last_cmd_time > self.cmd_timeout
        ):
            self._timed_out = True
            self._send('STOP')
            self._publish_motor_targets(0, 0)
            self.get_logger().warning(
                f'/cmd_vel timeout ({self.cmd_timeout:.3f} s): '
                'STOP sent to ESP32'
            )

    def _poll_serial(self):
        try:
            waiting = self.serial.in_waiting
            if waiting:
                self._rx_buffer.extend(self.serial.read(waiting))
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f'Serial read failed: {error}')
            self._publish_status(f'ERR,serial_read,{error}')
            return

        while b'\n' in self._rx_buffer:
            raw_line, _, remainder = self._rx_buffer.partition(b'\n')
            self._rx_buffer = bytearray(remainder)
            line = raw_line.decode('utf-8', errors='replace').strip('\r ')
            if line:
                self._parse_line(line)

    def _parse_line(self, line):
        fields = line.split(',')
        if not fields:
            return

        message_type = fields[0]
        if message_type == 'ENC' and len(fields) == 4:
            try:
                left_count = int(fields[2])
                right_count = int(fields[3])
            except ValueError:
                self.get_logger().warning(f'Malformed ENC message: {line}')
                return
            message = Int64MultiArray()
            message.data = [left_count, right_count]
            self.ticks_publisher.publish(message)
            return

        if message_type == 'ENC_ABS' and len(fields) >= 6:
            # Accept the absolute-encoder format without noisy console output.
            self._publish_status(line)
            return

        if message_type == 'US' and len(fields) == 4:
            try:
                measured_distance = float(fields[2])
                valid = int(fields[3]) == 1
            except ValueError:
                self.get_logger().warning(f'Malformed US message: {line}')
                return
            message = Range()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = self.ultrasonic_frame
            message.radiation_type = Range.ULTRASOUND
            message.field_of_view = self.ultrasonic_field_of_view
            message.min_range = self.ultrasonic_min_range
            message.max_range = self.ultrasonic_max_range
            if (
                valid
                and math.isfinite(measured_distance)
                and self.ultrasonic_min_range <= measured_distance
                and measured_distance <= self.ultrasonic_max_range
            ):
                message.range = measured_distance
            else:
                message.range = math.inf
            self.ultrasonic_publisher.publish(message)
            return

        if message_type == 'ERR':
            now = time.monotonic()
            error_key = ','.join(fields[:2]) if len(fields) >= 2 else line
            if (
                error_key == self._last_error_line
                and now - self._last_error_time < self.repeat_error_interval
            ):
                return
            self._last_error_line = error_key
            self._last_error_time = now
            self._publish_status(line)
            self.get_logger().error(f'ESP32: {line}')
            return

        if message_type in ('ACK', 'STAT'):
            self._publish_status(line)
            return

        # Some firmware versions emit a fragmented line when serial is busy.
        # Ignore those rather than spamming warnings.
        if (
            line.startswith('ACK,') or line.startswith('STAT,')
            or line.startswith('ERR,')
        ):
            self._publish_status(line)
            return

        self.get_logger().debug(f'Ignoring unexpected ESP32 line: {line}')

    def _publish_status(self, text):
        message = String()
        message.data = text
        self.status_publisher.publish(message)

    def destroy_node(self):
        if rclpy.ok():
            self._publish_motor_targets(0, 0)
        if hasattr(self, 'serial') and self.serial.is_open:
            self._send('STOP')
            try:
                self.serial.close()
            except (serial.SerialException, OSError):
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelToEsp32Serial()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'cmdvel_to_esp32_serial: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
