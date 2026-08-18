"""Safety mux: exactly one of keyboard or autonomous control reaches the ESP32."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String


MODE_LABELS = {
    1: 'KEYBOARD',
    2: 'WAYPOINT',
    3: 'RESCUE WAYPOINT',
}


def command_source_for_mode(mode: int) -> int:
    """Map operator mode to the physical velocity input channel."""

    if mode not in MODE_LABELS:
        raise ValueError('drive_mode must be 1, 2, or 3')
    return 1 if mode == 1 else 2


def inspection_stops_output(mode: int, inspection_hold: bool) -> bool:
    return int(mode) == 3 and bool(inspection_hold)


class CmdVelModeMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mode_mux')
        self.declare_parameter('input_timeout_sec', 0.35)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.timeout = float(self.get_parameter('input_timeout_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if self.timeout <= 0.0 or rate <= 0.0:
            raise ValueError('timeout and publish rate must be positive')
        self.mode = 1
        self.inspection_hold = False
        self.commands = {1: Twist(), 2: Twist()}
        self.received = {1: 0.0, 2: 0.0}
        self.output = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status = self.create_publisher(String, '/drive_mode_status', 10)
        self.create_subscription(Twist, '/cmd_vel_keyboard', lambda m: self._cmd(1, m), 10)
        self.create_subscription(Twist, '/cmd_vel_auto', lambda m: self._cmd(2, m), 10)
        self.create_subscription(Int32, '/drive_mode', self._mode, 10)
        self.create_subscription(
            Bool, '/mode3_inspection_hold', self._inspection_hold, 10
        )
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info('Drive mode 1 (keyboard) selected')

    def _cmd(self, source, message):
        self.commands[source] = message
        self.received[source] = time.monotonic()

    def _mode(self, message):
        if message.data not in MODE_LABELS:
            self.get_logger().warning('drive_mode must be 1, 2, or 3')
            return
        self.output.publish(Twist())
        self.mode = int(message.data)
        label = MODE_LABELS[self.mode]
        self.status.publish(String(data=f'{self.mode}:{label}'))
        self.get_logger().info(f'Drive mode {self.mode} ({label}) selected')

    def _publish(self):
        if inspection_stops_output(
            self.mode, self.inspection_hold
        ):
            self.output.publish(Twist())
            return
        source = command_source_for_mode(self.mode)
        command = self.commands[source]
        if time.monotonic() - self.received[source] > self.timeout:
            command = Twist()
        self.output.publish(command)

    def _inspection_hold(self, message):
        self.inspection_hold = bool(message.data)

    def destroy_node(self):
        if rclpy.ok():
            self.output.publish(Twist())
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelModeMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
