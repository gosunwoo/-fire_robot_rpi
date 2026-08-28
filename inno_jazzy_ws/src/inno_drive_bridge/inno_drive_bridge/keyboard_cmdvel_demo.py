import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Empty, Int32, String

from .named_waypoint_input import (
    command_source_for_drive_mode,
    parse_named_waypoints,
)


class KeyboardCmdVelDemo(Node):
    def __init__(self):
        super().__init__('keyboard_cmdvel_demo')
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('angular_speed', 0.35)
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_keyboard')
        self.declare_parameter('mode6_enabled', False)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.mode6_enabled = bool(self.get_parameter('mode6_enabled').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        if publish_rate <= 0.0:
            raise ValueError('publish_rate_hz must be greater than zero')
        self._owns_input_stream = False
        self._owns_terminal_output = False
        if sys.stdin.isatty():
            self._input_stream = sys.stdin
        else:
            try:
                self._input_stream = open('/dev/tty', encoding='utf-8')
                self._owns_input_stream = True
            except OSError as error:
                raise RuntimeError(
                    'keyboard input requires an interactive terminal (TTY)'
                ) from error
        try:
            self._terminal_output = open(
                '/dev/tty', 'w', encoding='utf-8', buffering=1
            )
            self._owns_terminal_output = True
        except OSError:
            self._terminal_output = sys.stdout

        self.publisher = self.create_publisher(
            Twist, str(self.get_parameter('cmd_vel_topic').value), 10
        )
        self.mode_publisher = self.create_publisher(Int32, '/drive_mode', 10)
        self.autonomy_cancel_publisher = self.create_publisher(
            Empty, '/autonomy_cancel', 10
        )
        self.waypoint_command_publisher = self.create_publisher(
            String, '/waypoint_queue_command', 10
        )
        self.inspection_command_publisher = self.create_publisher(
            String, '/obstacle_inspection_command', 10
        )
        self.mode6_toggle_publisher = self.create_publisher(
            Empty, '/mode6/toggle', 10
        )
        self.drive_mode = 1
        self.create_subscription(
            Int32, '/drive_mode', self._external_drive_mode, 10
        )
        self.create_subscription(
            String, '/waypoint_queue_status', self._waypoint_status, 10
        )
        self.command = Twist()
        self._waypoint_collecting = False
        self._waypoint_buffer = ''
        self._terminal_settings = termios.tcgetattr(self._input_stream)
        tty.setcbreak(self._input_stream.fileno())

        self.create_timer(1.0 / publish_rate, self._publish_command)
        self.create_timer(0.02, self._poll_keyboard)
        self.add_on_set_parameters_callback(self._set_speed_parameters)
        self.get_logger().info(
            'Keyboard ready: 1=manual, 2=select named waypoints, '
            '3=mmWave inspection, 4=camera+LiDAR inspection, '
            '5=automatic evacuation demo, '
            '6=smoke auxiliary driving, '
            'SPACE=start/next, '
            'c=cancel mission, w/x/a/d/s, q=quit'
        )

    def _write_terminal(self, text):
        try:
            self._terminal_output.write(text)
            self._terminal_output.flush()
        except (OSError, ValueError):
            pass

    def _render_waypoint_prompt(self):
        self._write_terminal(
            '\r\033[2K[모드 2] 웨이포인트 입력 (예: w1,w5,w6) > '
            + self._waypoint_buffer
        )

    def _begin_waypoint_input(self):
        self._waypoint_collecting = True
        self._waypoint_buffer = ''
        self._write_terminal('\n')
        self._render_waypoint_prompt()

    def _poll_waypoint_input(self, key):
        if key in ('\r', '\n'):
            self._write_terminal('\n')
            try:
                labels = parse_named_waypoints(self._waypoint_buffer)
            except ValueError as error:
                self.get_logger().warning(f'MODE 2 input rejected: {error}')
                self._waypoint_buffer = ''
                self._render_waypoint_prompt()
                return
            self._waypoint_collecting = False
            command = 'MODE2_SET:' + ','.join(labels)
            self.waypoint_command_publisher.publish(String(data=command))
            self.get_logger().info(
                'MODE 2 requested: ' + ' -> '.join(labels)
            )
            return
        if key == '\x1b':
            self._waypoint_collecting = False
            self._waypoint_buffer = ''
            self._write_terminal('\r\033[2K[모드 2] 입력 취소\n')
            return
        if key in ('\x7f', '\b'):
            self._waypoint_buffer = self._waypoint_buffer[:-1]
            self._render_waypoint_prompt()
            return
        if key.isprintable() and len(self._waypoint_buffer) < 1024:
            self._waypoint_buffer += key.lower()
            self._render_waypoint_prompt()

    def _waypoint_status(self, message):
        if not message.data.startswith('MODE2_'):
            return
        # The operator console translates these status codes into Korean.
        # Do not leak the raw internal code directly to /dev/tty.
        if self._waypoint_collecting:
            self._render_waypoint_prompt()

    def _external_drive_mode(self, message):
        """Keep emergency keys aware of a launch-selected autonomous mode."""
        try:
            command_source_for_drive_mode(message.data)
        except ValueError:
            return
        new_mode = int(message.data)
        if new_mode == self.drive_mode:
            return
        if self._waypoint_collecting and new_mode != 2:
            self._waypoint_collecting = False
            self._waypoint_buffer = ''
            self._write_terminal('\r\033[2K[모드 2] 입력 취소\n')
        self.drive_mode = new_mode
        self.command = Twist()
        if new_mode == 5:
            self.get_logger().info('MODE 5: EVACUATION_DEMO selected externally')
        elif new_mode == 6:
            self.get_logger().info('MODE 6: SMOKE_AUXILIARY_DRIVE selected externally')

    def _set_speed_parameters(self, parameters):
        linear_speed = self.linear_speed
        angular_speed = self.angular_speed
        changed = False
        for parameter in parameters:
            if parameter.name == 'linear_speed':
                linear_speed = float(parameter.value)
                changed = True
            elif parameter.name == 'angular_speed':
                angular_speed = float(parameter.value)
                changed = True
        if linear_speed <= 0.0 or angular_speed <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='linear_speed and angular_speed must be positive',
            )
        if changed:
            self.linear_speed = linear_speed
            self.angular_speed = angular_speed
            self.get_logger().info(
                f'Manual speed updated: linear={linear_speed:.3f} m/s, '
                f'angular={angular_speed:.3f} rad/s'
            )
        return SetParametersResult(successful=True)

    def _poll_keyboard(self):
        readable, _, _ = select.select([self._input_stream], [], [], 0.0)
        if not readable:
            return
        key = self._input_stream.read(1)
        if self._waypoint_collecting:
            self._poll_waypoint_input(key)
            return
        key = key.lower()

        command = Twist()
        label = None
        if key in ('1', '2', '3', '4', '5', '6'):
            if key == '6' and not self.mode6_enabled:
                self.get_logger().warning(
                    'MODE 6 is disabled. Launch with mode6_enabled:=true.'
                )
                return
            previous_mode = self.drive_mode
            # MODE 6 is an overlay for a route already started in MODE 2. It
            # deliberately keeps the active goal and path. Every other mode
            # selection preserves the original cancel-before-switch behavior.
            if key != '6':
                self.autonomy_cancel_publisher.publish(Empty())
                # Clear any stale planner goal before selecting/reselecting the
                # autonomous velocity source.
                if previous_mode in (2, 6) or key == '2':
                    self.waypoint_command_publisher.publish(
                        String(data='MODE2_CANCEL')
                    )
            self.drive_mode = int(key)
            self.command = command
            self._publish_command()
            self.mode_publisher.publish(Int32(data=self.drive_mode))
            if self.drive_mode == 1:
                label = 'MODE 1: KEYBOARD'
            elif self.drive_mode == 2:
                label = 'MODE 2: NAMED WAYPOINT STEP MISSION'
            elif self.drive_mode == 3:
                label = 'MODE 3: MMWAVE OBSTACLE INSPECTION'
            elif self.drive_mode == 4:
                label = 'MODE 4: CAMERA + LIDAR SURVIVOR INSPECTION'
            elif self.drive_mode == 5:
                label = 'MODE 5: AUTOMATIC EVACUATION DEMO'
            else:
                label = 'MODE 6: SMOKE AUXILIARY DRIVE (SPACE=TOGGLE)'
            self.get_logger().info(label)
            if self.drive_mode == 2:
                self._begin_waypoint_input()
            return
        if key == ' ':
            if self.drive_mode == 2:
                self.waypoint_command_publisher.publish(
                    String(data='MODE2_NEXT')
                )
                self.get_logger().info(
                    'MODE 2 requested next selected waypoint'
                )
                return
            if self.drive_mode in (3, 4):
                command = f'MODE{self.drive_mode}_START'
                self.inspection_command_publisher.publish(String(data=command))
                self.get_logger().info(
                    f'MODE {self.drive_mode} requested nearest-obstacle inspection'
                )
                return
            if self.drive_mode == 6:
                self.mode6_toggle_publisher.publish(Empty())
                self.get_logger().info(
                    'MODE 6 smoke auxiliary driving toggle requested'
                )
                return
            if self.drive_mode == 1:
                self.get_logger().warning(
                    'Select MODE 2, 3, 4, or 6 before pressing Space.'
                )
            return
        if key == 'c':
            if self.drive_mode in (2, 3, 4, 5, 6):
                cancelled_mode = self.drive_mode
                self._stop_all_motion()
                self.get_logger().warning(
                    f'MODE {cancelled_mode} CANCELLED: '
                    'zero velocity and MODE 1 selected'
                )
                return
            return
        if key == 's' and self.drive_mode != 1:
            self._stop_all_motion()
            self.get_logger().warning(
                'AUTONOMOUS STOP: zero velocity and MODE 1 selected'
            )
            return
        if self.drive_mode != 1 and key in ('w', 'x', 'a', 'd'):
            self.get_logger().warning(
                'Press 1 before using manual drive keys.'
            )
            return
        if key == 'w':
            command.linear.x = self.linear_speed
            label = 'FORWARD'
        elif key == 'x':
            command.linear.x = -self.linear_speed
            label = 'REVERSE'
        elif key == 'a':
            command.angular.z = self.angular_speed
            label = 'TURN LEFT'
        elif key == 'd':
            command.angular.z = -self.angular_speed
            label = 'TURN RIGHT'
        elif key == 's':
            label = 'STOP'
        elif key == 'q':
            self._stop_all_motion()
            self.get_logger().info('STOP, then quit')
            self.restore_terminal()
            raise KeyboardInterrupt
        else:
            return

        self.command = command
        self._publish_command()
        self.get_logger().info(
            f'{label}: linear.x={command.linear.x:.3f} m/s, '
            f'angular.z={command.angular.z:.3f} rad/s'
        )

    def _publish_command(self):
        if self.drive_mode == 1:
            self.publisher.publish(self.command)

    def _stop_all_motion(self):
        """Select the manual source and publish zero even from auto modes."""
        self.autonomy_cancel_publisher.publish(Empty())
        if self.drive_mode in (2, 6):
            self.waypoint_command_publisher.publish(
                String(data='MODE2_CANCEL')
            )
        self.command = Twist()
        self.publisher.publish(self.command)
        self.mode_publisher.publish(Int32(data=1))
        self.drive_mode = 1

    def restore_terminal(self):
        if self._terminal_settings is not None:
            termios.tcsetattr(
                self._input_stream, termios.TCSADRAIN, self._terminal_settings
            )
            self._terminal_settings = None
        if self._owns_input_stream:
            self._input_stream.close()
            self._owns_input_stream = False
        if self._owns_terminal_output:
            self._terminal_output.close()
            self._owns_terminal_output = False

    def destroy_node(self):
        try:
            # ROS 2's SIGINT handler can invalidate the context before launch
            # asks this node to clean up. Publishing after that point raises
            # RCLError and turns a normal shutdown into a process failure.
            if rclpy.ok():
                self._stop_all_motion()
        finally:
            self.restore_terminal()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = KeyboardCmdVelDemo()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (RuntimeError, ValueError) as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f'keyboard_cmdvel_demo: {error}', file=sys.stderr)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
