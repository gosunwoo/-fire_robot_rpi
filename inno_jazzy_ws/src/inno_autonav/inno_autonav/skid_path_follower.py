"""Conservative rotate-then-drive follower for a skid-steer robot."""

import math
import time
from typing import Optional

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .grid_utils import normalize_angle, yaw_from_quaternion
from .tf_utils import TfHelper


def clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def requires_in_place_rotation(heading_error: float, threshold: float) -> bool:
    return abs(normalize_angle(float(heading_error))) >= float(threshold)


def nearest_scan_clearances(
    scan: LaserScan, front_angle_rad: float
) -> tuple[float, float]:
    """Return nearest valid front and all-around ranges."""

    angle = float(scan.angle_min)
    nearest_front = math.inf
    nearest_any = math.inf
    for measured_range in scan.ranges:
        distance = float(measured_range)
        if (
            math.isfinite(distance)
            and distance >= float(scan.range_min)
            and distance <= float(scan.range_max)
        ):
            nearest_any = min(nearest_any, distance)
            if abs(normalize_angle(angle)) <= front_angle_rad:
                nearest_front = min(nearest_front, distance)
        angle += float(scan.angle_increment)
    return nearest_front, nearest_any


class RotationProgressMonitor:
    """Detect a commanded in-place turn whose localization yaw is frozen."""

    def __init__(self, min_progress_rad: float, timeout_sec: float) -> None:
        if min_progress_rad <= 0.0 or timeout_sec <= 0.0:
            raise ValueError("rotation progress limits must be positive")
        self.min_progress_rad = float(min_progress_rad)
        self.timeout_sec = float(timeout_sec)
        self.reset()

    def reset(self) -> None:
        self.reference_yaw: Optional[float] = None
        self.reference_time: Optional[float] = None

    def stalled(self, yaw: float, now: float, active: bool = True) -> bool:
        if not active:
            self.reset()
            return False
        yaw = float(yaw)
        now = float(now)
        if self.reference_yaw is None or self.reference_time is None:
            self.reference_yaw = yaw
            self.reference_time = now
            return False
        progress = abs(normalize_angle(yaw - self.reference_yaw))
        if progress >= self.min_progress_rad:
            self.reference_yaw = yaw
            self.reference_time = now
            return False
        return now - self.reference_time >= self.timeout_sec


class SkidPathFollower(Node):
    def __init__(self) -> None:
        super().__init__('skid_path_follower')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel',
            'lookahead_distance': 0.35,
            'goal_tolerance': 0.12,
            'yaw_tolerance': 0.25,
            'align_goal_yaw': False,
            'rotate_in_place_threshold': 0.45,
            'rotate_exit_threshold': 0.20,
            'max_linear_speed': 0.06,
            'max_angular_speed': 0.45,
            'k_linear': 0.5,
            'k_angular': 1.2,
            'emergency_stop_distance': 0.28,
            'emergency_front_angle_deg': 35.0,
            "rotation_clearance_distance": 0.32,
            "minimum_turn_speed_ratio": 0.40,
            "rotation_progress_min_rad": 0.04,
            "rotation_progress_timeout_sec": 3.0,
            'control_rate_hz': 10.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.lookahead = float(self.get_parameter('lookahead_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.rotate_threshold = float(
            self.get_parameter('rotate_in_place_threshold').value
        )
        self.align_goal_yaw = bool(self.get_parameter('align_goal_yaw').value)
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.k_linear = float(self.get_parameter('k_linear').value)
        self.rotate_exit_threshold = float(
            self.get_parameter('rotate_exit_threshold').value
        )
        self.k_angular = float(self.get_parameter('k_angular').value)
        self.emergency_distance = float(
            self.get_parameter('emergency_stop_distance').value
        )
        self.front_angle = math.radians(float(
            self.get_parameter('emergency_front_angle_deg').value
        ))
        self.rotation_clearance_distance = float(
            self.get_parameter("rotation_clearance_distance").value
        )
        self.minimum_turn_speed_ratio = float(
            self.get_parameter("minimum_turn_speed_ratio").value
        )
        rotation_progress_min = float(
            self.get_parameter("rotation_progress_min_rad").value
        )
        rotation_progress_timeout = float(
            self.get_parameter("rotation_progress_timeout_sec").value
        )
        control_rate = float(self.get_parameter('control_rate_hz').value)
        positive = (
            self.lookahead, self.goal_tolerance, self.yaw_tolerance,
            self.rotate_threshold, self.max_linear, self.max_angular,
            self.k_linear, self.k_angular, self.emergency_distance,
            self.front_angle, self.rotation_clearance_distance,
            rotation_progress_min, rotation_progress_timeout, control_rate,
        )
        if not 0.0 < self.rotate_exit_threshold < self.rotate_threshold:
            raise ValueError(
                'rotate_exit_threshold는 rotate_in_place_threshold보다 작아야 합니다.'
            )
        if not 0.0 < self.minimum_turn_speed_ratio <= 1.0:
            raise ValueError("minimum_turn_speed_ratio must be in (0, 1]")
        if any(value <= 0.0 for value in positive):
            raise ValueError('skid follower의 모든 거리/속도/gain/rate는 양수여야 합니다.')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf = TfHelper(self)
        self.path: Optional[Path] = None
        self.planner_state = 'WAITING_FOR_PATH'
        self.emergency_stop = False
        self.rotation_clearance_blocked = False
        self.rotating_in_place = False
        self.rotation_direction = 0.0
        self.rotation_stalled = False
        self.rotation_stall_yaw = 0.0
        self.rotation_progress = RotationProgressMonitor(
            rotation_progress_min, rotation_progress_timeout
        )
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_publisher = self.create_publisher(String, '/follower_state', 10)
        self.create_subscription(Path, '/planned_path', self._path_callback, qos)
        self.create_subscription(String, '/planner_state', self._planner_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.create_timer(1.0 / control_rate, self._control)
        self.add_on_set_parameters_callback(self._set_speed_parameters)
        self._publish_stop('WAITING_FOR_PATH')
        self.get_logger().info(
            f'skid follower -> {cmd_vel_topic}, max=({self.max_linear:.3f} m/s, '
            f'{self.max_angular:.3f} rad/s)'
        )

    def _set_speed_parameters(self, parameters):
        max_linear = self.max_linear
        max_angular = self.max_angular
        changed = False
        for parameter in parameters:
            if parameter.name == 'max_linear_speed':
                max_linear = float(parameter.value)
                changed = True
            elif parameter.name == 'max_angular_speed':
                max_angular = float(parameter.value)
                changed = True
        if max_linear <= 0.0 or max_angular <= 0.0:
            return SetParametersResult(
                successful=False,
                reason='max_linear_speed and max_angular_speed must be positive',
            )
        if changed:
            self.max_linear = max_linear
            self.max_angular = max_angular
            self.get_logger().info(
                f'Autonomous speed updated: linear={max_linear:.3f} m/s, '
                f'angular={max_angular:.3f} rad/s'
            )
        return SetParametersResult(successful=True)

    def _path_callback(self, message: Path) -> None:
        self.path = message if message.poses else None
        self.rotation_progress.reset()
        if not message.poses:
            self._publish_stop('EMPTY_PATH')
            return
        # Acknowledge every newly accepted path before control can report an
        # immediate GOAL_REACHED for a waypoint already inside tolerance.
        self._state('PATH_ACCEPTED')

    def _planner_callback(self, message: String) -> None:
        self.planner_state = message.data
        if message.data in ('NO_PATH', 'WAITING_FOR_TF', 'WAITING_FOR_GRID'):
            self.path = None
            self._publish_stop(message.data)

    def _scan_callback(self, scan: LaserScan) -> None:
        nearest, nearest_any = nearest_scan_clearances(
            scan, self.front_angle
        )
        was_stopped = self.emergency_stop
        self.emergency_stop = nearest < self.emergency_distance
        self.rotation_clearance_blocked = (
            nearest_any < self.rotation_clearance_distance
        )
        if self.emergency_stop and not was_stopped:
            self.get_logger().warning(
                f'EMERGENCY_STOP: front obstacle {nearest:.3f} m'
            )

    def _control(self) -> None:
        if self.emergency_stop:
            self._publish_stop('EMERGENCY_STOP')
            return
        if self.planner_state == 'NO_PATH':
            self._publish_stop('NO_PATH')
            return
        if self.path is None or not self.path.poses:
            self._publish_stop('WAITING_FOR_PATH')
            return
        current = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if current is None:
            self._publish_stop('WAITING_FOR_TF')
            return
        x, y, yaw = current
        if self.rotation_stalled:
            recovered = abs(normalize_angle(yaw - self.rotation_stall_yaw))
            if recovered < self.rotation_progress.min_progress_rad:
                self._publish_stop("ROTATION_ODOMETRY_STALE")
                return
            self.rotation_stalled = False
            self.rotation_progress.reset()
        goal = self.path.poses[-1].pose
        goal_distance = math.hypot(goal.position.x - x, goal.position.y - y)
        if goal_distance <= self.goal_tolerance:
            goal_yaw = yaw_from_quaternion(goal.orientation)
            if not self.align_goal_yaw:
                self.path = None
                self.rotating_in_place = False
                self._publish_stop('GOAL_REACHED')
                return
            yaw_error = normalize_angle(goal_yaw - yaw)
            if abs(yaw_error) <= self.yaw_tolerance:
                self.path = None
                self._publish_stop('GOAL_REACHED')
            else:
                command = Twist()
                command.angular.z = clip(
                    self.k_angular * yaw_error, self.max_angular
                )
                self.publisher.publish(command)
                self._state('ALIGNING_GOAL_YAW')
            return

        target = self.path.poses[-1].pose.position
        for stamped_pose in self.path.poses:
            candidate = stamped_pose.pose.position
            if math.hypot(candidate.x - x, candidate.y - y) >= self.lookahead:
                target = candidate
                break
        target_distance = math.hypot(target.x - x, target.y - y)
        target_heading = math.atan2(target.y - y, target.x - x)
        heading_error = normalize_angle(target_heading - yaw)
        if self.rotating_in_place:
            if abs(heading_error) <= self.rotate_exit_threshold:
                self.rotating_in_place = False
                self.rotation_direction = 0.0
                self.rotation_progress.reset()
        elif requires_in_place_rotation(heading_error, self.rotate_threshold):
            self.rotating_in_place = True
            self.rotation_direction = 1.0 if heading_error >= 0.0 else -1.0
            self.rotation_progress.reset()

        if self.rotating_in_place and self.rotation_progress.stalled(
            yaw, time.monotonic(), active=True
        ):
            self.rotating_in_place = False
            self.rotation_stalled = True
            self.rotation_stall_yaw = yaw
            self._publish_stop("ROTATION_ODOMETRY_STALE")
            self.get_logger().error(
                "In-place rotation stopped: map->base_link yaw did not update"
            )
            return
        if not self.rotating_in_place:
            self.rotation_progress.reset()

        if self.rotating_in_place and self.rotation_clearance_blocked:
            self._publish_stop('ROTATION_BLOCKED')
            return

        command = Twist()
        if self.rotating_in_place:
            # Keep the chosen turn direction while near the +/-pi wrap point;
            # small TF yaw noise must not alternate left/right commands.
            magnitude = min(
                self.max_angular,
                max(0.10, self.k_angular * abs(heading_error)),
            )
            command.angular.z = self.rotation_direction * magnitude
            command.linear.x = 0.0
            state = 'ROTATING_IN_PLACE'
        else:
            command.angular.z = clip(
                self.k_angular * heading_error, self.max_angular
            )
            command.linear.x = min(
                self.max_linear, max(0.01, self.k_linear * target_distance)
            )
            # Slow down while steering to reduce skid and LiDAR motion distortion.
            command.linear.x *= max(
                self.minimum_turn_speed_ratio,
                1.0 - abs(heading_error) / self.rotate_threshold,
            )
            state = 'FOLLOWING_PATH'
        self.publisher.publish(command)
        self._state(state)

    def _publish_stop(self, reason: str) -> None:
        self.publisher.publish(Twist())
        self._state(reason)

    def _state(self, text: str) -> None:
        self.state_publisher.publish(String(data=text))

    def destroy_node(self):
        # A launch SIGINT can invalidate the rclpy context before this cleanup
        # callback runs. Publish the final stop only while the context is valid.
        if rclpy.ok():
            self.publisher.publish(Twist())
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SkidPathFollower()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f'skid_path_follower 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
