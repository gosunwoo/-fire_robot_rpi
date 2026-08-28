"""Conservative rotate-then-drive follower for a skid-steer robot."""

import math
import time
from typing import Optional, Sequence, Tuple

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, Empty, Int32, String

from .grid_utils import normalize_angle, yaw_from_quaternion
from .tf_utils import TfHelper


def clip(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


Point2D = Tuple[float, float]


def polyline_cumulative_lengths(points: Sequence[Point2D]) -> tuple[float, ...]:
    """Return monotonically increasing arc lengths for a path polyline."""
    if not points:
        return ()
    output = [0.0]
    for first, second in zip(points, points[1:]):
        output.append(
            output[-1] + math.hypot(second[0] - first[0], second[1] - first[1])
        )
    return tuple(output)


def project_progress_onto_path(
    points: Sequence[Point2D],
    cumulative: Sequence[float],
    robot_x: float,
    robot_y: float,
    minimum_progress: float = 0.0,
) -> float:
    """Project the robot onto the remaining path without moving backwards."""
    if not points:
        return 0.0
    if len(points) == 1:
        return 0.0
    minimum = max(0.0, min(float(minimum_progress), float(cumulative[-1])))
    best_distance = math.inf
    best_progress = minimum
    for index, (first, second) in enumerate(zip(points, points[1:])):
        segment_start = float(cumulative[index])
        segment_end = float(cumulative[index + 1])
        if segment_end + 1e-9 < minimum:
            continue
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 1e-12:
            continue
        ratio = ((robot_x - first[0]) * dx + (robot_y - first[1]) * dy) / length_squared
        ratio = max(0.0, min(1.0, ratio))
        progress = segment_start + ratio * (segment_end - segment_start)
        if progress + 1e-9 < minimum:
            progress = minimum
        segment_length = segment_end - segment_start
        local_ratio = 0.0 if segment_length <= 1e-12 else (
            progress - segment_start
        ) / segment_length
        projected_x = first[0] + local_ratio * dx
        projected_y = first[1] + local_ratio * dy
        distance = math.hypot(projected_x - robot_x, projected_y - robot_y)
        # Prefer the farther-ahead projection on equal-distance loops.
        if (
            distance < best_distance - 1e-9
            or (abs(distance - best_distance) <= 1e-9 and progress > best_progress)
        ):
            best_distance = distance
            best_progress = progress
    return max(minimum, best_progress)


def point_at_path_progress(
    points: Sequence[Point2D], cumulative: Sequence[float], progress: float
) -> Point2D:
    """Interpolate a point at an arc length along ``points``."""
    if not points:
        raise ValueError('path points must not be empty')
    if len(points) == 1:
        return float(points[0][0]), float(points[0][1])
    target = max(0.0, min(float(progress), float(cumulative[-1])))
    for index in range(len(points) - 1):
        end = float(cumulative[index + 1])
        if target > end + 1e-9:
            continue
        start = float(cumulative[index])
        length = end - start
        if length <= 1e-12:
            continue
        ratio = (target - start) / length
        first, second = points[index], points[index + 1]
        return (
            first[0] + ratio * (second[0] - first[0]),
            first[1] + ratio * (second[1] - first[1]),
        )
    return float(points[-1][0]), float(points[-1][1])


def shortest_turn_direction(
    heading_error: float, previous_direction: float = 0.0,
    wrap_guard_rad: float = 0.08,
) -> float:
    """Follow current shortest-turn sign, retaining sign only at +/-pi wrap."""
    error = normalize_angle(float(heading_error))
    if (
        abs(abs(error) - math.pi) <= wrap_guard_rad
        and previous_direction != 0.0
    ):
        return 1.0 if previous_direction > 0.0 else -1.0
    if abs(error) <= 1e-12:
        return 0.0
    return 1.0 if error > 0.0 else -1.0


class SkidPathFollower(Node):
    def __init__(self) -> None:
        super().__init__('skid_path_follower')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'cmd_vel_topic': '/cmd_vel',
            'lookahead_distance': 0.35,
            'goal_tolerance': 0.25,
            'yaw_tolerance': 0.35,
            'align_goal_yaw': False,
            'rotate_in_place_threshold': 0.45,
            'rotate_exit_threshold': 0.20,
            'max_linear_speed': 0.06,
            'max_angular_speed': 0.45,
            'k_linear': 0.5,
            'k_angular': 1.2,
            'emergency_stop_distance': 0.28,
            'emergency_front_angle_deg': 35.0,
            'control_rate_hz': 10.0,
            'spin_guard_max_rotation_rad': 1.25 * math.pi,
            'spin_guard_progress_m': 0.03,
            'require_localization_ready': False,
            'localization_ready_topic': '/localization_ready',
            'replan_hold_topic': '/replanning/hold',
            'survivor_follow_hold_topic': '/survivor_follow_hold',
            'mode6_active_topic': '/mode6/smoke_assist_active',
            'mode6_safety_hold_topic': '/mode6/safety_hold',
            'mode6_obstacle_topic': '/mode6/ultrasonic_obstacle',
            'mode6_imu_topic': '/imu/data',
            'mode6_imu_timeout_sec': 1.0,
            'mode6_initial_stop_sec': 0.50,
            'mode6_use_imu_heading': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        replan_hold_topic = str(self.get_parameter('replan_hold_topic').value)
        survivor_follow_hold_topic = str(
            self.get_parameter('survivor_follow_hold_topic').value
        )
        mode6_active_topic = str(
            self.get_parameter('mode6_active_topic').value
        )
        mode6_safety_hold_topic = str(
            self.get_parameter('mode6_safety_hold_topic').value
        )
        mode6_obstacle_topic = str(
            self.get_parameter('mode6_obstacle_topic').value
        )
        mode6_imu_topic = str(self.get_parameter('mode6_imu_topic').value)
        self.mode6_imu_timeout = float(
            self.get_parameter('mode6_imu_timeout_sec').value
        )
        self.mode6_initial_stop = float(
            self.get_parameter('mode6_initial_stop_sec').value
        )
        self.mode6_use_imu_heading = bool(
            self.get_parameter('mode6_use_imu_heading').value
        )
        self.lookahead = float(self.get_parameter('lookahead_distance').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.rotate_threshold = float(
            self.get_parameter('rotate_in_place_threshold').value
        )
        self.default_align_goal_yaw = bool(
            self.get_parameter('align_goal_yaw').value
        )
        self.align_goal_yaw = self.default_align_goal_yaw
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
        control_rate = float(self.get_parameter('control_rate_hz').value)
        self.spin_guard_max_rotation = float(
            self.get_parameter('spin_guard_max_rotation_rad').value
        )
        self.spin_guard_progress = float(
            self.get_parameter('spin_guard_progress_m').value
        )
        self.require_localization_ready = bool(
            self.get_parameter('require_localization_ready').value
        )
        localization_ready_topic = str(
            self.get_parameter('localization_ready_topic').value
        )
        positive = (
            self.lookahead, self.goal_tolerance, self.yaw_tolerance,
            self.rotate_threshold, self.max_linear, self.max_angular,
            self.k_linear, self.k_angular, self.emergency_distance,
            self.front_angle, control_rate, self.spin_guard_max_rotation,
            self.spin_guard_progress,
            self.mode6_imu_timeout, self.mode6_initial_stop,
        )
        if not 0.0 < self.rotate_exit_threshold < self.rotate_threshold:
            raise ValueError(
                'rotate_exit_threshold는 rotate_in_place_threshold보다 작아야 합니다.'
            )
        if any(value <= 0.0 for value in positive):
            raise ValueError('skid follower의 모든 거리/속도/gain/rate는 양수여야 합니다.')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf = TfHelper(self)
        self.path: Optional[Path] = None
        self.planner_state = 'WAITING_FOR_PATH'
        self.emergency_stop = False
        # Stage 6 event-replanning hold: set by replan_supervisor_node while it stops
        # the robot, invalidates the active path, and requests+validates a new one.
        # Defaults false and stays false forever if nothing publishes this topic, so
        # Stage 1-5 behavior is unchanged when Stage 6 is absent/disabled.
        self.hold = False
        # Mode 5 escort uses an independent hold.  With no Mode 5 publisher it
        # remains false, preserving every standalone mode's follower behavior.
        self.survivor_follow_hold = False
        self.drive_mode = 1
        self.mode6_active = False
        self.mode6_safety_hold = False
        self.mode6_obstacle = False
        self.mode6_obstacle_stop_until = float('-inf')
        self.mode6_imu_yaw = None
        self.mode6_imu_received_at = float('-inf')
        self.mode6_imu_anchor = None
        self.mode6_map_yaw_anchor = None
        self.rotating_in_place = False
        self.rotation_direction = 0.0
        self.path_progress = 0.0
        self.active_goal_signature = None
        self.best_goal_distance = math.inf
        self.accumulated_rotation = 0.0
        self.last_control_yaw = None
        self.last_command_was_rotation = False
        self.localization_ready = not self.require_localization_ready
        self.publisher = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.state_publisher = self.create_publisher(String, '/follower_state', 10)
        self.create_subscription(Path, '/planned_path', self._path_callback, qos)
        self.create_subscription(String, '/planner_state', self._planner_callback, 10)
        self.create_subscription(LaserScan, self.scan_topic, self._scan_callback, 10)
        self.create_subscription(Int32, '/drive_mode', self._mode_callback, 10)
        self.create_subscription(
            Bool,
            localization_ready_topic,
            self._localization_ready_callback,
            qos,
        )
        self.create_subscription(
            Empty, '/autonomy_cancel', self._cancel_callback, 10
        )
        self.create_subscription(Bool, replan_hold_topic, self._hold_callback, 10)
        self.create_subscription(
            Bool,
            survivor_follow_hold_topic,
            self._survivor_follow_hold_callback,
            10,
        )
        self.create_subscription(
            Bool, mode6_active_topic, self._mode6_active_callback, 10
        )
        self.create_subscription(
            Bool, mode6_safety_hold_topic, self._mode6_safety_hold_callback, 10
        )
        self.create_subscription(
            Bool, mode6_obstacle_topic, self._mode6_obstacle_callback, 10
        )
        self.create_subscription(
            Imu, mode6_imu_topic, self._mode6_imu_callback, 10
        )
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
        if not message.poses:
            self.path_progress = 0.0
            self._publish_stop('EMPTY_PATH')
            return
        goal = message.poses[-1].pose
        signature = (
            round(float(goal.position.x), 4),
            round(float(goal.position.y), 4),
            round(float(yaw_from_quaternion(goal.orientation)), 4),
        )
        if signature != getattr(self, 'active_goal_signature', None):
            self.active_goal_signature = signature
            self._reset_motion_watchdog()
        self.path_progress = 0.0
        # Acknowledge every newly accepted path before control can report an
        # immediate GOAL_REACHED for a waypoint already inside tolerance.
        self._state('PATH_ACCEPTED')

    def _planner_callback(self, message: String) -> None:
        self.planner_state = message.data
        if message.data in ('NO_PATH', 'WAITING_FOR_TF', 'WAITING_FOR_GRID'):
            self.path = None
            self._publish_stop(message.data)

    def _hold_callback(self, message: Bool) -> None:
        self.hold = bool(message.data)

    def _survivor_follow_hold_callback(self, message: Bool) -> None:
        self.survivor_follow_hold = bool(message.data)

    def _localization_ready_callback(self, message: Bool) -> None:
        self.localization_ready = bool(message.data)

    def _mode_callback(self, message: Int32) -> None:
        # MODE 3/4 must finish facing the inspected obstacle so the forward
        # mmWave sensor or camera observes it at the standoff point.
        self.drive_mode = int(message.data)
        self.align_goal_yaw = (
            self.default_align_goal_yaw or self.drive_mode in (3, 4)
        )
        if self.drive_mode != 6:
            self.mode6_imu_anchor = None
            self.mode6_map_yaw_anchor = None

    def _mode6_active_callback(self, message: Bool) -> None:
        active = bool(message.data)
        if active != self.mode6_active:
            self.mode6_imu_anchor = None
            self.mode6_map_yaw_anchor = None
        self.mode6_active = active
        if not active:
            self.mode6_obstacle = False

    def _mode6_safety_hold_callback(self, message: Bool) -> None:
        self.mode6_safety_hold = bool(message.data)

    def _mode6_obstacle_callback(self, message: Bool) -> None:
        blocked = bool(message.data)
        if blocked and not self.mode6_obstacle:
            self.mode6_obstacle_stop_until = (
                time.monotonic() + self.mode6_initial_stop
            )
        self.mode6_obstacle = blocked

    def _mode6_imu_callback(self, message: Imu) -> None:
        try:
            yaw = yaw_from_quaternion(message.orientation)
        except (TypeError, ValueError):
            return
        if not math.isfinite(yaw):
            return
        self.mode6_imu_yaw = yaw
        self.mode6_imu_received_at = time.monotonic()

    def _mode6_heading(self, tf_yaw: float, now: float) -> float:
        """Use relative IMU yaw anchored to the latest trusted map heading."""
        if (
            self.drive_mode != 6
            or not self.mode6_active
            or not self.mode6_use_imu_heading
            or self.mode6_imu_yaw is None
            or now - self.mode6_imu_received_at > self.mode6_imu_timeout
        ):
            return tf_yaw
        if self.mode6_imu_anchor is None or self.mode6_map_yaw_anchor is None:
            self.mode6_imu_anchor = self.mode6_imu_yaw
            self.mode6_map_yaw_anchor = tf_yaw
        return normalize_angle(
            self.mode6_map_yaw_anchor
            + normalize_angle(self.mode6_imu_yaw - self.mode6_imu_anchor)
        )

    def _cancel_callback(self, _message: Empty) -> None:
        """Stop immediately and forget the route selected before cancellation."""
        self.path = None
        self.rotating_in_place = False
        self.rotation_direction = 0.0
        self.path_progress = 0.0
        self.active_goal_signature = None
        self._reset_motion_watchdog()
        self._publish_stop('CANCELLED')

    def _reset_motion_watchdog(self) -> None:
        self.best_goal_distance = math.inf
        self.accumulated_rotation = 0.0
        self.last_control_yaw = None
        self.last_command_was_rotation = False

    def _spin_guard_triggered(self, yaw: float, goal_distance: float) -> bool:
        if goal_distance < self.best_goal_distance - self.spin_guard_progress:
            self.best_goal_distance = goal_distance
            self.accumulated_rotation = 0.0
        if self.last_control_yaw is not None and self.last_command_was_rotation:
            self.accumulated_rotation += abs(
                normalize_angle(yaw - self.last_control_yaw)
            )
        self.last_control_yaw = yaw
        if self.accumulated_rotation <= self.spin_guard_max_rotation:
            return False
        self.path = None
        self.path_progress = 0.0
        self.rotating_in_place = False
        self.rotation_direction = 0.0
        self.last_command_was_rotation = False
        self.get_logger().error(
            'SPIN_GUARD_STOP: 목표 거리 개선 없이 과도한 회전을 감지했습니다.'
        )
        self._publish_stop('SPIN_GUARD_STOP')
        return True

    def _scan_callback(self, scan: LaserScan) -> None:
        angle = float(scan.angle_min)
        nearest = math.inf
        for measured_range in scan.ranges:
            if abs(normalize_angle(angle)) <= self.front_angle:
                distance = float(measured_range)
                if (
                    math.isfinite(distance)
                    and distance >= float(scan.range_min)
                    and distance <= float(scan.range_max)
                ):
                    nearest = min(nearest, distance)
            angle += float(scan.angle_increment)
        was_stopped = self.emergency_stop
        self.emergency_stop = nearest < self.emergency_distance
        if self.emergency_stop and not was_stopped:
            self.get_logger().warning(
                f'EMERGENCY_STOP: front obstacle {nearest:.3f} m'
            )

    def _control(self) -> None:
        if self.require_localization_ready and not self.localization_ready:
            self._publish_stop('WAITING_FOR_LOCALIZATION')
            return
        if self.emergency_stop:
            self._publish_stop('EMERGENCY_STOP')
            return
        if self.hold:
            self._publish_stop('REPLAN_HOLD')
            return
        if self.survivor_follow_hold:
            self._publish_stop('SURVIVOR_FOLLOW_HOLD')
            return
        now_monotonic = time.monotonic()
        if self.drive_mode == 6 and self.mode6_active:
            if self.mode6_safety_hold:
                self._publish_stop('MODE6_SENSOR_SAFETY_HOLD')
                return
            if (
                self.mode6_use_imu_heading
                and now_monotonic - self.mode6_imu_received_at
                > self.mode6_imu_timeout
            ):
                self._publish_stop('MODE6_IMU_STALE_STOP')
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
        x, y, tf_yaw = current
        yaw = self._mode6_heading(tf_yaw, now_monotonic)
        goal = self.path.poses[-1].pose
        goal_distance = math.hypot(goal.position.x - x, goal.position.y - y)
        if self._spin_guard_triggered(yaw, goal_distance):
            return
        if goal_distance <= self.goal_tolerance:
            goal_yaw = yaw_from_quaternion(goal.orientation)
            if not self.align_goal_yaw:
                self.path = None
                self.rotating_in_place = False
                self.last_command_was_rotation = False
                self._publish_stop('GOAL_REACHED')
                return
            yaw_error = normalize_angle(goal_yaw - yaw)
            if abs(yaw_error) <= self.yaw_tolerance:
                self.path = None
                self.last_command_was_rotation = False
                self._publish_stop('GOAL_REACHED')
            else:
                command = Twist()
                command.angular.z = clip(
                    self.k_angular * yaw_error, self.max_angular
                )
                self.publisher.publish(command)
                self.last_command_was_rotation = True
                self._state('ALIGNING_GOAL_YAW')
            return

        points = tuple(
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in self.path.poses
        )
        cumulative = polyline_cumulative_lengths(points)
        self.path_progress = project_progress_onto_path(
            points, cumulative, x, y, self.path_progress
        )
        target_x, target_y = point_at_path_progress(
            points,
            cumulative,
            min(cumulative[-1], self.path_progress + self.lookahead),
        )
        target_distance = math.hypot(target_x - x, target_y - y)
        target_heading = math.atan2(target_y - y, target_x - x)
        heading_error = normalize_angle(target_heading - yaw)
        if self.rotating_in_place:
            if abs(heading_error) <= self.rotate_exit_threshold:
                self.rotating_in_place = False
                self.rotation_direction = 0.0
            else:
                self.rotation_direction = shortest_turn_direction(
                    heading_error, self.rotation_direction
                )
        elif abs(heading_error) >= self.rotate_threshold:
            self.rotating_in_place = True
            self.rotation_direction = shortest_turn_direction(heading_error)

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
            self.last_command_was_rotation = True
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
                0.25, 1.0 - abs(heading_error) / self.rotate_threshold
            )
            self.last_command_was_rotation = False
            state = 'FOLLOWING_PATH'
        if self.drive_mode == 6 and self.mode6_active and self.mode6_obstacle:
            if now_monotonic < self.mode6_obstacle_stop_until:
                self._publish_stop('MODE6_ULTRASONIC_INITIAL_STOP')
                return
            # A single front sensor cannot certify either side as free. Keep
            # translation stopped and allow only the turn commanded by the
            # newly planned map path. Once the object leaves the 0.9 m release
            # gate, normal path following resumes.
            command.linear.x = 0.0
            if abs(command.angular.z) <= 1e-3:
                self._publish_stop('MODE6_WAITING_FOR_AVOIDANCE_PATH')
                return
            state = 'MODE6_ULTRASONIC_AVOIDANCE_TURN'
        self.publisher.publish(command)
        self._state(state)

    def _publish_stop(self, reason: str) -> None:
        self.last_command_was_rotation = False
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
