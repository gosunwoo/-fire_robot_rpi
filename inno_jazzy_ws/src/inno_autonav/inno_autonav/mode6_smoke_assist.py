"""Mode 6 smoke-assist state, safety gate, logs, and RViz scan coloring."""

from __future__ import annotations

import copy
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Imu, LaserScan, Range
from std_msgs.msg import Bool, Empty, Int32, String


class ObstacleHysteresis:
    """Stable close-obstacle state for a noisy single-beam range sensor."""

    def __init__(self, stop_distance_m=0.8, release_distance_m=0.9):
        self.stop_distance = float(stop_distance_m)
        self.release_distance = float(release_distance_m)
        if not 0.0 < self.stop_distance < self.release_distance:
            raise ValueError('release distance must be greater than stop distance')
        self.blocked = False

    def update(self, distance_m):
        distance = float(distance_m)
        if math.isnan(distance):
            return self.blocked
        if distance <= self.stop_distance:
            self.blocked = True
        elif distance >= self.release_distance:
            self.blocked = False
        return self.blocked


class Mode6SmokeAssist(Node):
    def __init__(self):
        super().__init__('mode6_smoke_assist')
        defaults = {
            'drive_mode': 6,
            'sensor_timeout_sec': 1.0,
            'ultrasonic_stop_distance_m': 0.8,
            'ultrasonic_release_distance_m': 0.9,
            'status_rate_hz': 5.0,
            'scan_topic': '/scan',
            'normal_scan_topic': '/scan_rviz_red',
            'smoke_scan_topic': '/scan_rviz_blue',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        value = lambda name: self.get_parameter(name).value
        self.required_mode = int(value('drive_mode'))
        self.sensor_timeout = float(value('sensor_timeout_sec'))
        rate = float(value('status_rate_hz'))
        if self.required_mode != 6 or self.sensor_timeout <= 0.0 or rate <= 0.0:
            raise ValueError('Mode 6 supervisor parameters are invalid')
        self.obstacle = ObstacleHysteresis(
            value('ultrasonic_stop_distance_m'),
            value('ultrasonic_release_distance_m'),
        )

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.active_publisher = self.create_publisher(
            Bool, '/mode6/smoke_assist_active', transient
        )
        self.hold_publisher = self.create_publisher(
            Bool, '/mode6/safety_hold', transient
        )
        self.obstacle_publisher = self.create_publisher(
            Bool, '/mode6/ultrasonic_obstacle', transient
        )
        self.status_publisher = self.create_publisher(
            String, '/mode6/status', transient
        )
        self.log_publisher = self.create_publisher(
            String, '/mode6/log', transient
        )
        self.normal_scan_publisher = self.create_publisher(
            LaserScan, str(value('normal_scan_topic')), qos_profile_sensor_data
        )
        self.smoke_scan_publisher = self.create_publisher(
            LaserScan, str(value('smoke_scan_topic')), qos_profile_sensor_data
        )

        self.create_subscription(Int32, '/drive_mode', self._on_mode, 10)
        self.create_subscription(Empty, '/mode6/toggle', self._on_toggle, 10)
        self.create_subscription(
            Range, '/ultrasonic/front/range', self._on_range, 10
        )
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)
        self.create_subscription(
            LaserScan, str(value('scan_topic')), self._on_scan,
            qos_profile_sensor_data,
        )
        self.create_timer(1.0 / rate, self._on_timer)

        self.mode = 1
        self.active = False
        self.last_range_at = float('-inf')
        self.last_imu_at = float('-inf')
        self.last_range = math.inf
        self.last_status = None
        self.last_hold = None
        self.last_obstacle = None
        self._publish_state('MODE6_READY:SELECT_MODE6_AFTER_STARTING_MODE2_ROUTE')

    def _log(self, text, warning=False):
        message = str(text)
        self.log_publisher.publish(String(data=message))
        if warning:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)

    def _publish_state(self, status, hold=False):
        if status != self.last_status:
            self.status_publisher.publish(String(data=status))
            self.last_status = status
        if bool(hold) != self.last_hold:
            self.hold_publisher.publish(Bool(data=bool(hold)))
            self.last_hold = bool(hold)
        if self.obstacle.blocked != self.last_obstacle:
            self.obstacle_publisher.publish(Bool(data=self.obstacle.blocked))
            self.last_obstacle = self.obstacle.blocked
        self.active_publisher.publish(Bool(data=self.active))

    def _on_mode(self, message):
        new_mode = int(message.data)
        if new_mode == self.mode:
            return
        self.mode = new_mode
        if new_mode == self.required_mode:
            self.active = False
            self.obstacle.blocked = False
            self._log(
                '모드 6 선택: 기존 웨이포인트 경로를 유지합니다. '
                'Space를 누르면 연기 보조주행을 시작합니다.'
            )
            self._publish_state('MODE6_SELECTED:SPACE_TO_ENABLE')
            return
        if self.active:
            self._log('모드 6 종료: LiDAR 중심의 기존 주행 방식으로 복귀합니다.')
        self.active = False
        self.obstacle.blocked = False
        self._publish_state('MODE6_INACTIVE')

    def _sensor_health(self, now=None):
        now = time.monotonic() if now is None else float(now)
        range_online = now - self.last_range_at <= self.sensor_timeout
        imu_online = now - self.last_imu_at <= self.sensor_timeout
        return range_online, imu_online

    def _on_toggle(self, _message):
        if self.mode != self.required_mode:
            self._log('모드 6을 먼저 선택해야 합니다.', warning=True)
            return
        if self.active:
            self.active = False
            self.obstacle.blocked = False
            self._log(
                '연기 보조주행을 해제했습니다. RViz LiDAR 스캔을 빨간색으로 '
                '복원하고 기존 주행 방식으로 복귀합니다.'
            )
            self._publish_state('MODE6_ASSIST_DISABLED')
            return
        range_online, imu_online = self._sensor_health()
        missing = []
        if not range_online:
            missing.append('HC-SR04')
        if not imu_online:
            missing.append('BNO055 IMU')
        if missing:
            self._log(
                '모드 6 시작 거부: 센서 데이터 없음 - ' + ', '.join(missing),
                warning=True,
            )
            self._publish_state('MODE6_START_REJECTED:SENSOR_OFFLINE', hold=True)
            return
        self.active = True
        self._log(
            '연기 농도가 높아 LiDAR 신뢰도가 낮습니다. 초음파 센서와 IMU를 '
            '보조적으로 활용해 저장된 SLAM 지도를 기준으로 경로를 계획합니다.'
        )
        self._publish_state('MODE6_SMOKE_ASSIST_ACTIVE')

    def _on_range(self, message):
        self.last_range_at = time.monotonic()
        distance = float(message.range)
        if not math.isfinite(distance):
            distance = math.inf
        self.last_range = distance
        was_blocked = self.obstacle.blocked
        self.obstacle.update(distance)
        if self.active and self.obstacle.blocked and not was_blocked:
            self._log(
                f'전방 {distance:.2f}m에서 초음파 장애물을 감지했습니다. '
                '로봇을 정지하고 코스트맵에 반영하여 우회 경로를 재계획합니다.',
                warning=True,
            )
        elif self.active and was_blocked and not self.obstacle.blocked:
            self._log('초음파 전방 안전거리가 확보되어 재계획 경로 주행을 재개합니다.')

    def _on_imu(self, message):
        quaternion = message.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
            + quaternion.w * quaternion.w
        )
        if math.isfinite(norm) and norm > 0.5:
            self.last_imu_at = time.monotonic()

    @staticmethod
    def _empty_scan(message):
        empty = copy.deepcopy(message)
        empty.ranges = [math.nan] * len(message.ranges)
        empty.intensities = []
        return empty

    def _on_scan(self, message):
        empty = self._empty_scan(message)
        if self.active:
            self.normal_scan_publisher.publish(empty)
            self.smoke_scan_publisher.publish(message)
        else:
            self.normal_scan_publisher.publish(message)
            self.smoke_scan_publisher.publish(empty)

    def _on_timer(self):
        if not self.active:
            self._publish_state(self.last_status or 'MODE6_INACTIVE')
            return
        range_online, imu_online = self._sensor_health()
        if not range_online or not imu_online:
            missing = []
            if not range_online:
                missing.append('HC-SR04')
            if not imu_online:
                missing.append('BNO055')
            status = 'MODE6_SAFETY_STOP:SENSOR_STALE:' + ','.join(missing)
            if status != self.last_status:
                self._log(
                    '보조센서 데이터가 끊겨 안전 정지합니다: ' + ', '.join(missing),
                    warning=True,
                )
            self._publish_state(status, hold=True)
            return
        status = (
            f'MODE6_OBSTACLE_STOP:{self.last_range:.3f}M'
            if self.obstacle.blocked
            else f'MODE6_ASSIST_DRIVING:RANGE={self.last_range:.3f}M'
        )
        self._publish_state(status, hold=False)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mode6SmokeAssist()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as error:
        print(f'mode6_smoke_assist: {error}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
