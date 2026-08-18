"""Compact, event-driven console output for the field driving demo."""

import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, String


MODE_TITLES = {
    1: 'KEYBOARD',
    2: 'WAYPOINT ONLY',
    3: 'RESCUE + DYNAMIC AVOIDANCE',
}
FILTERED_PRESENCE_TOPIC = '/mmwave/filtered_presence'
FILTERED_DISTANCE_TOPIC = '/mmwave/filtered_distance_m'
DYNAMIC_OBSTACLE_TOPIC = '/dynamic_obstacle_detected'
VICTIM_FUSION_TOPIC = '/victim_fusion_status'
THERMAL_ROUTE_TOPIC = "/thermal_route_status"


def waypoint_log_text(state: str) -> Optional[str]:
    """Return one concise Korean line for operator-relevant queue events."""

    raw = state.strip().upper()
    if raw.startswith('RESTORED:'):
        try:
            return f'[웨이포인트] {int(raw.split(":", 1)[1])}개 준비됨'
        except (TypeError, ValueError):
            return None
    if raw.startswith(('RUNNING:', 'REACHED:')):
        try:
            event, progress = raw.split(':', 1)
            current, total = (int(value) for value in progress.split('/', 1))
            if event == 'RUNNING':
                return f'[웨이포인트] {current}/{total} 주행 중'
            return f'[웨이포인트] {current}/{total} 도착'
        except (TypeError, ValueError):
            return None
    if raw in ('MISSION_COMPLETE', 'STEP_MISSION_COMPLETE'):
        return '[웨이포인트] 주행 완료'
    if raw == 'CLEARED':
        return '[웨이포인트] 대기열 초기화'
    return None


class StatusConsole(Node):
    """Show operator-relevant changes without repeating steady-state messages."""

    def __init__(self) -> None:
        super().__init__('mmwave_status_console')
        self.declare_parameter('distance_log_interval_sec', 1.0)
        self.declare_parameter('distance_log_delta_m', 0.10)
        self.declare_parameter('detection_distance_wait_sec', 0.15)
        self.distance_log_interval = float(
            self.get_parameter('distance_log_interval_sec').value
        )
        self.distance_log_delta = float(
            self.get_parameter('distance_log_delta_m').value
        )
        self.detection_distance_wait = float(
            self.get_parameter('detection_distance_wait_sec').value
        )
        if self.distance_log_interval <= 0.0:
            raise ValueError('distance_log_interval_sec must be positive')
        if self.distance_log_delta < 0.0:
            raise ValueError('distance_log_delta_m must not be negative')
        if self.detection_distance_wait <= 0.0:
            raise ValueError('detection_distance_wait_sec must be positive')

        self._mode: Optional[int] = 1
        self._unknown_mode_state: Optional[str] = None
        self._waypoint_state: Optional[str] = None
        self._follower_state: Optional[str] = None
        self._sensor_state: Optional[str] = None
        self._presence: Optional[bool] = None
        self._distance_m: Optional[float] = None
        self._last_distance_log_m: Optional[float] = None
        self._last_distance_log_time = 0.0
        self._pending_detection = False
        self._dynamic_detected = False
        self._victim_state: Optional[str] = None
        self._thermal_route_state: Optional[str] = None

        self.create_subscription(
            String, '/drive_mode_status', self._on_drive_mode, 10
        )
        waypoint_qos = QoSProfile(depth=1)
        waypoint_qos.reliability = ReliabilityPolicy.RELIABLE
        waypoint_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String, '/waypoint_queue_status', self._on_waypoint_state,
            waypoint_qos,
        )
        self.create_subscription(
            String, '/follower_state', self._on_follower_state, 10
        )
        self.create_subscription(
            Bool, FILTERED_PRESENCE_TOPIC, self._on_presence, 10
        )
        self.create_subscription(
            Float32, FILTERED_DISTANCE_TOPIC, self._on_distance, 10
        )
        self.create_subscription(
            String, '/mmwave/sensor_state', self._on_sensor_state, 10
        )
        self.create_subscription(
            Bool, DYNAMIC_OBSTACLE_TOPIC, self._on_dynamic_obstacle, 10
        )
        self.create_subscription(
            String, VICTIM_FUSION_TOPIC, self._on_victim_fusion, 10
        )
        self.create_subscription(
            String, THERMAL_ROUTE_TOPIC, self._on_thermal_route, 10
        )
        self._detection_timer = self.create_timer(
            self.detection_distance_wait, self._flush_pending_detection
        )
        self._detection_timer.cancel()

        self._print_mode(1)

    @staticmethod
    def _write(text: str) -> None:
        print(text, flush=True)

    def _print_mode(self, mode: int) -> None:
        title = MODE_TITLES[mode]
        self._write(f'\n{"═" * 12} MODE {mode} | {title} {"═" * 12}')

    def _on_drive_mode(self, message: String) -> None:
        raw = message.data.strip()
        try:
            mode = int(raw.split(':', 1)[0])
        except (TypeError, ValueError):
            if raw != self._unknown_mode_state:
                self._unknown_mode_state = raw
                self._write(f'[DRIVE MODE] {raw or "UNKNOWN"}')
            return
        if mode not in MODE_TITLES or mode == self._mode:
            return
        self._mode = mode
        self._pending_detection = False
        self._detection_timer.cancel()
        self._last_distance_log_m = None
        self._last_distance_log_time = 0.0
        self._print_mode(mode)
        if mode in (2, 3):
            text = waypoint_log_text(self._waypoint_state or '')
            if text:
                self._write(text)
            self._write('[조작] g=전체 웨이포인트 주행 | SPACE=다음 1개')
        if mode == 3:
            if self._sensor_state and self._sensor_state.upper() != 'ONLINE':
                self._write(f'[센서 경고] MMWAVE {self._sensor_state}')
            if self._presence:
                self._emit_detection()

    def _on_waypoint_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._waypoint_state:
            return
        self._waypoint_state = state
        if self._mode not in (2, 3):
            return
        text = waypoint_log_text(state)
        if text:
            self._write(text)

    def _on_follower_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._follower_state:
            return
        self._follower_state = state
        important = {
            'EMERGENCY_STOP': '전방 안전 정지',
            'NO_PATH': '주행 가능한 경로 없음',
            "ROTATION_BLOCKED": "제자리 회전 공간 부족",
            "ROTATION_ODOMETRY_STALE": "회전각 갱신 없음 - 안전 정지",
        }
        if self._mode in (2, 3) and state in important:
            self._write(f'[주행 경고] {important[state]}')

    def _on_dynamic_obstacle(self, message: Bool) -> None:
        detected = bool(message.data)
        if detected == self._dynamic_detected:
            return
        self._dynamic_detected = detected
        if self._mode == 3:
            self._write(
                '[동적장애물] 감지됨' if detected
                else '[동적장애물] 감지 해제'
            )

    def _on_sensor_state(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._sensor_state:
            return
        self._sensor_state = state
        if self._mode == 3 and state.upper() != 'ONLINE':
            self._write(f'[센서 경고] MMWAVE {state}')

    def _on_victim_fusion(self, message: String) -> None:
        state = message.data.strip()
        if not state or state == self._victim_state:
            return
        self._victim_state = state
        if self._mode != 3:
            return
        upper = state.upper()
        if upper.startswith('INSPECTING:'):
            try:
                distance = float(state.split(':', 1)[1])
                self._write(
                    f'[판별] 동적 후보 {distance:.1f}m - 정지 후 MMWAVE 확인 중'
                )
            except (TypeError, ValueError):
                self._write('[판별] 정지 후 MMWAVE 확인 중')
            return
        if upper.startswith('OBSTACLE:'):
            self._write('[판별] 사람 아님 - 동적장애물 회피 재개')
            return
        if not upper.startswith('DETECTED:'):
            return
        try:
            coordinates = state.split(':', 1)[1]
            x_text, y_text = coordinates.split(',', 1)
            self._write(
                f'[요구조자] 확정됨 (x={float(x_text):.1f}, '
                f'y={float(y_text):.1f})'
            )
        except (TypeError, ValueError):
            self._write('[요구조자] 확정됨')

    def _on_thermal_route(self, message: String) -> None:
        state = message.data.strip().upper()
        if not state or state == self._thermal_route_state:
            return
        self._thermal_route_state = state
        if self._mode == 3 and state == "THERMAL_DANGER:EXIT3":
            self._write("이동경로 중 온도 증가 감지! ->exit2 danger expected")
            self._write("exit3으로 경로를 변경합니다.")

    @staticmethod
    def _valid_distance(value: float) -> bool:
        return math.isfinite(value) and value > 0.0

    def _detection_text(self) -> str:
        if self._distance_m is None:
            return '[사람] 감지됨'
        return f'[사람] 감지됨, 거리 약 {self._distance_m:.1f}m'

    def _record_distance_log(self, now: float) -> None:
        self._last_distance_log_time = now
        self._last_distance_log_m = self._distance_m

    def _emit_detection(self) -> None:
        self._write(self._detection_text())
        self._record_distance_log(time.monotonic())
        self._pending_detection = False
        self._detection_timer.cancel()

    def _flush_pending_detection(self) -> None:
        if self._pending_detection and self._presence:
            self._emit_detection()
        else:
            self._detection_timer.cancel()

    def _on_presence(self, message: Bool) -> None:
        present = bool(message.data)
        previous = self._presence
        self._presence = present
        if self._mode != 3 or present == previous:
            return
        if present:
            self._pending_detection = True
            if self._distance_m is not None:
                self._emit_detection()
            else:
                self._detection_timer.reset()
            return
        if self._pending_detection:
            self._pending_detection = False
            self._detection_timer.cancel()
        self._distance_m = None
        self._last_distance_log_m = None
        self._last_distance_log_time = 0.0
        if previous:
            self._write('[사람] 감지 해제')

    def _on_distance(self, message: Float32) -> None:
        measured = float(message.data)
        if not self._valid_distance(measured):
            return
        self._distance_m = measured
        if self._mode != 3 or not self._presence:
            return

        if self._pending_detection:
            self._emit_detection()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = StatusConsole()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f'mmwave_status_console error: {exc}', flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
