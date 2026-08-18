"""Parse human mission text and publish a semantic PoseStamped goal."""

from __future__ import annotations

import math
import re
from pathlib import Path
from typing import Dict, Optional, Tuple

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, String
import yaml

from .grid_utils import quaternion_from_yaw
from .project_paths import project_path
from .tf_utils import TfHelper


DEFAULT_SEMANTIC = project_path(
    'inno_jazzy_ws', 'src', 'inno_autonav', 'config', 'semantic_points.yaml'
)


class ThermalRerouteTrigger:
    """Confirm sustained heat and latch one reroute per mode-3 session."""

    def __init__(self, threshold_c: float = 40.0, required_hits: int = 3) -> None:
        if threshold_c <= 0.0 or required_hits < 1:
            raise ValueError("thermal threshold and hit count must be positive")
        self.threshold_c = float(threshold_c)
        self.required_hits = int(required_hits)
        self.hits = 0
        self.triggered = False

    def reset(self) -> None:
        self.hits = 0
        self.triggered = False

    def update(self, temperature_c: float, enabled: bool) -> bool:
        if not enabled or self.triggered:
            self.hits = 0
            return False
        if not math.isfinite(temperature_c) or temperature_c < self.threshold_c:
            self.hits = 0
            return False
        self.hits += 1
        if self.hits < self.required_hits:
            return False
        self.triggered = True
        return True


def normalize_label(label: str, aliases: Optional[Dict[str, str]] = None) -> str:
    normalized = re.sub(r'\s+', '', label).strip().lower()
    shorthand = re.fullmatch(r'e(\d+)', normalized)
    if shorthand:
        normalized = f'exit{shorthand.group(1)}'
    if aliases:
        normalized = aliases.get(normalized, normalized)
    return normalized


def parse_mission(text: str) -> Tuple[Optional[str], str]:
    cleaned = text.strip()
    if not cleaned:
        raise ValueError('빈 mission 명령입니다.')
    korean = re.fullmatch(
        r'\s*([A-Za-z0-9_-]+)\s*에서\s*([A-Za-z0-9_-]+)\s*로\s*가(?:줘)?\s*',
        cleaned,
        flags=re.IGNORECASE,
    )
    if korean:
        return korean.group(1), korean.group(2)
    english_to = re.fullmatch(
        r'\s*([A-Za-z0-9_-]+)\s+to\s+([A-Za-z0-9_-]+)\s*',
        cleaned,
        flags=re.IGNORECASE,
    )
    if english_to:
        return english_to.group(1), english_to.group(2)
    tokens = cleaned.split()
    if tokens and tokens[0].lower() == 'go':
        tokens = tokens[1:]
    if len(tokens) == 1:
        return None, tokens[0]
    if len(tokens) == 2:
        return tokens[0], tokens[1]
    raise ValueError(
        '지원 형식: go exit2 | go exit1 exit2 | exit1에서 exit2로가 | exit1 to exit2'
    )


def load_semantic_points(path: str) -> Tuple[Dict[str, Dict], Dict[str, str]]:
    semantic_path = Path(path).expanduser().resolve(strict=False)
    if not semantic_path.is_file():
        raise ValueError(f'semantic YAML 파일이 없습니다: {semantic_path}')
    try:
        document = yaml.safe_load(semantic_path.read_text(encoding='utf-8'))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError(f'semantic YAML을 읽을 수 없습니다: {exc}') from exc
    if not isinstance(document, dict):
        raise ValueError('semantic YAML 최상위 값은 mapping이어야 합니다.')
    raw_points = document.get('semantic_points')
    if raw_points is None:
        raw_points = document.get('poses')
    if not isinstance(raw_points, dict):
        raise ValueError('semantic_points 또는 poses 항목이 필요합니다.')
    points = {}
    for name, value in raw_points.items():
        if not isinstance(value, dict):
            raise ValueError(f'{name} 좌표는 mapping이어야 합니다.')
        try:
            points[str(name).lower()] = {
                'frame_id': str(value.get('frame_id', document.get('frame_id', 'map'))),
                'x': float(value['x']),
                'y': float(value['y']),
                'yaw': float(value.get('yaw', 0.0)),
            }
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'{name} 좌표가 올바르지 않습니다.') from exc
    raw_aliases = document.get('aliases', {})
    if not isinstance(raw_aliases, dict):
        raise ValueError('aliases 항목은 mapping이어야 합니다.')
    aliases = {str(k).lower(): str(v).lower() for k, v in raw_aliases.items()}
    return points, aliases


class MissionCommander(Node):
    def __init__(self) -> None:
        super().__init__('mission_commander')
        self.declare_parameter('semantic_yaml', DEFAULT_SEMANTIC)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_source_if_no_tf', False)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.use_source_if_no_tf = bool(
            self.get_parameter('use_source_if_no_tf').value
        )
        semantic_yaml = str(self.get_parameter('semantic_yaml').value)
        self.points, self.aliases = load_semantic_points(semantic_yaml)
        self.tf = TfHelper(self)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.state_publisher = self.create_publisher(String, '/mission_state', 10)
        self.create_subscription(String, '/mission_text', self._mission_callback, 10)
        self.declare_parameter("thermal_threshold_c", 40.0)
        self.declare_parameter("thermal_confirmation_hits", 3)
        threshold = float(self.get_parameter("thermal_threshold_c").value)
        hits = int(self.get_parameter("thermal_confirmation_hits").value)
        self.thermal_trigger = ThermalRerouteTrigger(threshold, hits)
        self.drive_mode = 1
        self.thermal_status_publisher = self.create_publisher(
            String, "/thermal_route_status", 10
        )
        self.create_subscription(
            Float32, "/thermal/max_temperature_c", self._thermal_callback, 10
        )
        self.create_subscription(Int32, "/drive_mode", self._drive_mode_callback, 10)
        self._state("READY")
        self.get_logger().info(
            f'mission commander: {len(self.points)} semantic points, {semantic_yaml}'
        )

    def _drive_mode_callback(self, message: Int32) -> None:
        mode = int(message.data)
        if mode == self.drive_mode:
            return
        self.drive_mode = mode
        self.thermal_trigger.reset()

    def _thermal_callback(self, message: Float32) -> None:
        if self.thermal_trigger.update(
            float(message.data), enabled=self.drive_mode == 3
        ):
            self.thermal_status_publisher.publish(String(data="THERMAL_DANGER:EXIT3"))

    def _resolve(self, label: str) -> Tuple[str, Dict]:
        normalized = normalize_label(label, self.aliases)
        if normalized not in self.points:
            raise ValueError(
                f'알 수 없는 위치 {label!r}; 사용 가능: {", ".join(sorted(self.points))}'
            )
        return normalized, self.points[normalized]

    def _mission_callback(self, message: String) -> None:
        try:
            source_label, destination_label = parse_mission(message.data)
            destination_name, destination = self._resolve(destination_label)
            current_pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
            start_description = 'CURRENT_LOCALIZATION'
            if current_pose is None:
                if source_label and self.use_source_if_no_tf:
                    source_name, source = self._resolve(source_label)
                    current_pose = (source['x'], source['y'], source['yaw'])
                    start_description = f'DEBUG_SOURCE:{source_name}'
                else:
                    self._state('WAITING_FOR_LOCALIZATION')
                    self.get_logger().error(
                        '현재 map->base_link TF가 없어 mission을 시작하지 않았습니다.'
                    )
                    return
            if source_label:
                self.get_logger().info(
                    f'요청 source={normalize_label(source_label, self.aliases)}, '
                    f'실제 시작점={start_description} '
                    f'({current_pose[0]:.3f}, {current_pose[1]:.3f})'
                )

            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = destination['frame_id'] or self.map_frame
            goal.pose.position.x = destination['x']
            goal.pose.position.y = destination['y']
            qx, qy, qz, qw = quaternion_from_yaw(destination['yaw'])
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
            self.goal_publisher.publish(goal)
            self._state(f'GOAL_PUBLISHED:{destination_name}')
            self.get_logger().info(
                f'goal={destination_name}: x={destination["x"]:.3f}, '
                f'y={destination["y"]:.3f}, yaw={destination["yaw"]:.3f}'
            )
        except ValueError as exc:
            self._state(f'ERROR:{exc}')
            self.get_logger().error(str(exc))

    def _state(self, text: str) -> None:
        self.state_publisher.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = MissionCommander()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f'mission_commander 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
