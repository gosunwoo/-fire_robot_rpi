"""Infer and latch rescuee markers from mmWave presence and LiDAR obstacles."""

from copy import deepcopy
from dataclasses import dataclass
import math
import time
from typing import Iterable, List, Optional, Sequence, Tuple

from geometry_msgs.msg import Point
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


XY = Tuple[float, float]


@dataclass(frozen=True)
class LidarCluster:
    x: float
    y: float
    point_count: int
    range_m: float
    range_error_m: float


@dataclass
class InspectionWindow:
    settle_sec: float
    timeout_sec: float
    required_hits: int
    confirmation_sec: float
    candidate: Optional[LidarCluster] = None
    started_at: float = 0.0
    matched_hits: int = 0
    first_match_at: Optional[float] = None

    def begin(self, candidate: LidarCluster, now: float) -> None:
        self.candidate = candidate
        self.started_at = float(now)
        self.matched_hits = 0
        self.first_match_at = None

    def clear(self) -> None:
        self.candidate = None
        self.matched_hits = 0
        self.first_match_at = None

    def evaluate(self, now: float, matched: bool) -> str:
        if self.candidate is None:
            return "IDLE"
        now = float(now)
        if now - self.started_at < self.settle_sec:
            return "SETTLING"
        if matched:
            if self.first_match_at is None:
                self.first_match_at = now
                self.matched_hits = 1
            else:
                self.matched_hits += 1
            if (
                self.matched_hits >= self.required_hits
                and now - self.first_match_at >= self.confirmation_sec
            ):
                return "PERSON"
        else:
            self.matched_hits = 0
            self.first_match_at = None
        if now - self.started_at >= self.timeout_sec:
            return "OBSTACLE"
        return "INSPECTING"



@dataclass
class EvidenceTrack:
    x: float
    y: float
    hits: int
    first_seen: float
    last_seen: float


def cluster_points(points: Sequence[XY], connection_radius_m: float) -> List[List[XY]]:
    """Group nearby dynamic-map cells using a small spatial hash."""

    if connection_radius_m <= 0.0:
        raise ValueError('connection_radius_m must be positive')
    if not points:
        return []
    size = float(connection_radius_m)
    buckets = {}
    for index, (x, y) in enumerate(points):
        key = (math.floor(x / size), math.floor(y / size))
        buckets.setdefault(key, []).append(index)

    visited = set()
    clusters = []
    for start in range(len(points)):
        if start in visited:
            continue
        visited.add(start)
        pending = [start]
        members = []
        while pending:
            current = pending.pop()
            members.append(points[current])
            x, y = points[current]
            bx, by = math.floor(x / size), math.floor(y / size)
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for candidate in buckets.get((bx + dx, by + dy), ()):
                        if candidate in visited:
                            continue
                        px, py = points[candidate]
                        if math.hypot(px - x, py - y) <= size:
                            visited.add(candidate)
                            pending.append(candidate)
        clusters.append(members)
    return clusters


def valid_mmwave_match_distance(distance_m: float, maximum_m: float) -> bool:
    """Accept only finite positive mmWave ranges inside the rescue zone."""

    return (
        math.isfinite(float(distance_m))
        and math.isfinite(float(maximum_m))
        and 0.0 < float(distance_m) <= float(maximum_m)
    )


def select_unique_range_match(
    points: Sequence[XY],
    *,
    robot_x: float,
    robot_y: float,
    sensor_yaw_rad: float,
    horizontal_fov_rad: float,
    mmwave_distance_m: float,
    cluster_radius_m: float,
    min_cluster_points: int,
    max_cluster_points: int,
    absolute_tolerance_m: float,
    relative_tolerance: float,
    maximum_tolerance_m: float,
    ambiguity_margin_m: float,
) -> Optional[LidarCluster]:
    """Pick one small LiDAR cluster only when its range match is unambiguous."""

    tolerance = min(
        maximum_tolerance_m,
        max(absolute_tolerance_m, mmwave_distance_m * relative_tolerance),
    )
    matches = []
    for members in cluster_points(points, cluster_radius_m):
        if not min_cluster_points <= len(members) <= max_cluster_points:
            continue
        x = sum(point[0] for point in members) / len(members)
        y = sum(point[1] for point in members) / len(members)
        bearing = math.atan2(y - robot_y, x - robot_x)
        bearing_error = math.atan2(
            math.sin(bearing - sensor_yaw_rad),
            math.cos(bearing - sensor_yaw_rad),
        )
        if abs(bearing_error) > horizontal_fov_rad / 2.0:
            continue
        lidar_range = math.hypot(x - robot_x, y - robot_y)
        error = abs(lidar_range - mmwave_distance_m)
        if error <= tolerance:
            matches.append(LidarCluster(x, y, len(members), lidar_range, error))
    matches.sort(key=lambda item: item.range_error_m)
    if not matches:
        return None
    if (
        len(matches) > 1
        and matches[1].range_error_m - matches[0].range_error_m
        < ambiguity_margin_m
    ):
        return None
    return matches[0]


def select_forward_inspection_cluster(
    points: Sequence[XY],
    *,
    robot_x: float,
    robot_y: float,
    sensor_yaw_rad: float,
    horizontal_fov_rad: float,
    min_range_m: float,
    max_range_m: float,
    cluster_radius_m: float,
    excluded_positions: Sequence[XY] = (),
    exclusion_radius_m: float = 0.60,
    ambiguity_margin_m: float = 0.35,
) -> Optional[LidarCluster]:
    """Pick one nearest confirmed front obstacle for stopped inspection."""

    options = []
    for members in cluster_points(points, cluster_radius_m):
        x = sum(point[0] for point in members) / len(members)
        y = sum(point[1] for point in members) / len(members)
        if any(
            math.hypot(x - px, y - py) <= exclusion_radius_m
            for px, py in excluded_positions
        ):
            continue
        distance = math.hypot(x - robot_x, y - robot_y)
        if not min_range_m <= distance <= max_range_m:
            continue
        bearing = math.atan2(y - robot_y, x - robot_x)
        error = math.atan2(
            math.sin(bearing - sensor_yaw_rad),
            math.cos(bearing - sensor_yaw_rad),
        )
        if abs(error) > horizontal_fov_rad / 2.0:
            continue
        options.append(LidarCluster(x, y, len(members), distance, 0.0))
    options.sort(key=lambda item: item.range_m)
    if not options:
        return None
    if (
        len(options) > 1
        and options[1].range_m - options[0].range_m < ambiguity_margin_m
    ):
        return None
    return options[0]


class RescueeTracker:
    """Require repeated spatial matches, then latch map positions until reset."""

    def __init__(
        self,
        *,
        confirm_hits: int,
        confirm_sec: float,
        evidence_timeout_sec: float,
        track_radius_m: float,
        merge_radius_m: float,
    ) -> None:
        if confirm_hits < 2:
            raise ValueError('confirm_hits must be at least two')
        if min(confirm_sec, evidence_timeout_sec, track_radius_m, merge_radius_m) <= 0.0:
            raise ValueError('tracker durations and radii must be positive')
        self.confirm_hits = int(confirm_hits)
        self.confirm_sec = float(confirm_sec)
        self.evidence_timeout = float(evidence_timeout_sec)
        self.track_radius = float(track_radius_m)
        self.merge_radius = float(merge_radius_m)
        self.evidence: List[EvidenceTrack] = []
        self.victims: List[XY] = []

    def _near(self, x: float, y: float, positions: Iterable[XY], radius: float) -> bool:
        return any(math.hypot(px - x, py - y) <= radius for px, py in positions)

    def expire(self, now: float) -> None:
        cutoff = float(now) - self.evidence_timeout
        self.evidence = [item for item in self.evidence if item.last_seen >= cutoff]

    def observe(self, cluster: LidarCluster, now: float) -> Optional[XY]:
        now = float(now)
        self.expire(now)
        if self._near(cluster.x, cluster.y, self.victims, self.merge_radius):
            return None
        nearby = [
            item for item in self.evidence
            if math.hypot(item.x - cluster.x, item.y - cluster.y)
            <= self.track_radius
        ]
        if nearby:
            track = min(
                nearby,
                key=lambda item: math.hypot(item.x - cluster.x, item.y - cluster.y),
            )
            total = track.hits + 1
            track.x = (track.x * track.hits + cluster.x) / total
            track.y = (track.y * track.hits + cluster.y) / total
            track.hits = total
            track.last_seen = now
        else:
            track = EvidenceTrack(cluster.x, cluster.y, 1, now, now)
            self.evidence.append(track)
        if (
            track.hits >= self.confirm_hits
            and track.last_seen - track.first_seen >= self.confirm_sec
        ):
            victim = (track.x, track.y)
            self.victims.append(victim)
            self.evidence = [item for item in self.evidence if item is not track]
            return victim
        return None

    def clear(self) -> None:
        self.evidence.clear()
        self.victims.clear()


def extract_dynamic_points(message: MarkerArray) -> List[XY]:
    points = []
    for marker in message.markers:
        if marker.action != Marker.ADD or marker.type != Marker.SPHERE_LIST:
            continue
        points.extend((float(point.x), float(point.y)) for point in marker.points)
    return points


def follow_victim_positions(
    victims: Sequence[XY],
    observations: Sequence[XY],
    *,
    cluster_radius_m: float,
    min_cluster_points: int,
    max_cluster_points: int,
    follow_radius_m: float,
    ambiguity_margin_m: float,
) -> List[XY]:
    """Follow each rescuee with one unique nearby configured-size scan cluster."""

    centers = []
    for members in cluster_points(observations, cluster_radius_m):
        if not min_cluster_points <= len(members) <= max_cluster_points:
            continue
        centers.append((
            sum(point[0] for point in members) / len(members),
            sum(point[1] for point in members) / len(members),
        ))
    updated = list(victims)
    used = set()
    for victim_index, (old_x, old_y) in enumerate(victims):
        options = sorted(
            (
                (math.hypot(x - old_x, y - old_y), index, (x, y))
                for index, (x, y) in enumerate(centers)
                if index not in used
                and math.hypot(x - old_x, y - old_y) <= follow_radius_m
            ),
            key=lambda item: item[0],
        )
        if not options:
            continue
        if len(options) > 1 and options[1][0] - options[0][0] < ambiguity_margin_m:
            continue
        _, center_index, position = options[0]
        used.add(center_index)
        updated[victim_index] = position
    return updated


def filtered_dynamic_markers(
    message: MarkerArray, victims: Sequence[XY], suppression_radius_m: float
) -> MarkerArray:
    """Hide red points near latched victims while preserving all marker metadata."""

    result = deepcopy(message)
    for marker in result.markers:
        if marker.action != Marker.ADD or marker.type != Marker.SPHERE_LIST:
            continue
        marker.points = [
            point for point in marker.points
            if not any(
                math.hypot(point.x - x, point.y - y) <= suppression_radius_m
                for x, y in victims
            )
        ]
    return result


def recolored_dynamic_markers(
    message: MarkerArray, red: float, green: float, blue: float, alpha: float
) -> MarkerArray:
    """Return a copy with every obstacle sphere recolored for display."""

    result = deepcopy(message)
    for marker in result.markers:
        if marker.action != Marker.ADD or marker.type != Marker.SPHERE_LIST:
            continue
        marker.color.r = float(red)
        marker.color.g = float(green)
        marker.color.b = float(blue)
        marker.color.a = float(alpha)
    return result


class VictimFusionNode(Node):
    """Fuse coarse mmWave range with small confirmed LiDAR obstacle clusters."""

    def __init__(self) -> None:
        super().__init__('victim_fusion')
        defaults = {
            'fixed_frame': 'map',
            'base_frame': 'base_link',
            'mmwave_stale_timeout_sec': 1.5,
            'max_match_distance_m': 4.0,
            'sensor_yaw_offset_deg': 0.0,
            'sensor_horizontal_fov_deg': 100.0,
            'cluster_radius_m': 0.22,
            'min_cluster_points': 2,
            'max_cluster_points': 15,
            'max_follow_cluster_points': 80,
            'victim_follow_radius_m': 0.65,
            'victim_follow_ambiguity_margin_m': 0.15,
            'victim_history_spacing_m': 0.10,
            'distance_tolerance_m': 0.80,
            'relative_distance_tolerance': 0.25,
            'maximum_distance_tolerance_m': 1.50,
            'ambiguity_margin_m': 0.35,
            'confirmation_hits': 3,
            'confirmation_sec': 0.50,
            'evidence_timeout_sec': 1.50,
            'track_radius_m': 0.35,
            'victim_merge_radius_m': 0.55,
            'red_suppression_radius_m': 0.50,
            'victim_marker_diameter_m': 0.75,
            'inspection_min_range_m': 1.5,
            'inspection_max_range_m': 4.0,
            'inspection_settle_sec': 1.0,
            'inspection_timeout_sec': 4.0,
            'inspection_confirmation_hits': 3,
            'inspection_confirmation_sec': 0.5,
            'inspection_reject_hold_sec': 60.0,
            'inspection_reject_radius_m': 0.60,
            'publish_rate_hz': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.fixed_frame = str(self.get_parameter('fixed_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.mmwave_stale_timeout = float(
            self.get_parameter('mmwave_stale_timeout_sec').value
        )
        self.max_match_distance = float(
            self.get_parameter('max_match_distance_m').value
        )
        self.sensor_yaw_offset = math.radians(float(
            self.get_parameter('sensor_yaw_offset_deg').value
        ))
        self.sensor_horizontal_fov = math.radians(float(
            self.get_parameter('sensor_horizontal_fov_deg').value
        ))
        self.cluster_radius = float(self.get_parameter('cluster_radius_m').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.max_cluster_points = int(self.get_parameter('max_cluster_points').value)
        self.max_follow_cluster_points = int(
            self.get_parameter('max_follow_cluster_points').value
        )
        self.victim_follow_radius = float(
            self.get_parameter('victim_follow_radius_m').value
        )
        self.victim_follow_ambiguity_margin = float(
            self.get_parameter('victim_follow_ambiguity_margin_m').value
        )
        self.victim_history_spacing = float(
            self.get_parameter('victim_history_spacing_m').value
        )
        self.distance_tolerance = float(
            self.get_parameter('distance_tolerance_m').value
        )
        self.relative_tolerance = float(
            self.get_parameter('relative_distance_tolerance').value
        )
        self.maximum_tolerance = float(
            self.get_parameter('maximum_distance_tolerance_m').value
        )
        self.ambiguity_margin = float(
            self.get_parameter('ambiguity_margin_m').value
        )
        self.red_suppression_radius = float(
            self.get_parameter('red_suppression_radius_m').value
        )
        self.marker_diameter = float(
            self.get_parameter('victim_marker_diameter_m').value
        )
        self.inspection_min_range = float(
            self.get_parameter('inspection_min_range_m').value
        )
        self.inspection_max_range = float(
            self.get_parameter('inspection_max_range_m').value
        )
        self.inspection_reject_hold = float(
            self.get_parameter('inspection_reject_hold_sec').value
        )
        self.inspection_reject_radius = float(
            self.get_parameter('inspection_reject_radius_m').value
        )
        inspection_hits = int(
            self.get_parameter('inspection_confirmation_hits').value
        )
        self.inspection = InspectionWindow(
            settle_sec=float(self.get_parameter('inspection_settle_sec').value),
            timeout_sec=float(self.get_parameter('inspection_timeout_sec').value),
            required_hits=inspection_hits,
            confirmation_sec=float(
                self.get_parameter('inspection_confirmation_sec').value
            ),
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        numeric = (
            self.mmwave_stale_timeout, self.max_match_distance,
            self.cluster_radius,
            self.distance_tolerance, self.relative_tolerance,
            self.maximum_tolerance, self.ambiguity_margin,
            self.victim_follow_radius,
            self.victim_follow_ambiguity_margin, self.victim_history_spacing,
            self.red_suppression_radius, self.marker_diameter,
            self.inspection_min_range, self.inspection_max_range,
            self.inspection_reject_hold, self.inspection_reject_radius,
            self.inspection.settle_sec, self.inspection.timeout_sec,
            self.inspection.confirmation_sec, rate,
        )
        if (
            not self.fixed_frame
            or not self.base_frame
            or self.min_cluster_points < 2
            or self.max_cluster_points < self.min_cluster_points
            or self.max_follow_cluster_points < self.max_cluster_points
            or any(value <= 0.0 for value in numeric)
            or self.inspection_min_range >= self.inspection_max_range
            or self.inspection.required_hits < 2
            or not math.isfinite(self.sensor_yaw_offset)
            or not 0.0 < self.sensor_horizontal_fov <= 2.0 * math.pi
        ):
            raise ValueError('victim fusion parameters are invalid')
        self.tracker = RescueeTracker(
            confirm_hits=int(self.get_parameter('confirmation_hits').value),
            confirm_sec=float(self.get_parameter('confirmation_sec').value),
            evidence_timeout_sec=float(
                self.get_parameter('evidence_timeout_sec').value
            ),
            track_radius_m=float(self.get_parameter('track_radius_m').value),
            merge_radius_m=float(
                self.get_parameter('victim_merge_radius_m').value
            ),
        )

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.red_publisher = self.create_publisher(
            MarkerArray, '/dynamic_obstacle_markers_display', qos
        )
        self.victim_publisher = self.create_publisher(
            MarkerArray, '/victim_markers', qos
        )
        self.status_publisher = self.create_publisher(
            String, '/victim_fusion_status', qos
        )
        self.victim_detected_publisher = self.create_publisher(
            Bool, '/victim_detected', qos
        )
        self.inspection_hold_publisher = self.create_publisher(
            Bool, '/mode3_inspection_hold', qos
        )
        self.create_subscription(
            MarkerArray, '/dynamic_obstacle_markers', self._dynamic_callback, qos
        )
        live_qos = QoSProfile(depth=1)
        live_qos.reliability = ReliabilityPolicy.RELIABLE
        self.create_subscription(
            MarkerArray,
            '/dynamic_obstacle_observations',
            self._observation_callback,
            live_qos,
        )
        self.create_subscription(
            Bool, '/mmwave/filtered_presence', self._presence_callback, qos
        )
        self.create_subscription(
            Float32, '/mmwave/filtered_distance_m', self._distance_callback, qos
        )
        self.create_subscription(
            String, '/mmwave/sensor_state', self._sensor_callback, qos
        )
        self.create_subscription(Int32, '/drive_mode', self._mode_callback, 10)
        self.create_subscription(
            String, '/waypoint_queue_status', self._mission_callback, 10
        )
        self.create_service(Trigger, '/clear_victims', self._clear_callback)
        self.drive_mode = 1
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.mmwave_presence = False
        self.mmwave_distance: Optional[float] = None
        self.sensor_online = False
        self.presence_updated = float('-inf')
        self.distance_updated = float('-inf')
        self.last_dynamic: Optional[MarkerArray] = None
        self.victim_history: List[XY] = []
        self.rejected_inspections: List[Tuple[float, float, float]] = []
        self.inspection_hold = False
        self.create_timer(1.0 / rate, self._publish_victims)
        self._publish_victims()

    def _presence_callback(self, message: Bool) -> None:
        self.mmwave_presence = bool(message.data)
        self.presence_updated = time.monotonic()

    def _distance_callback(self, message: Float32) -> None:
        distance = float(message.data)
        self.mmwave_distance = distance if math.isfinite(distance) and distance > 0.0 else None
        self.distance_updated = time.monotonic()

    def _sensor_callback(self, message: String) -> None:
        self.sensor_online = message.data.strip().upper() == 'ONLINE'

    def _mmwave_ready(self, now: float) -> bool:
        return (
            self.sensor_online
            and self.mmwave_presence
            and self.mmwave_distance is not None
            and valid_mmwave_match_distance(
                self.mmwave_distance, self.max_match_distance
            )
            and now - self.presence_updated <= self.mmwave_stale_timeout
            and now - self.distance_updated <= self.mmwave_stale_timeout
        )

    def _robot_pose(self):
        try:
            transform = self.buffer.lookup_transform(
                self.fixed_frame, self.base_frame, rclpy.time.Time()
            )
        except TransformException:
            return None
        origin = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
            1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
        )
        return float(origin.x), float(origin.y), yaw

    def _set_inspection_hold(self, enabled: bool) -> None:
        self.inspection_hold = bool(enabled)
        self.inspection_hold_publisher.publish(
            Bool(data=self.inspection_hold)
        )

    def _cancel_inspection(self, clear_rejections: bool = False) -> None:
        self.inspection.clear()
        self._set_inspection_hold(False)
        if clear_rejections:
            self.rejected_inspections.clear()

    def _consider_inspection(self, message: MarkerArray) -> None:
        if self.drive_mode != 3 or self.inspection.candidate is not None:
            return
        pose = self._robot_pose()
        if pose is None:
            return
        now = time.monotonic()
        self.rejected_inspections = [
            item for item in self.rejected_inspections
            if now - item[2] <= self.inspection_reject_hold
        ]
        excluded = list(self.tracker.victims) + [
            (x, y) for x, y, _ in self.rejected_inspections
        ]
        candidate = select_forward_inspection_cluster(
            extract_dynamic_points(message),
            robot_x=pose[0],
            robot_y=pose[1],
            sensor_yaw_rad=pose[2] + self.sensor_yaw_offset,
            horizontal_fov_rad=self.sensor_horizontal_fov,
            min_range_m=self.inspection_min_range,
            max_range_m=self.inspection_max_range,
            cluster_radius_m=self.cluster_radius,
            excluded_positions=excluded,
            exclusion_radius_m=self.inspection_reject_radius,
            ambiguity_margin_m=self.ambiguity_margin,
        )
        if candidate is None:
            return
        self.inspection.begin(candidate, now)
        self._set_inspection_hold(True)
        self.status_publisher.publish(
            String(data=f'INSPECTING:{candidate.range_m:.2f}')
        )
        self.get_logger().warning(
            f'DYNAMIC INSPECTION HOLD: lidar={candidate.range_m:.2f}m'
        )

    def _advance_inspection(self, now: float) -> None:
        candidate = self.inspection.candidate
        if candidate is None:
            return
        tolerance = min(
            self.maximum_tolerance,
            max(
                self.distance_tolerance,
                candidate.range_m * self.relative_tolerance,
            ),
        )
        matched = (
            self._mmwave_ready(now)
            and self.mmwave_distance is not None
            and abs(self.mmwave_distance - candidate.range_m) <= tolerance
        )
        decision = self.inspection.evaluate(now, matched)
        if decision == 'PERSON':
            victim = (candidate.x, candidate.y)
            if not self.tracker._near(
                candidate.x, candidate.y,
                self.tracker.victims, self.tracker.merge_radius,
            ):
                self.tracker.victims.append(victim)
                self.victim_history.append(victim)
            self.get_logger().warning(
                'RESCUEE INFERRED AFTER STOP at '
                f'({candidate.x:.2f}, {candidate.y:.2f}); '
                f'lidar={candidate.range_m:.2f}m, '
                f'mmwave={self.mmwave_distance:.2f}m'
            )
            self.status_publisher.publish(
                String(data=f'DETECTED:{candidate.x:.2f},{candidate.y:.2f}')
            )
            self._cancel_inspection()
        elif decision == 'OBSTACLE':
            self.rejected_inspections.append(
                (candidate.x, candidate.y, float(now))
            )
            self.status_publisher.publish(
                String(data=f'OBSTACLE:{candidate.range_m:.2f}')
            )
            self.get_logger().info(
                f'NON-PERSON OBSTACLE: lidar={candidate.range_m:.2f}m; '
                'avoidance resumed'
            )
            self._cancel_inspection()

    def _mode_callback(self, message: Int32) -> None:
        mode = int(message.data)
        if mode not in (1, 2, 3) or mode == self.drive_mode:
            return
        self.drive_mode = mode
        self._cancel_inspection(clear_rejections=True)
        self.last_dynamic = None
        self._publish_dynamic_display()
        self._publish_victims()
        states = {
            1: 'INACTIVE:MODE1',
            2: 'ACTIVE:MODE2_DYNAMIC_CYAN',
            3: 'ACTIVE:MODE3_RESCUE',
        }
        self.status_publisher.publish(String(data=states[mode]))

    def _mission_callback(self, message: String) -> None:
        state = message.data.strip().upper()
        if state not in ('MISSION_COMPLETE', 'STEP_MISSION_COMPLETE', 'CLEARED'):
            return
        self.tracker.clear()
        self.victim_history.clear()
        self._cancel_inspection(clear_rejections=True)
        self.status_publisher.publish(String(data=f'CLEARED:{state}'))
        self._publish_dynamic_display()
        self._publish_victims()

    def _dynamic_callback(self, message: MarkerArray) -> None:
        self.last_dynamic = message
        self._publish_dynamic_display()
        self._consider_inspection(message)

    def _observation_callback(self, message: MarkerArray) -> None:
        if self.drive_mode != 3:
            return
        now = time.monotonic()
        points = extract_dynamic_points(message)
        updated = follow_victim_positions(
            self.tracker.victims,
            points,
            cluster_radius_m=self.cluster_radius,
            min_cluster_points=self.min_cluster_points,
            max_cluster_points=self.max_follow_cluster_points,
            follow_radius_m=self.victim_follow_radius,
            ambiguity_margin_m=self.victim_follow_ambiguity_margin,
        )
        self.tracker.victims[:] = updated
        for position in updated:
            if not any(
                math.hypot(position[0] - x, position[1] - y)
                < self.victim_history_spacing
                for x, y in self.victim_history
            ):
                self.victim_history.append(position)

        # Person inference remains independent from the mode-specific
        # generic obstacle channel: C4001 candidates are 2..15 scan points.
        discovery_points = points
        if self._mmwave_ready(now) and discovery_points:
            try:
                transform = self.buffer.lookup_transform(
                    self.fixed_frame, self.base_frame, rclpy.time.Time()
                )
            except TransformException:
                transform = None
            if transform is not None:
                origin = transform.transform.translation
                rotation = transform.transform.rotation
                robot_yaw = math.atan2(
                    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                )
                cluster = select_unique_range_match(
                    discovery_points,
                    robot_x=float(origin.x),
                    robot_y=float(origin.y),
                    sensor_yaw_rad=robot_yaw + self.sensor_yaw_offset,
                    horizontal_fov_rad=self.sensor_horizontal_fov,
                    mmwave_distance_m=float(self.mmwave_distance),
                    cluster_radius_m=self.cluster_radius,
                    min_cluster_points=self.min_cluster_points,
                    max_cluster_points=self.max_cluster_points,
                    absolute_tolerance_m=self.distance_tolerance,
                    relative_tolerance=self.relative_tolerance,
                    maximum_tolerance_m=self.maximum_tolerance,
                    ambiguity_margin_m=self.ambiguity_margin,
                )
                if cluster is not None:
                    victim = self.tracker.observe(cluster, now)
                    if victim is not None:
                        self.victim_history.append(victim)
                        self.get_logger().warning(
                            'RESCUEE INFERRED at '
                            f'({victim[0]:.2f}, {victim[1]:.2f}); '
                            f'lidar={cluster.range_m:.2f}m, '
                            f'mmwave={self.mmwave_distance:.2f}m'
                        )
                        self.status_publisher.publish(
                            String(data=f'DETECTED:{victim[0]:.2f},{victim[1]:.2f}')
                        )
                else:
                    self.tracker.expire(now)
        else:
            self.tracker.expire(now)
        self._publish_dynamic_display()
        self._publish_victims()

    def _clear_callback(self, request, response):
        del request
        count = len(self.tracker.victims)
        self.tracker.clear()
        self.victim_history.clear()
        self._cancel_inspection(clear_rejections=True)
        self.status_publisher.publish(String(data='CLEARED:MANUAL'))
        self._publish_dynamic_display()
        self._publish_victims()
        response.success = True
        response.message = f'{count}개 요구조자 기록을 삭제했습니다.'
        return response

    def _publish_dynamic_display(self) -> None:
        if self.drive_mode == 1 or self.last_dynamic is None:
            clear = Marker()
            clear.header.frame_id = self.fixed_frame
            clear.header.stamp = self.get_clock().now().to_msg()
            clear.action = Marker.DELETEALL
            self.red_publisher.publish(MarkerArray(markers=[clear]))
            return
        if self.drive_mode == 2:
            display = recolored_dynamic_markers(
                self.last_dynamic, 0.05, 0.85, 1.0, 0.90
            )
        else:
            display = filtered_dynamic_markers(
                self.last_dynamic,
                self.victim_history,
                self.red_suppression_radius,
            )
        self.red_publisher.publish(display)

    def _publish_victims(self) -> None:
        self._advance_inspection(time.monotonic())
        self.inspection_hold_publisher.publish(
            Bool(data=self.inspection_hold)
        )
        stamp = self.get_clock().now().to_msg()
        clear = Marker()
        clear.header.frame_id = self.fixed_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        marker = Marker()
        marker.header = clear.header
        marker.ns = 'inferred_rescuees'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_diameter
        marker.scale.y = self.marker_diameter
        marker.scale.z = 0.25
        marker.color.r = 0.05
        marker.color.g = 0.85
        marker.color.b = 1.0
        marker.color.a = 0.95
        positions = self.tracker.victims if self.drive_mode == 3 else ()
        self.victim_detected_publisher.publish(Bool(data=bool(positions)))
        marker.points = [
            Point(x=x, y=y, z=0.15) for x, y in positions
        ]
        self.victim_publisher.publish(MarkerArray(markers=[clear, marker]))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VictimFusionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
