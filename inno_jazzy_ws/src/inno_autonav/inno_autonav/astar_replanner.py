"""Merge static/dynamic grids and continuously replan an 8-connected A* path."""

from __future__ import annotations

import heapq
import math
import time
from typing import Dict, List, Optional, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .grid_utils import (
    MapGrid,
    bresenham,
    grid_to_world,
    inflate_occupied_cells,
    is_inside_grid,
    normalize_angle,
    path_cells_collision,
    quaternion_from_yaw,
    world_to_grid,
    yaw_from_quaternion,
)
from .tf_utils import TfHelper


Cell = Tuple[int, int]


def astar_search(
    data: np.ndarray,
    start: Cell,
    goal: Cell,
    unknown_is_occupied: bool = True,
    allow_diagonal: bool = True,
) -> List[Cell]:
    height, width = data.shape

    def blocked(cell: Cell) -> bool:
        x, y = cell
        if not (0 <= x < width and 0 <= y < height):
            return True
        value = int(data[y, x])
        return value >= 100 or (value < 0 and unknown_is_occupied)

    if blocked(start) or blocked(goal):
        return []
    straight = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0)]
    diagonal = [
        (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)), (-1, -1, math.sqrt(2.0)),
    ]
    neighbors = straight + diagonal if allow_diagonal else straight
    open_heap = [(0.0, 0.0, start)]
    came_from: Dict[Cell, Cell] = {}
    costs: Dict[Cell, float] = {start: 0.0}
    closed = set()
    while open_heap:
        _, current_cost, current = heapq.heappop(open_heap)
        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))
        closed.add(current)
        for dx, dy, move_cost in neighbors:
            candidate = current[0] + dx, current[1] + dy
            if blocked(candidate):
                continue
            # Do not cut across the corner of two occupied cells.
            if dx and dy and (
                blocked((current[0] + dx, current[1]))
                or blocked((current[0], current[1] + dy))
            ):
                continue
            new_cost = current_cost + move_cost
            if new_cost >= costs.get(candidate, math.inf):
                continue
            costs[candidate] = new_cost
            came_from[candidate] = current
            heuristic = math.hypot(goal[0] - candidate[0], goal[1] - candidate[1])
            heapq.heappush(open_heap, (new_cost + heuristic, new_cost, candidate))
    return []


def simplify_path(
    path: List[Cell], data: np.ndarray, unknown_is_occupied: bool
) -> List[Cell]:
    if len(path) <= 2:
        return path
    simplified = [path[0]]
    anchor = 0
    while anchor < len(path) - 1:
        candidate = len(path) - 1
        while candidate > anchor + 1:
            cells = bresenham(path[anchor], path[candidate])
            if not path_cells_collision(cells, data, unknown_is_occupied):
                break
            candidate -= 1
        simplified.append(path[candidate])
        anchor = candidate
    return simplified


def remaining_path_from_nearest(path: List[Cell], current: Cell) -> List[Cell]:
    """Drop cells already passed so obstacles behind cannot trigger replans."""

    if not path:
        return []
    nearest = min(
        range(len(path)),
        key=lambda index: (
            (path[index][0] - current[0]) ** 2
            + (path[index][1] - current[1]) ** 2
        ),
    )
    return path[nearest:]


def message_to_grid(message: OccupancyGrid) -> MapGrid:
    width, height = int(message.info.width), int(message.info.height)
    if width <= 0 or height <= 0 or len(message.data) != width * height:
        raise ValueError('OccupancyGrid geometry/data 길이가 올바르지 않습니다.')
    return MapGrid(
        width=width,
        height=height,
        resolution=float(message.info.resolution),
        origin_x=float(message.info.origin.position.x),
        origin_y=float(message.info.origin.position.y),
        origin_yaw=yaw_from_quaternion(message.info.origin.orientation),
        frame_id=message.header.frame_id,
        data=np.asarray(message.data, dtype=np.int8).reshape(height, width),
    )


def footprint_clearance_radius(
    robot_length_m: float, robot_width_m: float, safety_margin_m: float
) -> float:
    """Conservative circular footprint for in-place skid steering."""

    if robot_length_m <= 0.0 or robot_width_m <= 0.0 or safety_margin_m < 0.0:
        raise ValueError('robot dimensions must be positive and margin nonnegative')
    return 0.5 * math.hypot(robot_length_m, robot_width_m) + safety_margin_m


class AstarReplanner(Node):
    def __init__(self) -> None:
        super().__init__('astar_replanner')
        defaults = {
            'map_frame': 'map',
            'base_frame': 'base_link',
            'unknown_is_occupied': True,
            'replan_rate_hz': 2.0,
            'path_block_check_radius': 0.0,
            'robot_length_m': 0.39,
            'robot_width_m': 0.20,
            'safety_margin_m': 0.10,
            'dynamic_replan_stop_sec': 0.35,
            'start_clearance_radius': 0.18,
            'allow_diagonal': True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.unknown_is_occupied = bool(
            self.get_parameter('unknown_is_occupied').value
        )
        self.replan_rate = float(self.get_parameter('replan_rate_hz').value)
        configured_clearance = float(
            self.get_parameter('path_block_check_radius').value
        )
        self.robot_length = float(self.get_parameter('robot_length_m').value)
        self.robot_width = float(self.get_parameter('robot_width_m').value)
        self.safety_margin = float(self.get_parameter('safety_margin_m').value)
        automatic_clearance = footprint_clearance_radius(
            self.robot_length, self.robot_width, self.safety_margin
        )
        self.clearance_radius = (
            configured_clearance if configured_clearance > 0.0
            else automatic_clearance
        )
        self.dynamic_replan_stop = float(
            self.get_parameter('dynamic_replan_stop_sec').value
        )
        self.start_clearance_radius = float(
            self.get_parameter('start_clearance_radius').value
        )
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        if (
            self.replan_rate <= 0.0
            or configured_clearance < 0.0
            or self.start_clearance_radius < 0.0
            or self.dynamic_replan_stop < 0.0
        ):
            raise ValueError('replan_rate는 양수, radius는 0 이상이어야 합니다.')

        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.tf = TfHelper(self)
        self.static_grid: Optional[MapGrid] = None
        self.dynamic_grid: Optional[MapGrid] = None
        self.combined_grid: Optional[MapGrid] = None
        self.goal: Optional[PoseStamped] = None
        self.current_path_cells: List[Cell] = []
        self.victim_positions: List[Tuple[float, float]] = []
        self._dirty = False
        self._last_plan = 0.0
        self._waiting_dynamic_replan = False
        self._replan_not_before = 0.0
        self.create_subscription(
            OccupancyGrid, '/planning_grid_static', self._static_callback, qos
        )
        self.create_subscription(
            OccupancyGrid, '/dynamic_obstacle_grid', self._dynamic_callback, qos
        )
        self.create_subscription(
            MarkerArray, '/victim_markers', self._victim_callback, qos
        )
        self.create_subscription(PoseStamped, '/goal_pose', self._goal_callback, 10)
        self.grid_publisher = self.create_publisher(
            OccupancyGrid, '/planning_grid', qos
        )
        self.path_publisher = self.create_publisher(Path, '/planned_path', qos)
        self.state_publisher = self.create_publisher(String, '/planner_state', 10)
        self.create_timer(1.0 / self.replan_rate, self._timer_callback)
        self._state('WAITING_FOR_GRID')
        self.get_logger().info(
            f'skid footprint clearance={self.clearance_radius:.3f}m '
            f'(L={self.robot_length:.3f}, W={self.robot_width:.3f}, '
            f'margin={self.safety_margin:.3f})'
        )

    @staticmethod
    def _same_geometry(first: MapGrid, second: MapGrid) -> bool:
        return (
            first.width == second.width
            and first.height == second.height
            and abs(first.resolution - second.resolution) < 1e-9
            and abs(first.origin_x - second.origin_x) < 1e-6
            and abs(first.origin_y - second.origin_y) < 1e-6
        )

    def _static_callback(self, message: OccupancyGrid) -> None:
        try:
            grid = message_to_grid(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        changed = self.static_grid is None or not np.array_equal(
            self.static_grid.data, grid.data
        )
        self.static_grid = grid
        if changed:
            self._dirty = True
            self._combine_and_publish()

    def _dynamic_callback(self, message: OccupancyGrid) -> None:
        try:
            grid = message_to_grid(message)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return
        if self.static_grid is not None and not self._same_geometry(
            self.static_grid, grid
        ):
            self.get_logger().error(
                'dynamic grid geometry가 static grid와 다릅니다.'
            )
            return
        changed = self.dynamic_grid is None or not np.array_equal(
            self.dynamic_grid.data, grid.data
        )
        self.dynamic_grid = grid
        if changed:
            self._dirty = True
            self._combine_and_publish()
            self._stop_if_current_path_blocked()

    def _victim_callback(self, message: MarkerArray) -> None:
        positions = []
        for marker in message.markers:
            if marker.action != Marker.ADD or marker.type != Marker.SPHERE_LIST:
                continue
            if marker.header.frame_id and marker.header.frame_id != self.map_frame:
                continue
            positions.extend((float(point.x), float(point.y)) for point in marker.points)
        if positions == self.victim_positions:
            return
        self.victim_positions = positions
        self._dirty = True
        self._combine_and_publish()
        self._stop_if_current_path_blocked()

    def _stop_if_current_path_blocked(self) -> None:
        if self._waiting_dynamic_replan or self.goal is None:
            return
        if self.combined_grid is None or not self.current_path_cells:
            return
        path_to_check = self.current_path_cells
        tf_helper = getattr(self, 'tf', None)
        pose = (
            tf_helper.lookup_pose_2d(self.map_frame, self.base_frame)
            if tf_helper is not None else None
        )
        if pose is not None:
            current = world_to_grid(pose[0], pose[1], self.combined_grid)
            path_to_check = remaining_path_from_nearest(path_to_check, current)
        if not path_cells_collision(
            path_to_check,
            self.combined_grid.data,
            self.unknown_is_occupied,
        ):
            # Off-path scan jitter must not replace an otherwise valid route.
            self._dirty = False
            return
        if self.dynamic_replan_stop <= 0.0:
            self._state('REPLANNING')
            self._plan('DYNAMIC_BLOCKED')
            return
        self._waiting_dynamic_replan = True
        self._replan_not_before = time.monotonic() + self.dynamic_replan_stop
        self._state('REPLANNING')
        self._publish_empty_path()

    def _combine_and_publish(self) -> None:
        if self.static_grid is None:
            return
        combined = self.static_grid.data.copy()
        if self.dynamic_grid is not None:
            combined[self.dynamic_grid.data >= 100] = 100
        for x, y in self.victim_positions:
            grid_x, grid_y = world_to_grid(x, y, self.static_grid)
            if is_inside_grid(grid_x, grid_y, self.static_grid):
                combined[grid_y, grid_x] = 100
        planning_source = combined.copy()
        if self.unknown_is_occupied:
            planning_source[planning_source < 0] = 100
        radius_cells = int(math.ceil(
            self.clearance_radius / self.static_grid.resolution
        ))
        planning_source = inflate_occupied_cells(planning_source, radius_cells)
        self.combined_grid = MapGrid(
            width=self.static_grid.width,
            height=self.static_grid.height,
            resolution=self.static_grid.resolution,
            origin_x=self.static_grid.origin_x,
            origin_y=self.static_grid.origin_y,
            origin_yaw=self.static_grid.origin_yaw,
            frame_id=self.map_frame,
            data=planning_source,
        )
        self._publish_grid()

    def _publish_grid(self) -> None:
        if self.combined_grid is None:
            return
        stamp = self.get_clock().now().to_msg()
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame
        message.info.map_load_time = stamp
        message.info.resolution = self.combined_grid.resolution
        message.info.width = self.combined_grid.width
        message.info.height = self.combined_grid.height
        message.info.origin.position.x = self.combined_grid.origin_x
        message.info.origin.position.y = self.combined_grid.origin_y
        qx, qy, qz, qw = quaternion_from_yaw(self.combined_grid.origin_yaw)
        message.info.origin.orientation.x = qx
        message.info.origin.orientation.y = qy
        message.info.origin.orientation.z = qz
        message.info.origin.orientation.w = qw
        message.data = self.combined_grid.data.reshape(-1).astype(int).tolist()
        self.grid_publisher.publish(message)

    def _goal_callback(self, message: PoseStamped) -> None:
        if message.header.frame_id and message.header.frame_id != self.map_frame:
            self._state('INVALID_GOAL_FRAME')
            self.get_logger().error(
                f'goal frame={message.header.frame_id!r}; {self.map_frame!r}만 지원합니다.'
            )
            return
        self.goal = message
        self._waiting_dynamic_replan = False
        self._dirty = True
        self._plan('NEW_GOAL')

    def _timer_callback(self) -> None:
        if self.goal is None or not self._dirty:
            return
        if (
            self._waiting_dynamic_replan
            and time.monotonic() < self._replan_not_before
        ):
            return
        self._waiting_dynamic_replan = False
        # Periodic replanning also corrects for robot motion.
        reason = 'GRID_UPDATE' if self._dirty else 'PERIODIC'
        self._plan(reason)

    def _plan(self, reason: str) -> None:
        if self.goal is None or self.combined_grid is None:
            self._state('WAITING_FOR_GRID')
            return
        pose = self.tf.lookup_pose_2d(self.map_frame, self.base_frame)
        if pose is None:
            self._state('WAITING_FOR_TF')
            self._publish_empty_path()
            return
        start = world_to_grid(pose[0], pose[1], self.combined_grid)
        goal = world_to_grid(
            self.goal.pose.position.x,
            self.goal.pose.position.y,
            self.combined_grid,
        )
        if not is_inside_grid(*start, self.combined_grid) or not is_inside_grid(
            *goal, self.combined_grid
        ):
            self._state('NO_PATH')
            self.get_logger().error(f'start={start} 또는 goal={goal}이 지도 밖입니다.')
            self._publish_empty_path()
            return
        if reason == 'GRID_UPDATE' and self.current_path_cells:
            if path_cells_collision(
                self.current_path_cells,
                self.combined_grid.data,
                self.unknown_is_occupied,
            ):
                self._state('REPLANNING')
        else:
            self._state('PLANNING')
        # Inflation can mark the cell occupied by the physical robot itself.
        # Clear only its already-occupied local footprint so A* can safely leave
        # the start, while all cells beyond this small disk remain protected.
        planning_data = self.combined_grid.data.copy()
        start_radius_cells = int(math.ceil(
            self.start_clearance_radius / self.combined_grid.resolution
        ))
        for dy in range(-start_radius_cells, start_radius_cells + 1):
            for dx in range(-start_radius_cells, start_radius_cells + 1):
                if math.hypot(dx, dy) > start_radius_cells:
                    continue
                sx, sy = start[0] + dx, start[1] + dy
                if 0 <= sy < planning_data.shape[0] and 0 <= sx < planning_data.shape[1]:
                    planning_data[sy, sx] = 0
        path = astar_search(
            planning_data,
            start,
            goal,
            self.unknown_is_occupied,
            self.allow_diagonal,
        )
        if not path:
            self.current_path_cells = []
            # Retry only after the grid actually changes, rather than issuing
            # NO_PATH and stop commands at the periodic timer rate.
            self._dirty = False
            self._state('NO_PATH')
            self._publish_empty_path()
            self.get_logger().error(f'A* 경로 없음: start={start}, goal={goal}')
            return
        simplified = simplify_path(
            path, planning_data, self.unknown_is_occupied
        )
        self.current_path_cells = path
        self._dirty = False
        self._last_plan = time.monotonic()
        self._publish_path(simplified)
        self._state('PATH_READY')
        self.get_logger().debug(
            f'A* {reason}: raw={len(path)} cells, simplified={len(simplified)} poses'
        )

    def _publish_path(self, cells: List[Cell]) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        for index, cell in enumerate(cells):
            x, y = grid_to_world(cell[0], cell[1], self.combined_grid)
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            if index + 1 < len(cells):
                next_x, next_y = grid_to_world(
                    cells[index + 1][0], cells[index + 1][1], self.combined_grid
                )
                yaw = math.atan2(next_y - y, next_x - x)
            else:
                yaw = yaw_from_quaternion(self.goal.pose.orientation)
                pose.pose.position.x = self.goal.pose.position.x
                pose.pose.position.y = self.goal.pose.position.y
            qx, qy, qz, qw = quaternion_from_yaw(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.path_publisher.publish(message)

    def _publish_empty_path(self) -> None:
        message = Path()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        self.path_publisher.publish(message)

    def _state(self, text: str) -> None:
        self.state_publisher.publish(String(data=text))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = AstarReplanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ValueError as exc:
        print(f'astar_replanner 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
