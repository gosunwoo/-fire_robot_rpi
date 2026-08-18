from pathlib import Path
import math

import numpy as np
from PIL import Image
import pytest
from sensor_msgs.msg import LaserScan
import yaml

from inno_autonav.astar_replanner import (
    AstarReplanner,
    astar_search,
    footprint_clearance_radius,
    remaining_path_from_nearest,
    simplify_path,
)
from inno_autonav.skid_path_follower import (
    RotationProgressMonitor, nearest_scan_clearances,
    requires_in_place_rotation,
)
from inno_autonav.grid_utils import (
    MapGrid,
    grid_to_world,
    inflate_occupied_cells,
    load_pgm_as_occupancy,
    world_to_grid,
)


def test_pgm_y_axis_is_flipped(tmp_path: Path):
    pixels = np.full((3, 4), 254, dtype=np.uint8)
    pixels[0, 1] = 0  # top PGM row must become highest OccupancyGrid y row.
    Image.fromarray(pixels, mode='L').save(tmp_path / 'map.pgm', format='PPM')
    metadata = {
        'image': 'map.pgm', 'resolution': 0.5, 'origin': [-1.0, -2.0, 0.0],
        'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196,
    }
    (tmp_path / 'map.yaml').write_text(
        yaml.safe_dump(metadata), encoding='utf-8'
    )
    grid = load_pgm_as_occupancy(tmp_path / 'map.yaml')
    assert grid.data[2, 1] == 100
    assert grid.data[0, 1] == 0


def test_world_grid_round_trip_uses_cell_center():
    grid = MapGrid(10, 10, 0.1, -1.0, -2.0, 0.0, 'map', np.zeros((10, 10)))
    assert world_to_grid(-0.75, -1.65, grid) == (2, 3)
    assert grid_to_world(2, 3, grid) == pytest.approx((-0.75, -1.65))


def test_inflation_and_astar_avoid_wall_gap():
    data = np.zeros((12, 12), dtype=np.int8)
    data[:, 6] = 100
    data[7, 6] = 0
    path = astar_search(data, (1, 1), (10, 10), True, True)
    assert path
    assert (6, 7) in path
    simplified = simplify_path(path, data, True)
    assert simplified[0] == (1, 1)
    assert simplified[-1] == (10, 10)
    inflated = inflate_occupied_cells(data, 1)
    assert inflated[7, 6] == 100
    assert not astar_search(inflated, (1, 1), (10, 10), True, True)


def test_unknown_is_occupied_switch():
    data = np.zeros((3, 5), dtype=np.int8)
    data[:, 2] = -1
    assert not astar_search(data, (0, 1), (4, 1), True, False)
    assert astar_search(data, (0, 1), (4, 1), False, False)


def test_skid_footprint_clearance_uses_half_diagonal_plus_margin():
    radius = footprint_clearance_radius(0.39, 0.20, 0.10)
    assert radius == pytest.approx(0.3192, abs=0.0002)


def test_sharp_waypoint_corner_uses_skid_arc_not_in_place_rotation():
    assert not requires_in_place_rotation(math.radians(121.0), 2.70)
    assert requires_in_place_rotation(math.radians(170.0), 2.70)


def test_rotation_monitor_stops_frozen_yaw_but_accepts_progress():
    monitor = RotationProgressMonitor(0.04, 3.0)
    assert not monitor.stalled(0.0, 0.0)
    assert not monitor.stalled(0.01, 2.9)
    assert monitor.stalled(0.01, 3.0)

    monitor.reset()
    assert not monitor.stalled(0.0, 0.0)
    assert not monitor.stalled(0.05, 2.0)
    assert not monitor.stalled(0.05, 4.9)
    assert monitor.stalled(0.05, 5.0)


def test_single_scan_point_still_triggers_immediate_safety_ranges():
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_increment = 0.1
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [0.25]

    front, all_around = nearest_scan_clearances(scan, 0.61)
    assert front == pytest.approx(0.25)
    assert all_around == pytest.approx(0.25)


def test_side_scan_point_blocks_rotation_but_not_front_emergency():
    scan = LaserScan()
    scan.angle_min = 1.0
    scan.angle_increment = 0.1
    scan.range_min = 0.05
    scan.range_max = 12.0
    scan.ranges = [0.25]

    front, all_around = nearest_scan_clearances(scan, 0.61)
    assert math.isinf(front)
    assert all_around == pytest.approx(0.25)


def test_blocked_dynamic_path_replans_without_publishing_empty_path():
    planner = object.__new__(AstarReplanner)
    planner._waiting_dynamic_replan = False
    planner.goal = object()
    planner.combined_grid = MapGrid(
        1, 1, 0.05, 0.0, 0.0, 0.0, "map",
        np.asarray([[100]], dtype=np.int8),
    )
    planner.current_path_cells = [(0, 0)]
    planner.unknown_is_occupied = True
    planner.dynamic_replan_stop = 0.0
    planner.tf = None
    events = []
    planner._state = lambda state: events.append(("state", state))
    planner._plan = lambda reason: events.append(("plan", reason))
    planner._publish_empty_path = lambda: events.append(("empty", None))

    planner._stop_if_current_path_blocked()

    assert events == [
        ("state", "REPLANNING"), ("plan", "DYNAMIC_BLOCKED")
    ]


def test_remaining_path_ignores_cells_already_passed():
    path = [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)]
    assert remaining_path_from_nearest(path, (3, 1)) == [(3, 0), (4, 0)]


def test_periodic_timer_does_not_replace_clean_path():
    planner = object.__new__(AstarReplanner)
    planner.goal = object()
    planner._dirty = False
    planner._waiting_dynamic_replan = False
    events = []
    planner._plan = events.append

    planner._timer_callback()

    assert events == []
