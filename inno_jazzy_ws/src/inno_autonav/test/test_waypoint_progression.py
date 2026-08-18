from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from inno_autonav.skid_path_follower import (
    RotationProgressMonitor, SkidPathFollower,
)
from inno_autonav.waypoint_queue import WaypointQueue


def test_new_path_is_acknowledged_before_control_cycle():
    follower = object.__new__(SkidPathFollower)
    states = []
    follower._state = states.append
    follower.rotation_progress = RotationProgressMonitor(0.04, 3.0)

    path = Path()
    path.poses.append(PoseStamped())
    follower._path_callback(path)

    assert follower.path is path
    assert states == ['PATH_ACCEPTED']


def test_121_degree_corner_commands_forward_skid_arc_not_pure_rotation():
    follower = object.__new__(SkidPathFollower)
    follower.emergency_stop = False
    follower.planner_state = "PATH_READY"
    follower.rotation_stalled = False
    follower.rotation_clearance_blocked = False
    follower.rotating_in_place = False
    follower.rotation_direction = 0.0
    follower.rotation_progress = RotationProgressMonitor(0.04, 3.0)
    follower.rotate_threshold = 2.70
    follower.rotate_exit_threshold = 0.35
    follower.lookahead = 0.55
    follower.goal_tolerance = 0.12
    follower.align_goal_yaw = False
    follower.max_linear = 0.20
    follower.max_angular = 0.35
    follower.k_linear = 0.5
    follower.k_angular = 0.9
    follower.minimum_turn_speed_ratio = 0.40
    follower.map_frame = "map"
    follower.base_frame = "base_link"
    follower.tf = type(
        "Tf", (), {"lookup_pose_2d": lambda _self, _map, _base: (0.0, 0.0, 0.0)}
    )()
    target = PoseStamped()
    target.pose.position.x = -1.03
    target.pose.position.y = -1.72
    path = Path()
    path.poses = [target]
    follower.path = path
    commands = []
    states = []
    follower.publisher = type(
        "Publisher", (), {"publish": lambda _self, message: commands.append(message)}
    )()
    follower._state = states.append

    follower._control()

    assert states == ["FOLLOWING_PATH"]
    assert commands[-1].linear.x > 0.0
    assert commands[-1].angular.z < 0.0


def test_immediately_reached_waypoint_advances_after_path_acceptance():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped(), PoseStamped()]
    queue.current_index = 0
    queue.waiting_for_departure = True
    queue.execution_mode = 'continuous'
    queue.step_index = 0
    sent_indices = []
    queue._send_current_goal = lambda: sent_indices.append(queue.current_index)
    queue._state = lambda _state: None

    queue._follower(String(data='GOAL_REACHED'))
    assert queue.current_index == 0
    assert sent_indices == []

    queue._follower(String(data='PATH_ACCEPTED'))
    queue._follower(String(data='GOAL_REACHED'))

    assert queue.current_index == 1
    assert sent_indices == [1]


def test_unrelated_stop_state_does_not_unlock_stale_goal_reached():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped(), PoseStamped()]
    queue.current_index = 0
    queue.waiting_for_departure = True
    queue.execution_mode = 'continuous'
    queue.step_index = 0
    queue._send_current_goal = lambda: None
    queue._state = lambda _state: None



def test_step_mode_stops_after_one_waypoint():
    queue = object.__new__(WaypointQueue)
    queue.queue = [PoseStamped(), PoseStamped()]
    queue.current_index = 0
    queue.step_index = 0
    queue.execution_mode = 'step'
    queue.waiting_for_departure = False
    states = []
    queue._state = states.append
    queue._publish_queue = lambda: None

    queue._follower(String(data='GOAL_REACHED'))

    assert queue.current_index is None
    assert queue.step_index == 1
    assert states == [
        'REACHED:1/2',
        'STEP_COMPLETE:1/2:SPACE_FOR:2',
    ]
