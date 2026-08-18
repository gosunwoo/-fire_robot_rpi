import pytest

from inno_autonav.mission_commander import (
    ThermalRerouteTrigger, normalize_label, parse_mission,
)
from geometry_msgs.msg import PoseStamped
import yaml

from inno_autonav.waypoint_queue import (
    document_from_poses,
    poses_from_document,
    replacement_indices_from_text,
    save_pose_document,
)


@pytest.mark.parametrize(
    'text, expected',
    [
        ('go exit2', (None, 'exit2')),
        ('go exit1 exit2', ('exit1', 'exit2')),
        ('exit1에서 exit2로가', ('exit1', 'exit2')),
        ('EXIT1 to E2', ('EXIT1', 'E2')),
    ],
)
def test_parse_mission(text, expected):
    assert parse_mission(text) == expected


def test_thermal_reroute_requires_three_hot_frames_and_latches():
    trigger = ThermalRerouteTrigger(threshold_c=40.0, required_hits=3)
    assert not trigger.update(40.0, enabled=True)
    assert not trigger.update(39.9, enabled=True)
    assert not trigger.update(44.0, enabled=True)
    assert not trigger.update(44.0, enabled=True)
    assert trigger.update(44.0, enabled=True)
    assert not trigger.update(50.0, enabled=True)
    trigger.reset()
    assert not trigger.update(50.0, enabled=False)


def test_alias_normalization():
    assert normalize_label('E1') == 'exit1'
    assert normalize_label('EXIT2') == 'exit2'
    assert normalize_label('door', {'door': 'exit3'}) == 'exit3'


def test_bad_mission_rejected():
    with pytest.raises(ValueError):
        parse_mission('go exit1 exit2 exit3')


def test_load_named_semantic_waypoints():
    poses = poses_from_document(
        {
            'frame_id': 'map',
            'poses': {
                'INIT': {'x': 1.0, 'y': 2.0, 'yaw': 0.0},
                'EXIT1': {'x': 3.0, 'y': 4.0, 'yaw': 1.57},
            },
        },
        'map',
    )
    assert len(poses) == 2
    assert poses[0].pose.position.x == 1.0
    assert poses[1].pose.position.y == 4.0
    assert poses[1].pose.orientation.w != 0.0


def test_load_pose_array_snapshot():
    poses = poses_from_document(
        {
            'header': {'frame_id': 'map'},
            'poses': [
                {
                    'position': {'x': 5.0, 'y': -2.0, 'z': 0.0},
                    'orientation': {'z': 0.5, 'w': 0.866},
                }
            ],
        },
        'map',
    )
    assert len(poses) == 1
    assert poses[0].pose.position.x == 5.0
    assert poses[0].pose.orientation.z == 0.5


def test_saved_queue_round_trip(tmp_path):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = 1.25
    pose.pose.position.y = -3.5
    pose.pose.orientation.w = 1.0
    output = tmp_path / 'waypoint_queue_latest.yaml'

    save_pose_document(output, [pose], 'map')

    document = yaml.safe_load(output.read_text(encoding='utf-8'))
    restored = poses_from_document(document, 'map')
    assert document_from_poses(restored, 'map') == document
    assert restored[0].pose.position.y == -3.5


def test_replacement_indices_preserve_requested_order():
    assert replacement_indices_from_text("2,3,9", 9) == [1, 2, 8]
    assert replacement_indices_from_text("", 9) == []


@pytest.mark.parametrize("value", ["0", "10", "2,2", "two"])
def test_replacement_indices_reject_invalid_input(value):
    with pytest.raises(ValueError):
        replacement_indices_from_text(value, 9)
