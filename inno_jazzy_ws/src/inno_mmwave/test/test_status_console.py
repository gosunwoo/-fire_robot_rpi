"""Focused tests for stable mmWave status-console output."""

from pathlib import Path
import sys

from std_msgs.msg import Bool, String


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from inno_mmwave.status_console import (  # noqa: E402
    DYNAMIC_OBSTACLE_TOPIC,
    FILTERED_DISTANCE_TOPIC,
    FILTERED_PRESENCE_TOPIC,
    MODE_TITLES,
    StatusConsole,
    VICTIM_FUSION_TOPIC,
    THERMAL_ROUTE_TOPIC,
    waypoint_log_text,
)


def test_console_has_three_explicit_operator_modes() -> None:
    assert MODE_TITLES == {
        1: 'KEYBOARD',
        2: 'WAYPOINT ONLY',
        3: 'RESCUE + DYNAMIC AVOIDANCE',
    }


def test_console_uses_filtered_target_topics() -> None:
    assert FILTERED_PRESENCE_TOPIC == '/mmwave/filtered_presence'
    assert FILTERED_DISTANCE_TOPIC == '/mmwave/filtered_distance_m'
    assert DYNAMIC_OBSTACLE_TOPIC == '/dynamic_obstacle_detected'
    assert VICTIM_FUSION_TOPIC == '/victim_fusion_status'
    assert THERMAL_ROUTE_TOPIC == "/thermal_route_status"


def test_waypoint_log_formats_queue_progress_and_completion_events() -> None:
    assert waypoint_log_text('RESTORED:15') == '[웨이포인트] 15개 준비됨'
    assert waypoint_log_text('RUNNING:3/15') == '[웨이포인트] 3/15 주행 중'
    assert waypoint_log_text('REACHED:2/5') == '[웨이포인트] 2/5 도착'
    assert waypoint_log_text('MISSION_COMPLETE') == '[웨이포인트] 주행 완료'


def test_mode_two_and_three_report_waypoint_progress() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 2
    console._waypoint_state = None
    lines = []
    console._write = lines.append

    console._on_waypoint_state(String(data='RUNNING:1/3'))
    console._on_waypoint_state(String(data='REACHED:1/3'))
    console._mode = 3
    console._on_waypoint_state(String(data='REACHED:2/3'))

    assert lines == [
        '[웨이포인트] 1/3 주행 중',
        '[웨이포인트] 1/3 도착',
        '[웨이포인트] 2/3 도착',
    ]


def test_mode_three_reports_newly_confirmed_rescuee_once() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 3
    console._victim_state = None
    lines = []
    console._write = lines.append

    message = String(data='DETECTED:1.24,-3.56')
    console._on_victim_fusion(message)
    console._on_victim_fusion(message)

    assert lines == ['[요구조자] 확정됨 (x=1.2, y=-3.6)']


def test_mode_three_reports_stopped_inspection_and_avoidance_resume() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 3
    console._victim_state = None
    lines = []
    console._write = lines.append

    console._on_victim_fusion(String(data='INSPECTING:2.15'))
    console._on_victim_fusion(String(data='OBSTACLE:2.15'))

    assert lines == [
        '[판별] 동적 후보 2.1m - 정지 후 MMWAVE 확인 중',
        '[판별] 사람 아님 - 동적장애물 회피 재개',
    ]


def test_rotation_odometry_stale_has_clear_safety_log() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 2
    console._follower_state = None
    lines = []
    console._write = lines.append

    message = String(data="ROTATION_ODOMETRY_STALE")
    console._on_follower_state(message)
    console._on_follower_state(message)

    assert lines == ["[주행 경고] 회전각 갱신 없음 - 안전 정지"]


def test_mode_three_dynamic_obstacle_log_is_transition_only() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 3
    console._dynamic_detected = False
    lines = []
    console._write = lines.append

    console._on_dynamic_obstacle(Bool(data=True))
    console._on_dynamic_obstacle(Bool(data=True))
    console._on_dynamic_obstacle(Bool(data=False))

    assert lines == [
        '[동적장애물] 감지됨',
        '[동적장애물] 감지 해제',
    ]


def test_mode_three_prints_requested_thermal_reroute_message_once() -> None:
    console = object.__new__(StatusConsole)
    console._mode = 3
    console._thermal_route_state = None
    lines = []
    console._write = lines.append
    message = String(data="THERMAL_DANGER:EXIT3")

    console._on_thermal_route(message)
    console._on_thermal_route(message)

    assert lines == [
        "이동경로 중 온도 증가 감지! ->exit2 danger expected",
        "exit3으로 경로를 변경합니다.",
    ]


def test_detection_distance_is_reported_to_one_decimal_metre() -> None:
    console = object.__new__(StatusConsole)
    console._distance_m = 2.04
    assert console._detection_text() == '[사람] 감지됨, 거리 약 2.0m'

    console._distance_m = 2.06
    assert console._detection_text() == '[사람] 감지됨, 거리 약 2.1m'

    console._distance_m = None
    assert console._detection_text() == '[사람] 감지됨'
