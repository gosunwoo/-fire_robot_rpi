#!/usr/bin/env bash
set -euo pipefail

project_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
timestamp="$(date '+%Y%m%d_%H%M%S')"
output="${project_root}/bags/modes234_${timestamp}"
check_seconds='2.0'
check_only='false'
declare -a extra_topics=()

declare -a topics=(
  # ROS graph/logs: correlate node failures and parameter changes with motion.
  /rosout
  /parameter_events
  /diagnostics

  # LiDAR localization and the exact TF chain used by every mode.
  /tf
  /tf_static
  /scan
  /map
  /odom_rf2o
  /amcl_pose
  /localization_ready
  /localization_status
  /lidar_path

  # ESP32 feedback and commands actually selected by the mode mux.
  /wheel_odom
  /wheel_path
  /cmd_vel
  /cmd_vel_keyboard
  /cmd_vel_auto
  /motor/left_steps_per_sec
  /motor/right_steps_per_sec
  /wheel_ticks
  /esp32/status
  /drive_mode
  /drive_mode_status

  # Mode 2 waypoint input, planner output, and follower decisions.
  /initialpose
  /goal_pose
  /waypoint_click
  /waypoint_queue
  /waypoint_poses
  /waypoint_markers
  /waypoint_route_markers
  /waypoint_queue_status
  /waypoint_queue_command
  /obstacle_inspection_command
  /waypoint_path
  /astar_path
  /planned_path
  /planner_state
  /follower_state
  /path_selector/mode
  /waypoint_planner/route_status
  /replanning/hold
  /replanning/status
  /survivor_follow_hold
  /planning_grid_static
  /planning_grid

  # Mode 5 operator decisions and exit-selection history.
  /evacuation_demo/status
  /evacuation_demo/log
  /evacuation/selected_exit
  /evacuation/blocked_exits
  /evacuation/plan
  /exit_switching/status

  # Shared Mode 3/4 LiDAR obstacle classification and moving-person track.
  /dynamic_obstacle_grid
  /dynamic_obstacle_markers
  /dynamic_obstacle_candidates
  /dynamic_obstacle_all_candidates
  /dynamic_obstacle_person
  /dynamic_obstacle_person_track
  /dynamic_obstacle_observations
  /dynamic_obstacle_markers_display
  /dynamic_obstacle_detected

  # Mode 3/4 state machines and their inspection results.
  /mode3_status
  /mode3_classification
  /mode3_audio_status
  /mode4_status
  /mode4_classification
  /autonomy_cancel

  # Mode 4 camera input, YOLO evidence, and annotated debug frames.
  /camera/image_raw
  /camera/camera_info
  /camera/person_detections
  /camera/person_detector_status
  /camera/person_detection_image
  /victim_markers
  /victim_detected
  /victim_fusion_status

  # Mode 3 raw/filtered mmWave evidence and mobility decision.
  /mmwave/raw/presence
  /mmwave/raw/distance_m
  /mmwave/raw/speed_mps
  /mmwave/raw/energy_raw
  /mmwave/filtered_presence
  /mmwave/filtered_distance_m
  /mmwave/filtered_speed_mps
  /mmwave/calibrated_distance_m
  /mmwave/human_presence
  /mmwave/motion_activity
  /mmwave/filter_state
  /mmwave/presence
  /mmwave/distance_m
  /mmwave/speed_mps
  /mmwave/energy_raw
  /mmwave/sensor_state
  /mmwave/mobility_state
  /mmwave/human_state
  /mmwave/still_duration_sec

  # Classification messages include this revision in Mode 3/4.
  /hazard/revision

  # Mode 6 smoke-assist sensor evidence, trigger state, and RViz color feeds.
  /imu/data
  /imu/magnetic_field
  /imu/status
  /ultrasonic/front/range
  /mode6/smoke_assist_active
  /mode6/ultrasonic_obstacle
  /mode6/safety_hold
  /mode6/status
  /mode6/log
  /scan_rviz_red
  /scan_rviz_blue
)

die() {
  printf 'ERROR: %s\n' "$*" >&2
  exit 1
}

usage() {
  printf '%s\n' \
    'Record a Mode 2/3/4 fire-robot debugging rosbag.' \
    '' \
    'Usage: ./record_robot_bag.sh [options]' \
    '' \
    '  --output PATH         Bag directory (default: bags/modes234_TIMESTAMP)' \
    '  --check-seconds SEC   Preflight sample window (default: 2.0)' \
    '  --extra-topic TOPIC   Add another topic; may be repeated' \
    '  --check-only          Print topic status without starting rosbag' \
    '  -h, --help            Show this help' \
    '' \
    'Missing publishers or samples are warnings only. Every listed topic remains' \
    'in the rosbag request so it can be recorded if it appears later.'
}

positive_number() {
  [[ "$1" =~ ^([0-9]+([.][0-9]*)?|[.][0-9]+)$ ]] && \
    awk -v value="$1" 'BEGIN { exit !(value > 0) }'
}

source_ros() {
  local ros_setup="${ROS_SETUP_FILE:-/opt/ros/jazzy/setup.bash}"
  local robot_setup="${ROBOT_WS_SETUP_FILE:-${project_root}/inno_jazzy_ws/install/local_setup.bash}"
  local discovery_range="${ROBOT_DISCOVERY_RANGE:-LOCALHOST}"
  [[ -r "${ros_setup}" ]] || die "ROS 2 Jazzy is missing: ${ros_setup}"
  [[ -r "${robot_setup}" ]] || die \
    "inno_jazzy_ws is not built: ${robot_setup}"
  set +u
  # shellcheck disable=SC1090
  source "${ros_setup}"
  # shellcheck disable=SC1090
  source "${robot_setup}"
  set -u
  # The integrated launch intentionally uses LOCALHOST discovery.  Recording
  # with the shell's inherited SUBNET setting creates an apparently valid but
  # empty bag containing only the recorder's own /rosout messages.
  export ROS_AUTOMATIC_DISCOVERY_RANGE="${discovery_range}"
  unset ROS_LOCALHOST_ONLY || true
  command -v ros2 >/dev/null 2>&1 || die 'ros2 is unavailable.'
  ros2 daemon stop >/dev/null 2>&1 || true
  ros2 pkg prefix inno_robot_bringup >/dev/null 2>&1 || \
    die 'inno_robot_bringup is unavailable in the sourced workspace.'
}

while (( $# > 0 )); do
  case "$1" in
    --output|--check-seconds|--extra-topic)
      (( $# >= 2 )) || die "$1 requires a value."
      case "$1" in
        --output) output="$2" ;;
        --check-seconds) check_seconds="$2" ;;
        --extra-topic) extra_topics+=("$2") ;;
      esac
      shift 2
      ;;
    --check-only)
      check_only='true'
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "Unknown option: $1 (use --help)"
      ;;
  esac
done

positive_number "${check_seconds}" || die '--check-seconds must be positive.'
if [[ "${output}" != /* ]]; then
  output="${project_root}/${output}"
fi
for topic in "${extra_topics[@]}"; do
  [[ "${topic}" == /* ]] || die "Topic must start with '/': ${topic}"
  topics+=("${topic}")
done

source_ros
status_file="${output}.topics.txt"
mkdir -p -- "$(dirname -- "${output}")"
if [[ "${check_only}" != 'true' ]]; then
  [[ ! -e "${output}" ]] || die "Bag output already exists: ${output}"
  [[ ! -e "${status_file}" ]] || die "Status file already exists: ${status_file}"
fi

printf 'Rosbag output: %s\n' "${output}"
printf 'ROS discovery range: %s\n' "${ROS_AUTOMATIC_DISCOVERY_RANGE}"
printf 'Checking %d topics; missing data will not stop recording.\n' "${#topics[@]}"
if [[ "${check_only}" == 'true' ]]; then
  ros2 run inno_robot_bringup bag_topic_preflight \
    --wait "${check_seconds}" "${topics[@]}"
  exit 0
fi

ros2 run inno_robot_bringup bag_topic_preflight \
  --wait "${check_seconds}" "${topics[@]}" | tee "${status_file}"

printf '\nStarting rosbag. Stop safely with Ctrl-C.\n'
printf 'Topic status sidecar: %s\n' "${status_file}"
exec ros2 bag record \
  --storage mcap \
  --storage-preset-profile fastwrite \
  --output "${output}" \
  --topics "${topics[@]}"
