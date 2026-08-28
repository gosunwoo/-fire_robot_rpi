# Fire Robot RPi

Raspberry Pi 5와 ROS 2 Jazzy를 사용하는 산업현장 화재 대피·요구조자 탐색 로봇
소프트웨어다. RPLIDAR C1 지도 위치추정과 동적장애물 회피, ESP32 모터 제어,
C4001 mmWave 사람 판별, Camera Module 3 YOLO 요구조자 판별을 하나의 통합 launch로
실행한다.

## 현재 주행 모드

| 모드 | 시작 입력 | 필수 장치·상태 | 핵심 기능 |
|---|---|---|---|
| 1 | `1` | ESP32와 구동 모터 | `w/x/a/d/s` 키보드 수동주행 |
| 2 | `2` → waypoint 입력 | 모드 1 장치 + LiDAR, AMCL, 지도 | 지정한 `w1,w5,...`를 입력 순서대로 단계주행 |
| 3 | `3` → `Space` | 모드 2 장치 + C4001 mmWave | 가장 가까운 동적장애물 2.0m 앞에서 사람 여부 판별 |
| 4 | `4` → `Space` | 모드 2 장치 + Camera Module 3, YOLO | 카메라 바운딩박스와 LiDAR 점을 결합해 요구조자 판별 |
| 5 | 전용 launch 실행 즉시 | 모드 2·3 장치 + MLX90640 | 출구를 실제 순차 탐색하고 장애물을 2m에서 mmWave 검사한 뒤 안전 경로로 대피 |
| 6 | 모드 2 경로 실행 중 `6` → `Space` | BNO055 + HC-SR04 | 연기 상황을 모사하여 IMU·초음파 보조주행 및 0.8m 장애물 재계획 시험 |

모든 입력은 통합 launch를 실행한 터미널에서 받는다. 일반 모드 변경은 진행 중인
자율주행을 먼저 취소하고 속도를 0으로 만든다. 모드 6만 모드 2의 진행 경로를 유지하는
시험용 보조주행 오버레이다. 자율주행 중에는 다음 공통 키를 사용한다.

| 키 | 동작 |
|---|---|
| `1` | 즉시 자율주행 취소, 정지, 모드 1 복귀 |
| `s` | 자율주행 취소, 정지, 모드 1 복귀 |
| `c` | 현재 모드 2·3·4·5·6 mission 취소 |
| `Space` | 모드 2의 다음 waypoint, 모드 3·4 검사 시작 또는 모드 6 보조주행 전환 |

RViz의 **큰 빨간 점**은 아직 분류하지 않은 LiDAR 동적장애물이고, **큰 파란 점**은
모드 3 또는 4에서 사람·요구조자로 확인한 위치다. 파란 점은 실행 중 timeout 없이
유지되며 `/clear_dynamic_obstacles` 서비스 호출 또는 노드 재시작 시 초기화된다.

## 하드웨어와 기본 환경

- Raspberry Pi 5, Ubuntu 24.04, ROS 2 Jazzy
- Raspberry Pi Camera Module 3 Wide (IMX708)
- RPLIDAR C1
- C4001 mmWave presence sensor
- ESP32 + TB6600 4륜 skid-steer 구동부
- 선택 장치: MLX90640 열화상 카메라
- 모드 6: MCU-055(BNO055) 9축 IMU + HC-SR04 전면 초음파 센서

현재 개발 PC처럼 센서가 연결되지 않은 환경에서 카메라·LiDAR·mmWave 미검출이
나오는 것은 정상이다. 실제 포트와 TF, 모델 정확도, 제동거리는 Raspberry Pi 5에서
별도로 검증해야 한다.

## 저장소 구성

| 경로 | 내용 |
|---|---|
| `inno_jazzy_ws/src/inno_robot_bringup` | 통합 launch, AMCL, RViz 설정 |
| `inno_jazzy_ws/src/inno_autonav` | A*, 동적장애물, 모드 3·4 검사, path follower |
| `inno_jazzy_ws/src/inno_drive_bridge` | 모드 선택, 키보드, ESP32 직렬 모터 bridge |
| `inno_jazzy_ws/src/inno_mmwave` | C4001 수신·필터링·상태 출력 |
| `inno_jazzy_ws/src/inno_camera_tools` | Camera Module 3 ROS 실행·YOLO 추론 |
| `maps` | 주행 지도, 약 1m 간격 waypoint YAML, no-go 설정 |
| `models` | 모드 4 YOLO weight 배치 안내 |
| `data/model_eval` | 임시 ONNX 모델의 공개사진 검출 결과와 출처 |
| `firmware/esp32_tb6600_bridge` | ESP32 모터 bridge 펌웨어 |
| `record_robot_bag.sh` | 센서·주행·분류 토픽 통합 rosbag 기록 |

## 1. 최초 설치와 빌드

Ubuntu 24.04에서 ROS 2 Jazzy와 기본 의존성을 처음 설치할 때 사용한다.

```bash
cd ~/fire_robot_rpi
sudo bash ./install_ubuntu2404_dependencies.sh
```

Camera Module 3를 사용할 Raspberry Pi에서는 호환 카메라 런타임도 한 번 빌드한다.

```bash
cd ~/fire_robot_rpi
./build_rpi_camera_runtime.sh
```

ROS workspace를 빌드한다.

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to inno_robot_bringup
source install/setup.bash
```

새 터미널을 열 때마다 `/opt/ros/jazzy`와 workspace의 `install/setup.bash`를 다시
source해야 한다.

## 2. 통합 현장 실행

먼저 장치별 고정 포트를 확인한다.

```bash
ls -l /dev/serial/by-id/
```

처음에는 바퀴를 바닥에서 띄우고 저속으로 실행한다. MLX90640을 사용하지 않으면
`start_thermal_viewer:=false`를 유지한다.

```bash
export FIRE_ROBOT_RPI_ROOT="$HOME/fire_robot_rpi"

cd "$FIRE_ROBOT_RPI_ROOT/inno_jazzy_ws"
source /opt/ros/jazzy/setup.bash
source install/setup.bash

ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  esp32_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  mmwave_port:=/dev/ttyAMA0 \
  map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_raw.yaml" \
  planning_map_yaml:="$FIRE_ROBOT_RPI_ROOT/maps/inno_map_nav.yaml" \
  waypoint_file:="$FIRE_ROBOT_RPI_ROOT/maps/waypoint_queue_latest.yaml" \
  drive_speed:=0.06 \
  turn_speed:=0.35 \
  start_thermal_viewer:=false
```

RViz가 열리면 다음 순서로 준비한다.

1. Fixed Frame이 `map`인지 확인한다.
2. **2D Pose Estimate**로 지도상의 실제 로봇 위치와 방향을 지정한다.
3. `/scan`, `/lidar_path`, `/planned_path`와 빨간 동적장애물 marker를 확인한다.
4. 통합 launch를 실행한 터미널에 포커스를 두고 모드 키를 입력한다.

기본 상태 확인:

```bash
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /drive_mode_status
ros2 topic echo /follower_state
```

### 모드 5: EVACUATION_DEMO 한 번에 실행

모드 5는 모드 1·2·3·4 launch와 입력 동작을 변경하지 않는 별도 통합 프로필이다.
아래 명령을 실행하면 `inno_jazzy_ws/src/inno_autonav/config/semantic_points.yaml`의
`init`을 AMCL 초기 자세로 넣고
모드 5를 자동 선택한다. 센서, `map → base_link`, 화재 belief와 출구 평가기가 모두
준비되면 키 입력 없이 `SEARCH_EXITS`에서 아직 확인하지 않은 출구 중 현재
cost-aware A* 경로가 가장 짧은 출구를 선택해 실제로 이동한다. 출구 도착 후에는
현재 위치에서 남은 출구를 다시 평가하므로 저장된 탐색 순서를 재생하지 않는다.
`init`은 위치추정의 초기값일 뿐이다.

처음에는 바퀴를 띄우고 `use_serial:=false`로 확인한다.

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch inno_robot_bringup evacuation_demo.launch.py \
  esp32_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  mmwave_port:=/dev/ttyAMA0 \
  use_serial:=false \
  event_replanning_enabled:=true \
  exit_switching_enabled:=true \
  waypoint_planning_enabled:=true
```

모드 5 launch도 기본적으로 같은 Pi 내부 discovery를 사용한다. 상태 확인용 두 번째
터미널에도 위 `export`를 적용한다. 다른 PC의 RViz/ROS 노드와 직접 통신해야 할 때만
launch에 `discovery_range:=SUBNET`을 넘기고 두 PC의 ROS 환경도 SUBNET으로 맞춘다.

센서와 ESP32를 하나도 연결하지 않은 상태의 소프트웨어 안전 기동은 다음과 같다.
지도와 22개 핵심 노드가 뜨지만 모터·센서 드라이버·RViz는 실행하지 않고, 모드 5는
화재 센서를 기다리거나 수동 시작 대기 상태에 머문다.

```bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ros2 launch inno_robot_bringup evacuation_demo.launch.py \
  use_serial:=false use_lidar:=false use_mmwave:=false \
  use_thermal_sensor:=false use_camera_mode4:=false use_rviz:=false \
  evacuation_demo_auto_start:=false
```

RViz의 위치와 경로가 맞는 것을 확인한 뒤에만 `use_serial:=true`로 실행한다. 실제
시작 위치가 바뀌면 `initial_pose_x`, `initial_pose_y`, `initial_pose_yaw`를 함께
넘긴다. 출구 근처에서 빨간 LiDAR 동적장애물이 발견되면 진행 중인 출구 경로를
취소하고 모드 3을 자동 호출한다. 단독 모드 3과 모드 5 모두 장애물 2.0m 앞에
정지하므로 단독 시험의 센서 기하 조건이 통합 주행에도 그대로 적용된다. mmWave를
검사해 `DYNAMIC_OBSTACLE`이면 해당 출구를 막힘으로 기록하고 다음 미확인 출구를
재평가한다. `PERSON`이면 RViz 점을 파란색으로 바꾸고 즉시 동행 대피 상태로
전환한다.

출구로 이동하는 중에는 `/dynamic_obstacle_all_candidates`의 현재 LiDAR cluster를
시간축으로 연결한다. 기본값은 2초 창 안에서 3회 이상 관측되고 0.20m 이상 이동한
track만 움직이는 사람 후보로 만든다. 후보가 생기면 현재 경로를 취소하고 해당 좌표를
`MODE4_START_AT:x,y`로 모드 4에 전달한다. 모드 4는 모드 5에서만 2.0m standoff와
canonical waypoint plan을 사용하며, 단독 모드 4의 `MODE4_START`와 1.5m 기본값은
유지된다.

카메라·LiDAR로 요구조자가 확인되면 파란 marker 위치를 LiDAR로 계속 갱신하면서
안전 출구로 안내한다. 로봇-요구조자 거리가 2.5m보다 커지거나 LiDAR track이 2초
이상 끊기면 `/survivor_follow_hold`로 모터를 정지하고, 추적이 정상이며 2.0m 안으로
돌아오면 주행을 재개한다. 로봇과 요구조자가 모두 출구에 도착해야
`EVACUATION_COMPLETE:SURVIVOR:<exit>`가 된다. 이 기능은 사람의 자율적인 보행을
제어하는 기능이 아니라, LiDAR로 따라오는지를 확인하며 기다려 주는 선도 주행이다.

동적장애물 inflation은 `final_simulator/factory_v5`와 같은 0.50m다. 실제 로봇의
외형과 통로 폭이 이 여유거리에 맞는지 저속 시험으로 확인해야 한다.

모드 5 상태와 정지·재시작 인터페이스는 다음과 같다.

```bash
ros2 topic echo /evacuation_demo/status
ros2 topic echo /evacuation_demo/log
ros2 topic echo /waypoint_planner/route_status
ros2 service call /evacuation_demo/stop std_srvs/srv/Trigger '{}'
ros2 service call /evacuation_demo/start std_srvs/srv/Trigger '{}'
```

통합 launch의 화면 로그에는 방문 중인 출구, 장애물 접근, mmWave/카메라 판별,
출구 막힘, 동행 대피 정지·재개가 한국어로 순서대로 출력된다. waypoint planner는
최초 경로를 `경로 생성: w1 -> w5 -> ...`, costmap 변화 후 경로를
`상황 변화로 경로 재생성: ...` 형식으로 출력하고 같은 내용을 JSON topic으로도
발행한다. RViz에는 저장 SLAM 지도 위에 모든 저장 waypoint 번호, 현재 선택된
waypoint 경로와 번호(청록색), 동적장애물(빨강), 확인된 요구조자(파랑)가 함께
표시된다.

통합 터미널에서 `1`, `s`, `c` 중 하나를 눌러도 자율주행을 취소하고 모드 1로
복귀한다.

권장 현장 개발 순서는 모드 2 waypoint 주행, 모드 3 mmWave 빨강→파랑 분류,
모드 4 카메라·LiDAR 분류, 마지막으로 모드 5 통합주행 순서다. 각 단계가 실패하면
다음 단계로 넘어가지 않는다. 모드 5 연결 코드는 미리 들어가 있지만 실제 모터·센서
환경에서는 이 순서로 각 하위 기능을 통과한 결과를 기준으로 파라미터를 확정해야 한다.

## 3. 모드별 사용법

### 모드 1: 키보드 수동주행

목적은 지도 목적지 없이 운전자가 직접 로봇을 이동시키는 것이다. 통합 launch가
시작되면 기본값이 모드 1이며, 다른 모드에서 `1`을 누르면 자율주행 goal을 취소하고
0 속도를 보낸 뒤 수동 입력으로 전환한다. 실제 이동에는 ESP32 직렬 연결과 모터
전원이 필요하지만 LiDAR·mmWave·카메라 값이 없어도 키 입력 자체는 사용할 수 있다.

| 키 | 동작 |
|---|---|
| `w` | 전진 |
| `x` | 후진 |
| `a` | 제자리 좌회전 |
| `d` | 제자리 우회전 |
| `s` | 정지 |
| `q` | 정지 후 키보드 노드 종료 |

모드 1의 키는 한 번 누른 속도를 계속 발행하므로 방향 전환 후 반드시 `s`로 정지한다.
출력은 `/cmd_vel_keyboard` → mode mux → `/cmd_vel` → ESP32 좌·우 STEP/s 순서로
전달된다.

```bash
ros2 topic echo /drive_mode_status
ros2 topic echo /cmd_vel
ros2 topic echo /motor/left_steps_per_sec
ros2 topic echo /motor/right_steps_per_sec
```

### 모드 2: 선택 waypoint 단계주행

목적은 지도 전체에 약 1m 간격으로 저장한 waypoint 중 필요한 지점만 골라 순서대로
이동하는 것이다. LiDAR `/scan`, `map → odom → base_link` TF와 AMCL 위치가 정상이어야
한다. waypoint는 [maps/waypoint_queue_latest.yaml](maps/waypoint_queue_latest.yaml)의
`w1`, `w2`, … 이름을 사용한다.

1. RViz에서 **2D Pose Estimate**로 로봇의 실제 위치와 방향을 맞춘다.
2. 통합 launch 터미널에서 `2`를 누른다.
3. waypoint를 쉼표로 두 개 이상 입력하고 Enter를 누른다.
4. 첫 waypoint로 즉시 출발하며, 각 지점 도착 후 Space를 눌러 다음 지점으로 간다.

```text
MODE 2 waypoints (example w1,w5,w6) > w1,w5,w6
```

`w1`에 도착해 `MODE2_REACHED:w1:SPACE_FOR:w5`가 표시되면 Space를 눌러 `w5`로
출발한다. 입력한 순서만 사용하며 주행 중 Space를 눌러도 다음 목적지를 건너뛰지
않는다. 마지막 지점에서는 `MODE2_MISSION_COMPLETE`가 표시된다.

A*는 정적 지도와 LiDAR 동적장애물 layer를 함께 사용하고, 주행 중 경로가 막히면
재계획한다. 정면 0.28m 이내에 장애물이 들어오면 path follower도 긴급 정지를
요청한다. 이는 보조 안전 기능이며 실제 제동거리 검증을 대신하지 않는다.

- `c`: 현재 모드 2 mission 취소
- `Esc`: waypoint 문자열 입력만 취소
- `s` 또는 `1`: mission 취소, 정지, 모드 1 복귀
- 상태 토픽: `/waypoint_queue_status`

약 1m 간격 waypoint 번호와 지도 시각화는
[docs/full_map_waypoints_1m_numbered.png](docs/full_map_waypoints_1m_numbered.png),
[maps/waypoint_queue_latest.yaml](maps/waypoint_queue_latest.yaml)에서 확인할 수 있다.

```bash
ros2 topic echo /waypoint_queue_status
ros2 topic echo /planner_state
ros2 topic echo /follower_state
```

### 모드 3: LiDAR + mmWave 사람 판별

목적은 LiDAR로 발견한 동적장애물에 접근한 뒤 C4001 mmWave presence로 사람 가능성을
판별하는 것이다. LiDAR·AMCL·동적장애물 marker와 C4001 `/mmwave/sensor_state = ONLINE`
상태가 필요하다.

LiDAR가 새 물체를 큰 빨간 점으로 표시하면 `3`을 누른 뒤 Space를 누른다. `3`만
입력하면 `MODE3_READY:PRESS_SPACE` 상태가 될 뿐 출발하지 않는다.

1. 현재 로봇과 가장 가까운 미분류 빨간 점을 선택한다.
2. A*로 대상의 2.0m 앞까지 이동하고 대상을 정면으로 바라본다.
3. 2초 정지 후 5초 동안 C4001의 튜닝된 presence·보정거리 샘플을 확인한다.
4. presence가 충분하면 `사람 감지!`를 출력하고 해당 점만 파란색으로 바꾼다.
5. 센서가 ONLINE이지만 presence가 없으면 `동적장애물!`을 출력하고 빨간색을
   유지한다.

센서가 OFFLINE이거나 샘플이 부족하면 사람 아님으로 오판하지 않고 판정을 보류한다.
C4001 사람 후보는 보정거리 0.6~6.0m, energy 3000 이상을 3프레임 연속 확인하고
6프레임 연속 미충족 시 해제하는 실험용 heuristic이다. 원본 거리는 보존하며 보정
거리는 `raw × 1.0 - 0.1m`이다. 이는 의료용 생체신호 측정이나 AI 사람 분류가 아니다.
한 검사가 끝난 뒤 Space를 다시 누르면 그 시점에서 가장 가까운 미분류 빨간 점을
다시 선택한다.

```bash
ros2 topic echo /mode3_status
ros2 topic echo /mode3_classification
ros2 topic echo /mmwave/sensor_state
ros2 topic echo /mmwave/human_presence
ros2 topic echo /mmwave/human_state
ros2 topic echo /mmwave/calibrated_distance_m
ros2 topic echo /mmwave/raw/distance_m
ros2 topic echo /mmwave/raw/energy_raw
```

`/mmwave/human_state`는 정상 관측 중 `MOVING`, `STILL`, `NO_HUMAN`을 발행한다.
로봇 자체가 움직이는 동안에는 `ROBOT_MOVING`, 센서가 끊기면 `SENSOR_OFFLINE`을
발행해 사람 상태로 오판하지 않는다.

### 모드 4: Camera Module 3 YOLO + LiDAR 요구조자 판별

목적은 LiDAR 동적장애물 좌표와 카메라 사람 바운딩박스를 결합해 실제 요구조자에
해당하는 지도 점만 파란색으로 바꾸는 것이다. 모드 2의 센서·TF에 더해 Camera
Module 3 Wide, 카메라 런타임, ONNX Runtime과 사람 모델이 필요하다. 카메라 노드는
기본 통합 명령에서는 꺼져 있으므로 `use_camera_mode4:=true`를 반드시 넘긴다.

저장소에는 임시 시험용 ONNX 모델이 다음 기본 경로에 포함되어 있다.

```text
~/fire_robot_rpi/models/yolov8n_best.onnx
```

모델 메타데이터상 YOLOv8n detect, 입력 640×640, class `{0: person}` 구성이다. 이
프로젝트에서 직접 학습한 모델은 아니므로 실제 배포 전 출처·재배포 권한과 현장
정확도를 다시 확인하고 자체 데이터 모델로 교체한다.

ONNX 모델은 Ultralytics 없이 ONNX Runtime CPU backend로 실행한다. ROS 노드를
실행하는 Python 환경에서 다음 import가 성공해야 한다.

```bash
python3 -c "import onnxruntime; print('YOLO ONNX runtime OK')"
```

모드 4를 사용할 때의 통합 실행 예시는 다음과 같다. 나머지 포트·지도 인자는 앞의
통합 실행 명령과 동일하게 함께 넘길 수 있다.

```bash
ros2 launch inno_robot_bringup field_waypoint_test.launch.py \
  use_camera_mode4:=true \
  yolo_model_path:="$HOME/fire_robot_rpi/models/yolov8n_best.onnx" \
  start_thermal_viewer:=false
```

`yolo_confidence` 기본값은 `0.50`이다. `mode4_inspector`의 association 최소값도
`inno_jazzy_ws/src/inno_autonav/config/autonav_params.yaml`에서 `0.50`으로 설정되어
있다. `yolo_confidence`만 낮추면 inspector가 낮은 confidence 박스를 다시 제외하므로,
임계값 실험 시 두 값을 같게 바꾸고 workspace를 다시 빌드·실행한다.

빨간 점이 보이면 `4`를 누른 뒤 Space를 누른다. `4`만 입력하면 준비 상태이며
Space를 눌러야 다음 순서가 시작된다.

1. 가장 가까운 미분류 빨간 점을 고른다.
2. A*로 대상의 1.5m 앞까지 이동하고 정면 정렬한다.
3. 2초간 정지해 흔들림을 줄인 뒤 5초 동안 YOLO 결과를 모은다.
4. 여러 frame에서 반복된 사람 검출만 후보로 확정한다.
5. 바운딩박스 수평 위치와 LiDAR 후보 방위각을 일대일로 연결한다.
6. 사람 방향의 빨간 점만 파란 점으로 바꾸고 나머지는 빨간색으로 유지한다.

사람과 일반 장애물이 한 카메라 프레임에 동시에 있고 LiDAR 빨간 점 두 개가 가까운
경우, 사람 바운딩박스의 수평 위치와 각 LiDAR 후보의 로봇 기준 방위각을 비교한다.
한 바운딩박스를 한 LiDAR 후보에만 연결하므로 사람 방향의 점만 파란색으로 바뀌고
다른 점은 빨간색으로 남는다.

- 요구조자 확인: `요구조자 감지!`, `/mode4_classification = SURVIVOR:...`
- 사람 미검출: `요구조자 미감지!`, 빨간색 유지
- 모델·카메라·추론 프레임 없음: 판정 보류, 빨간색 유지
- `c`, `s`, `1`: 검사 취소 및 모드 1 복귀

한 검사가 끝난 뒤 Space를 다시 누르면 다음으로 가까운 미분류 빨간 점을 검사한다.

두 물체가 LiDAR 단계에서 이미 하나의 cluster로 합쳐져 빨간 점이 하나만 만들어지면
카메라만으로 지도 점을 둘로 나눌 수 없다. 현장에서 이런 경우
`inno_jazzy_ws/src/inno_autonav/config/autonav_params.yaml`의
`cluster_radius_m`을 낮춰 다시 검증한다.
내부 파라미터가 없을 때 사용하는 `fallback_horizontal_fov_deg`와 카메라 장착
`camera_yaw_offset_deg`도 실제 화각·장착각에 맞춰야 한다.

```bash
ros2 topic echo /mode4_status
ros2 topic echo /mode4_classification
ros2 topic echo /camera/person_detector_status
ros2 topic echo /camera/person_detections
```

#### 임시 ONNX 모델 공개사진 점검 결과

현재 모델을 실제 모드 4와 같은 ONNX backend, 입력 `640×640`, confidence `0.50`으로
공개 저시점 사진에 적용했다. 이는 정답 라벨이 있는 성능평가가 아니라 배포 전 육안
점검이다.

- 일반 저시점 6장: 사람 박스 총 33개 출력
- 약 1.5m 접근 화면 크기를 근사한 6장: 3장 검출, 3장 미검출
- 검출된 근거리 장면 confidence: `0.87~0.90`
- 뒷모습과 극단적인 로우앵글은 놓쳐 현장 데이터 재학습이 필요함
- 인터넷 사진에는 실제 촬영 거리가 없으므로 1.5m 표기는 사람의 화면 점유율을
  기준으로 한 근사치임

![약 1.5m 근거리 저시점 사람 검출 결과](data/model_eval/yolov8n_best_robot_view_2026-08-19/near_1p5m/yolov8n_best_approx_1p5m_contact_sheet.jpg)

전체 결과와 원본 출처:

- [일반 저시점 결과](data/model_eval/yolov8n_best_robot_view_2026-08-19/yolov8n_best_robot_view_contact_sheet.jpg)
- [근거리 개별 결과](data/model_eval/yolov8n_best_robot_view_2026-08-19/near_1p5m/annotated)
- [근거리 검출 좌표·confidence](data/model_eval/yolov8n_best_robot_view_2026-08-19/near_1p5m/results.json)
- [공개사진 출처와 사용 조건](data/model_eval/yolov8n_best_robot_view_2026-08-19/near_1p5m/SOURCES.md)

## 4. Camera Module 3 사진 촬영과 보정

ROS 없이 카메라 화면을 열고 학습용 사진을 저장하려면 다음만 실행한다.

```bash
sudo apt install -y python3-picamera2 python3-opencv
cd ~/fire_robot_rpi
./run_camera.sh
```

- `s`: `data/camera_capture/`에 JPEG 한 장 저장
- `q` 또는 `Esc`: 종료
- 해상도 변경 예: `./run_camera.sh --width 1920 --height 1080`

YOLO 데이터 수집 해상도와 실제 추론 해상도·카메라 sensor mode는 일치시키는 것이
좋다. 내부·외부 보정 작업은 다음 순서로 실행한다.

```bash
./capture_intrinsic_images.sh
./calibrate_camera.sh
./view_calibration_result.sh
./run_lidar_camera_calibration.sh
./run_lidar_camera_distance.sh
```

세부 조건은 [체커보드 Rational 캘리브레이션 가이드](docs/checkerboard_rational_calibration.md)와
[RPLIDAR C1–카메라 외부 보정 가이드](docs/lidar_camera_extrinsic_and_distance.md)를
참고한다.

## 5. 전체 주행 rosbag 기록

통합 launch 실행 후 새 터미널에서 실행한다.

```bash
cd ~/fire_robot_rpi
./record_robot_bag.sh
```

다음 항목을 한 번에 기록한다.

- 좌·우 모터 목표 STEP/s와 `/cmd_vel` 계열
- LiDAR `/scan`, TF, AMCL, 지도, 실제·계획 경로
- waypoint 명령·상태와 동적장애물 후보·marker
- mmWave raw·filtered 값과 모드 3 판정
- 카메라 영상·YOLO 바운딩박스와 모드 4 판정

시작 전 각 토픽을 `수신`, `대기`, `없음`으로 표시한다. 일부 센서가 없더라도 기록을
중단하지 않으며, 나중에 나타난 토픽도 저장한다. 결과는 `bags/fire_robot_날짜_시간/`,
점검 결과는 같은 이름의 `.topics.txt`에 생성된다. 카메라 원본 영상은 용량이 크므로
저장 공간을 먼저 확인한다.

토픽 상태만 검사하려면:

```bash
./record_robot_bag.sh --check-only
```

## 6. 테스트

센서가 없는 PC에서도 핵심 파싱, 경로, 모드 전환, mmWave 판정, 카메라–LiDAR
일대일 결합을 테스트할 수 있다.

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

colcon test --packages-select \
  inno_camera_tools inno_drive_bridge inno_mmwave inno_autonav \
  inno_robot_bringup
colcon test-result --verbose
```

현재 변경 기준으로 관련 7개 패키지 빌드와 149개 테스트가 통과했다. 개발 PC에서는
ROS 2 Humble 호환 빌드·토픽 통합 테스트까지 수행했으며, Raspberry Pi 5의 Jazzy와
실제 센서·모터 검증은 현장에서 진행해야 한다.

## 안전 주의사항

- 첫 모터 시험은 반드시 바퀴를 바닥에서 띄우고 `drive_speed:=0.06` 이하에서 한다.
- `s` 또는 `1`로 자율주행을 취소할 수 있는지 먼저 확인한다.
- `/cmd_vel` publisher와 `odom → base_link`, `map → odom` TF publisher는 각각 하나만
  활성화한다.
- 2D LiDAR가 보지 못하는 낮은 물체와 실제 제동거리를 별도로 고려한다.
- 사람 주변 실제 운용에는 소프트웨어 외의 비상정지와 안전 감시자가 필요하다.
- YOLO·mmWave 판정은 구조 보조 정보이며 사람의 안전이나 신원을 단독으로 보장하지
  않는다.

현장 점검 순서는 [FIELD_TEST_TODAY_KO.md](FIELD_TEST_TODAY_KO.md)에 더 자세히
정리되어 있다.
