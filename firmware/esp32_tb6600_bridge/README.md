# ESP32 TB6600 bridge firmware

대상 보드는 ESP32 DevKit V4의 ESP32-WROOM-32E 모듈이다. 실제 wheel encoder 없이 TB6600에 발생한 STEP pulse 수를 임시 encoder count로 사용한다.

## 핀 배치

| 기능 | ESP32 GPIO |
|---|---:|
| Left STEP/PUL | 25 |
| Left DIR | 26 |
| Left ENA | 27 |
| Right STEP/PUL | 14 |
| Right DIR | 12 |
| Right ENA | 13 |
| HC-SR04 TRIG | 32 |
| HC-SR04 ECHO | 33 (1kΩ/2kΩ 분압 필수) |

GPIO12는 ESP32 strapping pin이다. TB6600 연결 상태 때문에 업로드 또는 부팅이
불안정하면 Right DIR 배선을 GPIO16 등 검증한 출력 핀으로 옮기고 펌웨어의 `R_DIR`도
같이 변경한다. GPIO32와 GPIO33은 모드 6 초음파 센서가 사용하므로 모터에 사용하지
않는다.

ESP32 GPIO는 3.3V 신호다. 사용하는 TB6600 모듈의 PUL/DIR/ENA 입력이 3.3V 신호를 확실히 인식하는지 데이터시트나 실제 입력 회로를 확인한다. 모터 전원과 USB 5V를 직접 연결하지 않는다. TB6600 신호 연결 방식에 맞는 공통 기준과 절연 입력 배선을 사용한다.

HC-SR04는 5V로 동작하며 ECHO도 5V로 출력한다. `ECHO → 1kΩ → GPIO33`으로 연결하고
`GPIO33 → 2kΩ → GND`를 추가하여 약 3.3V로 낮춘다. ECHO를 ESP32 또는 Raspberry Pi
GPIO에 직접 연결하지 않는다. 펌웨어는 인터럽트로 ECHO 폭을 측정하므로 모터 STEP
발생 루프를 `pulseIn()`으로 막지 않는다.

## encoder 없는 동작

`AccelStepper::runSpeed()`가 STEP pulse를 발생시킬 때 `currentPosition()`도 함께 증가하거나 감소한다. 펌웨어는 이 값을 100ms마다 다음 형식으로 보낸다.

```text
ENC,<millis>,<left_virtual_count>,<right_virtual_count>
```

전면 초음파 거리는 200ms 텔레메트리에 다음 형식으로 함께 전송된다.

```text
US,<millis>,<distance_m>,<valid_0_or_1>
```

이 값은 명령한 pulse의 누적값이지 실제 바퀴 회전 측정값은 아니다. 탈조, 미끄러짐, 바퀴 걸림은 검출하지 못하므로 SLAM/내비게이션의 최종 odometry로 사용하지 않는다.

## Arduino IDE 설정

1. Espressif ESP32 board package를 설치한다.
2. 보드는 `DOIT ESP32 DEVKIT V1` 또는 설치된 패키지에서 DevKit V4/WROOM-32E와 호환되는 항목을 선택한다.
3. Library Manager에서 `AccelStepper`를 설치한다.
4. Upload Speed는 우선 `115200` 또는 `460800`을 사용한다.
5. 해당 serial port를 선택하고 `esp32_tb6600_bridge.ino`를 업로드한다.

## 모터 연결 전 serial 시험

반드시 TB6600 모터 전원을 끈 상태에서 먼저 시험한다.

```bash
python3 ~/esp32_serial_check.py /dev/ttyUSB0 PING,1
python3 ~/esp32_serial_check.py /dev/ttyUSB0 ZERO,2
```

picocom에서는 CR/LF 어느 쪽으로 보내도 펌웨어가 명령을 분리한다.

```bash
picocom --baud 115200 /dev/ttyUSB0
```

수동 명령 예시(좌/우 step/s):

```text
PING,1
M,2,80,80
STOP,3
M,4,-80,80
STOP,5
ZERO,6
```

## 안전 시험 순서

1. 모터 전원을 끄고 `PING`, `ZERO`, `M,...`의 ACK/STAT/ENC만 확인한다.
2. 바퀴를 공중에 띄우고 TB6600 모터 전원을 켠다.
3. 낮은 `M` 명령 직후 `STOP`을 보내 한쪽씩 방향을 확인한다.
4. 반대 방향이면 우선 `INVERT_LEFT_DIR` 또는 `INVERT_RIGHT_DIR`을 수정한다.
5. 명령을 보내지 않았을 때 500ms 이내 `ERR,COMMAND_TIMEOUT_STOP`과 정지를 확인한다.
6. 모든 정지 시험이 성공한 뒤에만 최저 속도로 지상 시험한다.

`M` 명령이 갱신되지 않으면 500ms 후 자동 정지한다. ROS bridge가 10Hz 이상으로
계속 명령을 보내므로 정상 운전 중에는 watchdog이 갱신된다. 펌웨어의
`MAX_STEP_ACCEL`이 급격한 STEP 주파수 변화를 ramp 처리한다. 너무 둔하면 값을 조금
올리고, 꺾을 때 충격이 남으면 내린다.

## 엔코더에 관한 중요한 구분

`ENC`는 ESP32가 **발생시킨 step 수**다. 실제 AS5048A 측정값이 아니므로 탈조와
미끄러짐을 알 수 없다. 불안정하게 설치된 엔코더가 주행을 조기에 끝내는 문제를 막기
위해 오늘의 waypoint 주행은 이 값을 완료 조건으로 사용하지 않고 LiDAR TF를 쓴다.
AS5048A를 기계적으로 고정한 뒤에는 별도 검증을 거쳐 센서 odometry 입력을 추가한다.

## 모드의 위치

펌웨어 안에 고정 거리 waypoint나 `1/2` 모드는 없다. `1`(키보드)과 `2`(자율)는 ROS의
`cmd_vel_mode_mux`가 안전하게 선택한다. ESP32가 ROS 위치 추정과 별도의 waypoint를
실행하면 두 제어기가 충돌하므로 모터 출력만 담당하게 한 것이다.
