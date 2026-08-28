# 모드 6 연기 보조주행 배선 및 시험

모드 6은 실제 CO 농도 대신 `Space`를 연기 발생 트리거로 사용하는 독립 시험 모드다.
모드 2에서 시작한 저장 지도 웨이포인트 경로를 유지하고, BNO055 상대 방위각과 전면
HC-SR04 거리를 보조 입력으로 사용한다. 모드 1~5의 입력과 동작은 그대로 유지한다.

## 1. 배선 전 필수 안전사항

- 모든 배선은 Raspberry Pi, ESP32, TB6600 모터 전원을 모두 끈 상태에서 한다.
- HC-SR04의 ECHO는 5V이므로 ESP32나 Raspberry Pi GPIO에 직접 연결하지 않는다.
- BNO055와 HC-SR04, ESP32의 GND는 각 제어기 기준에 맞게 공통으로 연결한다.
- 센서 배선을 모터 전원선·TB6600 출력선과 떨어뜨리고 IMU는 자석과 모터에서 최대한
  멀리 고정한다.
- 첫 시험은 바퀴를 바닥에서 띄우고 `drive_speed:=0.03`으로 수행한다.

## 2. MCU-055(BNO055)와 Raspberry Pi 5 배선

링크의 제품은 MCU-055 계열 BNO055 9축 IMU다. Raspberry Pi의 I2C-1을 사용한다.
모듈 복제품마다 실크 인쇄가 다르므로 `SDA/SCL`이 직접 적혀 있으면 그 핀을 우선한다.
`ATX/LRX`로 적힌 제품은 I2C 모드에서 각각 SDA/SCL로 쓰이는 형식인지 판매 사진과
기판 실크를 다시 확인한다.

| MCU-055 | Raspberry Pi 5 물리 핀 | GPIO 기능 |
|---|---:|---|
| `VCC` 또는 `VIN` | 1 | 3.3V |
| `GND` | 6 | GND |
| `SDA` 또는 I2C 모드의 `ATX` | 3 | GPIO2 / SDA1 |
| `SCL` 또는 I2C 모드의 `LRX` | 5 | GPIO3 / SCL1 |
| `S0/PS0` | GND | I2C 모드 선택, 핀이 노출된 경우 |
| `S1/PS1` | GND | I2C 모드 선택, 핀이 노출된 경우 |
| `INT`, `RES/RST`, `BOOT` | 연결하지 않음 | 현재 소프트웨어에서 미사용 |

MCU-055 보드는 3.3~5V 입력을 지원하는 형식이지만, Raspberry Pi I2C 논리 전압을
확실히 3.3V로 유지하기 위해 이 배선에서는 물리 핀 1의 3.3V를 사용한다. S0/S1 패드가
납땜 선택식이면 전원을 넣기 전에 둘 다 I2C 위치인지 확인한다.

```text
Raspberry Pi 5                 MCU-055 (BNO055)

물리 1  3.3V  --------------> VCC/VIN
물리 3  GPIO2/SDA1 ----------> SDA 또는 ATX
물리 5  GPIO3/SCL1 ----------> SCL 또는 LRX
물리 6  GND   ---------------> GND
                                  S0/PS0 ─┐
                                  S1/PS1 ─┴── GND (노출된 경우)
```

Ubuntu에서 `/boot/firmware/config.txt`에 `dtparam=i2c_arm=on`이 있는지 확인하고 재부팅한
뒤 다음 명령을 실행한다.

```bash
ls -l /dev/i2c-1
sudo i2cdetect -y 1
```

주소 표에 `28` 또는 `29`가 보여야 한다. 소프트웨어는 기본 0x28과 보조 0x29를 모두
확인한다. 둘 다 보이지 않으면 ROS를 실행하지 말고 VCC/GND, I2C 모드 패드와 SDA/SCL
순서를 다시 확인한다.

## 3. HC-SR04와 ESP32 배선

HC-SR04 펄스 폭은 기존 모터용 ESP32에서 인터럽트로 측정한다. Linux GPIO에서 직접
펄스 폭을 재지 않으므로 Raspberry Pi 부하에 따른 거리 오차를 줄이고, ESP32 모터
STEP 루프도 `pulseIn()`으로 막지 않는다.

| HC-SR04 | 연결 | 설명 |
|---|---|---|
| `VCC` | ESP32 `VIN/5V` | USB에서 공급되는 5V |
| `GND` | ESP32 `GND` | 공통 GND |
| `TRIG` | ESP32 GPIO32 | 3.3V 출력, 직접 연결 |
| `ECHO` | 1kΩ/2kΩ 분압 후 GPIO33 | 5V를 약 3.3V로 낮춤 |

```text
ESP32 VIN/5V ------------------------> HC-SR04 VCC
ESP32 GND ---------------------------> HC-SR04 GND
ESP32 GPIO32 ------------------------> HC-SR04 TRIG

HC-SR04 ECHO ----[ 1kΩ ]----+--------> ESP32 GPIO33
                             |
                           [ 2kΩ ]
                             |
ESP32 GND -------------------+ 
```

초음파 센서는 로봇 전면 중앙에 수평으로 고정하고 송수신 원통 앞을 가리지 않는다.
기본 TF는 센서가 `base_link` 원점보다 전방 0.20m, 높이 0.20m에 있다고 가정한다.
실측값이 다르면 launch 인자 `ultrasonic_x`, `ultrasonic_y`, `ultrasonic_z`를 수정한다.

GPIO32/33을 사용하려면 변경된 아래 펌웨어를 ESP32에 다시 업로드해야 한다.

```text
firmware/esp32_tb6600_bridge/esp32_tb6600_bridge.ino
```

업로드 후 직렬 출력에 다음 형식이 약 200ms마다 나타나야 한다.

```text
US,<ESP32_millis>,<distance_m>,<valid>
```

## 4. 모드 6 소프트웨어 동작

1. 기존 RPLIDAR, RF2O와 AMCL로 저장 SLAM 지도에서 위치를 잡는다.
2. 모드 2에서 시험할 웨이포인트 경로를 입력하여 주행을 시작한다.
3. 주행 중 `6`을 누르면 현재 목표와 경로를 취소하지 않고 모드 6으로 전환한다.
4. `Space`를 누르면 IMU와 초음파 데이터가 모두 최신인지 검사한 뒤 보조주행을 켠다.
5. RViz의 LiDAR 점을 빨간색에서 파란색으로 바꾸고 연기 보조주행 로그를 출력한다.
6. 초음파 거리가 0.8m 이하가 되면 즉시 0.5초 이상 정지한다.
7. 측정 위치를 지도 좌표로 변환하여 동적 코스트맵에 0.5m inflation으로 반영한다.
8. A*가 현재 목표까지 새 경로를 만들면 IMU 상대 방위각을 이용한 제자리 회전만 먼저
   허용하고, 초음파 거리가 0.9m 이상 확보된 뒤 전진을 재개한다.
9. 다시 `Space`를 누르면 보조주행을 끄고 RViz 점을 빨간색으로 복원한다.

LiDAR를 완전히 끄지는 않는다. 저장 지도 기반 위치추정은 계속 사용하고, 연기 상황에서
전방 충돌 검출과 회전 방위각을 HC-SR04와 IMU가 보조한다. 센서 데이터가 1초 이상
끊기면 모터 명령을 0으로 만들어 안전 정지한다.

## 5. 빌드와 실행 명령어

```bash
cd ~/fire_robot_rpi/inno_jazzy_ws && \
source /opt/ros/jazzy/setup.bash && \
colcon build --symlink-install --packages-up-to inno_robot_bringup && \
source install/setup.bash && \
ros2 launch inno_robot_bringup mode6_smoke_assist.launch.py \
  esp32_port:=/dev/ttyUSB0 \
  lidar_port:=/dev/ttyUSB1 \
  use_serial:=true \
  use_bno055_imu:=true \
  drive_speed:=0.03 \
  turn_speed:=0.30
```

실행 후 터미널 입력 순서는 다음과 같다.

```text
2
w1,w5,w9       # 실제 지도에서 안전한 시험 웨이포인트로 바꾼다.
6
Space          # 연기 보조주행 ON
Space          # 연기 보조주행 OFF
```

## 6. 확인 토픽

```bash
ros2 topic echo /imu/status
ros2 topic hz /imu/data
ros2 topic echo /ultrasonic/front/range
ros2 topic echo /mode6/status
ros2 topic echo /mode6/log
ros2 topic echo /follower_state
ros2 topic echo /dynamic_obstacle_grid
```

정상 시작 로그는 다음 문장을 포함한다.

```text
연기 농도가 높아 LiDAR 신뢰도가 낮습니다. 초음파 센서와 IMU를 보조적으로 활용해
저장된 SLAM 지도를 기준으로 경로를 계획합니다.
```

## 7. 시험 합격 조건

- `i2cdetect`에 0x28 또는 0x29가 표시되고 `/imu/data`가 연속 발행된다.
- 전방 물체 거리를 줄였을 때 `/ultrasonic/front/range`가 실제 거리와 비슷하게 변한다.
- `Space` ON일 때만 RViz LiDAR 점이 파란색이고 OFF일 때 빨간색이다.
- 0.8m 이하에서 로봇이 먼저 정지하며 코스트맵과 `/planned_path`가 갱신된다.
- 회전 중 전진 명령이 차단되고 0.9m 이상 확보된 뒤에만 전진한다.
- IMU 또는 초음파 토픽을 끊으면 1초 이내 안전 정지한다.
- 모드 1~5의 기존 시험 결과가 그대로 유지된다.

전면 초음파 한 개는 좌우 측면의 빈 공간을 직접 측정할 수 없다. 따라서 모드 6은 임의로
왼쪽이나 오른쪽으로 외워서 회피하지 않고, 저장 지도와 코스트맵으로 생성된 우회 경로의
회전만 허용한다. 옷감, 비스듬한 판, 흡음재는 HC-SR04 반사가 약할 수 있으므로 실제
장애물 재질별 거리 검증이 필요하다.

## 참고 사양

- [Bosch BNO055 데이터시트](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [Raspberry Pi GPIO 전압 및 40핀 헤더 문서](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio)
- [HC-SR04 데이터시트](https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf)
