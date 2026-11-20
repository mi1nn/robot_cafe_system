# Robot Cafe System

Doosan M0609 based autonomous barista robot system.

## Environment

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- Doosan DSR_ROBOT2
- PyQt5
- Flask
- WebSocket / Socket.IO
- SQLite3

## Architecture

Browser
    |
    v
Flask Backend
    |
    v
ROS2
    |
    v
coffee_process
    |
    v
robot_control
    |
    v
robot_driver
    |
    v
Doosan DSR_ROBOT2
    |
    v
M0609

## Important

Doosan API access is isolated in:

    robot_driver/doosan_adapter.py

Application code must not directly import DSR_ROBOT2.

Robot coordinates are stored in YAML configuration files.

Never use the sample coordinates on a real robot.

Real robot configuration must be created after:

1. Teaching
2. TCP verification
3. Tool verification
4. I/O verification
5. Safety verification
6. Dry-run testing

0. 필요 패키지 존재 확인(requirement.txt 파일에 있는 설치 패키지)
```bash
apt search python3-flask 
apt search python3-yaml 
apt search python3-requests 
apt search python3-socketio 
apt search python3-eventlet 
apt search python3-flask-socketio
```

1. 필요 패키지 설치
```bash
sudo apt update && sudo apt install \
    python3-flask \
    python3-flask-socketio \
    python3-yaml \
    python3-requests \
    python3-socketio \
    python3-eventlet \
    python3-pytest
```
2. rosbridge 설치

```bash
sudo apt update
sudo apt install ros-jazzy-rosbridge-suite
```

rosbridge_suite는 rosbridge_server, rosbridge_library, rosapi 등을 포함하는 메타 패키지입니다.

3. 설치 후 확인
```bash
ros2 pkg list | grep rosbridge

#정상이라면 대략:
#rosbridge_msgs
#rosbridge_server
#rosbridge_library
#rosapi
```
4. rosbridge_server 실행
```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## Execution

### Simulation

```bash
cd <저장소 경로>
./scripts/build.sh          # 처음 한 번
./scripts/start_sim.sh      # 가상 로봇 + ROS 노드 + 비전(mock) + rosbridge + 백엔드 (로그: log/sim/)
```

로봇은 STOPPED 상태로 시작하므로 첫 주문 전에 초기화합니다.

```bash
curl -X POST localhost:5000/api/control/resume   # 홈 이동 → HOMED
curl -X POST localhost:5000/api/control/start    # IDLE (주문 대기)
```

브라우저에서 `http://localhost:5000` 접속 후 주문합니다.

시뮬레이션에서는 카메라 대신 가짜 탐지 결과를 씁니다 (기본: 뒤집힌 컵). 다른 컵 상태로 해 보려면:

```bash
MOCK_DETECTION=config/robot/virtual/mock/lying.yaml ./scripts/start_sim.sh   # upright / inverted / lying
```

가상 로봇에 실제 RealSense 와 YOLO 모델을 붙여 탐지만 확인하려면 (`models/cup_pose.pt` 필요):

```bash
REAL_CAMERA=1 ./scripts/start_sim.sh
```

로봇이 가상이라 TF 가 실제 손목 위치와 다르므로, 나온 좌표는 의미가 없습니다. 탐지 상태(`log/sim/vision.log`)만 보세요.

### Real

실행 전에 아래 순서로 준비합니다. 자세한 절차는 `cobot1_ws/src/cafe_vision/README.md` 에 있습니다.

1. **핸드-아이 캘리브레이션** → `config/robot/real/camera_extrinsics.yaml`
2. **컵 모델 학습** → `cobot1_ws/src/cafe_vision/models/cup_pose.pt`
3. **티칭·배선**: `config/robot/real/` 의 포즈(`poses.yaml`: `cup_look` 포함, `workspace` 범위), I/O(`io.yaml`), 컵 집기 설정(`cup_grasp.yaml`: `station_position` 은 `cup_pick` 과 같은 자리)

값이 0 이거나 `TODO` 로 남아 있거나, 캘리브레이션 결과·모델 파일이 없으면 스크립트가 실행을 거부합니다.

```bash
ROBOT_HOST=<로봇 IP> ./scripts/start_real.sh   # 설정 검사 → yes 입력 → 실행 (로그: log/real/)
```

초기화와 주문 방법은 Simulation 과 같습니다. 처음에는 낮은 속도로, 비상정지 버튼을 준비한 상태에서 실행하세요.
