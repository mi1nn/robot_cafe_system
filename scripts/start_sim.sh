#!/bin/bash
# 가상 로봇 + ROS 노드 + rosbridge + 백엔드를 한 번에 실행. Ctrl+C 한 번으로 전부 종료.
# 각 프로세스 로그: log/sim/<이름>.log
# 실제 로봇은 start_real.sh 사용 (ROBOT_MODE=real 로 이 스크립트를 실행)
# REAL_CAMERA=1: 가상 로봇 + 실제 RealSense + YOLO (탐지 확인용. 가상 로봇 기준이라 좌표는 의미 없음)

source "$(dirname "$0")/cafe_env.sh" || exit 1
ROOT="$CAFE_ROOT"
MODE="${ROBOT_MODE:-virtual}"
CAMERA="${REAL_CAMERA:-0}"
[ "$MODE" = real ] && CAMERA=1
MODEL="$ROOT/cobot1_ws/src/cafe_vision/models/cup_pose.pt"
if [ "$CAMERA" = 1 ] && [ ! -f "$MODEL" ]; then
    echo "YOLO 모델이 없습니다: $MODEL (cobot1_ws/src/cafe_vision/README.md 학습 절 참고)"
    exit 1
fi
LOG="$ROOT/log/sim"
[ "$MODE" = real ] && LOG="$ROOT/log/real"
mkdir -p "$LOG"

if pgrep -f "ros2_control_node|run_emulator|driver_node|robot_control_node|coffee_process_node|vision_node|rosbridge_websocket|backend.app" > /dev/null; then
    echo "이미 실행 중인 로봇/노드가 있습니다. 먼저 종료하세요."
    exit 1
fi

trap 'echo; echo "종료 중..."; kill -INT 0 2>/dev/null; wait' EXIT
trap 'exit' INT TERM

wait_log() {
    for _ in $(seq "$3"); do grep -q "$2" "$1" 2>/dev/null && return 0; sleep 1; done
    echo "시간 초과: $1 에서 '$2' 를 못 찾았습니다."; exit 1
}

# 카메라를 쓰면 손목 RealSense 까지 띄우는 bringup_camera 사용 (camera:=true 는 가상 모드에서도 카메라를 띄움).
BRINGUP=(bringup.launch.py)
[ "$CAMERA" = 1 ] && BRINGUP=(bringup_camera.launch.py camera:=true)

echo "[1/7] 로봇 bringup ($MODE ${ROBOT_HOST:-127.0.0.1}, camera=$CAMERA)"
ros2 launch m0609_rg2_bringup "${BRINGUP[@]}" mode:="$MODE" host:="${ROBOT_HOST:-127.0.0.1}" > "$LOG/bringup.log" 2>&1 &
wait_log "$LOG/bringup.log" "Configured and activated dsr_controller2" 90

echo "[2/7] robot_driver"
ros2 run robot_driver driver_node > "$LOG/driver.log" 2>&1 &
echo "[3/7] robot_control"
ros2 run robot_control robot_control_node --ros-args \
    -p poses_file:="$ROOT/config/robot/$MODE/poses.yaml" > "$LOG/control.log" 2>&1 &
echo "[4/7] coffee_process"
ros2 run coffee_process coffee_process_node --ros-args \
    -p recipe_dir:="$ROOT/cobot1_ws/src/coffee_process/config/recipes" \
    -p io_file:="$ROOT/config/robot/$MODE/io.yaml" \
    -p grasp_file:="$ROOT/config/robot/$MODE/cup_grasp.yaml" > "$LOG/coffee.log" 2>&1 &
if [ "$CAMERA" = 1 ]; then
    echo "[5/7] cafe_vision (YOLO + RealSense)"
    ros2 run cafe_vision vision_node --ros-args \
        -p model_path:="$MODEL" \
        -p extrinsics_file:="$ROOT/config/robot/$MODE/camera_extrinsics.yaml" > "$LOG/vision.log" 2>&1 &
else
    MOCK="${MOCK_DETECTION:-$ROOT/config/robot/virtual/mock/inverted.yaml}"
    echo "[5/7] cafe_vision (mock: $(basename "$MOCK"))"
    ros2 run cafe_vision vision_node --ros-args -p mock_file:="$(realpath "$MOCK")" > "$LOG/vision.log" 2>&1 &
fi
echo "[6/7] rosbridge"
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > "$LOG/rosbridge.log" 2>&1 &
wait_log "$LOG/rosbridge.log" "Rosbridge WebSocket server started" 30

echo "[7/7] 백엔드 → http://localhost:${FLASK_PORT:-5000}"
echo "로그: $LOG   (종료: Ctrl+C)"
"$ROOT/scripts/start_backend.sh" 2>&1 | tee "$LOG/backend.log"
