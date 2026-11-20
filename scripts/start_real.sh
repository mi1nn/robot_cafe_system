#!/bin/bash
# 실제 M0609(+ 손목 RealSense) + ROS 노드 + 비전 + rosbridge + 백엔드를 한 번에 실행. Ctrl+C 한 번으로 전부 종료.
# 사용: ROBOT_HOST=192.168.1.100 ./scripts/start_real.sh   (설정: config/robot/real/)
# 로그: log/real/<이름>.log

ROOT="$(cd "$(dirname "$0")/.." && pwd)"

if [ -z "$ROBOT_HOST" ]; then
    echo "ROBOT_HOST 가 필요합니다. 예: ROBOT_HOST=192.168.1.100 $0"
    exit 1
fi

echo "실제 로봇 설정 검사: config/robot/real/"
if ! python3 "$ROOT/scripts/check_robot_config.py" "$ROOT/config/robot/real"; then
    echo "티칭/배선 확인 후 위 값을 채우세요. 실행을 중단합니다."
    exit 1
fi

MODEL="$ROOT/cobot1_ws/src/cafe_vision/models/cup_pose.pt"
if [ ! -f "$MODEL" ]; then
    echo "모델 없음: $MODEL (cobot1_ws/src/cafe_vision/README.md 학습 절 참고). 실행을 중단합니다."
    exit 1
fi

echo
echo "실제 로봇($ROBOT_HOST)이 움직입니다. 주변 안전, 비상정지 버튼, 협동 속도 설정을 확인하세요."
read -r -p "계속하려면 yes 입력: " answer
[ "$answer" = "yes" ] || { echo "취소했습니다."; exit 1; }

ROBOT_MODE=real exec "$ROOT/scripts/start_sim.sh"
