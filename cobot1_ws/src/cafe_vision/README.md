# cafe_vision

손목 RealSense 카메라로 종이컵을 찾아 로봇이 집을 좌표를 계산하는 패키지입니다.
비전 관련 코드는 모두 이 패키지에만 있고, 다른 패키지는 `/vision/detect_cup` 서비스로만 사용합니다.

| 노드 | 역할 |
|---|---|
| `vision_node` | `/vision/detect_cup` 서비스. 컵 상태(upright / inverted / lying)와 base 기준 집는 점 |
| `calibrate_node` | 체커보드 핸드-아이 캘리브레이션 (카메라가 손목에 어떻게 달렸는지) |
| `capture_node` | 학습용 사진 촬영 |

| 코드 | 역할 |
|---|---|
| `geometry.py` | 픽셀 → 3D, TF → 행렬, 집는 점 계산 (순수 함수) |
| `detection.py` | 탐지 후보 → 서비스 응답 |
| `detector.py` | YOLO 추론 (ultralytics 는 여기서만 사용) |
| `hand_eye.py` | 체커보드 포즈, 핸드-아이 계산, 검증 편차 |

실제 로봇에서 쓰기까지의 순서: **설치 → 핸드-아이 캘리브레이션 → 데이터 촬영·라벨링·학습 → 실행**.
시뮬레이션(`scripts/start_sim.sh`)은 카메라 없이 mock 결과를 쓰므로 아래 과정이 필요 없습니다.

## 1. 설치

YOLO(ultralytics)는 실제 탐지와 학습에만 필요합니다. numpy 2 가 설치되면 ROS 의 cv_bridge 가 깨지므로 반드시 함께 고정합니다.

```bash
pip install --break-system-packages --user ultralytics "numpy<2"
python3 -c "import cv_bridge, numpy; print('ok', numpy.__version__)"   # 1.x 이어야 함
```

## 2. 핸드-아이 캘리브레이션

카메라 좌표를 로봇 좌표로 바꾸려면 **카메라가 손목(`link_6`)에 어떤 위치·방향으로 달려 있는지(X)** 를 알아야 합니다.
URDF 의 장착값은 실측값이 아니므로 체커보드로 직접 구합니다. 이 노드는 로봇을 움직이지 않습니다.

1. **보드 준비**: 체커보드를 인쇄해 테이블에 단단히 고정합니다 (캘리브레이션 중 움직이면 안 됨).
   `calibration/board.yaml` 의 내부 코너 수(`cols`, `rows`)와 칸 크기(`square_mm`, 자로 실측)를 맞춥니다.
2. **실행** (저장소 루트에서, 각각 다른 터미널)

   ```bash
   source scripts/cafe_env.sh
   ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=<로봇 IP>
   ros2 run robot_driver driver_node
   ros2 run cafe_vision calibrate_node --ros-args \
       -p output_file:=$PWD/config/robot/real/camera_extrinsics.yaml
   ```
3. **촬영**: 티치펜던트나 직접 교시로 로봇을 옮겨 보드 전체가 화면에 보이게 하고, 로봇이 멈춘 상태에서 Enter.
   - 15장 이상, 보드를 보는 각도를 다양하게(기울기 ±30° 안팎, 손목 회전 섞기). 거리만 바꾼 샘플은 도움이 적습니다.
   - "체커보드를 찾지 못했습니다"가 나오면 보드가 잘리지 않게 다시 찍습니다.
4. **계산**: `q` 를 누르면 결과와 검증 편차를 출력하고 `output_file` 에 저장합니다.
   - 보드 위치 편차가 5 mm 를 넘으면 경고합니다. 샘플을 다시 찍는 것을 권장합니다.
   - 원본 샘플(영상, 행렬)은 `log/calibration/<날짜>/` 에 남습니다.
5. **확인**: 실제 로봇 시험 0단계로, 비전으로 계산한 보드 코너 위치 50 mm 위로 로봇을 보내 눈으로 맞는지 확인합니다.

TF 프레임 이름이 다르면 `-p base_frame:=... -p flange_frame:=...` 로 바꿉니다 (`ros2 run tf2_ros tf2_echo base_link link_6` 로 확인).

## 3. 데이터 촬영 · 라벨링 · 학습

모델은 컵 하나마다 **상자 + 상태(3클래스) + 키포인트 2개**(0 = `rim` 입구 중심, 1 = `bottom` 바닥 중심)를 찾습니다.
두 키포인트를 이으면 누운 컵의 방향과 입구 쪽을 알 수 있습니다. 정의는 `training/cup.yaml` 입니다.

1. **촬영**: 실제 탐지와 같은 `cup_look` 자세로 로봇을 옮긴 뒤

   ```bash
   ros2 run cafe_vision capture_node      # s: 저장, q: 종료 → ~/cafe_vision_dataset/raw/
   ```
   - 상태별로 100장 이상: 뒤집힌 컵 더미(1–10개 높이), 누운 컵(여러 방향, 입구 방향 섞기), 바로 선 컵 1개.
   - 테이블 위 위치, 조명, 주변 물건을 바꿔 가며 찍습니다. 컵이 여러 개 함께 보이는 사진도 넣습니다.
2. **라벨링**: [CVAT](https://www.cvat.ai/) 등에서 컵마다 상자, 클래스(`upright`/`inverted`/`lying`), 키포인트 `rim`, `bottom` 을 답니다.
   - 위에서 봐서 가려지는 키포인트(예: 뒤집힌 컵의 `rim`)는 visibility 0 으로 표시합니다.
   - "Ultralytics YOLO Pose" 형식으로 내보내 `~/cafe_vision_dataset/images/{train,val}`, `labels/{train,val}` 로 둡니다 (val 은 약 20%).
3. **학습** (RTX 3060 6GB 기준 yolov8n-pose 권장)

   ```bash
   yolo pose train data=cobot1_ws/src/cafe_vision/training/cup.yaml model=yolov8n-pose.pt imgsz=640 epochs=100
   cp runs/pose/train/weights/best.pt cobot1_ws/src/cafe_vision/models/cup_pose.pt
   ```
   가중치 파일(`*.pt`)은 git 에 올리지 않습니다.
4. **탐지 확인** (로봇을 움직이지 않음)

   ```bash
   ros2 run cafe_vision vision_node --ros-args \
       -p model_path:=$PWD/cobot1_ws/src/cafe_vision/models/cup_pose.pt \
       -p extrinsics_file:=$PWD/config/robot/real/camera_extrinsics.yaml
   ros2 service call /vision/detect_cup cafe_interfaces/srv/DetectCup
   ```
   출력된 `position` 을 자로 잰 실제 컵 위치와 비교합니다.
