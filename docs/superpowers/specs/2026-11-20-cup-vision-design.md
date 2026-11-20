# 컵 비전 픽킹 설계

작성일: 2026-11-20 · 브랜치: `feat/cup-vision` (PR #4 `feat/start-real-script` 위에서 시작)

## 1. 목적과 배경

이 프로젝트는 강사님의 **강의 예시 프로젝트**입니다. 지금은 컵이 항상 `cup_pick` 고정 좌표에 있다고 가정하고 움직이지만, 실제로는 컵 위치가 매번 달라집니다. 손목의 RealSense 카메라로 컵을 찾아서 집도록 바꿉니다.

강의용이므로 "잘 동작하는 것"만큼 **학생이 구조를 읽고 따라 할 수 있는 것**이 중요합니다. 객체 탐지의 전 과정(데이터 수집 → 라벨링 → 학습 → 로봇 연동)이 코드와 문서에 드러나야 합니다.

## 2. 목표 / 비목표 / 안티골

**목표**
- 세 가지 컵 상태를 모두 집는다.

  | 상태 | 상황 | 비전이 알아낼 것 |
  |---|---|---|
  | 뒤집힘 (입구가 아래, 기본) | 쌓아 둔 컵 더미의 맨 위 컵 | 위치 x, y, 맨 위 높이 z |
  | 누움 | 컵이 쓰러짐 | 위치, 누운 방향 각도, 입구 쪽 방향 |
  | 바로 섬 (한 개) | 정상 자세 | 위치, 높이 |
- 집은 컵은 손 안에서 한 번 방향을 돌려 **`cup_station`에 바로 세워 내려놓는다**(사용자 결정, 방식 B). 그 다음 공정(커피머신 → 서빙)은 지금 코드를 그대로 쓴다.
- 카메라 없는 시뮬레이션에서도 전체 주문 흐름이 돈다(가짜 탐지 결과 사용).
- 일시정지, 재개, 강제 정지가 비전 단계에서도 지금과 똑같이 동작한다.

**비목표**
- 움직이는 컵 추적이나 비주얼 서보잉
- 여러 잔 동시 처리
- MoveIt 경로 계획
- 컵 외 다른 물체 인식
- 로봇이 스스로 자세를 바꿔 가며 찍는 완전 자동 캘리브레이션 (자세는 사람이 옮긴다)

**안티골** (동작은 해도 이렇게 되면 실패)
- 잘못된 탐지 때문에 로봇이 작업 영역 밖이나 테이블 아래로 이동하는 것. 탐지 좌표는 이동 전에 반드시 작업 영역 검사를 거친다.
- 컵을 못 잡았는데 커피를 내리는 것.
- YOLO나 카메라 코드가 비전 노드 밖으로 새어 나가는 것. 지금 DSR API를 `doosan_adapter.py`에만 두는 원칙과 같다.

## 3. 제약

- **카메라:** RealSense가 `link_6`(손목)에 달린 eye-in-hand 구조이고, `bringup_camera.launch.py`가 드라이버를 띄운다. URDF의 카메라 장착값은 `xyz="0 0 0"`으로 **실측값이 아니다.** → §5.6 체커보드 핸드-아이 캘리브레이션으로 구한다
- **학습 장비:** RTX 3060 Laptop 6GB. YOLOv8n/s-pose 학습이 가능하다.
- **환경:** ROS 2 Jazzy, Python 노드. `ultralytics`는 새 의존성이다. 시스템 Python(ROS)에 설치해야 하므로 설치 방법을 README에 적는다.
- **그리퍼:** RG2 최대 개폐 폭은 110mm이다. 종이컵 입구는 약 70–80mm이므로 옆면을 잡을 수 있다.
- **이동:** 모든 이동은 기존 `robot_control`을 거친다. 일시정지와 정지 처리가 그곳에 있다.

## 4. 방식 검토

| 방식 | 내용 | 장단점 |
|---|---|---|
| **1. YOLO 키포인트 직접 학습 (선택)** | 클래스 3종(upright / inverted / lying), 키포인트 2개(입구 중심, 바닥 중심). z는 깊이값 | 모델 하나로 위치·자세·방향·입구 쪽이 모두 나온다. 강의에서 학습 전 과정을 보여줄 수 있다. 대신 사진 수백 장과 라벨링이 필요하다 |
| 2. 기존 YOLO "cup" + 깊이 규칙 | 학습 없이 깊이 모양으로 자세를 판단 | 바로 시작할 수 있다. 하지만 뒤집힌·누운 종이컵 인식이 보장되지 않고, 규칙이 많아 환경에 민감하다 |
| 3. 회전 상자(OBB) 학습 + 깊이 규칙 | 각도는 OBB, 입구 쪽은 깊이로 판단 | 라벨링은 쉽지만 규칙이 하나 더 붙는다 |

사용자가 1번을 선택했다. "객체 탐지로 좌표 계산"이라는 요구에 가장 직접적으로 맞고, 결과가 모델 하나에서 나와 구조가 단순하다.

## 5. 설계

### 5.1 전체 흐름

레시피 맨 앞에 단계 하나를 **추가**한다. 나머지는 바꾸지 않는다. `cup_pick` 포즈는 `cup_station`과 같은 자리로 다시 티칭한다.

```yaml
steps:
  - name: Find Cup          # 신규: 찾기 → 집기 → 바로 세우기 → cup_station에 놓기
    action: vision_pick
  - name: Cup Pick          # 기존 그대로 (cup_pick = cup_station 자리)
    action: move
    pose: cup_pick
  ...
```

```
coffee_process ──vision_pick──▶ CupPicker
   CupPicker ── /robot_control/move_named_pose (cup_look, cup_station)
             ── /vision/detect_cup            ──▶ cafe_vision/vision_node ◀── RealSense (color + aligned depth)
             ── /robot_control/move_cartesian  (신규, 작업 영역 검사 포함)
             ── /onrobot/sendCommand           (기존 그리퍼)
```

### 5.2 `cafe_vision` 패키지 (신규)

- **위치와 분리 원칙:** `cobot1_ws/src/cafe_vision`에 독립 ROS 패키지(ament_python)로 만든다. 비전 관련 코드는 모두 이 패키지에 둔다: 탐지 노드, 학습 도구(§5.5), 캘리브레이션 도구(§5.6).
  - YOLO(`ultralytics`), OpenCV, RealSense 의존성은 이 패키지에만 둔다.
  - 다른 패키지는 `cafe_interfaces`의 `DetectCup.srv` 서비스로만 비전을 사용하고, `cafe_vision`을 import하지 않는다.
  - 따라서 비전을 쓰지 않는 레시피는 이 패키지가 없어도 동작한다.
  - `scripts/build.sh`의 `colcon build`에 자동으로 포함된다.
- **`vision_node`:** RealSense의 컬러 영상과 정렬된 깊이 영상을 구독한다. `/vision/detect_cup` 서비스를 제공한다.
- **`DetectCup.srv` (cafe_interfaces에 추가)**

  ```
  ---
  bool found
  string state            # upright | inverted | lying
  float64[3] position     # 로봇 base 기준 mm, 집을 지점
  float64 yaw             # 그리퍼 회전각 deg (lying일 때 컵 축 방향)
  float64 rim_direction   # lying일 때 입구 쪽 방향 deg (base 기준)
  float64 confidence
  string message
  ```
- **처리 순서**
  1. YOLO-pose로 추론한다.
  2. 신뢰도 기준값(파라미터) 이상인 것 중 가장 높은 하나를 고른다.
  3. 키포인트 픽셀과 깊이로 카메라 좌표를 구한다.
  4. 카메라→`link_6` 변환과 `base`→`link_6`(TF)를 곱해 base 좌표로 바꾼다.
- **상태별 집는 점**

  | 상태 | 집는 점 |
  |---|---|
  | inverted | 윗면(컵 바닥) 중심. z는 윗면 깊이 |
  | upright | 윗면(입구) 중심. z는 윗면 깊이, 옆면 높이는 grasp_offset 으로 |
  | lying | 두 키포인트의 중점. yaw는 두 점을 이은 방향, rim_direction은 바닥→입구 방향 |
- **파라미터:** `model_path`, `confidence_threshold`, `camera_extrinsics_file`(§5.6에서 만든 파일), `mock_file`
- **mock 모드:** `mock_file`(yaml)을 주면 카메라와 모델 없이 그 값을 돌려준다. 시뮬레이션과 테스트용이다.
- **좌표 변환 코드:** 픽셀→3D 변환과 상태별 집는 점 계산은 ROS와 무관한 순수 함수(`geometry.py`)로 분리해 단위 테스트한다.

### 5.3 `robot_control`: 좌표 이동 서비스 (신규)

- **서비스:** `/robot_control/move_cartesian`, `MoveCartesian.srv`

  ```
  float64[6] pose        # x y z rx ry rz (base, mm/deg)
  float64 velocity
  float64 acceleration
  ---
  bool success
  string message
  ```
- **이동 방식:** 기존 `MotionManager` → `DriverClient.move_line`을 쓴다. 그래서 일시정지와 정지 처리도 기존 이름 포즈 이동과 똑같이 적용된다.
- **작업 영역 검사:** 이동 전에 검사하고, 범위 밖이면 이동하지 않고 실패를 돌려준다. 범위 값(x, y, z 최소·최대)은 `config/robot/<mode>/poses.yaml`의 `workspace:`에 둔다. `check_robot_config.py`도 이 키를 검사하도록 넓힌다.

### 5.4 `coffee_process`: `vision_pick` 단계

- `ProcessExecutor`에 `vision_pick` 액션을 추가한다. 실제 동작은 새 파일 `cup_picker.py`의 `CupPicker`가 맡는다.
- **`CupPicker` 순서**
  1. `cup_look` 포즈로 이동해 `detect_cup`을 호출한다. 실패하면 최대 3번까지 다시 시도한다(이 값은 빌더가 정함).
  2. 끝내 못 찾으면 예외를 던진다. 그러면 기존 흐름대로 주문은 FAILED가 되고, 메시지는 "컵을 찾지 못했습니다"이다.
  3. 상태별로 집는다: 위 접근점(z + 여유) → 하강 → 그리퍼 닫기 → 상승.

     | 상태 | 접근 자세 | 바로 세우기 |
     |---|---|---|
     | upright | 툴 수평, 옆에서 접근 (inverted와 같음) | 없음 (180° 회전 안 함) |
     | inverted | 툴 수평, 옆에서 접근 | J6 180° 회전 |
     | lying | 툴 수직, yaw를 컵 축에 맞춤 | rim_direction에 따라 손목 ±90° |
  4. `cup_station` 위로 이동 → 하강 → 그리퍼 열기 → 상승.
- **포즈 추가:** `cup_look`(탐지용 관찰 자세)과 `cup_station`을 `config/robot/{virtual,real}/poses.yaml`에 추가한다. 실제 로봇용 값은 0으로 두어, 티칭하기 전에는 `check_robot_config.py`가 실행을 거부하게 한다.
- **상태별 이동 순서는 데이터로 둔다.** 접근 높이와 툴 자세 같은 값은 코드에 하드코딩하지 않고 `config/robot/<mode>/cup_grasp.yaml`에 둔다. 학생은 이 파일 하나만 보면 상태별 동작을 이해할 수 있다.
- **진행률:** `vision_pick`은 진행률에서 한 단계로 센다. 세부 상황은 상태 메시지로 보낸다(예: "컵 탐지: inverted").

### 5.5 학습 파이프라인 (`cafe_vision/training/`)

| 파일 | 역할 |
|---|---|
| `capture.py` | RealSense 컬러 영상을 키 입력마다 저장한다. 실제 `cup_look` 자세에서 찍는다 |
| `README.md` | 촬영 가이드(상태별 장수, 조명, 배경, 쌓인 높이), 라벨링 방법, 학습·검증 명령 |
| `cup.yaml` | 데이터셋 정의. 클래스 3개, 키포인트 2개(`rim`, `bottom`) |

- **라벨링 규칙:** upright와 inverted에서는 위에서 봤을 때 가려지는 키포인트를 "보이지 않음"으로 표시한다.
- **가중치 파일:** `cafe_vision/models/`에 둔다. 저장소에 올릴지는 크기를 보고 빌더가 정한다.

### 5.6 핸드-아이 캘리브레이션 (`cafe_vision/calibration/`)

체커보드로 **카메라가 손목(`link_6`)에 대해 어디에 어떻게 달려 있는지**를 구한다. 이 값을 변환 행렬 X라 부른다. 카메라가 손목에 달린 eye-in-hand 방식이다.

- **준비물:** 체커보드를 인쇄해 테이블에 고정한다. 캘리브레이션하는 동안 보드가 움직이면 안 된다. 내부 코너 수와 칸 크기(mm)는 `cafe_vision/calibration/board.yaml`에 둔다.
- **카메라 내부 파라미터:** RealSense 공장 보정값(`camera_info`)을 쓴다. 내부 보정은 따로 하지 않는다.
- **절차:** `hand_eye_calibrate` 노드가 터미널에서 대화형으로 진행한다. **이 노드는 로봇을 움직이지 않는다.**
  1. 사람이 티치펜던트나 직접 교시로 로봇을 옮겨, 체커보드 전체가 화면에 보이게 한다. 로봇이 멈춘 상태여야 한다.
  2. Enter를 누르면 샘플 하나를 기록한다.
     - 컬러 영상에서 코너를 찾고(`findChessboardCorners` + `cornerSubPix`), `solvePnP`로 보드→카메라 변환을 구한다.
     - 같은 순간의 TF `base`→`link_6`을 함께 기록한다.
     - 코너를 못 찾으면 그 샘플은 버리고 다시 찍으라고 안내한다.
  3. 15장 이상 찍는다. 기울기(±30° 안팎)와 회전을 골고루 섞어야 한다. 촬영 가이드는 README에 쓴다.
  4. `q`를 누르면 `cv2.calibrateHandEye`(기본 PARK 방식)로 X를 계산한다.
  5. **검증:** 샘플마다 보드의 base 좌표(`base→link_6 · X · 카메라→보드`)를 계산하고, 그 흩어진 정도(위치 mm, 회전 deg)를 출력한다. 위치 편차가 기준(빌더가 정함, 약 5mm)을 넘으면 경고한다.
  6. **저장**
     - 결과는 `config/robot/real/camera_extrinsics.yaml`에 저장한다. 위치 xyz(mm), 쿼터니언, 날짜, 샘플 수, 검증 편차가 들어간다.
     - 원본 샘플(영상, TF)은 `log/calibration/<날짜>/`에 남겨 나중에 다시 계산할 수 있게 한다.
- **`base`→`link_6`을 TF에서 얻는 이유:** 실행 중 비전 노드도 TF를 쓴다. 캘리브레이션과 실제 사용이 같은 좌표 기준을 써야 한다. DSR의 `get_current_pose()`는 TCP 기준이라 툴 설정에 따라 값이 달라질 수 있다.
- **코드 구조:** 샘플에서 X를 구하는 계산과 검증 편차 계산은 순수 함수(`hand_eye.py`)로 분리한다.
- **가상 모드:** `config/robot/virtual/camera_extrinsics.yaml`에 예시 값을 둔다. 가상 모드는 mock 비전이라 실제로 쓰이지는 않는다.

### 5.7 실행 스크립트

- **`start_sim.sh`:** `vision_node`를 mock 모드로 추가로 띄운다.
- **`start_real.sh`(실제 모드):**
  - bringup을 `bringup_camera.launch.py`로 바꾸고, `vision_node`를 학습된 모델로 띄운다.
  - 실행 전 설정 검사에 모델 파일 존재, `camera_extrinsics.yaml` 존재(캘리브레이션 여부), `workspace`를 추가한다.

### 5.8 시험

- **단위 테스트**
  - 픽셀→base 좌표 변환
  - 핸드-아이 계산: 정답 X를 정해 합성 샘플을 만들고, 계산 결과가 X와 같은지와 검증 편차 계산을 확인
  - 상태별 집는 점과 각도 계산
  - 작업 영역 검사
  - mock 비전으로 `CupPicker`의 상태별 이동 순서
  - 못 찾았을 때 FAILED 처리
- **시뮬레이션 E2E:** 상태 3종 mock 파일 각각으로 주문이 DONE이 되는지 확인한다. 일시정지와 강제 정지도 `vision_pick` 도중에 확인한다.
- **실제 로봇 (단계적으로)**
  0. 캘리브레이션 확인: 비전으로 체커보드 코너 한 점의 base 좌표를 구하고, 로봇을 그 점 50mm 위로 `move_cartesian`해 눈으로 확인한다.
  1. 로봇을 움직이지 않고 탐지 좌표만 출력해 실제 위치와 비교한다.
  2. 컵 위 높은 곳까지만 이동한다.
  3. 상태별로 집는다.
  4. 전체 주문을 진행한다.

## 6. 미결 (사용자 확인 필요)

없음. 카메라 장착값은 체커보드 핸드-아이 캘리브레이션을 이번 범위에 넣기로 했다(§5.6).

## 7. 결정 내역

**사용자가 정한 것**
- 컵 상태 세 가지 처리
- 내려놓고 다시 집는 방식 B
- 방식 1(YOLO 키포인트 직접 학습)
- 비전 노드는 별도 패키지 `cafe_vision`으로 분리한다
- 카메라 장착값은 체커보드 핸드-아이 캘리브레이션으로 구한다

**설계자 판단** (이견이 있으면 수정)
- 레시피에 `vision_pick` 한 단계를 추가하고, `cup_pick` = `cup_station`으로 둔다. 세 레시피(americano / cappuccino / latte) 모두에 넣는다.
- 컵이 여러 개 보이면 신뢰도가 가장 높은 것 하나를 고른다.
- 못 찾으면 3번 다시 시도한 뒤 FAILED로 처리한다.
- `cup_station`은 받침대 없이 테이블 위의 티칭 위치로 둔다.
- 작업 영역 검사는 `robot_control`에서 한다. 비전 결과뿐 아니라 모든 좌표 이동을 보호하기 위해서다.
- 시뮬레이션은 mock 비전으로 돌린다.
- 상태별 접근값은 yaml로 둔다.
- 캘리브레이션 중 로봇 이동은 사람이 한다(노드는 로봇을 움직이지 않음). 계산은 OpenCV `calibrateHandEye`(PARK)를 쓰고, `base`→`link_6`은 TF에서 얻는다. 카메라 내부 파라미터는 RealSense 공장값을 쓴다.

**빌더에게 맡기는 것**
- YOLO 모델 크기(n/s)
- 재시도 횟수
- 신뢰도 기준값
- 가중치 파일을 저장소에 올릴지
- `bringup_camera.launch.py`의 준비 완료 로그 확인
- 체커보드 크기, 최소 샘플 수, 캘리브레이션 검증 기준값
- 직접 교시 중에 ROS 드라이버 연결이 유지되는지 실물 확인
- 쌓인 컵에서 **한 장만** 빠지는지 실물 검증. 겹친 컵이 같이 딸려 올라오면 들어 올린 뒤 살짝 흔드는 동작 등을 추가
- `cup_station`에서 다시 탐지해 컵이 놓였는지 확인하는 기능(선택)
