# 컵 비전 픽킹 작업 정리

- 브랜치: `feat/cup-vision` (PR #4 `feat/start-real-script` 위에서 시작, fork 레포에 푸시됨)
- 설계 문서: `docs/superpowers/specs/2026-11-20-cup-vision-design.md`
- 구현 계획: `docs/superpowers/plans/2026-11-20-cup-vision.md`
- 상태: pytest 112개 통과, 시뮬레이터에서 컵 상태 3종 주문·일시정지·강제 정지 확인 (탐지는 mock). YOLO 모델(`cup_pose.pt`)은 아직 없음. **실제 로봇·카메라에서는 아직 실행하지 않음.**

---

## 1. 무엇을 만들었나

컵이 매번 다른 자리, 다른 자세로 놓여도 손목 RealSense 카메라로 찾아서 집습니다.

| 컵 상태 | 상황 | 집는 법 | 바로 세우는 법 |
|---|---|---|---|
| `upright` (바로 섬) | 정상 자세 컵 1개 | 툴을 수평으로 옆면 (inverted와 같음) | 필요 없음 (회전 안 함) |
| `inverted` (뒤집힘, 기본) | 쌓아 둔 컵 더미의 맨 위 | 툴을 수평으로 | 손목(J6) 180° 회전 |
| `lying` (누움) | 쓰러진 컵 | 컵 축 방향(yaw)에 맞춰 위에서 | 입구 방향에 따라 손목 ±90° |

집은 컵은 바로 세워 `cup_station`에 내려놓습니다. 그다음은 기존 레시피(Cup Pick → 커피머신 → 서빙)를 **그대로** 진행합니다.
그래서 레시피 맨 앞에 `Find Cup`(`vision_pick`) 단계 하나만 추가했습니다.

---

## 2. 전체 통신 구조

```
[웹 키오스크] ──HTTP/Socket.IO──▶ [backend (Flask)]
                                      │ rosbridge (ws://localhost:9090)
                                      ▼
                       /coffee_process/execute_recipe (srv)
                       /coffee_process/{pause,resume,force_stop,start} (srv)
                       /coffee_process/status (topic) ◀──────────────┐
                                      │                                │
                                      ▼                                │
                            ┌──────────────────────┐                   │
                            │ coffee_process        │───────────────────┘
                            │  ProcessRunner        │
                            │   └ ProcessExecutor   │
                            │       └ CupPicker (신규)│
                            └──┬──────────┬────────┬┘
       /robot_control/          │          │        │ /vision/detect_cup (srv, 신규)
       move_named_pose (srv)    │          │        ▼
       move_cartesian (srv,신규)│          │   ┌───────────────────────┐
       pause/resume/stop/...    │          │   │ cafe_vision/vision_node│ (신규 패키지)
                                ▼          │   │  mock 모드 또는 YOLO    │
                       ┌───────────────┐   │   └───┬───────────────────┘
                       │ robot_control  │   │       │ 구독: /camera/color/image_raw/compressed
                       │  작업 영역 검사 │   │       │       /camera/aligned_depth_to_color/image_raw/compressedDepth
                       └──────┬────────┘   │       │       /camera/color/camera_info
         /robot_driver/       │            │       │ TF:   base_link → link_6
         move_line (action)   │            │       ▼
         move_joint (action)  │            │   [RealSense 손목 카메라]
         pause/resume/stop    ▼            │
                       ┌───────────────┐   │ /onrobot/sendCommand (srv, 그리퍼)
                       │ robot_driver   │◀──┘ /robot_driver/set_digital_output (srv, DO)
                       │  DoosanAdapter │
                       └──────┬────────┘
                              ▼
                       Doosan M0609 (DSR_ROBOT2)
```

### 2.1 이번에 새로 생긴 통신

| 이름 | 종류 | 제공 | 호출 | 내용 |
|---|---|---|---|---|
| `/vision/detect_cup` | 서비스 `cafe_interfaces/srv/DetectCup` | `cafe_vision/vision_node` | `coffee_process` (`VisionClient`) | 요청 없음 → 응답 `found, state, position[3], yaw, rim_direction, confidence, message` |
| `/robot_control/move_cartesian` | 서비스 `cafe_interfaces/srv/MoveCartesian` | `robot_control` | `coffee_process` (`RobotClient`) | `pose[6], velocity, acceleration, check_only` → `success, message`. `check_only=true`면 작업 영역 검사만 함 |
| `/camera/color/image_raw/compressed` | 토픽 `sensor_msgs/CompressedImage` | RealSense 드라이버 (image_transport) | `vision_node`, `calibrate_node`, `capture_node` | 컬러 영상 (jpeg). 네트워크 부담 때문에 압축 토픽 사용 |
| `/camera/aligned_depth_to_color/image_raw/compressedDepth` | 토픽 `sensor_msgs/CompressedImage` | RealSense 드라이버 (image_transport) | `vision_node` | 컬러에 정렬된 깊이 영상 (16UC1 mm, 무손실 png. jpeg는 깊이를 깨뜨려 사용 안 함) |
| `/camera/color/camera_info` | 토픽 `sensor_msgs/CameraInfo` | RealSense 드라이버 | `vision_node`, `calibrate_node` | 카메라 내부 파라미터 (fx, fy, cx, cy, 왜곡) |
| `/tf` (`base_link`→`link_6`) | TF | robot_state_publisher | `vision_node`, `calibrate_node` | 지금 손목 위치 (m 단위 → 코드에서 mm로 변환) |

### 2.2 그대로 쓰는 기존 통신

| 이름 | 종류 | 용도 |
|---|---|---|
| `/robot_control/move_named_pose` | 서비스 | `cup_look`(관찰 자세), `trash`, `home` 등 저장된 이름의 자세로 이동 |
| `/robot_control/{pause,resume,stop_process,clear_stop}` | 서비스 | 일시정지·재개·강제 정지 |
| `/robot_driver/move_line`, `move_joint`, `move_home` | 액션 | robot_control → robot_driver 실제 이동 |
| `/onrobot/sendCommand` | 서비스 (`onrobot_rg_msgs/SetCommand`) | 그리퍼 열기(`o`)/닫기(`c`) |
| `/robot_driver/set_digital_output` | 서비스 | 커피머신 등 DO |
| `/coffee_process/execute_recipe`, `/coffee_process/status` | 서비스, 토픽 | 백엔드가 rosbridge로 주문을 보내고 진행 상태를 받음 |

### 2.3 단위와 좌표 약속

- 위치는 모두 **mm**, 각도는 **deg**.
- 로봇 좌표는 Doosan posx `[x, y, z, A, B, C]`. 자세는 ZYZ 오일러 각입니다. 툴이 아래를 향할 때(B=180) base에서 본 그리퍼 방향은 A−C이므로, **누운 컵의 yaw는 A에 더합니다.**
- 카메라 외부 파라미터 X = `link_6` → `camera_color_optical_frame` (4×4). 카메라 좌표 → 로봇 좌표: `T_base_cam = T_base_link6(TF) @ X`.
- 모든 좌표는 **활성 TCP = 그리퍼 손가락 끝 중심**을 전제로 합니다.

---

## 3. 주문 1건이 처리되는 순서 (Find Cup 단계)

1. **백엔드 → coffee_process:** `/coffee_process/execute_recipe`로 주문이 오면, `ProcessRunner`가 레시피 단계를 순서대로 실행합니다.
2. **첫 단계 `vision_pick`:** `ProcessExecutor`가 `CupPicker.run(checkpoint, report)`를 부릅니다.
3. **관찰 자세로 이동:** `CupPicker`가 `move_named_pose("cup_look")`으로 이동합니다.
4. **컵 탐지:** `/vision/detect_cup`을 최대 3번 호출합니다. 매번 `checkpoint()`로 일시정지·강제 정지를 확인합니다.
   - `vision_node`는 이동이 끝난 뒤 들어온 **새 영상**을 기다립니다(최대 1초).
   - YOLO 추론 → 신뢰도가 가장 높은 후보 하나 → 깊이로 3D 좌표 → TF와 X로 base 좌표 → 상태별 집는 점·yaw·입구 방향을 계산합니다.
   - 끝내 못 찾으면 주문은 FAILED가 됩니다("컵을 찾지 못했습니다: …").
5. **상태 보고:** `report("컵 탐지: <state>")`로 상태 메시지를 갱신합니다. 이 메시지는 웹 화면까지 전달됩니다.
6. **동작 계획:** `plan_pick`이 10단계 동작을 만듭니다.
   열기 → 컵 위 → 내려가기 → 닫기 → 들기 → 손목 돌려 세우기 → 스테이션 위 → 내려놓기 → 열기 → 빠지기
7. **사전 검사:** 실제로 움직이기 **전에** 이동 좌표 7개를 모두 `move_cartesian(check_only=true)`로 작업 영역과 대조합니다. 하나라도 밖이면 로봇은 움직이지 않고 FAILED가 됩니다.
8. **실행:** 각 동작 앞에서 `checkpoint()`를 부른 뒤 그리퍼(`/onrobot/sendCommand`) 또는 `move_cartesian`을 실행합니다.
9. **기존 공정:** 다음 단계부터는 기존 레시피(`Cup Pick`은 `cup_station`과 같은 자리)를 그대로 진행합니다.

**일시정지·강제 정지**
- `vision_pick` 도중 일시정지하면 로봇도 멈추고, 재개 전에는 다음 동작이 나가지 않습니다.
- 강제 정지하면 주문이 FAILED가 됩니다. 컵을 들고 있을 수 있으므로, 다음 초기화 때 `trash`로 가서 그리퍼를 연 뒤 `home`으로 갑니다.

---

## 4. 파일별 역할과 함수

### 4.1 `cobot1_ws/src/cafe_vision/` (신규 패키지)

비전 관련 코드는 모두 이 패키지에 있습니다. 다른 패키지는 `/vision/detect_cup` 서비스로만 사용하고, 이 패키지를 import하지 않습니다.
YOLO(ultralytics)는 `detector.py`에서만 쓰므로 mock 모드와 테스트는 ultralytics 없이 돕니다.

#### `cafe_vision/geometry.py` — 좌표 계산 (ROS 없이 테스트 가능한 순수 함수)

| 함수 | 기능 |
|---|---|
| `deproject(u, v, depth_mm, fx, fy, cx, cy)` | 픽셀 (u, v)와 깊이(mm)를 카메라 광학 좌표 `[x, y, z]` mm로 바꿉니다 |
| `top_depth(depth_mm, box, percentile=5.0)` | 상자 안의 0이 아닌 깊이 중 가장 가까운 값(컵 맨 위). 유효 픽셀이 10개 미만이면 `None`(반사, 너무 가까움) |
| `tf_to_matrix(translation_m, quat_xyzw)` | TF(m, 쿼터니언)를 4×4 행렬(mm)로 바꿉니다 |
| `transform_point(T, p)` | 4×4 행렬로 점 하나를 변환합니다 |
| `pick_target(state, center, rim, bottom)` | base 좌표의 상자 중심·입구·바닥 → (집는 점, 그리퍼 yaw, 입구 방향). upright/inverted는 상자 중심. lying은 두 키포인트의 중점, 바닥→입구 방향(`rim_direction`, [-180, 180)), yaw는 그 값을 [-90, 90)로 접은 값 |
| `choose_best(candidates, threshold)` | 신뢰도가 기준 이상인 후보 중 가장 높은 것. 없으면 `None` |

#### `cafe_vision/detection.py` — 탐지 후보 → 서비스 응답 (순수 함수)

| 함수 | 기능 |
|---|---|
| `not_found(message)` | `found=False` 응답 dict |
| `build_response(candidate, depth_mm, K, T_base_cam)` | 후보 하나를 base 기준 응답으로 바꿉니다. 후보가 없거나, 누운 컵 키포인트가 (0,0)(불확실)이거나, 깊이가 없으면 `not_found`. 상자 중심·입구·바닥 세 픽셀을 **컵 맨 위 깊이**로 역투영합니다. 키포인트 픽셀의 깊이는 컵 안쪽을 볼 수 있기 때문입니다 |
| `decode_compressed_depth(fmt, data)` | compressedDepth 메시지 → uint16 깊이(mm). 앞 12바이트 헤더(ConfigHeader)를 건너뛰고 png 디코딩. 16UC1 png가 아니면 `ValueError` |
| `load_mock(path)` | mock 응답 yaml을 읽습니다 (시뮬레이션) |

#### `cafe_vision/detector.py` — YOLO 추론

| 이름 | 기능 |
|---|---|
| `STATES = ("upright", "inverted", "lying")` | 클래스 번호 순서 (학습 데이터셋 `cup.yaml`과 같아야 함) |
| `YoloCupDetector(model_path)` | 생성자 안에서만 `ultralytics`를 import해 모델을 불러옵니다 |
| `YoloCupDetector.detect(color_bgr)` | 컬러 영상 → 후보 목록 `[{state, conf, box, rim_px, bottom_px}]` (키포인트 0 = 입구, 1 = 바닥) |

#### `cafe_vision/vision_node.py` — `/vision/detect_cup` 서비스 노드

| 이름 | 기능 |
|---|---|
| `VisionNode.__init__` | 파라미터를 읽고 서비스를 엽니다. `mock_file`이 있으면 mock 모드(카메라·모델·TF 없음), 없으면 `_setup_camera` |
| `VisionNode._setup_camera` | `model_path`, `extrinsics_file`이 없으면 시작을 거부합니다. YOLO, X, cv_bridge, TF 리스너, 카메라 토픽 3개 구독 |
| `cb_color` / `cb_depth` / `cb_info` | 최신 컬러·깊이 영상과 카메라 내부 파라미터를 저장합니다 |
| `cb_detect` | 서비스 콜백. mock 값 또는 `_detect()` 결과를 응답에 채웁니다 |
| `_detect` | 요청 이후의 새 컬러 영상을 최대 1초 기다림 → TF `base_link→link_6` 조회 → `T_base_cam` → YOLO → `choose_best` → `build_response` |
| `main` | `MultiThreadedExecutor`로 실행 (서비스 처리 중에도 영상 구독이 갱신되도록) |

파라미터: `mock_file`, `model_path`, `extrinsics_file`, `confidence_threshold`(0.5), `color_topic`, `depth_topic`, `info_topic`, `base_frame`(`base_link`), `flange_frame`(`link_6`)

#### `cafe_vision/hand_eye.py` — 핸드-아이 캘리브레이션 계산 (순수 함수)

| 함수 | 기능 |
|---|---|
| `find_board_pose(gray, cols, rows, square_mm, K, dist)` | 체커보드 코너 검출(`findChessboardCorners` + `cornerSubPix`) → `solvePnP` → `T_cam_board`(mm). 못 찾으면 `None` |
| `solve_hand_eye(T_base_flange, T_cam_board)` | 샘플 쌍들로 X(`link_6`→카메라)를 구합니다 (`cv2.calibrateHandEye`, PARK). 3개 미만이면 오류 |
| `board_spread(T_base_flange, T_cam_board, X)` | 샘플마다 계산한 보드의 base 위치가 얼마나 흩어지는지 (위치 mm, 회전 deg). X가 맞으면 0에 가깝습니다 |
| `save_extrinsics(path, X, samples, spread_mm, spread_deg)` | X를 yaml(위치 mm, 쿼터니언, 날짜, 샘플 수, 편차)로 저장 |
| `load_extrinsics(path)` | yaml → 4×4 X |

#### `cafe_vision/calibrate_node.py` — 대화형 캘리브레이션 노드 (로봇을 움직이지 않음)

| 이름 | 기능 |
|---|---|
| `CalibrateNode.__init__` | 파라미터(`output_file` 필수, `board_file`, `sample_dir`, `min_samples`=15, `max_spread_mm`=5.0 등), 보드 설정, TF, 카메라 구독 |
| `cb_color` / `cb_info` | 최신 영상, 내부 파라미터·왜곡 저장 |
| `capture()` | Enter 입력 시: 보드 포즈 + TF 손목 위치를 샘플로 기록하고, 영상(png)과 행렬(npz)을 `log/calibration/<날짜>/`에 저장 |
| `finish()` | `q` 입력 시: 샘플 수 확인 → `solve_hand_eye` → `board_spread` 출력(기준 초과면 경고) → `save_extrinsics` |
| `main` | spin은 별도 스레드, 메인 스레드는 터미널 입력. 종료 시 executor를 먼저 정리 |

#### `cafe_vision/capture_node.py` — 학습용 사진 촬영

| 이름 | 기능 |
|---|---|
| `CaptureNode` | 컬러 영상 구독, `out_dir`(기본 `~/cafe_vision_dataset/raw`) 준비 |
| `save()` | 현재 영상을 `<시각>.jpg`로 저장 |
| `main` | OpenCV 창 표시. `s` 저장, `q` 종료 |

#### 데이터·설정 파일

| 파일 | 역할 |
|---|---|
| `calibration/board.yaml` | 체커보드 내부 코너 수(`cols` 9, `rows` 6), 칸 크기(`square_mm` 25) |
| `training/cup.yaml` | YOLO-pose 데이터셋 정의. 클래스 3개, 키포인트 2개(`kpt_shape [2,3]`, `flip_idx [0,1]`) |
| `models/` | 학습한 가중치 `cup_pose.pt`를 두는 곳 (`*.pt`는 git 제외) |
| `README.md` | 설치(`numpy<2` 고정), 핸드-아이 캘리브레이션, 촬영·라벨링·학습, 실제 로봇 전 확인 사항(TCP, 빈 그리퍼, 내려놓는 높이) |
| `package.xml`, `setup.py`, `setup.cfg`, `resource/` | ROS 패키지 정의. 실행 파일 `vision_node`, `calibrate_node`, `capture_node` |

### 4.2 `cobot1_ws/src/coffee_process/` (수정)

#### `coffee_process/cup_picker.py` (신규)

| 이름 | 기능 |
|---|---|
| `plan_pick(detection, cfg)` | 탐지 결과 + `cup_grasp.yaml` → 10단계 동작 목록 `("move", posx[6])` / `("gripper", "open"/"close")`. 집는 점 = 탐지 위치 + `grasp_offset`. 놓는 점 = `station_position` + 상태별 `place_offset`. lying이면 접근 자세 A에 yaw를 더하고, 입구 방향으로 운반 자세(`_rim_forward`/`_rim_backward`)를 고릅니다 |
| `_add(a, b)` | 벡터 덧셈 |
| `CupPicker(robot, io, vision, cfg, retries=3)` | 집기 실행기 |
| `CupPicker.run(checkpoint, report)` | `cup_look` 이동 → `_detect` → `report("컵 탐지: …")` → 모든 이동 좌표 사전 검사(`check_cartesian`) → 각 동작 앞에서 `checkpoint()` 후 실행 |
| `CupPicker._detect(checkpoint)` | `detect_cup`을 최대 `retries`번 시도. 비전 노드 예외도 실패 1회로 셉니다. 끝내 실패하면 `RuntimeError("컵을 찾지 못했습니다: …")` |

#### `coffee_process/ros_clients.py` (추가)

| 이름 | 통신 | 기능 |
|---|---|---|
| `RobotClient.check_cartesian(pose)` | `/robot_control/move_cartesian` (`check_only=true`) | 작업 영역 안인지 검사만 (5초 타임아웃) |
| `RobotClient.move_cartesian(pose, velocity, acceleration)` | `/robot_control/move_cartesian` | 좌표로 직선 이동 (일시정지 중에도 기다리도록 타임아웃 없음) |
| `VisionClient.detect_cup()` | `/vision/detect_cup` | 응답을 dict로 (10초 타임아웃) |

#### `coffee_process/process_executor.py` (수정)

- `ProcessExecutor(robot, io, wait, picker=None, checkpoint=None, report=None)`
- `execute_step`에 `vision_pick` 액션을 추가했습니다. `picker.run(checkpoint, report)`를 호출하고, picker가 없으면 "grasp_file 파라미터가 없어 CupPicker가 없습니다" 오류를 냅니다.

#### `coffee_process/process_runner.py` (수정)

- `ProcessRunner(..., picker=None)`: executor에 `checkpoint=self._checkpoint`, `report=self._report`를 넘깁니다.
- `_report(message)`(신규): 상태 메시지를 갱신하고 `/coffee_process/status`로 내보냅니다.
- `pause()`: 현재 단계가 `move` **또는 `vision_pick`**이면 로봇도 멈춥니다.
- `_run_recipe`: `vision_pick` 시작 시 `_holding_cup = True`, 끝나면 `False`. 도중에 강제 정지하면 초기화 때 `trash`로 가서 컵을 버립니다.

#### `coffee_process/coffee_process_node.py` (수정)

- `_make_picker(robot, io, client_group)`(신규): `grasp_file` 파라미터가 있으면 yaml을 읽어 `CupPicker(RobotClient, IOClient, VisionClient, cfg)`를 만듭니다. 없으면 경고 후 `None`.

#### `config/recipes/{americano,cappuccino,latte}.yaml` (수정)

- 맨 앞에 `- name: Find Cup / action: vision_pick`을 추가했습니다. 진행률 단계가 1개 늘었습니다.

### 4.3 `cobot1_ws/src/robot_control/` (수정)

| 파일 | 함수 | 기능 |
|---|---|---|
| `motion_manager.py` | `MotionManager(robot_driver, poses, workspace=None)` | `poses.yaml`의 `workspace`를 받습니다 |
| | `move_cartesian(pose, velocity, acceleration, check_only=False)` | workspace가 없으면 거부. x/y/z가 범위 밖이면 `ValueError("Target outside workspace: …")`를 내고 driver를 부르지 않습니다. `check_only`면 검사만 하고 끝. 통과하면 `robot.move_line` (→ `/robot_driver/move_line` 액션) |
| `robot_controller.py` | `move_cartesian(...)` | `MotionManager`로 그대로 전달 |
| `robot_control_node.py` | `__init__` | yaml 전체를 읽어 `poses`와 `workspace`를 넘기고, `/robot_control/move_cartesian` 서비스를 엽니다 |
| | `cb_move_cartesian(request, response)` | 서비스 콜백. 성공/실패 메시지 |

### 4.4 `cobot1_ws/src/cafe_interfaces/srv/` (신규)

| 파일 | 정의 |
|---|---|
| `MoveCartesian.srv` | `float64[6] pose, float64 velocity, float64 acceleration, bool check_only` → `bool success, string message` |
| `DetectCup.srv` | (요청 없음) → `bool found, string state, float64[3] position, float64 yaw, float64 rim_direction, float64 confidence, string message` |

`CMakeLists.txt`의 `srv_files`에 두 파일을 추가했습니다.

### 4.5 `config/robot/` (설정)

| 파일 | 역할 |
|---|---|
| `{virtual,real}/poses.yaml` | 추가: `workspace`(좌표 이동 허용 범위, mm), `cup_look`(관찰 자세). real은 0으로 두어 티칭 전 실행 거부 |
| `{virtual,real}/cup_grasp.yaml` | 신규: `station_position`, `approach_height`, 속도, 상태별 `grasp_offset`·`place_offset`·`approach_orientation`·`carry_orientation(_rim_forward/_backward)` |
| `virtual/camera_extrinsics.yaml` | 예시 X (가상 모드는 mock이라 실제로 쓰이지 않음). real은 캘리브레이션으로 생성 |
| `virtual/mock/{upright,inverted,lying}.yaml` | 시뮬레이션용 가짜 탐지 결과. `MOCK_DETECTION`으로 선택 |

### 4.6 `scripts/` (수정)

| 파일 | 변경 |
|---|---|
| `start_sim.sh` | 단계 7개. `vision_node`를 추가했습니다(가상 = mock, 기본 inverted / 실제 = YOLO + 외부 파라미터). coffee_process에 `grasp_file`을 넘기고, 실제 모드 또는 `REAL_CAMERA=1`이면 `bringup_camera.launch.py`(가상은 `camera:=true`)로 카메라까지 띄우고, 모델 파일이 없으면 시작 전에 멈춥니다. 중복 실행 검사에 `vision_node`를 추가했습니다 |
| `start_real.sh` | 설정 검사 후 모델 파일(`models/cup_pose.pt`)이 없으면 중단합니다 |
| `check_robot_config.py` | `check(config_dir)`에 추가: workspace 범위(min < max), `camera_extrinsics.yaml` 존재, `station_position`이 0, 그립 자세(`*orientation*`)가 0이면 오류 |
| `check_project.py` | 필수 패키지 목록에 `cafe_vision`을 추가했습니다 |

### 4.7 `tests/` (pytest 111개, ROS 없이 실행)

| 파일 | 검증 내용 |
|---|---|
| `test_motion_cartesian.py` | 작업 영역 안/밖, workspace 없음, `check_only`면 이동 안 함 |
| `test_vision_geometry.py` | 역투영, 맨 위 깊이, m→mm, 회전 변환, lying 축·입구 방향, 후보 선택 |
| `test_hand_eye.py` | 정답 X를 아는 합성 샘플로 계산 복원, 편차 0/증가, 그린 체커보드로 거리 375mm |
| `test_vision_node_logic.py` | 응답 생성(inverted, lying), 후보 없음, 깊이 없음, 키포인트 불확실, mock이 작업 영역 안인지 |
| `test_cup_picker.py` | 10단계 순서, lying yaw를 **실제 그리퍼 방향**으로 검증, 스테이션·`place_offset`, 재시도, 비전 노드 꺼짐, 이동 오류, 사전 검사로 움직이기 전 실패, checkpoint 호출 |
| `test_process_runner.py` (추가) | `vision_pick` 실행, picker 없음 → FAILED, 강제 정지 → trash, 일시정지 시 다음 동작 차단, 상태 메시지 |
| `test_robot_config_check.py` (추가) | 외부 파라미터·workspace 누락, 빈 범위, 미티칭 real 설정, 0인 그립 자세 거부 |
| `test_training_config.py` | 데이터셋 클래스·키포인트 정의가 코드 상수와 같은지 |

---

## 5. 실행 방법

```bash
./scripts/build.sh

# 시뮬레이션 (카메라 없이 가짜 탐지)
./scripts/start_sim.sh                                                     # 기본: 뒤집힌 컵
MOCK_DETECTION=config/robot/virtual/mock/lying.yaml ./scripts/start_sim.sh # 누운 컵
REAL_CAMERA=1 ./scripts/start_sim.sh   # 가상 로봇 + 실제 RealSense + YOLO (탐지 확인용, cup_pose.pt 필요)
curl -X POST localhost:5000/api/control/resume   # 홈 이동 → HOMED
curl -X POST localhost:5000/api/control/start    # IDLE
# 브라우저 http://localhost:5000 에서 주문

# 실제 로봇 (준비가 끝나야 실행됨)
ROBOT_HOST=<로봇 IP> ./scripts/start_real.sh
```

실제 로봇 준비 순서는 `cobot1_ws/src/cafe_vision/README.md`에 있습니다.
1. 핸드-아이 캘리브레이션
2. 사진 촬영·라벨링·학습 → `models/cup_pose.pt`
3. `cup_look`·`cup_station`·작업 영역·그립 자세 티칭
4. TCP를 손가락 끝으로 설정

---

## 6. 남은 위험과 현장 확인 사항

**실물 확인 필요**
- 쌓인 종이컵에서 맨 위 한 장만 빠지는지
- 직접 교시 중 ROS 드라이버 연결이 유지되는지
- 카메라 토픽 이름(`/camera/...`), TF 프레임 이름(`base_link`, `link_6`), `bringup_camera.launch.py`의 준비 완료 로그

**알려진 위험: 빈 그리퍼 감지 없음**
- 컵을 놓쳐도 다음 단계로 진행해, 컵 없이 커피를 내릴 수 있습니다.
- 실물 시험 4단계(전체 주문) 전에 RG2 폭 피드백으로 확인하는 기능을 넣거나, 사람이 지켜보며 진행해야 합니다.

**미룬 사소한 항목**

| 번호 | 내용 |
|---|---|
| M1 | 로봇이 멈춰 있는 구간(탐지 중, 그리퍼 동작 중)에 일시정지를 누르면 컨트롤러가 거부할 수 있음 (실물 확인) |
| M2 | 뒤집힌 컵은 툴 수평 상태로 수직 하강해 집음. 그리퍼 손가락이 수평으로 벌어질 때만 동작 |
| M3 | 내려놓은 뒤 `cup_pick` 관절 이동 중 그리퍼가 컵을 쓸 수 있음. `cup_pick`을 위에서 내려오는 동작으로 티칭 |
| M4 | 실물 시험 0단계(체커보드 코너 50mm 위로 이동) 전용 도구 없음 |
| M5 | 깊이 영상은 최신인지 따로 확인하지 않음 |
| M6 | 탐지 중 예외가 나면 비전 노드가 죽음. mock 위치를 정수로 쓰면 실패 |
| M7 | 컵을 못 찾아 실패해도 초기화 때 trash를 거쳐 감 |
