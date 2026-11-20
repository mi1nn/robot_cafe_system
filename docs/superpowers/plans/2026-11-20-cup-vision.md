# 컵 비전 픽킹 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 손목 RealSense와 직접 학습한 YOLO 키포인트 모델로 종이컵(뒤집힘, 누움, 바로 섬)을 찾아 집는다. 집은 컵은 바로 세워 `cup_station`에 내려놓고, 이후 공정은 기존 레시피대로 진행한다.

**Architecture:** 비전은 새 패키지 `cafe_vision`에 모두 넣는다(탐지 노드, 핸드-아이 캘리브레이션, 학습 도구). 다른 패키지는 `/vision/detect_cup` 서비스로만 비전을 쓴다. `robot_control`에는 작업 영역 검사가 붙은 좌표 이동 서비스를 추가한다. `coffee_process`에는 새 레시피 액션 `vision_pick`을 추가하고, 실제 동작은 `CupPicker`가 맡는다. 순수 계산(좌표 변환, 집기 순서, 핸드-아이)은 ROS와 분리해 pytest로 검증한다.

**Tech Stack:** ROS 2 Jazzy (rclpy, tf2_ros, cv_bridge), OpenCV 4.6, numpy 1.26, scipy 1.11, ultralytics (YOLOv8-pose, 신규), pytest

**Spec:** `docs/superpowers/specs/2026-11-20-cup-vision-design.md`

## Global Constraints

- 위치 단위는 모두 **mm**, 각도는 **deg**다. TF의 translation은 m이므로 `tf_to_matrix`에서 1000을 곱해 mm로 바꾼다. RealSense 정렬 깊이 영상(16UC1)은 mm다.
- 로봇 좌표는 Doosan posx `[x, y, z, rx, ry, rz]`(ZYZ 오일러, deg)를 쓴다.
- 컵 상태 문자열은 정확히 `upright`, `inverted`, `lying`이다. YOLO 클래스 순서도 0 = upright, 1 = inverted, 2 = lying이다.
- 키포인트는 2개, 순서는 0 = `rim`(입구 중심), 1 = `bottom`(바닥 중심)이다.
- **의존성 격리**
  - `ultralytics`, `cv2`, `cv_bridge`, RealSense 관련 import는 `cobot1_ws/src/cafe_vision` 안에만 둔다.
  - `ultralytics`는 실제 탐지기(`detector.py`) 안에서만 import한다. 그래서 mock 모드와 테스트는 ultralytics 없이 돈다.
- ultralytics 설치 명령은 `pip install --break-system-packages --user ultralytics "numpy<2"`로 고정한다. numpy 2가 설치되면 ROS의 cv_bridge가 깨진다.
- 카메라 외부 파라미터 X는 `link_6`→`camera_color_optical_frame` 변환(4x4, mm)이다. 파일은 `config/robot/<mode>/camera_extrinsics.yaml`이다.
- 프레임 이름은 파라미터로 받고, 기본값은 `base_frame="base_link"`, `flange_frame="link_6"`이다.
- 새 실제 로봇용 값(포즈, 스테이션 위치)은 0으로 둔다. 그래야 `check_robot_config.py`가 티칭 전 실행을 거부한다.
- 커밋 메시지 끝에는 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`을 붙인다.

## Review Focus

1. **컵을 든 채 강제 정지(`vision_pick` 도중):** 복구할 때 `trash`로 가서 그리퍼를 열어야 한다. 테스트는 Task 6 `test_force_stop_during_vision_pick_discards_cup`.
2. **`vision_pick` 도중 일시정지:** 로봇이 멈추고, 재개 전에는 다음 동작이 나가지 않아야 한다. 테스트는 Task 6 `test_pause_during_vision_pick_blocks_next_motion`.
3. **상자 안에 유효한 깊이가 없을 때**(반사, 너무 가까움): z=0으로 이동하면 안 되고 "찾지 못함"으로 처리해야 한다. 테스트는 Task 2 `test_top_depth_none_without_valid_pixels`, Task 4 `test_build_response_not_found_without_depth`.
4. **작업 영역 밖 탐지 좌표:** 로봇은 움직이지 않고 주문이 FAILED가 되어야 한다. 테스트는 Task 1 `test_move_cartesian_rejects_outside_workspace`, Task 5 `test_picker_propagates_motion_error`.
5. **비전 노드가 꺼져 있을 때:** 멈춰 있지 말고 명확한 메시지로 FAILED가 되어야 한다. 테스트는 Task 5 `test_picker_fails_after_retries_when_vision_errors`.

---

## File Structure

| 파일 | 책임 |
|---|---|
| `cobot1_ws/src/cafe_interfaces/srv/MoveCartesian.srv`, `DetectCup.srv` (신규) | 서비스 정의 |
| `cobot1_ws/src/robot_control/robot_control/motion_manager.py` | `move_cartesian` + 작업 영역 검사 |
| `cobot1_ws/src/robot_control/robot_control/robot_controller.py`, `robot_control_node.py` | 서비스 연결, `workspace` 로드 |
| `cobot1_ws/src/cafe_vision/` (신규 패키지) | 비전 전부 |
| ├ `cafe_vision/geometry.py` | 픽셀→3D, TF→행렬, 집는 점 계산, 후보 선택 (순수 함수) |
| ├ `cafe_vision/hand_eye.py` | 체커보드 포즈, 핸드-아이 계산, 편차, 외부 파라미터 저장·로드 (순수 함수) |
| ├ `cafe_vision/detector.py` | YOLO 추론 → 후보 목록 (ultralytics는 여기서만 사용) |
| ├ `cafe_vision/vision_node.py` | `/vision/detect_cup`, mock 모드 |
| ├ `cafe_vision/calibrate_node.py` | 대화형 핸드-아이 캘리브레이션 |
| ├ `cafe_vision/capture_node.py` | 학습용 사진 촬영 |
| ├ `calibration/board.yaml`, `training/cup.yaml`, `models/` | 데이터 파일 |
| └ `README.md` | 설치, 촬영, 라벨링, 학습, 캘리브레이션 절차 |
| `cobot1_ws/src/coffee_process/coffee_process/cup_picker.py` (신규) | `plan_pick`(순수), `CupPicker` |
| `cobot1_ws/src/coffee_process/coffee_process/ros_clients.py` | `RobotClient.move_cartesian`, `VisionClient` |
| `process_executor.py`, `process_runner.py`, `coffee_process_node.py` | `vision_pick` 연결 |
| `config/robot/{virtual,real}/` | `poses.yaml`(workspace, cup_look), `cup_grasp.yaml`, `camera_extrinsics.yaml`, `virtual/mock/*.yaml` |
| `scripts/check_robot_config.py`, `start_sim.sh`, `start_real.sh`, `README.md` | 실행과 검사 |

**스펙과 다른 결정 (설계자 판단)**
- `cup_station` 위치는 `poses.yaml`이 아니라 `cup_grasp.yaml`의 `station_position`에 둔다. `CupPicker`가 coffee_process 안에서 직접 읽어야 하기 때문이다. 기존 `cup_pick` 포즈는 같은 자리로 티칭한다(README에 명시).
- 학습 README와 캘리브레이션 README는 패키지 README 하나로 합친다.
- 가중치 파일(`*.pt`)은 커밋하지 않는다(`.gitignore`). README에 둘 경로(`models/cup_pose.pt`)를 적는다.

---

### Task 1: `robot_control` 좌표 이동 + 작업 영역 검사

**Files:**
- Create: `cobot1_ws/src/cafe_interfaces/srv/MoveCartesian.srv`
- Modify: `cobot1_ws/src/cafe_interfaces/CMakeLists.txt` (srv_files)
- Modify: `cobot1_ws/src/robot_control/robot_control/motion_manager.py`, `robot_controller.py`, `robot_control_node.py`
- Modify: `config/robot/virtual/poses.yaml`, `config/robot/real/poses.yaml`
- Test: `tests/test_motion_cartesian.py`

**Interfaces:**
- Produces:
  - `MoveCartesian.srv`: `float64[6] pose`, `float64 velocity`, `float64 acceleration` / `bool success`, `string message`
  - 서비스 `/robot_control/move_cartesian`
  - `MotionManager(robot_driver, poses, workspace=None)`
  - `MotionManager.move_cartesian(pose: list[float], velocity=30.0, acceleration=30.0)`. 범위 밖이면 `ValueError("Target outside workspace: ...")`를 던지고 driver를 호출하지 않는다.
- poses.yaml 최상위에 다음 키를 추가한다(`poses:`와 같은 수준).

  ```yaml
  workspace:   # mm, base 기준
    x: [min, max]
    y: [min, max]
    z: [min, max]
  ```
  - virtual 값: x [200, 700], y [-400, 400], z [50, 600]
  - real 값: 모두 `[0, 0]`
- `poses:`에 `cup_look`(joint)을 추가한다. virtual은 `[0, 0, 90, 0, 90, 0]`, real은 0.

- [ ] **Step 1: Write the failing test** (`FakeDriver`는 `move_line` 호출을 기록한다)

```python
def test_move_cartesian_inside_workspace_calls_move_line():
    m = MotionManager(driver, {}, {"x": [200, 700], "y": [-400, 400], "z": [50, 600]})
    m.move_cartesian([400, 0, 200, 0, 180, 0], 20, 20)
    assert driver.calls == [("move_line", [400, 0, 200, 0, 180, 0], 20, 20)]

def test_move_cartesian_rejects_outside_workspace():
    with pytest.raises(ValueError, match="outside workspace"):
        m.move_cartesian([400, 0, 10, 0, 180, 0])     # z < 50
    assert driver.calls == []

def test_move_cartesian_without_workspace_is_refused():
    with pytest.raises(ValueError, match="workspace"):
        MotionManager(driver, {}).move_cartesian([400, 0, 200, 0, 180, 0])
```

- [ ] **Step 2: Run** `python3 -m pytest tests/test_motion_cartesian.py -v` → FAIL (`move_cartesian` 없음)
- [ ] **Step 3: Implement**
  - `MotionManager.move_cartesian`을 만든다. workspace가 없으면 거부한다. 정의 없이 임의 좌표로 움직이는 것을 막기 위해서다.
  - `RobotController.move_cartesian`은 `MotionManager.move_cartesian`을 그대로 넘긴다.
  - 노드에서는 yaml 전체를 읽어 `poses`와 `workspace`를 넘기고, `cb_move_cartesian`을 기존 `cb_move_named_pose` 형식으로 만든다.
  - `MoveCartesian.srv`를 CMake `srv_files`에 추가한다.
- [ ] **Step 4: Run** `python3 -m pytest tests/test_motion_cartesian.py -v` → PASS. 이어서 `./scripts/build.sh` 성공, `ros2 interface show cafe_interfaces/srv/MoveCartesian`이 출력되는지 확인.
- [ ] **Step 5: Commit** `feat(robot_control): move_cartesian service with workspace check`

---

### Task 2: `cafe_vision` 패키지 + 좌표 계산 (`geometry.py`)

**Files:**
- Create: `cobot1_ws/src/cafe_vision/{package.xml, setup.py, setup.cfg, resource/cafe_vision, cafe_vision/__init__.py, cafe_vision/geometry.py, models/.gitkeep}`
- Modify: `tests/conftest.py` (sys.path 패키지 목록에 `cafe_vision` 추가), `tests/test_project_structure.py`, `scripts/check_project.py` (패키지 목록에 추가), `.gitignore` (`cobot1_ws/src/cafe_vision/models/*.pt`)
- Test: `tests/test_vision_geometry.py`

**Interfaces:**
- Produces (`cafe_vision.geometry`)
  - `deproject(u: float, v: float, depth_mm: float, fx, fy, cx, cy) -> np.ndarray` (3,), 카메라 광학 프레임 mm
  - `top_depth(depth_mm: np.ndarray, box: tuple[int,int,int,int], percentile=5.0) -> float | None`
    - 상자 안의 0이 아닌 깊이값 중 하위 percentile 값(가장 가까운 면, 즉 컵 맨 위)을 돌려준다.
    - 유효 픽셀이 10개 미만이면 `None`이다.
  - `tf_to_matrix(translation_m: list[float], quat_xyzw: list[float]) -> np.ndarray` (4×4, translation은 mm)
  - `transform_point(T: np.ndarray, p: np.ndarray) -> np.ndarray` (3,)
  - `pick_target(state: str, center: np.ndarray, rim: np.ndarray, bottom: np.ndarray) -> tuple[list[float], float, float]`
    - 반환값은 `(position_mm[3], yaw_deg, rim_direction_deg)`이고, 입력은 모두 base 좌표다.
    - upright, inverted: `position = center`, yaw = 0, rim_direction = 0
    - lying:
      - `position = [(rim+bottom)/2 의 x, y, center의 z]`
      - `rim_direction = atan2(rim.y - bottom.y, rim.x - bottom.x)` → [-180, 180)
      - `yaw = rim_direction`을 [-90, 90)로 접은 값
  - `choose_best(candidates: list[dict], threshold: float) -> dict | None`: 신뢰도 `conf`가 threshold 이상인 후보 중 최댓값
- package.xml exec_depend: `rclpy`, `cafe_interfaces`, `sensor_msgs`, `cv_bridge`, `tf2_ros`, `python3-opencv`, `python3-numpy`, `python3-scipy`, `python3-yaml`
- setup.py
  - console_scripts: `vision_node`, `calibrate_node`, `capture_node`. 각 모듈의 `main`이고, 모듈은 Task 3, 4, 7, 8에서 만든다.
  - data_files: `calibration/board.yaml`, `training/cup.yaml`

- [ ] **Step 1: Write the failing tests**

```python
def test_deproject_center_pixel_is_on_optical_axis():
    assert np.allclose(deproject(320, 240, 500, 600, 600, 320, 240), [0, 0, 500])

def test_deproject_offset_pixel():
    assert np.allclose(deproject(380, 240, 600, 600, 600, 320, 240), [60, 0, 600])

def test_top_depth_ignores_zeros_and_takes_nearest_surface():
    d = np.zeros((100, 100), np.uint16); d[10:30, 10:30] = 400; d[15:20, 15:20] = 350
    assert top_depth(d, (10, 10, 30, 30)) == pytest.approx(350, abs=1)

def test_top_depth_none_without_valid_pixels():
    assert top_depth(np.zeros((50, 50), np.uint16), (0, 0, 50, 50)) is None

def test_tf_to_matrix_converts_meters_to_mm():
    T = tf_to_matrix([0.1, 0, 0.2], [0, 0, 0, 1])
    assert np.allclose(T[:3, 3], [100, 0, 200])

def test_pick_target_lying_axis_and_rim_direction():
    pos, yaw, rim = pick_target("lying", np.array([0, 0, 40.]), np.array([10, 10, 40.]), np.array([-10, -10, 40.]))
    assert np.allclose(pos, [0, 0, 40]) and yaw == pytest.approx(45) and rim == pytest.approx(45)

def test_pick_target_lying_rim_pointing_back_folds_yaw():
    _, yaw, rim = pick_target("lying", np.zeros(3), np.array([-10, -10, 0.]), np.array([10, 10, 0.]))
    assert rim == pytest.approx(-135) and yaw == pytest.approx(45)

def test_pick_target_inverted_uses_center():
    pos, yaw, _ = pick_target("inverted", np.array([400, 10, 120.]), np.zeros(3), np.zeros(3))
    assert np.allclose(pos, [400, 10, 120]) and yaw == 0

def test_choose_best_respects_threshold():
    c = [{"conf": 0.4}, {"conf": 0.9}, {"conf": 0.7}]
    assert choose_best(c, 0.5)["conf"] == 0.9 and choose_best(c, 0.95) is None
```

- [ ] **Step 2: Run** `python3 -m pytest tests/test_vision_geometry.py -v` → FAIL (모듈 없음)
- [ ] **Step 3: Implement** 패키지 골격과 `geometry.py`를 만든다. 쿼터니언 변환은 `scipy.spatial.transform.Rotation`을 쓴다.
- [ ] **Step 4: Run** `python3 -m pytest tests -q` → 전체 PASS. `./scripts/build.sh` 성공.
- [ ] **Step 5: Commit** `feat(cafe_vision): package skeleton and geometry helpers`

---

### Task 3: 핸드-아이 계산 (`hand_eye.py`)

**Files:**
- Create: `cobot1_ws/src/cafe_vision/cafe_vision/hand_eye.py`, `cobot1_ws/src/cafe_vision/calibration/board.yaml`
- Create: `config/robot/virtual/camera_extrinsics.yaml` (예시 값, `save_extrinsics` 형식)
- Test: `tests/test_hand_eye.py`

**Interfaces:**
- Produces (`cafe_vision.hand_eye`)
  - `board.yaml`: `cols: 9`, `rows: 6`(내부 코너 수), `square_mm: 25.0`
  - `find_board_pose(gray: np.ndarray, cols: int, rows: int, square_mm: float, K: np.ndarray, dist: np.ndarray) -> np.ndarray | None`
    - 반환값은 4×4 `T_cam_board`(mm)이고, 코너를 못 찾으면 `None`이다.
    - 순서: `findChessboardCorners` → `cornerSubPix` → `solvePnP`
  - `solve_hand_eye(T_base_flange: list[np.ndarray], T_cam_board: list[np.ndarray]) -> np.ndarray`
    - 반환값은 4×4 X = `T_flange_cam`이다.
    - `cv2.calibrateHandEye(..., method=cv2.CALIB_HAND_EYE_PARK)`를 쓴다. 입력 이름은 gripper2base, target2cam이다.
    - 샘플이 3개 미만이면 `ValueError`를 던진다.
  - `board_spread(T_base_flange, T_cam_board, X) -> tuple[float, float]`
    - 각 샘플의 `T_base_board = T_base_flange @ X @ T_cam_board`를 계산한다.
    - 반환값은 (위치 표준편차 mm, 회전 편차 deg)이다. 회전 편차는 평균 회전 대비 최대 각도다.
  - `save_extrinsics(path, X, samples: int, spread_mm: float, spread_deg: float)` / `load_extrinsics(path) -> np.ndarray`
    - yaml 형식: `parent: link_6`, `child: camera_color_optical_frame`, `translation_mm: [x, y, z]`, `quaternion_xyzw: [...]`, `samples`, `spread_mm`, `spread_deg`, `date`

- [ ] **Step 1: Write the failing tests** (합성 데이터)
  - 정답 X: 평행이동 `[30, -50, 80]`, ZYX 회전 `[10, -5, 90]`도
  - 고정된 `T_base_board`를 하나 정한다.
  - 서로 다른 회전을 가진 `T_base_flange` 10개를 만들고, `T_cam_board_i = inv(X) @ inv(T_base_flange_i) @ T_base_board`로 계산한다.

```python
def test_solve_hand_eye_recovers_known_transform():
    assert np.allclose(solve_hand_eye(flanges, boards), X, atol=1e-3)

def test_board_spread_is_zero_for_consistent_samples():
    pos, rot = board_spread(flanges, boards, X)
    assert pos < 1e-3 and rot < 1e-3

def test_board_spread_grows_with_wrong_extrinsics():
    wrong = X.copy(); wrong[0, 3] += 10
    assert board_spread(flanges, boards, wrong)[0] > 1.0

def test_solve_hand_eye_needs_three_samples():
    with pytest.raises(ValueError):
        solve_hand_eye(flanges[:2], boards[:2])

def test_extrinsics_round_trip(tmp_path):
    save_extrinsics(tmp_path / "x.yaml", X, 10, 0.5, 0.1)
    assert np.allclose(load_extrinsics(tmp_path / "x.yaml"), X, atol=1e-6)

def test_find_board_pose_on_rendered_board():
    # 9x6 내부 코너, 칸 40px 체커보드를 정면으로 그린 640x480 영상. K: f=600, c=(320,240)
    # 칸 25mm → 기대 거리 z = 600 * 25 / 40 = 375mm
    T = find_board_pose(img, 9, 6, 25.0, K, np.zeros(5))
    assert T is not None and T[2, 3] == pytest.approx(375, rel=0.02)

def test_find_board_pose_none_on_blank_image():
    assert find_board_pose(np.full((480, 640), 255, np.uint8), 9, 6, 25.0, K, np.zeros(5)) is None
```

- [ ] **Step 2: Run** `python3 -m pytest tests/test_hand_eye.py -v` → FAIL
- [ ] **Step 3: Implement** `hand_eye.py`와 `board.yaml`을 만들고, virtual 외부 파라미터 예시를 저장한다.
- [ ] **Step 4: Run** `python3 -m pytest tests/test_hand_eye.py -v` → PASS
- [ ] **Step 5: Commit** `feat(cafe_vision): hand-eye calibration math`

---

### Task 4: 탐지 노드 (`vision_node.py`, `detector.py`, `DetectCup.srv`)

**Files:**
- Create: `cobot1_ws/src/cafe_interfaces/srv/DetectCup.srv`, CMake srv_files에 추가
- Create: `cobot1_ws/src/cafe_vision/cafe_vision/detector.py`, `vision_node.py`
- Create: `config/robot/virtual/mock/{upright,inverted,lying}.yaml`
- Test: `tests/test_vision_node_logic.py`

**Interfaces:**
- Consumes: Task 2 `geometry.*`, Task 3 `load_extrinsics`
- Produces
  - `DetectCup.srv`: 요청은 비어 있다. 응답은 `bool found`, `string state`, `float64[3] position`, `float64 yaw`, `float64 rim_direction`, `float64 confidence`, `string message`
  - 서비스 `/vision/detect_cup`
  - `detector.YoloCupDetector(model_path)`
    - `.detect(color_bgr) -> list[dict]`. dict 키는 `state`(str), `conf`(float), `box`(x1, y1, x2, y2 int), `rim_px`(u, v), `bottom_px`(u, v)
    - ultralytics는 이 클래스의 `__init__` 안에서만 import한다.
  - `vision_node.build_response(candidate: dict | None, depth_mm, K: tuple[fx, fy, cx, cy], T_base_cam: np.ndarray) -> dict`
    - 순수 함수이고, 키는 srv 응답 필드와 같다.
    - candidate가 None이면 `found=False, message="컵을 찾지 못했습니다"`
    - `top_depth`가 None이면 `found=False, message="컵 깊이를 읽지 못했습니다"`
    - 그 외에는 상자 중심, rim, bottom 픽셀을 top_depth로 역투영하고 base로 변환한 뒤 `pick_target`을 적용한다.
  - `vision_node.load_mock(path) -> dict`: 응답 dict를 그대로 읽는다.
  - mock yaml 예(inverted): `found: true`, `state: inverted`, `position: [450, 0, 120]`, `yaw: 0`, `rim_direction: 0`, `confidence: 0.99`, `message: mock`. lying은 `yaw: 30`, `rim_direction: 30`. 모두 virtual workspace 안이어야 한다.
- 노드 파라미터
  - `mock_file`(있으면 카메라, TF, 모델을 쓰지 않는다)
  - `model_path`, `extrinsics_file`, `confidence_threshold`(기본 0.5)
  - `color_topic`(기본 `/camera/camera/color/image_raw`), `depth_topic`(기본 `/camera/camera/aligned_depth_to_color/image_raw`), `info_topic`(기본 `/camera/camera/color/camera_info`)
  - `base_frame`, `flange_frame`
- 실제 모드에서 `model_path`나 `extrinsics_file` 파일이 없으면 노드 시작 시 `RuntimeError`를 던진다. 메시지에 README 절을 적는다.
- 서비스 콜백 순서: 최신 컬러·깊이 프레임 → `detector.detect` → `choose_best` → TF로 `base→flange` 조회 → `@ X` → `build_response`. 프레임이나 TF가 없으면 `found=False`와 사유 메시지를 돌려준다.

- [ ] **Step 1: Write the failing tests**

```python
K = (600, 600, 320, 240)

def test_build_response_inverted_at_identity_pose():
    d = np.zeros((480, 640), np.uint16); d[200:280, 280:360] = 500
    c = {"state": "inverted", "conf": 0.9, "box": (280, 200, 360, 280), "rim_px": (320, 240), "bottom_px": (320, 240)}
    r = build_response(c, d, K, np.eye(4))
    assert r["found"] and r["state"] == "inverted" and np.allclose(r["position"], [0, 0, 500], atol=1)

def test_build_response_not_found_without_candidate():
    assert build_response(None, np.zeros((480, 640), np.uint16), K, np.eye(4))["found"] is False

def test_build_response_not_found_without_depth():
    c = {"state": "upright", "conf": 0.9, "box": (0, 0, 10, 10), "rim_px": (5, 5), "bottom_px": (5, 5)}
    r = build_response(c, np.zeros((480, 640), np.uint16), K, np.eye(4))
    assert r["found"] is False and "깊이" in r["message"]

def test_mock_files_are_inside_virtual_workspace():
    ws = yaml.safe_load(open("config/robot/virtual/poses.yaml"))["workspace"]
    for name in ("upright", "inverted", "lying"):
        p = load_mock(f"config/robot/virtual/mock/{name}.yaml")["position"]
        assert all(lo <= v <= hi for v, (lo, hi) in zip(p, (ws["x"], ws["y"], ws["z"])))
```

- [ ] **Step 2: Run** `python3 -m pytest tests/test_vision_node_logic.py -v` → FAIL
- [ ] **Step 3: Implement** `DetectCup.srv`, `detector.py`, `vision_node.py`(Node, mock 분기, `main`)를 만든다. YOLO 결과에서 `result.boxes.cls/conf/xyxy`와 `result.keypoints.xy`를 꺼내 dict로 만든다.
- [ ] **Step 4: Run**
  - `python3 -m pytest tests -q` → PASS
  - `./scripts/build.sh`
  - `ros2 run cafe_vision vision_node --ros-args -p mock_file:=$PWD/config/robot/virtual/mock/inverted.yaml`를 띄우고, 다른 터미널에서 `ros2 service call /vision/detect_cup cafe_interfaces/srv/DetectCup` 응답이 `found=True state='inverted'`인지 확인
- [ ] **Step 5: Commit** `feat(cafe_vision): detect_cup service with mock mode`

---

### Task 5: 집기 순서 (`cup_picker.py`) + 클라이언트

**Files:**
- Create: `cobot1_ws/src/coffee_process/coffee_process/cup_picker.py`
- Modify: `cobot1_ws/src/coffee_process/coffee_process/ros_clients.py`
- Create: `config/robot/virtual/cup_grasp.yaml`, `config/robot/real/cup_grasp.yaml`
- Test: `tests/test_cup_picker.py`

**Interfaces:**
- Consumes: Task 1 `/robot_control/move_cartesian`, Task 4 `/vision/detect_cup` 응답 필드
- Produces
  - `RobotClient.move_cartesian(pose: list[float], velocity: float, acceleration: float)`: 실패하면 `RuntimeError(message)`
  - `VisionClient(node, cb_group).detect_cup() -> dict`: srv 응답 필드 dict이고, 타임아웃은 10초다.
  - `plan_pick(detection: dict, cfg: dict) -> list[tuple]`: 각 원소는 `("move", [6 floats])` 또는 `("gripper", "open" | "close")`
  - `CupPicker(robot, io, vision, cfg, retries=3)`
    - `.run(checkpoint: Callable[[], None], report: Callable[[str], None])`
    - 동작: `robot.move_to_pose("cup_look", v, a)` → `detect_cup`(최대 retries번, 사이마다 `checkpoint()`) → `report(f"컵 탐지: {state}")` → `plan_pick`의 각 동작 앞에 `checkpoint()`를 부르고 실행
    - 끝내 못 찾으면 `RuntimeError("컵을 찾지 못했습니다: <마지막 message>")`
    - 비전 호출 예외도 실패 1회로 세고 재시도한다.
- `cup_grasp.yaml` 스키마 (virtual은 시뮬레이터에서 도달 가능한 값, real은 0)

```yaml
station_position: [x, y, z]   # mm. 기존 cup_pick 포즈와 같은 자리
approach_height: 100.0        # mm. 집기/놓기 전 위쪽 대기 높이
velocity: 30.0
acceleration: 30.0
states:
  upright:  {grasp_offset: [0, 0, -30], approach_orientation: [0, 180, 0], carry_orientation: [0, 180, 0]}
  inverted: {grasp_offset: [...], approach_orientation: [...], carry_orientation: [...]}
  lying:    {grasp_offset: [...], approach_orientation: [...],
             carry_orientation_rim_forward: [...], carry_orientation_rim_backward: [...]}
```

- `plan_pick` 규칙
  - `g = position + grasp_offset`, `above(p) = p + [0, 0, approach_height]`
  - `o_a = approach_orientation`. lying이면 마지막 각에 `yaw`를 더한다.
  - `o_c = carry_orientation`. lying이면 `cos(rim_direction - yaw) > 0`일 때 `_rim_forward`, 아니면 `_rim_backward`를 쓴다.
  - 순서 (10단계)
    1. open
    2. move above(g)+o_a
    3. move g+o_a
    4. close
    5. move above(g)+o_a
    6. move above(g)+o_c
    7. move above(station)+o_c
    8. move station+o_c
    9. open
    10. move above(station)+o_c

- [ ] **Step 1: Write the failing tests** (`cfg`는 테스트 안에서 숫자로 정의한다. FakeRobot, FakeIO, FakeVision은 호출을 기록한다)

```python
def test_plan_pick_upright_sequence():
    plan = plan_pick({"state": "upright", "position": [400, 0, 100], "yaw": 0, "rim_direction": 0}, cfg)
    assert plan[0] == ("gripper", "open") and plan[3] == ("gripper", "close") and plan[8] == ("gripper", "open")
    assert plan[2] == ("move", [400, 0, 70, 0, 180, 0])
    assert len(plan) == 10

def test_plan_pick_lying_applies_yaw_and_rim_side():
    fwd = plan_pick({"state": "lying", "position": [400, 0, 40], "yaw": 30, "rim_direction": 30}, cfg)
    back = plan_pick({"state": "lying", "position": [400, 0, 40], "yaw": 30, "rim_direction": -150}, cfg)
    assert fwd[1][1][5] == cfg["states"]["lying"]["approach_orientation"][2] + 30
    assert fwd[5][1][3:] == cfg["states"]["lying"]["carry_orientation_rim_forward"]
    assert back[5][1][3:] == cfg["states"]["lying"]["carry_orientation_rim_backward"]

def test_plan_pick_places_at_station():
    plan = plan_pick({"state": "inverted", "position": [400, 0, 120], "yaw": 0, "rim_direction": 0}, cfg)
    assert plan[7][1][:3] == cfg["station_position"]

def test_picker_moves_to_cup_look_then_executes_plan():
    CupPicker(robot, io, FakeVision([found_inverted]), cfg).run(lambda: None, reports.append)
    assert robot.calls[0] == ("pose", "cup_look") and reports == ["컵 탐지: inverted"]
    assert len([c for c in robot.calls if c[0] == "cartesian"]) == 7

def test_picker_retries_then_fails_when_not_found():
    v = FakeVision([{"found": False, "message": "x"}] * 3)
    with pytest.raises(RuntimeError, match="컵을 찾지 못했습니다"):
        CupPicker(robot, io, v, cfg).run(lambda: None, print)
    assert v.calls == 3 and not [c for c in robot.calls if c[0] == "cartesian"]

def test_picker_fails_after_retries_when_vision_errors():
    v = FakeVision([RuntimeError("/vision/detect_cup not available")] * 3)
    with pytest.raises(RuntimeError, match="not available"):
        CupPicker(robot, io, v, cfg).run(lambda: None, print)

def test_picker_propagates_motion_error():
    robot.cartesian_error = RuntimeError("Target outside workspace")
    with pytest.raises(RuntimeError, match="outside workspace"):
        CupPicker(robot, io, FakeVision([found_inverted]), cfg).run(lambda: None, print)

def test_picker_calls_checkpoint_before_each_motion():
    ticks = []
    CupPicker(robot, io, FakeVision([found_inverted]), cfg).run(lambda: ticks.append(1), print)
    assert len(ticks) >= 10
```

- [ ] **Step 2: Run** `python3 -m pytest tests/test_cup_picker.py -v` → FAIL
- [ ] **Step 3: Implement** `cup_picker.py`, `RobotClient.move_cartesian`, `VisionClient`(기존 `call_service` 사용), cup_grasp.yaml 두 개를 만든다.
- [ ] **Step 4: Run** `python3 -m pytest tests -q` → PASS
- [ ] **Step 5: Commit** `feat(coffee_process): CupPicker plans and runs vision pick`

---

### Task 6: `vision_pick` 레시피 액션 연결

**Files:**
- Modify: `cobot1_ws/src/coffee_process/coffee_process/process_executor.py`, `process_runner.py`, `coffee_process_node.py`
- Modify: `cobot1_ws/src/coffee_process/config/recipes/{americano,cappuccino,latte}.yaml`
- Test: `tests/test_process_runner.py` (추가)

**Interfaces:**
- Consumes: Task 5 `CupPicker.run(checkpoint, report)`
- Produces
  - `ProcessExecutor(robot, io, wait, picker=None, checkpoint=None, report=None)`
    - `vision_pick` 액션은 `picker.run(checkpoint, report)`를 부른다.
    - picker가 None이면 `RuntimeError("vision_pick: grasp_file 파라미터가 없어 CupPicker가 없습니다")`를 던진다.
  - `ProcessRunner(recipes, robot, io, on_status, picker=None)`
    - `report`는 상태 `message`를 갱신하고 emit한다.
    - `vision_pick` 단계 시작 시 `_holding_cup = True`, 끝나면 `False`로 둔다. 도중에 강제 정지하면 보수적으로 컵을 버린다.
    - `pause()`는 `_current_action`이 `"move"` 또는 `"vision_pick"`일 때 로봇도 멈춘다.
  - 노드 파라미터 `grasp_file`(선택): 있으면 yaml을 로드하고 `CupPicker(RobotClient, IOClient, VisionClient, cfg)`를 만든다.
  - 레시피 세 개 맨 앞에 `- {name: Find Cup, action: vision_pick}`를 추가한다. 이 단계가 진행률 1단계로 늘어난다.

- [ ] **Step 1: Write the failing tests** (기존 FakeRobot, FakeRecipes 재사용. FakePicker는 `run` 안에서 `checkpoint()`를 부르고, `gate` 이벤트로 멈출 수 있다)

```python
def test_vision_pick_step_runs_picker():
    # steps=[{"action": "vision_pick"}], execute → DONE, picker.runs == 1

def test_vision_pick_without_picker_fails_order():
    # picker=None → order_state FAILED, message에 "grasp_file"

def test_force_stop_during_vision_pick_discards_cup():
    # picker가 gate에서 대기 중 force_stop → resume(reset) 시 robot.calls에 ("move", "trash")와 gripper open

def test_pause_during_vision_pick_blocks_next_motion():
    # picker run: checkpoint → motion1 → (gate) → checkpoint → motion2
    # pause 후: robot.calls에 ("pause",)가 있고 motion2 없음 → resume → motion2 실행, DONE

def test_report_updates_status_message():
    # picker가 report("컵 탐지: lying") → snapshot()["message"] == "컵 탐지: lying"
```

- [ ] **Step 2: Run** `python3 -m pytest tests/test_process_runner.py -v` → 새 테스트 FAIL
- [ ] **Step 3: Implement** executor, runner, node, 레시피를 위 Interfaces대로 바꾼다.
- [ ] **Step 4: Run** `python3 -m pytest tests -q` → 전체 PASS. `tests/test_recipe_loader.py`가 단계 수를 검사하면 기대값을 갱신한다.
- [ ] **Step 5: Commit** `feat(coffee_process): vision_pick recipe action`

---

### Task 7: 핸드-아이 캘리브레이션 도구 (`calibrate_node.py`)

**Files:**
- Create: `cobot1_ws/src/cafe_vision/cafe_vision/calibrate_node.py`
- Create: `cobot1_ws/src/cafe_vision/README.md` (설치 절, 캘리브레이션 절)

**Interfaces:**
- Consumes: Task 3 `find_board_pose`, `solve_hand_eye`, `board_spread`, `save_extrinsics`, Task 2 `tf_to_matrix`
- 파라미터
  - `board_file`(기본: 패키지 share의 `calibration/board.yaml`)
  - `output_file`(필수, 예 `config/robot/real/camera_extrinsics.yaml`)
  - `sample_dir`(기본 `log/calibration/<YYYYmmdd_HHMMSS>`)
  - `color_topic`, `info_topic`, `base_frame`, `flange_frame`, `min_samples`(기본 15), `max_spread_mm`(기본 5.0)
- 동작
  - 콘솔 입력 스레드에서 Enter를 받으면 최신 영상에서 `find_board_pose`를 실행한다.
    - 실패하면 "체커보드를 찾지 못했습니다. 보드 전체가 보이게 다시"를 출력한다.
    - 성공하면 TF `base→flange`와 함께 샘플로 저장한다(영상 png와 두 행렬 npz).
  - `q` 입력 시
    - 샘플이 `min_samples` 미만이면 경고 후 계속 받는다.
    - 충분하면 계산 → 편차 출력 → `max_spread_mm` 초과 시 경고 → `save_extrinsics`
  - **로봇에 이동 명령을 보내지 않는다.**
- README 캘리브레이션 절
  - 보드 인쇄와 고정
  - 직접 교시나 티치펜던트로 15자세 이상(기울기 ±30°, 회전 섞기)
  - 실행 명령: `ros2 launch m0609_rg2_bringup bringup_camera.launch.py mode:=real host:=<IP>` + driver + `ros2 run cafe_vision calibrate_node --ros-args -p output_file:=$PWD/config/robot/real/camera_extrinsics.yaml`
  - 결과 확인법: 실물 시험 0단계, 코너 50mm 위로 이동

- [ ] **Step 1: Verify** `python3 -m pytest tests/test_hand_eye.py -q` → PASS (계산은 Task 3에서 검증됨)
- [ ] **Step 2: Implement** `calibrate_node.py`와 README 절을 쓴다.
- [ ] **Step 3: Run**
  - `./scripts/build.sh`
  - `ros2 run cafe_vision calibrate_node --ros-args -p output_file:=/tmp/x.yaml`이 카메라 없이도 시작해 "영상 대기 중"을 출력하는지 확인
  - Enter를 누르면 "영상이 아직 없습니다"를 출력하고 죽지 않는지 확인
- [ ] **Step 4: Commit** `feat(cafe_vision): interactive hand-eye calibration node`

---

### Task 8: 학습 도구 (`capture_node.py`, `cup.yaml`, README 학습 절)

**Files:**
- Create: `cobot1_ws/src/cafe_vision/cafe_vision/capture_node.py`, `cobot1_ws/src/cafe_vision/training/cup.yaml`
- Modify: `cobot1_ws/src/cafe_vision/README.md` (학습 절)
- Test: `tests/test_training_config.py`

**Interfaces:**
- `cup.yaml`
  - `path: ~/cafe_vision_dataset`, `train: images/train`, `val: images/val`
  - `kpt_shape: [2, 3]`, `flip_idx: [0, 1]`
  - `names: {0: upright, 1: inverted, 2: lying}`
- `capture_node` 파라미터: `color_topic`, `out_dir`(기본 `~/cafe_vision_dataset/raw`). OpenCV 창에서 `s`를 누르면 `<타임스탬프>.jpg`로 저장하고, `q`를 누르면 종료한다.
- README 학습 절
  - 설치: Global Constraints의 pip 명령 + `python3 -c "import cv_bridge"` 확인
  - 촬영: `cup_look` 자세에서 상태별 100장 이상. 쌓인 높이 1–10개, 테이블 위치, 조명을 다양하게
  - 라벨링: CVAT에서 상자, 클래스, 키포인트 `rim`, `bottom`을 단다. 가려진 키포인트는 visibility 0. "Ultralytics YOLO Pose" 형식으로 내보낸다.
  - 학습: `yolo pose train data=cobot1_ws/src/cafe_vision/training/cup.yaml model=yolov8n-pose.pt imgsz=640 epochs=100`
  - 결과 `runs/pose/train*/weights/best.pt`를 `cobot1_ws/src/cafe_vision/models/cup_pose.pt`로 복사

- [ ] **Step 1: Write the failing test**

```python
def test_dataset_config_matches_vision_constants():
    cfg = yaml.safe_load(open("cobot1_ws/src/cafe_vision/training/cup.yaml"))
    assert cfg["kpt_shape"] == [2, 3] and cfg["names"] == {0: "upright", 1: "inverted", 2: "lying"}
```

- [ ] **Step 2: Run** → FAIL. **Step 3: Implement** → **Step 4: Run** `python3 -m pytest tests -q` → PASS. `ros2 run cafe_vision capture_node`가 영상이 없을 때 대기 메시지를 출력하는지 확인
- [ ] **Step 5: Commit** `feat(cafe_vision): dataset capture tool and training guide`

---

### Task 9: 실행 스크립트, 설정 검사, 시뮬레이션 E2E

**Files:**
- Modify: `scripts/check_robot_config.py`, `tests/test_robot_config_check.py`
- Modify: `scripts/start_sim.sh`, `scripts/start_real.sh`, `README.md` (Execution 섹션만)

**Interfaces:**
- `check(config_dir)`에 다음 오류를 추가한다.
  - `poses.yaml`에 `workspace`가 없거나 어느 축이든 `min >= max`이면 → `"poses.yaml: workspace.<축> 범위 필요"`
  - `camera_extrinsics.yaml`이 없으면 → `"camera_extrinsics.yaml 없음 (핸드-아이 캘리브레이션 필요)"`
  - `cup_grasp.yaml`의 `station_position`이 모두 0이면 → `"cup_grasp.yaml: station_position 가 전부 0 (티칭 필요)"`
- `start_sim.sh`
  - 단계가 7개로 늘어난다. rosbridge 앞에 `cafe_vision` 단계를 넣는다.
    - virtual: `vision_node -p mock_file:="${MOCK_DETECTION:-$ROOT/config/robot/virtual/mock/inverted.yaml}"`
    - real: `-p model_path:=$ROOT/cobot1_ws/src/cafe_vision/models/cup_pose.pt -p extrinsics_file:=$ROOT/config/robot/real/camera_extrinsics.yaml`
  - coffee_process에 `-p grasp_file:="$ROOT/config/robot/$MODE/cup_grasp.yaml"`을 추가한다.
  - real 모드의 bringup은 `bringup_camera.launch.py`다.
  - 중복 실행 검사 패턴에 `vision_node`를 추가한다.
- `start_real.sh`: 설정 검사 뒤에 모델 파일 존재 검사를 넣는다. 없으면 "모델 없음: README 학습 절 참고"를 출력하고 종료한다.
- README Execution
  - Real 절에 순서를 적는다: ① 캘리브레이션 → ② 모델 학습 후 `models/cup_pose.pt` 배치 → ③ `cup_look`·`cup_station`(`cup_pick`과 같은 자리) 티칭 → ④ 실행
  - Simulation 절에 `MOCK_DETECTION=config/robot/virtual/mock/lying.yaml ./scripts/start_sim.sh` 예시를 넣는다.

- [ ] **Step 1: Write the failing tests** (`tests/test_robot_config_check.py`)

```python
def test_missing_extrinsics_is_rejected(tmp_path):  # 유효한 poses/io/cup_grasp만 두고 extrinsics 없음
    assert any("camera_extrinsics" in e for e in check(tmp_path))

def test_missing_workspace_is_rejected(tmp_path):
    assert any("workspace" in e for e in check(tmp_path))

def test_untaught_real_config_is_rejected():   # 기존 테스트에 추가
    errors = check(ROOT / "config/robot/real")
    assert any("station_position" in e for e in errors) and any("cup_look" in e for e in errors)
```
기존 `test_virtual_config_passes`와 `test_taught_values_pass`는 계속 통과해야 한다. tmp 설정에 새 파일들을 추가한다.

- [ ] **Step 2: Run** `python3 -m pytest tests/test_robot_config_check.py -v` → 새 테스트 FAIL
- [ ] **Step 3: Implement** 검사기, 스크립트 두 개, README를 고친다.
- [ ] **Step 4: Run (E2E)**
  - `python3 -m pytest tests -q` → 전체 PASS
  - `ROBOT_HOST=1.2.3.4 ./scripts/start_real.sh` → `station_position`, `cup_look`, `camera_extrinsics` 오류로 거부되는지 확인
  - mock 세 종류(`upright`, `inverted`, `lying`)마다 `MOCK_DETECTION=... ./scripts/start_sim.sh` 실행 → `resume` → `start` → 아메리카노 주문 → **DONE**, 진행 메시지에 `컵 탐지: <state>` 확인
  - inverted mock으로 `Find Cup` 도중 `pause` → 상태값이 10초간 PAUSED 유지 → `resume` → DONE
  - `Find Cup` 도중 `force_stop` → FAILED → `resume` 시 `trash` 경유 홈 이동 → `start` → IDLE
- [ ] **Step 5: Commit** `feat(scripts): launch cafe_vision in sim and real, extend config checks`

---

## 실물 시험 (코드 작업 이후, 현장)

스펙 §5.8 순서를 따른다.
- **0단계 캘리브레이션 확인:** 코너 50mm 위로 이동해 눈으로 확인한다.
- **1단계:** 로봇을 움직이지 않고 탐지 좌표만 출력해 실제 위치와 비교한다.
- **2단계:** 컵 위 높은 곳까지만 이동한다.
- **3단계:** 상태별로 집는다.
- **4단계:** 전체 주문을 진행한다.

**현장 확인 항목**
- 쌓인 컵에서 한 장만 빠지는지
- 직접 교시 중 드라이버 연결 유지
- `bringup_camera.launch.py`의 준비 완료 로그와 정렬 깊이(`align_depth`) 활성화 여부
- 실제 TF 프레임 이름(`ros2 run tf2_ros tf2_echo base_link link_6`)
