"""체커보드 핸드-아이 캘리브레이션 계산 (eye-in-hand, ROS 없이 테스트 가능).

X = T_flange_cam: link_6 → camera_color_optical_frame, 4x4, 위치 mm.
"""

import datetime

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def find_board_pose(gray, cols, rows, square_mm, K, dist):
    """영상에서 체커보드를 찾아 T_cam_board(4x4, mm)를 돌려준다. 못 찾으면 None."""

    found, corners = cv2.findChessboardCorners(gray, (cols, rows))
    if not found:
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_mm

    ok, rvec, tvec = cv2.solvePnP(grid, corners, K, dist)
    if not ok:
        return None

    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T


def solve_hand_eye(T_base_flange, T_cam_board):
    """샘플 쌍들로 X = T_flange_cam을 구한다 (OpenCV PARK)."""

    if len(T_base_flange) < 3 or len(T_base_flange) != len(T_cam_board):
        raise ValueError("핸드-아이 계산에는 같은 수의 샘플이 3개 이상 필요합니다")

    R, t = cv2.calibrateHandEye(
        [T[:3, :3] for T in T_base_flange],
        [T[:3, 3] for T in T_base_flange],
        [T[:3, :3] for T in T_cam_board],
        [T[:3, 3] for T in T_cam_board],
        method=cv2.CALIB_HAND_EYE_PARK,
    )

    X = np.eye(4)
    X[:3, :3] = R
    X[:3, 3] = t.ravel()
    return X


def board_spread(T_base_flange, T_cam_board, X):
    """샘플마다 계산한 보드의 base 포즈가 얼마나 흩어지는지: (위치 표준편차 mm, 최대 회전 편차 deg)."""

    boards = [F @ X @ B for F, B in zip(T_base_flange, T_cam_board)]

    positions = np.array([T[:3, 3] for T in boards])
    position_std = float(np.linalg.norm(positions.std(axis=0)))

    rotations = Rotation.from_matrix([T[:3, :3] for T in boards])
    mean = rotations.mean()
    rotation_dev = float(np.degrees(np.max((mean.inv() * rotations).magnitude())))

    return position_std, rotation_dev


def save_extrinsics(path, X, samples, spread_mm, spread_deg):

    data = {
        "parent": "link_6",
        "child": "camera_color_optical_frame",
        "translation_mm": [round(float(v), 4) for v in X[:3, 3]],
        "quaternion_xyzw": [float(v) for v in Rotation.from_matrix(X[:3, :3]).as_quat()],
        "samples": int(samples),
        "spread_mm": round(float(spread_mm), 3),
        "spread_deg": round(float(spread_deg), 3),
        "date": datetime.date.today().isoformat(),
    }

    with open(path, "w", encoding="utf-8") as file:
        yaml.safe_dump(data, file, sort_keys=False, allow_unicode=True)


def load_extrinsics(path):

    with open(path, "r", encoding="utf-8") as file:
        data = yaml.safe_load(file)

    X = np.eye(4)
    X[:3, :3] = Rotation.from_quat(data["quaternion_xyzw"]).as_matrix()
    X[:3, 3] = data["translation_mm"]
    return X
