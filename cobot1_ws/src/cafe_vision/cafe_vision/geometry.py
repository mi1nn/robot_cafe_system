"""비전 좌표 계산 (ROS 없이 테스트 가능한 순수 함수).

단위: 위치 mm, 각도 deg. 카메라 좌표는 광학 프레임(z = 앞, x = 오른쪽, y = 아래).
"""

import math

import numpy as np
from scipy.spatial.transform import Rotation


def deproject(u, v, depth_mm, fx, fy, cx, cy):
    """픽셀 (u, v)와 깊이(mm)를 카메라 좌표 [x, y, z] mm로 바꾼다."""

    z = float(depth_mm)
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])


def top_depth(depth_mm, box, percentile=5.0):
    """상자 안에서 가장 가까운 면(컵 맨 위)의 깊이. 유효 픽셀이 10개 미만이면 None."""

    x1, y1, x2, y2 = (int(round(value)) for value in box)
    patch = depth_mm[max(y1, 0):y2, max(x1, 0):x2]
    valid = patch[patch > 0]

    if valid.size < 10:
        return None

    return float(np.percentile(valid, percentile))


def tf_to_matrix(translation_m, quat_xyzw):
    """TF(translation m, quaternion xyzw)를 4x4 행렬(translation mm)로 바꾼다."""

    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat_xyzw).as_matrix()
    T[:3, 3] = np.asarray(translation_m, dtype=float) * 1000.0
    return T


def transform_point(T, p):

    return (T @ np.append(np.asarray(p, dtype=float), 1.0))[:3]


def pick_target(state, center, rim, bottom):
    """base 좌표의 상자 중심, 입구 중심, 바닥 중심 → (집는 점, 그리퍼 yaw, 입구 방향)."""

    if state != "lying":
        return [float(v) for v in center], 0.0, 0.0

    rim_direction = math.degrees(math.atan2(rim[1] - bottom[1], rim[0] - bottom[0]))
    rim_direction = (rim_direction + 180.0) % 360.0 - 180.0

    yaw = (rim_direction + 90.0) % 180.0 - 90.0

    position = [(rim[0] + bottom[0]) / 2.0, (rim[1] + bottom[1]) / 2.0, center[2]]

    return [float(v) for v in position], yaw, rim_direction


def choose_best(candidates, threshold):
    """신뢰도가 threshold 이상인 후보 중 가장 높은 것. 없으면 None."""

    passing = [c for c in candidates if c["conf"] >= threshold]
    return max(passing, key=lambda c: c["conf"]) if passing else None
