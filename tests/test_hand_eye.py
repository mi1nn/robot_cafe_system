import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from cafe_vision.hand_eye import (
    board_spread,
    find_board_pose,
    load_extrinsics,
    save_extrinsics,
    solve_hand_eye,
)

K = np.array([[600.0, 0, 320], [0, 600.0, 240], [0, 0, 1]])


def make_T(translation, euler_zyx_deg):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("ZYX", euler_zyx_deg, degrees=True).as_matrix()
    T[:3, 3] = translation
    return T


X = make_T([30, -50, 80], [10, -5, 90])
T_BASE_BOARD = make_T([500, 0, 0], [0, 0, 180])


def synthetic_samples(n=10):
    rng = np.random.default_rng(0)
    flanges, boards = [], []
    for _ in range(n):
        T_base_flange = make_T(
            [450 + rng.uniform(-80, 80), rng.uniform(-80, 80), 400 + rng.uniform(-50, 50)],
            [rng.uniform(-40, 40), rng.uniform(-25, 25), 180 + rng.uniform(-25, 25)],
        )
        flanges.append(T_base_flange)
        boards.append(np.linalg.inv(X) @ np.linalg.inv(T_base_flange) @ T_BASE_BOARD)
    return flanges, boards


def test_solve_hand_eye_recovers_known_transform():
    flanges, boards = synthetic_samples()
    assert np.allclose(solve_hand_eye(flanges, boards), X, atol=1e-3)


def test_board_spread_is_zero_for_consistent_samples():
    flanges, boards = synthetic_samples()
    pos, rot = board_spread(flanges, boards, X)
    assert pos < 1e-3 and rot < 1e-3


def test_board_spread_grows_with_wrong_extrinsics():
    flanges, boards = synthetic_samples()
    wrong = X.copy()
    wrong[0, 3] += 10
    assert board_spread(flanges, boards, wrong)[0] > 1.0


def test_solve_hand_eye_needs_three_samples():
    flanges, boards = synthetic_samples()
    with pytest.raises(ValueError):
        solve_hand_eye(flanges[:2], boards[:2])


def test_extrinsics_round_trip(tmp_path):
    save_extrinsics(tmp_path / "x.yaml", X, 10, 0.5, 0.1)
    assert np.allclose(load_extrinsics(tmp_path / "x.yaml"), X, atol=1e-6)


def rendered_board(cols=9, rows=6, square_px=40):
    img = np.full((480, 640), 255, np.uint8)
    w, h = (cols + 1) * square_px, (rows + 1) * square_px
    x0, y0 = 320 - w // 2, 240 - h // 2
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                img[y0 + r * square_px:y0 + (r + 1) * square_px, x0 + c * square_px:x0 + (c + 1) * square_px] = 0
    return img


def test_find_board_pose_on_rendered_board():
    T = find_board_pose(rendered_board(), 9, 6, 25.0, K, np.zeros(5))
    assert T is not None
    assert T[2, 3] == pytest.approx(375, rel=0.02)


def test_find_board_pose_none_on_blank_image():
    assert find_board_pose(np.full((480, 640), 255, np.uint8), 9, 6, 25.0, K, np.zeros(5)) is None
