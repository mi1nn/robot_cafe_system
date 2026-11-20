import numpy as np
import pytest

from cafe_vision.geometry import (
    choose_best,
    deproject,
    pick_target,
    tf_to_matrix,
    top_depth,
    transform_point,
)


def test_deproject_center_pixel_is_on_optical_axis():
    assert np.allclose(deproject(320, 240, 500, 600, 600, 320, 240), [0, 0, 500])


def test_deproject_offset_pixel():
    assert np.allclose(deproject(380, 240, 600, 600, 600, 320, 240), [60, 0, 600])


def test_top_depth_ignores_zeros_and_takes_nearest_surface():
    d = np.zeros((100, 100), np.uint16)
    d[10:30, 10:30] = 400
    d[15:20, 15:20] = 350
    assert top_depth(d, (10, 10, 30, 30)) == pytest.approx(350, abs=1)


def test_top_depth_none_without_valid_pixels():
    assert top_depth(np.zeros((50, 50), np.uint16), (0, 0, 50, 50)) is None


def test_tf_to_matrix_converts_meters_to_mm():
    T = tf_to_matrix([0.1, 0, 0.2], [0, 0, 0, 1])
    assert np.allclose(T[:3, 3], [100, 0, 200])
    assert np.allclose(T[:3, :3], np.eye(3))


def test_transform_point_applies_rotation_and_translation():
    T = tf_to_matrix([0.1, 0, 0], [0, 0, np.sin(np.pi / 4), np.cos(np.pi / 4)])
    assert np.allclose(transform_point(T, np.array([10.0, 0, 0])), [100, 10, 0])


def test_pick_target_lying_axis_and_rim_direction():
    pos, yaw, rim = pick_target(
        "lying", np.array([0, 0, 40.0]), np.array([10, 10, 40.0]), np.array([-10, -10, 40.0])
    )
    assert np.allclose(pos, [0, 0, 40])
    assert yaw == pytest.approx(45)
    assert rim == pytest.approx(45)


def test_pick_target_lying_rim_pointing_back_folds_yaw():
    _, yaw, rim = pick_target("lying", np.zeros(3), np.array([-10, -10, 0.0]), np.array([10, 10, 0.0]))
    assert rim == pytest.approx(-135)
    assert yaw == pytest.approx(45)


def test_pick_target_inverted_uses_center():
    pos, yaw, _ = pick_target("inverted", np.array([400, 10, 120.0]), np.zeros(3), np.zeros(3))
    assert np.allclose(pos, [400, 10, 120])
    assert yaw == 0


def test_choose_best_respects_threshold():
    c = [{"conf": 0.4}, {"conf": 0.9}, {"conf": 0.7}]
    assert choose_best(c, 0.5)["conf"] == 0.9
    assert choose_best(c, 0.95) is None
