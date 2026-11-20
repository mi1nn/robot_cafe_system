import numpy as np
import yaml

from cafe_vision.detection import build_response, load_mock

K = (600, 600, 320, 240)


def test_build_response_inverted_at_identity_pose():
    d = np.zeros((480, 640), np.uint16)
    d[200:280, 280:360] = 500
    c = {"state": "inverted", "conf": 0.9, "box": (280, 200, 360, 280), "rim_px": (320, 240), "bottom_px": (320, 240)}
    r = build_response(c, d, K, np.eye(4))
    assert r["found"] is True
    assert r["state"] == "inverted"
    assert np.allclose(r["position"], [0, 0, 500], atol=1)
    assert r["confidence"] == 0.9


def test_build_response_lying_reports_axis():
    d = np.zeros((480, 640), np.uint16)
    d[200:280, 260:380] = 400
    c = {"state": "lying", "conf": 0.8, "box": (260, 200, 380, 280), "rim_px": (370, 240), "bottom_px": (270, 240)}
    r = build_response(c, d, K, np.eye(4))
    assert r["found"] is True
    assert r["rim_direction"] == 0.0
    assert np.allclose(r["position"], [0, 0, 400], atol=1)


def test_build_response_not_found_without_candidate():
    r = build_response(None, np.zeros((480, 640), np.uint16), K, np.eye(4))
    assert r["found"] is False
    assert "찾지" in r["message"]


def test_build_response_not_found_without_depth():
    c = {"state": "upright", "conf": 0.9, "box": (0, 0, 10, 10), "rim_px": (5, 5), "bottom_px": (5, 5)}
    r = build_response(c, np.zeros((480, 640), np.uint16), K, np.eye(4))
    assert r["found"] is False
    assert "깊이" in r["message"]


def test_mock_files_are_inside_virtual_workspace():
    ws = yaml.safe_load(open("config/robot/virtual/poses.yaml"))["workspace"]
    for name in ("upright", "inverted", "lying"):
        mock = load_mock(f"config/robot/virtual/mock/{name}.yaml")
        assert mock["found"] is True and mock["state"] == name
        p = mock["position"]
        assert all(lo <= v <= hi for v, (lo, hi) in zip(p, (ws["x"], ws["y"], ws["z"])))


def test_build_response_lying_without_keypoint_is_not_found():
    d = np.zeros((480, 640), np.uint16)
    d[200:280, 260:380] = 400
    c = {"state": "lying", "conf": 0.8, "box": (260, 200, 380, 280), "rim_px": (0.0, 0.0), "bottom_px": (270, 240)}
    r = build_response(c, d, K, np.eye(4))
    assert r["found"] is False
    assert "키포인트" in r["message"]


def test_decode_compressed_depth_png_roundtrip():
    import cv2
    import pytest

    from cafe_vision.detection import decode_compressed_depth

    depth = np.zeros((480, 640), np.uint16)
    depth[100:200, 100:200] = 1234
    png = cv2.imencode(".png", depth)[1].tobytes()
    data = np.zeros(3, np.float32).tobytes() + png
    assert np.array_equal(decode_compressed_depth("16UC1; compressedDepth png", data), depth)

    with pytest.raises(ValueError):
        decode_compressed_depth("16UC1; compressedDepth rvl", data)
