"""탐지 후보 → DetectCup 응답 변환 (ROS 없이 테스트 가능).

후보(candidate) dict: state, conf, box(x1, y1, x2, y2), rim_px(u, v), bottom_px(u, v)
응답 dict 키는 DetectCup.srv 응답 필드와 같다.
"""

import cv2
import numpy as np
import yaml

from .geometry import deproject, pick_target, top_depth, transform_point


def not_found(message):

    return {
        "found": False,
        "state": "",
        "position": [0.0, 0.0, 0.0],
        "yaw": 0.0,
        "rim_direction": 0.0,
        "confidence": 0.0,
        "message": message,
    }


def build_response(candidate, depth_mm, K, T_base_cam):
    """후보 하나를 base 기준 집는 점으로 바꾼다. K = (fx, fy, cx, cy)."""

    if candidate is None:
        return not_found("컵을 찾지 못했습니다")

    if candidate["state"] == "lying" and (0.0, 0.0) in (tuple(candidate["rim_px"]), tuple(candidate["bottom_px"])):
        return not_found("누운 컵의 키포인트를 확신할 수 없습니다")

    z = top_depth(depth_mm, candidate["box"])
    if z is None:
        return not_found("컵 깊이를 읽지 못했습니다")

    x1, y1, x2, y2 = candidate["box"]
    pixels = {
        "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
        "rim": candidate["rim_px"],
        "bottom": candidate["bottom_px"],
    }
    base = {
        name: transform_point(T_base_cam, deproject(u, v, z, *K))
        for name, (u, v) in pixels.items()
    }

    position, yaw, rim_direction = pick_target(
        candidate["state"], base["center"], base["rim"], base["bottom"]
    )

    return {
        "found": True,
        "state": candidate["state"],
        "position": position,
        "yaw": float(yaw),
        "rim_direction": float(rim_direction),
        "confidence": float(candidate["conf"]),
        "message": f"{candidate['state']} ({candidate['conf']:.2f})",
    }


def decode_compressed_depth(fmt, data):
    """compressedDepth 토픽(16UC1, png) → uint16 깊이(mm). 앞 12바이트는 ConfigHeader라 건너뛴다."""

    if not fmt.startswith("16UC1") or "png" not in fmt:
        raise ValueError(f"지원하지 않는 compressedDepth 형식: '{fmt}' (16UC1 png만 지원)")

    return cv2.imdecode(np.frombuffer(bytes(data)[12:], np.uint8), cv2.IMREAD_UNCHANGED)


def load_mock(path):
    """mock 응답 yaml을 읽는다 (시뮬레이션, 테스트용)."""

    with open(path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file)
