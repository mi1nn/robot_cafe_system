"""실제 로봇 실행 전 설정 검사: TODO 값, 0으로 된 포즈, 작업 영역·카메라 캘리브레이션·컵 스테이션 누락이면 실패.

사용: python3 scripts/check_robot_config.py config/robot/real
"""

import sys
from pathlib import Path

import yaml


def check(config_dir):
    config_dir = Path(config_dir)
    errors = []

    pose_file = yaml.safe_load((config_dir / "poses.yaml").read_text())
    poses = pose_file["poses"] or {}

    workspace = pose_file.get("workspace") or {}
    for axis in ("x", "y", "z"):
        low, high = workspace.get(axis, [0, 0])
        if not float(low) < float(high):
            errors.append(f"poses.yaml: workspace.{axis} 범위 필요 (min < max)")

    for name, pose in poses.items():
        for kind in ("joint", "cartesian"):
            values = (pose or {}).get(kind)
            if values is not None and all(float(v) == 0.0 for v in values):
                errors.append(f"poses.yaml: {name}.{kind} 가 전부 0 (티칭 필요)")

    io = yaml.safe_load((config_dir / "io.yaml").read_text()) or {}
    for group, mapping in io.items():
        for name, value in (mapping or {}).items():
            if not isinstance(value, int):
                errors.append(f"io.yaml: {group}.{name} = {value} (번호 필요)")

    if not (config_dir / "camera_extrinsics.yaml").exists():
        errors.append("camera_extrinsics.yaml 없음 (핸드-아이 캘리브레이션 필요)")

    grasp_file = config_dir / "cup_grasp.yaml"
    station = (yaml.safe_load(grasp_file.read_text()) or {}).get("station_position") if grasp_file.exists() else None
    if not station or all(float(v) == 0.0 for v in station):
        errors.append("cup_grasp.yaml: station_position 가 전부 0 (티칭 필요)")

    return errors


if __name__ == "__main__":
    problems = check(sys.argv[1])
    for line in problems:
        print("  -", line)
    sys.exit(1 if problems else 0)
