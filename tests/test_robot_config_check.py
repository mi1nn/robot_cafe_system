import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))

from check_robot_config import check  # noqa: E402


def test_virtual_config_passes():
    assert check(ROOT / "config/robot/virtual") == []


def test_untaught_real_config_is_rejected():
    errors = check(ROOT / "config/robot/real")
    assert any("cup_pick.joint" in e for e in errors)
    assert any("coffee_start" in e for e in errors)


WORKSPACE = "workspace:\n  x: [200, 700]\n  y: [-400, 400]\n  z: [50, 600]\n"
POSES = "poses:\n  home:\n    joint: [0, 0, 90, 0, 90, 0]\n"


def write_taught(path, workspace=True, extrinsics=True):
    (path / "poses.yaml").write_text((WORKSPACE if workspace else "") + POSES)
    (path / "io.yaml").write_text("outputs:\n  coffee_start: 1\ninputs:\n  coffee_ready: 1\n")
    (path / "cup_grasp.yaml").write_text("station_position: [450, 200, 100]\n")
    if extrinsics:
        (path / "camera_extrinsics.yaml").write_text("translation_mm: [0, 60, 40]\n")


def test_taught_values_pass(tmp_path):
    write_taught(tmp_path)
    assert check(tmp_path) == []


def test_missing_extrinsics_is_rejected(tmp_path):
    write_taught(tmp_path, extrinsics=False)
    assert any("camera_extrinsics" in e for e in check(tmp_path))


def test_missing_workspace_is_rejected(tmp_path):
    write_taught(tmp_path, workspace=False)
    assert any("workspace" in e for e in check(tmp_path))


def test_empty_workspace_range_is_rejected(tmp_path):
    write_taught(tmp_path)
    (tmp_path / "poses.yaml").write_text(WORKSPACE.replace("[50, 600]", "[0, 0]") + POSES)
    assert any("workspace.z" in e for e in check(tmp_path))


def test_untaught_vision_values_in_real_config_are_rejected():
    errors = check(ROOT / "config/robot/real")
    assert any("station_position" in e for e in errors)
    assert any("cup_look" in e for e in errors)
    assert any("camera_extrinsics" in e for e in errors)
