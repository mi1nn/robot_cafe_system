import math

import pytest
from scipy.spatial.transform import Rotation

from coffee_process.cup_picker import CupPicker, plan_pick

CFG = {
    "station_position": [450.0, 200.0, 100.0],
    "approach_height": 100.0,
    "velocity": 30.0,
    "acceleration": 30.0,
    "states": {
        "upright": {"grasp_offset": [0, 0, -30], "approach_orientation": [0, 180, 0], "carry_orientation": [0, 180, 0]},
        "inverted": {"grasp_offset": [0, 0, -20], "place_offset": [0, 0, 40], "approach_orientation": [0, 90, 0], "carry_orientation": [0, 90, 180]},
        "lying": {
            "grasp_offset": [0, 0, -10],
            "approach_orientation": [0, 180, 0],
            "carry_orientation_rim_forward": [0, 90, 0],
            "carry_orientation_rim_backward": [180, 90, 0],
        },
    },
}

FOUND_INVERTED = {"found": True, "state": "inverted", "position": [400, 0, 120], "yaw": 0.0,
                  "rim_direction": 0.0, "confidence": 0.9, "message": "ok"}


class FakeRobot:

    def __init__(self):
        self.calls = []
        self.cartesian_error = None

    def move_to_pose(self, name, velocity, acceleration):
        self.calls.append(("pose", name))

    def move_cartesian(self, pose, velocity, acceleration):
        if self.cartesian_error:
            raise self.cartesian_error
        self.calls.append(("cartesian", list(pose)))


class FakeIO:

    def __init__(self):
        self.calls = []

    def set_gripper(self, command):
        self.calls.append(command)


class FakeVision:

    def __init__(self, results):
        self.results = list(results)
        self.calls = 0

    def detect_cup(self):
        result = self.results[min(self.calls, len(self.results) - 1)]
        self.calls += 1
        if isinstance(result, Exception):
            raise result
        return result


def test_plan_pick_upright_sequence():
    plan = plan_pick({"state": "upright", "position": [400, 0, 100], "yaw": 0, "rim_direction": 0}, CFG)
    assert len(plan) == 10
    assert plan[0] == ("gripper", "open")
    assert plan[3] == ("gripper", "close")
    assert plan[8] == ("gripper", "open")
    assert plan[2] == ("move", [400, 0, 70, 0, 180, 0])
    assert plan[1] == ("move", [400, 0, 170, 0, 180, 0])


def heading(posx):
    """Doosan posx(ZYZ) 툴 x축이 base 에서 가리키는 방향(deg). 3개면 자세만, 6개면 뒤 3개."""
    angles = list(posx)[-3:]
    x_axis = Rotation.from_euler("ZYZ", angles, degrees=True).as_matrix()[:, 0]
    return math.degrees(math.atan2(x_axis[1], x_axis[0])) % 360


def test_plan_pick_lying_applies_yaw_and_rim_side():
    fwd = plan_pick({"state": "lying", "position": [400, 0, 40], "yaw": 30, "rim_direction": 30}, CFG)
    back = plan_pick({"state": "lying", "position": [400, 0, 40], "yaw": 30, "rim_direction": -150}, CFG)
    assert heading(fwd[1][1]) - heading(CFG["states"]["lying"]["approach_orientation"]) == pytest.approx(30)
    assert fwd[5][1][3:] == CFG["states"]["lying"]["carry_orientation_rim_forward"]
    assert back[5][1][3:] == CFG["states"]["lying"]["carry_orientation_rim_backward"]


def test_plan_pick_places_at_station():
    plan = plan_pick({"state": "inverted", "position": [400, 0, 120], "yaw": 0, "rim_direction": 0}, CFG)
    assert plan[7][1][:3] == [450.0, 200.0, 140.0]  # station + inverted place_offset
    assert plan[6][1][:3] == [450.0, 200.0, 240.0]
    assert plan[7][1][3:] == CFG["states"]["inverted"]["carry_orientation"]


def test_picker_moves_to_cup_look_then_executes_plan():
    robot, io, reports = FakeRobot(), FakeIO(), []
    CupPicker(robot, io, FakeVision([FOUND_INVERTED]), CFG).run(lambda: None, reports.append)
    assert robot.calls[0] == ("pose", "cup_look")
    assert reports == ["컵 탐지: inverted"]
    assert len([c for c in robot.calls if c[0] == "cartesian"]) == 7
    assert io.calls == ["open", "close", "open"]


def test_picker_retries_then_fails_when_not_found():
    robot = FakeRobot()
    vision = FakeVision([{"found": False, "message": "x"}])
    with pytest.raises(RuntimeError, match="컵을 찾지 못했습니다"):
        CupPicker(robot, FakeIO(), vision, CFG).run(lambda: None, print)
    assert vision.calls == 3
    assert not [c for c in robot.calls if c[0] == "cartesian"]


def test_picker_succeeds_on_retry():
    vision = FakeVision([{"found": False, "message": "x"}, FOUND_INVERTED])
    CupPicker(FakeRobot(), FakeIO(), vision, CFG).run(lambda: None, print)
    assert vision.calls == 2


def test_picker_fails_after_retries_when_vision_errors():
    vision = FakeVision([RuntimeError("/vision/detect_cup not available")])
    with pytest.raises(RuntimeError, match="not available"):
        CupPicker(FakeRobot(), FakeIO(), vision, CFG).run(lambda: None, print)
    assert vision.calls == 3


def test_picker_propagates_motion_error():
    robot = FakeRobot()
    robot.cartesian_error = RuntimeError("Target outside workspace")
    with pytest.raises(RuntimeError, match="outside workspace"):
        CupPicker(robot, FakeIO(), FakeVision([FOUND_INVERTED]), CFG).run(lambda: None, print)


def test_picker_calls_checkpoint_before_each_motion():
    ticks = []
    CupPicker(FakeRobot(), FakeIO(), FakeVision([FOUND_INVERTED]), CFG).run(lambda: ticks.append(1), print)
    assert len(ticks) >= 10


def test_plan_pick_place_offset_defaults_to_zero():
    plan = plan_pick({"state": "upright", "position": [400, 0, 100], "yaw": 0, "rim_direction": 0}, CFG)
    assert plan[7][1][:3] == CFG["station_position"]
