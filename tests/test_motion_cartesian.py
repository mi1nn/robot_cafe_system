import pytest

from robot_control.motion_manager import MotionManager

WORKSPACE = {"x": [200, 700], "y": [-400, 400], "z": [50, 600]}


class FakeDriver:

    def __init__(self):
        self.calls = []

    def move_line(self, values, velocity, acceleration):
        self.calls.append(("move_line", list(values), velocity, acceleration))


def test_move_cartesian_inside_workspace_calls_move_line():
    driver = FakeDriver()
    MotionManager(driver, {}, WORKSPACE).move_cartesian([400, 0, 200, 0, 180, 0], 20, 20)
    assert driver.calls == [("move_line", [400, 0, 200, 0, 180, 0], 20, 20)]


def test_move_cartesian_rejects_outside_workspace():
    driver = FakeDriver()
    with pytest.raises(ValueError, match="outside workspace"):
        MotionManager(driver, {}, WORKSPACE).move_cartesian([400, 0, 10, 0, 180, 0])
    assert driver.calls == []


def test_move_cartesian_without_workspace_is_refused():
    driver = FakeDriver()
    with pytest.raises(ValueError, match="workspace"):
        MotionManager(driver, {}).move_cartesian([400, 0, 200, 0, 180, 0])
    assert driver.calls == []
