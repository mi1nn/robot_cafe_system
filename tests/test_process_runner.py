import threading
import time

import pytest

from coffee_process.process_runner import ProcessRunner


class FakeRecipes:

    def __init__(self, steps):
        self.steps = steps

    def load_recipe(self, recipe_id):
        if recipe_id != "latte":
            raise FileNotFoundError(recipe_id)
        return {"id": recipe_id, "name": "Latte", "steps": self.steps}


class FakeRobot:

    def __init__(self):
        self.calls = []
        self.block = None
        self.fail = None
        self.pause_error = None
        self.pause_gate = None
        self.stop_error = None
        self.moving = threading.Event()
        self._stopped = threading.Event()

    def move_to_pose(self, name, velocity, acceleration):
        self.calls.append(("move", name))
        self.moving.set()
        if self.fail:
            raise self.fail
        while self.block is not None and not self.block.wait(0.01):
            if self._stopped.is_set():
                raise RuntimeError("stopped")

    def pause(self):
        self.calls.append(("pause",))
        if self.pause_gate is not None:
            self.pause_gate.wait(2)
        if self.pause_error:
            raise self.pause_error

    def resume(self):
        self.calls.append(("resume",))

    def stop(self):
        self.calls.append(("stop",))
        if self.stop_error:
            raise self.stop_error
        self._stopped.set()

    def clear_stop(self):
        self.calls.append(("clear_stop",))
        self._stopped.clear()


class FakeIO:

    def __init__(self):
        self.calls = []
        self.gripper_block = None
        self.gripper_started = threading.Event()

    def set_output(self, name, value):
        self.calls.append(("output", name, value))

    def set_gripper(self, command):
        self.calls.append(("gripper", command))
        self.gripper_started.set()
        if self.gripper_block is not None:
            self.gripper_block.wait(2)


MOVE = {"name": "Cup Pick", "action": "move", "pose": "cup_pick"}
GRIP_CLOSE = {"name": "Grip Cup", "action": "gripper", "command": "close"}
TO_MACHINE = {"name": "Coffee Machine", "action": "move", "pose": "coffee_machine"}
OUTPUT_ON = {"name": "Start Espresso", "action": "io", "output": "coffee_start", "value": True}


def wait_until(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    raise AssertionError("condition not met")


@pytest.fixture
def env():
    robot = FakeRobot()
    io = FakeIO()
    statuses = []
    recipes = FakeRecipes([MOVE, GRIP_CLOSE, OUTPUT_ON])
    runner = ProcessRunner(recipes, robot, io, on_status=statuses.append)
    yield runner, robot, io, recipes, statuses
    if robot.block is not None:
        robot.block.set()
    runner.join(2)


def ready(runner):
    runner.resume()
    runner.join(2)
    assert runner.start()[0] is True
    assert runner.state == "IDLE"


def test_initial_state_is_stopped(env):
    runner = env[0]
    assert runner.state == "STOPPED"


def test_first_reset_goes_home_only(env):
    runner, robot, *_ = env
    assert runner.resume()[0] is True
    runner.join(2)
    assert robot.calls == [("clear_stop",), ("move", "home")]
    assert runner.state == "HOMED"


def test_full_recipe_done(env):
    runner, robot, io, *_ = env
    ready(runner)
    assert runner.execute("ORD-1", "latte") == (True, "accepted")
    runner.join(2)
    s = runner.snapshot()
    assert (s["state"], s["order_state"], s["progress"], s["order_id"]) == (
        "IDLE", "DONE", 100.0, "ORD-1")
    assert ("move", "cup_pick") in robot.calls
    assert io.calls == [("gripper", "close"), ("output", "coffee_start", True)]


def test_execute_unknown_recipe_rejected(env):
    runner = env[0]
    ready(runner)
    ok, _ = runner.execute("ORD-1", "mocha")
    assert ok is False
    assert runner.state == "IDLE"


def test_execute_rejected_when_not_idle(env):
    runner, robot, *_ = env
    ready(runner)
    robot.block = threading.Event()
    runner.execute("ORD-1", "latte")
    ok, _ = runner.execute("ORD-2", "latte")
    assert ok is False
    assert runner.snapshot()["order_id"] == "ORD-1"


def test_commands_rejected_in_wrong_state(env):
    runner = env[0]
    assert runner.pause()[0] is False
    assert runner.start()[0] is False
    assert runner.execute("ORD-1", "latte")[0] is False
    assert runner.state == "STOPPED"


def test_pause_resume_move_step(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [MOVE]
    ready(runner)
    robot.block = threading.Event()
    robot.moving.clear()
    runner.execute("ORD-1", "latte")
    wait_until(robot.moving.is_set)
    assert runner.pause()[0] is True
    assert runner.state == "PAUSED"
    assert runner.resume()[0] is True
    robot.block.set()
    runner.join(2)
    assert robot.calls.count(("pause",)) == 1
    assert robot.calls.count(("resume",)) == 1
    assert runner.snapshot()["order_state"] == "DONE"


def test_pause_during_wait_keeps_remaining_time(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [{"name": "Espresso Wait", "action": "wait", "seconds": 0.3}]
    ready(runner)
    started = time.monotonic()
    runner.execute("ORD-1", "latte")
    time.sleep(0.1)
    runner.pause()
    time.sleep(0.5)
    assert runner.state == "PAUSED"
    assert runner.snapshot()["order_state"] == "RUNNING"
    runner.resume()
    runner.join(2)
    assert time.monotonic() - started >= 0.8
    assert ("pause",) not in robot.calls
    assert runner.snapshot()["order_state"] == "DONE"


def test_pause_during_gripper_applies_after_step(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [GRIP_CLOSE, MOVE]
    ready(runner)
    io.gripper_block = threading.Event()
    runner.execute("ORD-1", "latte")
    wait_until(io.gripper_started.is_set)
    assert runner.pause()[0] is True
    io.gripper_block.set()
    time.sleep(0.1)
    assert ("move", "cup_pick") not in robot.calls
    assert runner.state == "PAUSED"
    runner.resume()
    runner.join(2)
    assert ("pause",) not in robot.calls
    assert ("resume",) not in robot.calls
    assert runner.snapshot()["order_state"] == "DONE"


def test_force_stop_running_marks_failed(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [MOVE]
    ready(runner)
    robot.block = threading.Event()
    robot.moving.clear()
    runner.execute("ORD-1", "latte")
    wait_until(robot.moving.is_set)
    assert runner.force_stop()[0] is True
    runner.join(2)
    s = runner.snapshot()
    assert (s["state"], s["order_state"]) == ("STOPPED", "FAILED")
    assert ("stop",) in robot.calls


def test_force_stop_while_paused(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [MOVE]
    ready(runner)
    robot.block = threading.Event()
    robot.moving.clear()
    runner.execute("ORD-1", "latte")
    wait_until(robot.moving.is_set)
    runner.pause()
    assert runner.force_stop()[0] is True
    runner.join(2)
    s = runner.snapshot()
    assert (s["state"], s["order_state"]) == ("STOPPED", "FAILED")
    assert ("stop",) in robot.calls
    assert ("resume",) not in robot.calls


def test_force_stop_during_wait(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [{"name": "Espresso Wait", "action": "wait", "seconds": 5}]
    ready(runner)
    runner.execute("ORD-1", "latte")
    time.sleep(0.05)
    runner.force_stop()
    runner.join(1)
    assert runner.state == "STOPPED"


def test_step_failure_stops(env):
    runner, robot, *_ = env
    ready(runner)
    robot.fail = RuntimeError("boom")
    runner.execute("ORD-1", "latte")
    runner.join(2)
    s = runner.snapshot()
    assert (s["state"], s["order_state"], s["message"]) == ("STOPPED", "FAILED", "boom")


def test_reset_with_cup_goes_to_trash(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [GRIP_CLOSE, TO_MACHINE]
    ready(runner)
    robot.block = threading.Event()
    runner.execute("ORD-1", "latte")
    wait_until(lambda: ("move", "coffee_machine") in robot.calls)
    runner.force_stop()
    runner.join(2)
    robot.block = None
    assert runner.resume()[0] is True
    runner.join(2)
    assert robot.calls[-2:] == [("move", "trash"), ("move", "home")]
    assert io.calls[-1] == ("gripper", "open")
    assert runner.state == "HOMED"


def test_force_stop_during_resetting(env):
    runner, robot, *_ = env
    robot.block = threading.Event()
    runner.resume()
    wait_until(robot.moving.is_set)
    assert runner.state == "RESETTING"
    assert runner.force_stop()[0] is True
    runner.join(2)
    assert runner.state == "STOPPED"


def test_next_order_requires_start_after_reset(env):
    runner = env[0]
    runner.resume()
    runner.join(2)
    assert runner.state == "HOMED"
    assert runner.execute("ORD-1", "latte")[0] is False
    assert runner.start()[0] is True
    assert runner.execute("ORD-1", "latte")[0] is True
    runner.join(2)


def test_status_callback_on_every_transition(env):
    runner, robot, io, recipes, statuses = env
    ready(runner)
    runner.execute("ORD-1", "latte")
    runner.join(2)
    states = []
    for status in statuses:
        if not states or states[-1] != status["state"]:
            states.append(status["state"])
    assert states == ["RESETTING", "HOMED", "IDLE", "RUNNING", "IDLE"]
    assert set(statuses[-1]) == {
        "order_id", "recipe_id", "state", "order_state", "current_step",
        "current_step_index", "total_steps", "progress", "message"}


def test_stop_latch_cleared_once_per_process(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [MOVE, TO_MACHINE]
    ready(runner)
    runner.execute("ORD-1", "latte")
    runner.join(2)
    calls = [c for c in robot.calls if c[0] in ("clear_stop", "move")]
    assert calls == [("clear_stop",), ("move", "home"),
                     ("clear_stop",), ("move", "cup_pick"), ("move", "coffee_machine")]


def test_force_stop_reports_failure_when_robot_stop_fails(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [{"name": "Espresso Wait", "action": "wait", "seconds": 5}]
    ready(runner)
    robot.stop_error = RuntimeError("driver down")
    runner.execute("ORD-1", "latte")
    ok, message = runner.force_stop()
    assert ok is False
    assert "driver down" in message
    runner.join(2)
    assert runner.state == "STOPPED"


def test_pause_failure_does_not_override_force_stop(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [MOVE]
    ready(runner)
    robot.block = threading.Event()
    robot.moving.clear()
    robot.pause_gate = threading.Event()
    robot.pause_error = RuntimeError("timeout")
    runner.execute("ORD-1", "latte")
    wait_until(robot.moving.is_set)
    result = {}
    t = threading.Thread(target=lambda: result.update(r=runner.pause()))
    t.start()
    wait_until(lambda: ("pause",) in robot.calls)
    runner.force_stop()
    runner.join(2)
    robot.pause_gate.set()
    t.join(2)
    assert result["r"][0] is False
    assert runner.state == "STOPPED"


def test_double_resume_from_stopped_starts_one_reset(env):
    runner, robot, *_ = env
    robot.block = threading.Event()
    results = []
    threads = [threading.Thread(target=lambda: results.append(runner.resume())) for _ in range(2)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(2)
    robot.block.set()
    runner.join(2)
    assert sorted(ok for ok, _ in results) == [False, True]
    assert robot.calls.count(("move", "home")) == 1


VISION_PICK = {"name": "Find Cup", "action": "vision_pick"}


class FakePicker:
    """CupPicker 대역: 이동 두 번 사이에 checkpoint를 부르고, report를 보낸다."""

    def __init__(self, robot, message=None):
        self.robot = robot
        self.message = message
        self.runs = 0

    def run(self, checkpoint, report):
        self.runs += 1
        checkpoint()
        self.robot.move_to_pose("cup_look", 30.0, 30.0)
        if self.message:
            report(self.message)
        checkpoint()
        self.robot.move_to_pose("cup_station", 30.0, 30.0)


@pytest.fixture
def vision_env():
    robot = FakeRobot()
    io = FakeIO()
    picker = FakePicker(robot)
    statuses = []
    runner = ProcessRunner(FakeRecipes([VISION_PICK]), robot, io, on_status=statuses.append, picker=picker)
    yield runner, robot, io, picker, statuses
    if robot.block is not None:
        robot.block.set()
    runner.join(2)


def test_vision_pick_step_runs_picker(vision_env):
    runner, robot, io, picker, _ = vision_env
    ready(runner)
    runner.execute("ORD-1", "latte")
    runner.join(2)
    assert picker.runs == 1
    assert runner.snapshot()["order_state"] == "DONE"
    assert ("move", "cup_station") in robot.calls


def test_vision_pick_without_picker_fails_order(env):
    runner, robot, io, recipes, _ = env
    recipes.steps = [VISION_PICK]
    ready(runner)
    runner.execute("ORD-1", "latte")
    runner.join(2)
    s = runner.snapshot()
    assert s["order_state"] == "FAILED"
    assert "grasp_file" in s["message"]


def test_force_stop_during_vision_pick_discards_cup(vision_env):
    runner, robot, io, picker, _ = vision_env
    ready(runner)
    robot.block = threading.Event()
    robot.moving.clear()
    runner.execute("ORD-1", "latte")
    wait_until(robot.moving.is_set)
    assert runner.force_stop()[0] is True
    runner.join(2)
    robot.block = None
    assert runner.resume()[0] is True
    runner.join(2)
    assert robot.calls[-2:] == [("move", "trash"), ("move", "home")]
    assert io.calls[-1] == ("gripper", "open")


def test_pause_during_vision_pick_blocks_next_motion(vision_env):
    runner, robot, io, picker, _ = vision_env
    ready(runner)
    robot.block = threading.Event()
    robot.moving.clear()
    runner.execute("ORD-1", "latte")
    wait_until(robot.moving.is_set)
    assert runner.pause()[0] is True
    assert ("pause",) in robot.calls
    robot.block.set()
    time.sleep(0.1)
    assert ("move", "cup_station") not in robot.calls
    assert runner.resume()[0] is True
    runner.join(2)
    assert ("move", "cup_station") in robot.calls
    assert runner.snapshot()["order_state"] == "DONE"


def test_report_updates_status_message(vision_env):
    runner, robot, io, picker, statuses = vision_env
    picker.message = "컵 탐지: lying"
    ready(runner)
    runner.execute("ORD-1", "latte")
    runner.join(2)
    assert any(s["message"] == "컵 탐지: lying" for s in statuses)
