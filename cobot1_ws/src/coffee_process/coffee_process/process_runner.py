import threading
import time

from .process_executor import ProcessExecutor


class _Aborted(Exception):
    pass


class ProcessRunner:
    """
    커피 제조 상태 머신. ROS에 의존하지 않는다.

    STOPPED -resume-> RESETTING -> HOMED -start-> IDLE -execute-> RUNNING
    RUNNING <-pause/resume-> PAUSED
    RUNNING/PAUSED/RESETTING -force_stop 또는 실패-> STOPPED
    """

    def __init__(self, recipe_manager, robot, io, on_status, picker=None):

        self.recipes = recipe_manager
        self.robot = robot
        self.io = io
        self.on_status = on_status

        self._cond = threading.Condition()
        self._thread = None
        self._abort = False
        self._robot_paused = False
        self._current_action = ""
        self._holding_cup = False

        self.state = "STOPPED"
        self._status = {
            "order_id": "",
            "recipe_id": "",
            "order_state": "",
            "current_step": "",
            "current_step_index": 0,
            "total_steps": 0,
            "progress": 0.0,
            "message": "초기화 필요",
        }

        self._executor = ProcessExecutor(
            robot, io, wait=self._wait,
            picker=picker, checkpoint=self._checkpoint, report=self._report,
        )


    def execute(self, order_id, recipe_id):

        with self._cond:
            if self.state != "IDLE":
                return self._reject("execute")

            try:
                recipe = self.recipes.load_recipe(recipe_id)
            except Exception as exc:
                return False, f"unknown recipe: {recipe_id} ({exc})"

            self._abort = False
            self.state = "RUNNING"
            self._status.update(
                order_id=order_id,
                recipe_id=recipe_id,
                order_state="RUNNING",
                current_step="",
                current_step_index=0,
                total_steps=len(recipe["steps"]),
                progress=0.0,
                message="",
            )
            self._start_thread(self._run_recipe, recipe["steps"])

        self._emit()
        return True, "accepted"

    def pause(self):

        with self._cond:
            if self.state != "RUNNING":
                return self._reject("pause")

            self.state = "PAUSED"
            pause_robot = self._current_action in ("move", "vision_pick")
            self._cond.notify_all()

        if pause_robot:
            try:
                self.robot.pause()
            except Exception as exc:
                with self._cond:
                    if self.state == "PAUSED" and not self._abort:
                        self.state = "RUNNING"
                        self._cond.notify_all()
                return False, f"robot pause failed: {exc}"

            with self._cond:
                if self.state == "PAUSED" and not self._abort:
                    self._robot_paused = True

        self._emit()
        return True, "paused"

    def resume(self):

        with self._cond:
            if self.state == "STOPPED":
                self._abort = False
                self.state = "RESETTING"
                self._status.update(message="초기화 중")
                self._start_thread(self._run_reset)
                reset = True
            elif self.state == "PAUSED":
                reset = False
                resume_robot = self._robot_paused
            else:
                return self._reject("resume")

        if reset:
            self._emit()
            return True, "resetting"

        if resume_robot:
            try:
                self.robot.resume()
            except Exception as exc:
                return False, f"robot resume failed: {exc}"

        with self._cond:
            if self.state != "PAUSED" or self._abort:
                return self._reject("resume")
            self._robot_paused = False
            self.state = "RUNNING"
            self._cond.notify_all()

        self._emit()
        return True, "resumed"

    def force_stop(self):

        with self._cond:
            if self.state not in ("RUNNING", "PAUSED", "RESETTING"):
                return self._reject("force_stop")

            self._abort = True
            self._cond.notify_all()

        try:
            self.robot.stop()
        except Exception as exc:
            return False, f"로봇 정지 실패: {exc} — 하드웨어 비상정지를 사용하세요"

        return True, "stopping"

    def start(self):

        with self._cond:
            if self.state != "HOMED":
                return self._reject("start")

            self.state = "IDLE"
            self._status.update(message="")

        self._emit()
        return True, "started"


    def snapshot(self):

        with self._cond:
            return dict(self._status, state=self.state)

    def join(self, timeout=None):

        thread = self._thread
        if thread is not None:
            thread.join(timeout)

    def _emit(self):
        self.on_status(self.snapshot())

    def _reject(self, command):
        return False, f"{self.state}에서는 {command} 불가"

    def _start_thread(self, target, *args):
        self._thread = threading.Thread(target=target, args=args, daemon=True)
        self._thread.start()


    def _run_recipe(self, steps):

        total = len(steps)

        try:
            self.robot.clear_stop()

            for index, step in enumerate(steps):
                self._checkpoint()

                with self._cond:
                    self._current_action = step["action"]
                    self._status.update(
                        current_step=step.get("name", step["action"]),
                        current_step_index=index,
                    )
                    if step["action"] == "gripper" and step.get("command") == "close":
                        self._holding_cup = True
                    if step["action"] == "vision_pick":
                        self._holding_cup = True
                self._emit()

                self._executor.execute_step(step)

                released = step["action"] == "gripper" and step.get("command") == "open"
                if released or step["action"] == "vision_pick":
                    self._holding_cup = False

                with self._cond:
                    self._current_action = ""
                    self._status.update(progress=(index + 1) / total * 100.0)
                self._emit()

            self._checkpoint()

        except Exception as exc:
            self._finish_stopped(
                "FAILED",
                "강제 정지" if self._abort else str(exc),
            )
            return

        with self._cond:
            self.state = "IDLE"
            self._status.update(order_state="DONE", progress=100.0, message="완료")
        self._emit()

    def _run_reset(self):

        try:
            self.robot.clear_stop()
            self._checkpoint()

            if self._holding_cup:
                self._step(lambda: self.robot.move_to_pose("trash", 30.0, 30.0))
                self._step(lambda: self.io.set_gripper("open"))
                self._holding_cup = False

            self._step(lambda: self.robot.move_to_pose("home", 30.0, 30.0))

        except Exception as exc:
            self._finish_stopped(None, "강제 정지" if self._abort else str(exc))
            return

        with self._cond:
            self.state = "HOMED"
            self._status.update(message="시작 대기")
        self._emit()

    def _report(self, message):

        with self._cond:
            self._status.update(message=message)
        self._emit()

    def _step(self, call):
        self._checkpoint()
        call()
        self._checkpoint()

    def _finish_stopped(self, order_state, message):

        with self._cond:
            self.state = "STOPPED"
            self._current_action = ""
            self._robot_paused = False
            self._status.update(message=message)
            if order_state:
                self._status.update(order_state=order_state)
        self._emit()

    def _checkpoint(self):

        with self._cond:
            while self.state == "PAUSED" and not self._abort:
                self._cond.wait()

            if self._abort:
                raise _Aborted()

    def _wait(self, seconds):

        remaining = seconds

        with self._cond:
            while True:
                if self._abort:
                    raise _Aborted()

                if self.state == "PAUSED":
                    self._cond.wait()
                    continue

                if remaining <= 0:
                    return

                started = time.monotonic()
                self._cond.wait(remaining)
                remaining -= time.monotonic() - started
