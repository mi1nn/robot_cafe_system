class ProcessExecutor:

    def __init__(
        self,
        robot_control,
        io_manager,
        wait,
        picker=None,
        checkpoint=None,
        report=None,
    ):

        self.robot = robot_control
        self.io = io_manager
        self.wait = wait
        self.picker = picker
        self.checkpoint = checkpoint
        self.report = report

    def execute_step(
        self,
        step,
    ):

        action = step["action"]

        if action == "move":

            return self.robot.move_to_pose(
                step["pose"],
                step.get(
                    "velocity",
                    30.0,
                ),
                step.get(
                    "acceleration",
                    30.0,
                ),
            )

        if action == "gripper":
            return self.io.set_gripper(step["command"])

        if action == "io":
            return self.io.set_output(
                step["output"],
                step["value"],
            )

        if action == "wait":
            return self.wait(float(step["seconds"]))

        if action == "vision_pick":

            if self.picker is None:
                raise RuntimeError("vision_pick: grasp_file 파라미터가 없어 CupPicker가 없습니다")

            return self.picker.run(self.checkpoint, self.report)

        if action == "home":
            return self.robot.move_to_pose("home", 30.0, 30.0)

        raise ValueError(f"Unknown process action: " f"{action}")
