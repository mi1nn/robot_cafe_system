class MotionManager:

    def __init__(
        self,
        robot_driver,
        poses,
        workspace=None,
    ):

        self.robot = robot_driver
        self.poses = poses
        self.workspace = workspace

    def move_cartesian(
        self,
        pose,
        velocity=30.0,
        acceleration=30.0,
        check_only=False,
    ):

        # 비전 등 외부에서 온 좌표는 작업 영역 안에서만 움직인다.
        if not self.workspace:
            raise ValueError("No workspace defined; refusing cartesian move")

        for axis, value in zip(("x", "y", "z"), pose[:3]):
            low, high = self.workspace[axis]
            if not low <= value <= high:
                raise ValueError(
                    f"Target outside workspace: {axis}={value} not in [{low}, {high}]"
                )

        if check_only:
            return None

        return self.robot.move_line(
            list(pose),
            velocity,
            acceleration,
        )

    def move_to_pose(
        self,
        pose_name,
        velocity=30.0,
        acceleration=30.0,
        motion_type=None,
    ):

        if pose_name not in self.poses:
            raise KeyError(f"Unknown pose: {pose_name}")

        pose = self.poses[pose_name]

        available_types = [key for key in ("joint", "cartesian") if key in pose]

        if not available_types:
            raise ValueError(f"Pose '{pose_name}' has no joint/cartesian values")

        if motion_type:

            if motion_type not in pose:
                raise ValueError(
                    f"Pose '{pose_name}' has no '{motion_type}' values "
                    f"(available: {available_types})"
                )

            selected_type = motion_type

        elif len(available_types) == 1:

            selected_type = available_types[0]

        else:
            raise ValueError(
                f"Pose '{pose_name}' has multiple motion types "
                f"{available_types}; specify motion_type explicitly"
            )

        values = pose[selected_type]

        if selected_type == "joint":

            return self.robot.move_joint(
                values,
                velocity,
                acceleration,
            )

        if selected_type == "cartesian":

            return self.robot.move_line(
                values,
                velocity,
                acceleration,
            )

        raise ValueError(f"Unknown pose type: {selected_type}")

    def move_home(self):

        return self.robot.move_home()

    def stop(self):

        return self.robot.stop()

    def pause(self):

        return self.robot.pause()

    def resume(self):

        return self.robot.resume()

    def clear_stop(self):

        return self.robot.clear_stop()
