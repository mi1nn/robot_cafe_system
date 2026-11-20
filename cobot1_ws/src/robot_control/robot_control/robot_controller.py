class RobotController:

    def __init__(
        self,
        robot_driver,
        motion_manager,
        safety_manager,
    ):
        self.robot = robot_driver
        self.motion = motion_manager
        self.safety = safety_manager

    def move_to_pose(
        self,
        pose_name,
        velocity=30.0,
        acceleration=30.0,
        motion_type=None,
    ):
        return self.motion.move_to_pose(
            pose_name,
            velocity,
            acceleration,
            motion_type,
        )

    def move_cartesian(self, pose, velocity=30.0, acceleration=30.0):
        return self.motion.move_cartesian(pose, velocity, acceleration)

    def move_home(self):
        return self.motion.move_home()

    def stop(self):
        return self.motion.stop()

    def pause(self):

        return self.motion.pause()

    def resume(self):

        return self.motion.resume()

    def clear_stop(self):

        return self.motion.clear_stop()
