import yaml

import rclpy

from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger

from cafe_interfaces.srv import MoveCartesian, MoveNamedPose, StopProcess

from .driver_client import DriverActionClient
from .robot_controller import RobotController
from .motion_manager import MotionManager
from .safety_manager import SafetyManager


class RobotControlNode(Node):

    def __init__(self):

        super().__init__("robot_control")

        self.declare_parameter("poses_file", "")

        poses_file = self.get_parameter("poses_file").get_parameter_value().string_value

        if not poses_file:
            raise RuntimeError(
                "'poses_file' 파라미터가 필요합니다. 예: "
                "ros2 run robot_control robot_control_node "
                "--ros-args -p poses_file:=/path/to/poses.yaml"
            )

        with open(poses_file, "r", encoding="utf-8") as file:
            config = yaml.safe_load(file)

        poses = config["poses"]
        workspace = config.get("workspace")

        self.get_logger().info(f"Loaded poses: {poses_file}")

        self.driver = DriverActionClient(self)
        self.motion = MotionManager(self.driver, poses, workspace)
        self.safety = SafetyManager()

        self.controller = RobotController(
            self.driver,
            self.motion,
            self.safety,
        )

        self.service_group = ReentrantCallbackGroup()

        self.move_named_pose_srv = self.create_service(
            MoveNamedPose,
            "/robot_control/move_named_pose",
            self.cb_move_named_pose,
            callback_group=self.service_group,
        )

        self.move_cartesian_srv = self.create_service(
            MoveCartesian,
            "/robot_control/move_cartesian",
            self.cb_move_cartesian,
            callback_group=self.service_group,
        )

        self.stop_process_srv = self.create_service(
            StopProcess,
            "/robot_control/stop_process",
            self.cb_stop_process,
            callback_group=self.service_group,
        )

        self.pause_srv = self.create_service(
            Trigger, "/robot_control/pause", self.cb_pause, callback_group=self.service_group
        )

        self.clear_stop_srv = self.create_service(
            Trigger, "/robot_control/clear_stop", self.cb_clear_stop, callback_group=self.service_group
        )

        self.resume_srv = self.create_service(
            Trigger, "/robot_control/resume", self.cb_resume, callback_group=self.service_group
        )

        self.get_logger().info("Robot control services ready")
        self.get_logger().info("  /robot_control/move_named_pose, /robot_control/move_cartesian")
        self.get_logger().info("  /robot_control/stop_process")
        self.get_logger().info("  /robot_control/pause, /robot_control/resume")

    def cb_move_named_pose(self, request, response):

        try:
            self.controller.move_to_pose(
                request.pose_name,
                request.velocity,
                request.acceleration,
                request.motion_type or None,
            )

            response.success = True
            response.message = f"Moved to {request.pose_name}"

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def cb_move_cartesian(self, request, response):

        try:
            self.controller.move_cartesian(
                list(request.pose),
                request.velocity,
                request.acceleration,
                request.check_only,
            )

            response.success = True
            response.message = "Target inside workspace" if request.check_only else "Moved to cartesian target"

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response

    def cb_stop_process(self, request, response):

        try:
            self.controller.stop()

            response.success = True
            response.message = "Stopped"

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response


    def cb_pause(self, request, response):
        return self._trigger(self.controller.pause, "Paused", response)

    def cb_clear_stop(self, request, response):
        return self._trigger(self.controller.clear_stop, "Stop cleared", response)

    def cb_resume(self, request, response):
        return self._trigger(self.controller.resume, "Resumed", response)

    def _trigger(self, action, message, response):

        try:
            action()
            response.success = True
            response.message = message

        except Exception as exc:
            response.success = False
            response.message = str(exc)

        return response


def main(args=None):

    rclpy.init(args=args)

    node = RobotControlNode()

    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()
