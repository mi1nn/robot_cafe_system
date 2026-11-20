#!/usr/bin/env python3

import rclpy
import yaml
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from cafe_interfaces.msg import ProcessStatus
from cafe_interfaces.srv import ExecuteRecipe

from .process_runner import ProcessRunner
from .recipe_manager import RecipeManager
from .cup_picker import CupPicker
from .ros_clients import IOClient, RobotClient, VisionClient


class CoffeeProcessNode(Node):

    def __init__(self):

        super().__init__("coffee_process")

        recipe_dir = self._required_parameter("recipe_dir", "/path/to/recipes")
        io_file = self._required_parameter("io_file", "/path/to/io.yaml")

        with open(io_file, "r", encoding="utf-8") as file:
            channels = yaml.safe_load(file)["outputs"]

        self.status_publisher = self.create_publisher(
            ProcessStatus, "/coffee_process/status", 10
        )

        group = ReentrantCallbackGroup()
        client_group = MutuallyExclusiveCallbackGroup()

        robot = RobotClient(self, client_group)
        io = IOClient(self, client_group, channels)

        self.runner = ProcessRunner(
            RecipeManager(recipe_dir),
            robot,
            io,
            on_status=self.publish_status,
            picker=self._make_picker(robot, io, client_group),
        )

        self.create_service(
            ExecuteRecipe, "/coffee_process/execute_recipe",
            self.cb_execute_recipe, callback_group=group,
        )

        for command in ("pause", "resume", "force_stop", "start"):
            self.create_service(
                Trigger, f"/coffee_process/{command}",
                self._trigger_callback(getattr(self.runner, command)),
                callback_group=group,
            )

        self.create_timer(
            1.0, lambda: self.publish_status(self.runner.snapshot()), callback_group=group
        )

        self.get_logger().info(f"Coffee process ready (recipes: {recipe_dir})")
        self.get_logger().info(
            "  /coffee_process/execute_recipe, pause, resume, force_stop, start"
        )
        self.publish_status(self.runner.snapshot())

    def _make_picker(self, robot, io, client_group):
        """grasp_file 이 있으면 vision_pick 단계용 CupPicker 를 만든다 (없으면 vision_pick 단계가 실패)."""

        self.declare_parameter("grasp_file", "")
        grasp_file = self.get_parameter("grasp_file").get_parameter_value().string_value

        if not grasp_file:
            self.get_logger().warn("grasp_file 없음: vision_pick 단계를 쓸 수 없습니다")
            return None

        with open(grasp_file, "r", encoding="utf-8") as file:
            cfg = yaml.safe_load(file)

        self.get_logger().info(f"Cup grasp config: {grasp_file}")
        return CupPicker(robot, io, VisionClient(self, client_group), cfg)

    def _required_parameter(self, name, example):

        self.declare_parameter(name, "")
        value = self.get_parameter(name).get_parameter_value().string_value

        if not value:
            raise RuntimeError(
                f"'{name}' 파라미터가 필요합니다. 예: "
                "ros2 run coffee_process coffee_process_node "
                f"--ros-args -p {name}:={example}"
            )

        return value

    def publish_status(self, status):

        msg = ProcessStatus()

        msg.order_id = status["order_id"]
        msg.recipe_id = status["recipe_id"]
        msg.state = status["state"]
        msg.order_state = status["order_state"]
        msg.current_step = status["current_step"]
        msg.current_step_index = int(status["current_step_index"])
        msg.total_steps = int(status["total_steps"])
        msg.progress = float(status["progress"])
        msg.message = status["message"]

        self.status_publisher.publish(msg)

    def cb_execute_recipe(self, request, response):

        response.accepted, response.message = self.runner.execute(
            request.order_id, request.recipe_id
        )
        self.get_logger().info(
            f"execute {request.order_id}/{request.recipe_id}: {response.message}"
        )
        return response

    def _trigger_callback(self, command):

        def callback(request, response):
            response.success, response.message = command()
            self.get_logger().info(f"{command.__name__}: {response.message}")
            return response

        return callback


def main(args=None):

    rclpy.init(args=args)

    node = CoffeeProcessNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
