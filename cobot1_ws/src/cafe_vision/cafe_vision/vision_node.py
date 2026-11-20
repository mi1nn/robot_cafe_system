"""/vision/detect_cup 서비스: 손목 카메라로 컵을 찾아 로봇 base 기준 집는 점을 돌려준다.

- mock 모드 (mock_file 지정): 카메라, 모델, TF 없이 yaml 값을 그대로 돌려준다 (시뮬레이션).
- 실제 모드: YOLO 추론 → 신뢰도 최고 후보 → 깊이로 3D → TF(base→link_6) · X(link_6→카메라).
"""

import os
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from cafe_interfaces.srv import DetectCup

from .detection import build_response, load_mock, not_found
from .geometry import choose_best, tf_to_matrix

FRAME_TIMEOUT = 1.0


class VisionNode(Node):

    def __init__(self):

        super().__init__("cafe_vision")

        params = {
            "mock_file": "",
            "model_path": "",
            "extrinsics_file": "",
            "confidence_threshold": 0.5,
            "color_topic": "/camera/camera/color/image_raw",
            "depth_topic": "/camera/camera/aligned_depth_to_color/image_raw",
            "info_topic": "/camera/camera/color/camera_info",
            "base_frame": "base_link",
            "flange_frame": "link_6",
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in params}

        group = ReentrantCallbackGroup()
        self.create_service(DetectCup, "/vision/detect_cup", self.cb_detect, callback_group=group)

        if self.p["mock_file"]:
            self.mock = load_mock(self.p["mock_file"])
            self.get_logger().info(f"mock 모드: {self.p['mock_file']}")
            return

        self.mock = None
        self._setup_camera(group)

    def _setup_camera(self, group):

        from cv_bridge import CvBridge
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros import Buffer, TransformListener

        from .detector import YoloCupDetector
        from .hand_eye import load_extrinsics

        for key in ("model_path", "extrinsics_file"):
            if not os.path.isfile(self.p[key]):
                raise RuntimeError(
                    f"{key} 파일이 없습니다: '{self.p[key]}' "
                    "(cobot1_ws/src/cafe_vision/README.md 의 학습/캘리브레이션 절 참고)"
                )

        self.detector = YoloCupDetector(self.p["model_path"])
        self.X = load_extrinsics(self.p["extrinsics_file"])
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.color = self.depth = self.K = None
        self.color_time = 0.0

        self.create_subscription(Image, self.p["color_topic"], self.cb_color, 10, callback_group=group)
        self.create_subscription(Image, self.p["depth_topic"], self.cb_depth, 10, callback_group=group)
        self.create_subscription(CameraInfo, self.p["info_topic"], self.cb_info, 10, callback_group=group)

        self.get_logger().info(f"실제 모드: model={self.p['model_path']}")

    def cb_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_time = time.monotonic()

    def cb_depth(self, msg):
        self.depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def cb_info(self, msg):
        self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def cb_detect(self, request, response):

        result = self.mock if self.mock is not None else self._detect()

        for key, value in result.items():
            setattr(response, key, value)

        self.get_logger().info(f"detect_cup: {result['message']}")
        return response

    def _detect(self):

        requested = time.monotonic()
        while self.color_time <= requested:
            if time.monotonic() - requested > FRAME_TIMEOUT:
                return not_found("카메라 영상이 들어오지 않습니다")
            time.sleep(0.02)

        if self.depth is None or self.K is None:
            return not_found("깊이 영상 또는 camera_info가 없습니다")

        try:
            tf = self.tf_buffer.lookup_transform(self.p["base_frame"], self.p["flange_frame"], Time())
        except Exception as exc:
            return not_found(f"TF 조회 실패: {exc}")

        t, q = tf.transform.translation, tf.transform.rotation
        T_base_cam = tf_to_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]) @ self.X

        candidate = choose_best(self.detector.detect(self.color), self.p["confidence_threshold"])
        return build_response(candidate, self.depth, self.K, T_base_cam)


def main(args=None):

    rclpy.init(args=args)
    node = VisionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
