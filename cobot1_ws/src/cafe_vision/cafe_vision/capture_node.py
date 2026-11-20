"""학습용 사진 촬영: 카메라 영상을 보여주고 s 키마다 jpg 로 저장한다 (q 종료).

실제로 탐지할 때와 같은 cup_look 자세에서 찍어야 학습한 모델이 잘 맞는다.
"""

import datetime
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CaptureNode(Node):

    def __init__(self):

        super().__init__("cafe_vision_capture")

        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("out_dir", os.path.expanduser("~/cafe_vision_dataset/raw"))

        self.out_dir = os.path.expanduser(self.get_parameter("out_dir").value)
        os.makedirs(self.out_dir, exist_ok=True)

        self.bridge = CvBridge()
        self.color = None
        self.saved = 0

        self.create_subscription(Image, self.get_parameter("color_topic").value, self.cb_color, 10)
        self.get_logger().info(f"영상 대기 중... (s: 저장, q: 종료, 저장 위치: {self.out_dir})")

    def cb_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def save(self):

        name = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".jpg"
        cv2.imwrite(os.path.join(self.out_dir, name), self.color)
        self.saved += 1
        self.get_logger().info(f"저장 {self.saved}장: {name}")


def main(args=None):

    rclpy.init(args=args)
    node = CaptureNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)

            if node.color is None:
                continue

            cv2.imshow("cafe_vision capture (s: save, q: quit)", node.color)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("s"):
                node.save()
            elif key == ord("q"):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
