"""체커보드 핸드-아이 캘리브레이션 (대화형). 이 노드는 로봇을 움직이지 않는다.

사람이 로봇을 옮겨 보드가 보이게 한 뒤 Enter → 샘플 기록, q → 계산·검증·저장.
결과: output_file (link_6 → camera_color_optical_frame), 원본 샘플: sample_dir.
"""

import datetime
import os
import threading

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from .geometry import tf_to_matrix
from .hand_eye import board_spread, find_board_pose, save_extrinsics, solve_hand_eye


class CalibrateNode(Node):

    def __init__(self):

        super().__init__("cafe_vision_calibrate")

        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        params = {
            "board_file": os.path.join(get_package_share_directory("cafe_vision"), "calibration", "board.yaml"),
            "output_file": "",
            "sample_dir": os.path.join("log", "calibration", stamp),
            "color_topic": "/camera/camera/color/image_raw",
            "info_topic": "/camera/camera/color/camera_info",
            "base_frame": "base_link",
            "flange_frame": "link_6",
            "min_samples": 15,
            "max_spread_mm": 5.0,
        }
        for name, default in params.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in params}

        if not self.p["output_file"]:
            raise RuntimeError(
                "'output_file' 파라미터가 필요합니다. 예: --ros-args "
                "-p output_file:=$PWD/config/robot/real/camera_extrinsics.yaml"
            )

        with open(self.p["board_file"], "r", encoding="utf-8") as file:
            self.board = yaml.safe_load(file)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.color = None
        self.K = self.dist = None
        self.flanges, self.boards = [], []

        self.create_subscription(Image, self.p["color_topic"], self.cb_color, 10)
        self.create_subscription(CameraInfo, self.p["info_topic"], self.cb_info, 10)

    def cb_color(self, msg):
        self.color = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def cb_info(self, msg):
        self.K = np.array(msg.k, dtype=float).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=float)

    def capture(self):
        """현재 영상과 손목 위치로 샘플 하나를 기록한다. 결과 메시지를 돌려준다."""

        if self.color is None or self.K is None:
            return "영상이 아직 없습니다 (카메라 토픽 확인)"

        gray = cv2.cvtColor(self.color, cv2.COLOR_BGR2GRAY)
        T_cam_board = find_board_pose(
            gray, self.board["cols"], self.board["rows"], self.board["square_mm"], self.K, self.dist
        )
        if T_cam_board is None:
            return "체커보드를 찾지 못했습니다. 보드 전체가 보이게 다시 찍으세요"

        try:
            tf = self.tf_buffer.lookup_transform(self.p["base_frame"], self.p["flange_frame"], Time())
        except Exception as exc:
            return f"TF 조회 실패: {exc}"

        t, q = tf.transform.translation, tf.transform.rotation
        T_base_flange = tf_to_matrix([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])

        self.flanges.append(T_base_flange)
        self.boards.append(T_cam_board)

        index = len(self.flanges)
        os.makedirs(self.p["sample_dir"], exist_ok=True)
        cv2.imwrite(os.path.join(self.p["sample_dir"], f"{index:02d}.png"), self.color)
        np.savez(
            os.path.join(self.p["sample_dir"], f"{index:02d}.npz"),
            T_base_flange=T_base_flange, T_cam_board=T_cam_board,
        )

        return f"샘플 {index}개 기록 (목표 {self.p['min_samples']}개 이상, 각도를 다양하게)"

    def finish(self):
        """충분하면 계산, 검증, 저장하고 True. 샘플이 부족하면 False."""

        count = len(self.flanges)
        if count < self.p["min_samples"]:
            print(f"샘플이 {count}개입니다. {self.p['min_samples']}개 이상 찍어 주세요.")
            return False

        X = solve_hand_eye(self.flanges, self.boards)
        spread_mm, spread_deg = board_spread(self.flanges, self.boards, X)

        print(f"결과 X(link_6→camera) 위치 mm: {np.round(X[:3, 3], 2)}")
        print(f"검증: 보드 위치 편차 {spread_mm:.2f} mm, 회전 편차 {spread_deg:.2f} deg")
        if spread_mm > self.p["max_spread_mm"]:
            print(f"경고: 편차가 기준({self.p['max_spread_mm']} mm)보다 큽니다. 샘플을 다시 찍는 것을 권장합니다.")

        save_extrinsics(self.p["output_file"], X, count, spread_mm, spread_deg)
        print(f"저장: {self.p['output_file']}  (원본 샘플: {self.p['sample_dir']})")
        return True


def main(args=None):

    rclpy.init(args=args)
    node = CalibrateNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()

    print("핸드-아이 캘리브레이션: 로봇을 옮겨 체커보드가 보이게 한 뒤 Enter (기록) / q (계산·저장)")
    print("영상 대기 중...")

    try:
        while True:
            command = input("> ").strip().lower()
            if command == "q":
                if node.finish():
                    break
            else:
                print(node.capture())
    except (KeyboardInterrupt, EOFError):
        print()
    finally:
        executor.shutdown()
        spinner.join(2)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
