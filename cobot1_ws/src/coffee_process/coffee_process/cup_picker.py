"""vision_pick 단계: 컵을 찾아 집고, 바로 세워 cup_station에 내려놓는다.

상태별 접근/운반 자세와 높이는 config/robot/<mode>/cup_grasp.yaml 에 있다.
이 파일은 그 값을 순서대로 이어 붙일 뿐이다.
"""

import math


def plan_pick(detection, cfg):
    """탐지 결과 → [("move", posx[6]) | ("gripper", "open"/"close")] 10단계."""

    state_cfg = cfg["states"][detection["state"]]
    lift = [0.0, 0.0, cfg["approach_height"]]

    grasp = _add(detection["position"], state_cfg["grasp_offset"])
    station = list(cfg["station_position"])

    approach = list(state_cfg["approach_orientation"])

    if detection["state"] == "lying":
        approach[0] += detection["yaw"]
        rim_forward = math.cos(math.radians(detection["rim_direction"] - detection["yaw"])) > 0
        carry = state_cfg["carry_orientation_rim_forward" if rim_forward else "carry_orientation_rim_backward"]
    else:
        carry = state_cfg["carry_orientation"]

    carry = list(carry)

    return [
        ("gripper", "open"),
        ("move", _add(grasp, lift) + approach),
        ("move", grasp + approach),
        ("gripper", "close"),
        ("move", _add(grasp, lift) + approach),
        ("move", _add(grasp, lift) + carry),
        ("move", _add(station, lift) + carry),
        ("move", station + carry),
        ("gripper", "open"),
        ("move", _add(station, lift) + carry),
    ]


def _add(a, b):
    return [float(x) + float(y) for x, y in zip(a, b)]


class CupPicker:

    def __init__(self, robot, io, vision, cfg, retries=3):

        self.robot = robot
        self.io = io
        self.vision = vision
        self.cfg = cfg
        self.retries = retries

    def run(self, checkpoint, report):

        velocity, acceleration = self.cfg["velocity"], self.cfg["acceleration"]

        checkpoint()
        self.robot.move_to_pose("cup_look", velocity, acceleration)

        detection = self._detect(checkpoint)
        report(f"컵 탐지: {detection['state']}")

        for kind, value in plan_pick(detection, self.cfg):
            checkpoint()
            if kind == "gripper":
                self.io.set_gripper(value)
            else:
                self.robot.move_cartesian(value, velocity, acceleration)

    def _detect(self, checkpoint):

        last = ""

        for _ in range(self.retries):
            checkpoint()
            try:
                result = self.vision.detect_cup()
            except Exception as exc:
                last = str(exc)
                continue

            if result["found"]:
                return result
            last = result["message"]

        raise RuntimeError(f"컵을 찾지 못했습니다: {last}")
