"""YOLO 키포인트 모델로 컵 후보를 찾는다. ultralytics는 이 파일에서만 쓴다."""

STATES = ("upright", "inverted", "lying")


class YoloCupDetector:

    def __init__(self, model_path):

        from ultralytics import YOLO

        self.model = YOLO(model_path)

    def detect(self, color_bgr):
        """컬러 영상 → 후보 목록 [{state, conf, box, rim_px, bottom_px}]."""

        result = self.model(color_bgr, verbose=False)[0]

        if result.boxes is None or result.keypoints is None:
            return []

        classes = result.boxes.cls.cpu().numpy().astype(int)
        confs = result.boxes.conf.cpu().numpy()
        boxes = result.boxes.xyxy.cpu().numpy()
        keypoints = result.keypoints.xy.cpu().numpy()

        return [
            {
                "state": STATES[cls],
                "conf": float(conf),
                "box": tuple(int(v) for v in box),
                "rim_px": tuple(float(v) for v in kpts[0]),
                "bottom_px": tuple(float(v) for v in kpts[1]),
            }
            for cls, conf, box, kpts in zip(classes, confs, boxes, keypoints)
        ]
