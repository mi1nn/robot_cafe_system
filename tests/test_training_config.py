import yaml

from cafe_vision.detector import STATES


def test_dataset_config_matches_vision_constants():
    cfg = yaml.safe_load(open("cobot1_ws/src/cafe_vision/training/cup.yaml"))
    assert cfg["kpt_shape"] == [2, 3]
    assert cfg["flip_idx"] == [0, 1]
    assert cfg["names"] == {0: "upright", 1: "inverted", 2: "lying"}
    assert tuple(cfg["names"][i] for i in range(3)) == STATES
