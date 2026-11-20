from pathlib import Path


def test_required_packages():

    root = Path(
        "cobot1_ws/src"
    )

    packages = [
        "cafe_interfaces",
        "robot_driver",
        "robot_control",
        "robot_monitor",
        "coffee_process",
        "io_controller",
        "robot_control_panel",
        "cafe_vision",
    ]

    for package in packages:

        assert (
            root / package
        ).exists()
