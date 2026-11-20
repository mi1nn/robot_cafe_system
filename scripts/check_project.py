from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

required = [
    "cobot1_ws/src/cafe_interfaces",
    "cobot1_ws/src/robot_driver",
    "cobot1_ws/src/robot_control",
    "cobot1_ws/src/robot_monitor",
    "cobot1_ws/src/coffee_process",
    "cobot1_ws/src/io_controller",
    "cobot1_ws/src/robot_control_panel",
    "cobot1_ws/src/cafe_vision",
    "config/robot/virtual",
    "config/robot/real",
]


failed = False

for item in required:
    path = ROOT / item

    if path.exists():
        print(f"[OK] {item}")

    else:
        print(f"[FAIL] {item}")

        failed = True


raise SystemExit(1 if failed else 0)
