import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

for package in ("robot_driver", "robot_control", "coffee_process", "cafe_vision"):
    sys.path.insert(0, str(ROOT / "cobot1_ws" / "src" / package))

try:
    import websocket
except ImportError:
    import types

    stub = types.ModuleType("websocket")

    class WebSocketException(Exception):
        pass

    def create_connection(*args, **kwargs):
        raise ConnectionRefusedError("websocket-client not installed (test stub)")

    stub.WebSocketException = WebSocketException
    stub.create_connection = create_connection
    sys.modules["websocket"] = stub
