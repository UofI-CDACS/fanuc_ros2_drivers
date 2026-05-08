"""
Start the conveyor, then stop it 1 second after the proximity sensor triggers.

Usage:
    python3 dice/conveyor.py [--robot-name <NAME>] [--direction <forward|reverse>]

Requires the robot nodes to already be running:
    ros2 launch start.launch.py robot_name:=<NAME> robot_ip:=<IP>
"""

import argparse
import os
import threading
import time

import rclpy
from pymodbus.client import ModbusTcpClient
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fanuc_interfaces.action import Conveyor
from fanuc_interfaces.msg import ProxReadings

# Load .env from the dice/ directory if present
_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
if os.path.isfile(_env_path):
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _k, _v = _line.split("=", 1)
                os.environ.setdefault(_k.strip(), _v.strip())

ROBOT_NAME  = os.getenv("ROBOT_NAME", "")
MODBUS_HOST = os.getenv("MODBUS_HOST", "localhost")
MODBUS_PORT = int(os.getenv("MODBUS_PORT", "5020"))

_SIGNALS = {
    # DJ
    "DJ_GRIPPER_CLOSED":        0,
    "DJ_HAS_DICE":              1,
    "DJ_READY_FOR_PICTURE":     2,
    "DJ_AT_CONVEYOR":           3,
    "DJ_CONVEYOR_ACTIVE":       4,
    "DJ_CONVEYOR_DICE_READY":   5,
    # Bill
    "BILL_GRIPPER_CLOSED":      10,
    "BILL_HAS_DICE":            11,
    "BILL_READY_FOR_PICTURE":   12,
    "BILL_AT_CONVEYOR":         13,
    "BILL_CONVEYOR_ACTIVE":     14,
    "BILL_CONVEYOR_DICE_READY": 15,
    # Camera
    "CAMERA_READY":             20,
    "CAMERA_DONE":              21,
    # Shared / Safety
    "FAULT":                    29,
    "RESET":                    30,
    "CYCLE_ACTIVE":             31,
}


class _ConveyorNode(Node):

    prox_triggered = False

    def __init__(self, robot_name):
        super().__init__("conveyor_node")
        self.conveyor_ac = ActionClient(self, Conveyor, f"/{robot_name}/conveyor")
        self.create_subscription(ProxReadings, f"/{robot_name}/prox_readings",
                                 self._prox_cb, 10)

    def _prox_cb(self, msg):
        if msg.right:
            self.prox_triggered = True

    def _send(self, command):
        self.conveyor_ac.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        print(f"[conveyor] {command}")
        self.conveyor_ac.send_goal_async(goal)


def main():
    parser = argparse.ArgumentParser(
        description="Run the conveyor until the proximity sensor triggers, then stop."
    )
    parser.add_argument("--robot-name", default=ROBOT_NAME,
                        help=f"Robot namespace (default: {ROBOT_NAME!r})")
    parser.add_argument("--direction", default="forward", choices=["forward", "reverse"],
                        help="Conveyor direction (default: forward)")
    parser.add_argument("--timeout", type=float, default=30.0,
                        help="Seconds to wait for sensor before giving up (default: 30)")
    args = parser.parse_args()

    rclpy.init()
    node = _ConveyorNode(args.robot_name)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    modbus = ModbusTcpClient(MODBUS_HOST, port=MODBUS_PORT)
    if not modbus.connect():
        print(f"[modbus] Could not connect to {MODBUS_HOST}:{MODBUS_PORT}")

    print(f"Robot namespace: /{args.robot_name}")

    try:
        node._send(args.direction)

        deadline = time.time() + args.timeout
        while not node.prox_triggered and time.time() < deadline:
            time.sleep(0.05)

        if node.prox_triggered:
            print("[conveyor] Proximity sensor triggered — stopping in 1 second...")
            time.sleep(1.0)
        else:
            print("[conveyor] Timeout waiting for proximity sensor — stopping now.")

        node._send("stop")
        modbus.write_coil(_SIGNALS["DJ_CONVEYOR_DICE_READY"], True, device_id=1)
        print("[modbus] DJ_CONVEYOR_DICE_READY = 1")

    except KeyboardInterrupt:
        print("\nInterrupted — stopping conveyor.")
        node._send("stop")
    finally:
        time.sleep(0.5)  # let the stop goal send before shutting down
        modbus.close()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
