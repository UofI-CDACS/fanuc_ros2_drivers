"""
Pick up a die, rotate through its faces, and return home once the target pip is found.

Usage:
    python3 dice/pick_and_home.py <target_pip> [--robot-name <NAME>] [--modbus-host <HOST>]

    target_pip  — the pip face you want to find (1–6)

Requires the robot nodes to already be running:
    ros2 launch start.launch.py robot_name:=<NAME> robot_ip:=<IP>

Requires the Modbus server (diceSlave.py) to be running.

Face-finding strategy (covers all 6 faces in at most 3 camera shots):
  Attempt 1 — r=30 pickup, camera at R=0  → sees face A, infers 7-A
  Attempt 2 — rotate wrist 90° in hand   → sees face B, infers 7-B
  Attempt 3 — place, re-grab at r=120    → sees face C, infers 7-C

Modbus signals used by this script:
    Coil  2  (DJ_READY_FOR PICTURE) — set before each photo request, cleared after
    Coil  23 (CYCLE_ACTIVE)         — set once at startup
    Reg   1  (PIP_COUNT)            — read after signaling ready; written by camera system
"""

import argparse
import os
import sys
import threading
import time

import rclpy
from pymodbus.client import ModbusTcpClient
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper
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


# ─────────────────────────────────────────────────────────────────────────────
# USER SECTION — edit here
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_NAME  = os.getenv("ROBOT_NAME", "")
MODBUS_HOST = os.getenv("MODBUS_HOST", "localhost")
MODBUS_PORT = int(os.getenv("MODBUS_PORT", "5020"))

# Coil addresses — must match diceSlave.py SIGNALS
_SIGNALS = {
    # DJ
    "DJ_GRIPPER_CLOSED":        0,
    "DJ_HAS_DICE":              1,
    "DJ_READY_FOR_PICTURE":      2,
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
    "CAMERA_READY":     20,
    "CAMERA_DONE":      21,
    # Shared / Safety
    "FAULT":            29,
    "RESET":            30,
    "CYCLE_ACTIVE":     31,
}


# Holding register addresses — must match diceSlave.py REGISTERS
_REGISTERS = {
    "PIP_COUNT": 1,
}


def run(control, modbus, target_pip):
    modbus.reset_coils()
    find_pip(control, modbus, target_pip)


def find_pip(control, modbus, target_pip):
    """
    Pick up the die, rotate through its faces until target_pip is found,
    then place it back and return home.

    Strategy — each attempt sees one face via camera and infers the opposite
    face via (7 - pip). Three attempts cover all six faces:
      Attempt 1: gripper r=30,  camera R=0
      Attempt 2: same grip,     wrist rolled 90° in hand
      Attempt 3: gripper r=120, camera R=0  (re-grab after place)
    """
    w = 44
    print(f"\n{'═'*w}")
    print(f"  TARGET PIP: {target_pip}")
    print(f"{'═'*w}")

    modbus.set_cycle_active()

    # ── Attempt 1: r=30 pickup, camera at R=0 ────────────────────────────────
    print("\n[Attempt 1]  gripper r=30  |  camera R=0")
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_gripper(control, 'open')
    time.sleep(2)

    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)  # approach
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0,  30.0)  # lower
    send_gripper(control, 'close')
    time.sleep(2)

    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)  # lift
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)             # carry
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0, 60.0, 0.0)  # photo pos

    if _check(_photo(modbus, 1), target_pip):
        _place_target(control, modbus)
        return

    # ── Attempt 2: rotate to second camera position ───────────────────────────
    print("\n[Attempt 2]  rotating to second camera position")
    send_cartesian(control, 200.0, 1050.0, 339.0960388183594,
                   0.4133031368255615, -1.2113802433013916, 60.2224006652832)

    if _check(_photo(modbus, 2), target_pip):
        _place_target(control, modbus)
        return

    # ── Attempt 3: place die, re-grab at r=120 ────────────────────────────────
    print("\n[Attempt 3]  re-grabbing with gripper r=120")
    send_cartesian(control, 200.0, 505.0, 195.0, -90.0, 60.0, 0.0)   # roll back
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)             # carry

    # Place die
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0,  30.0)
    send_gripper(control, 'open')
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)  # rotate to r=120

    # Re-grab
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0, 120.0)  # lower
    send_gripper(control, 'close')
    time.sleep(2)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)  # lift
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)             # carry
    send_cartesian(control, 200.0, 505.0, 195.0, -90.0, 60.0, 0.0)   # photo pos

    if _check(_photo(modbus, 3), target_pip):
        _place_target(control, modbus)
    else:
        print(f"\n  WARNING: pip {target_pip} was not detected on any face of this die.")
        _return_r120(control)


def _photo(modbus, attempt_num):
    """Signal DJ_READY_FOR PICTURE, wait for PIP_COUNT from Modbus, return pip count."""
    pip = modbus.request_pip_count()
    print(f"  Modbus result (attempt {attempt_num}): top={pip}  bottom(inferred)={7 - pip}")
    return pip


def _check(pip, target_pip):
    """Return True if target_pip matches the visible face or its inferred opposite."""
    if pip == target_pip:
        print(f"  TARGET {target_pip} found on top face — done.")
        return True
    if 7 - pip == target_pip:
        print(f"  TARGET {target_pip} inferred on bottom face — done.")
        return True
    print(f"  Neither {pip} nor {7 - pip} matches target {target_pip}.")
    return False


def _place_target(control, modbus):
    """Place die at the target drop-off location (called whenever the target pip is found)."""
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)                            # home
    send_cartesian(control, -52.70402908325195, 476.11212158203125, 550.0,
                   179.9, 1.0390314855612814e-05, 30.000024795532227)               # intermediate
    send_cartesian(control, -492.0802001953125, 629.2401733398438, 332.9703063964844,
                   179.9, -0.0027465890161693096, 30.574892044067383)               # approach
    send_cartesian(control, -492.0802001953125, 629.2401733398438, 272.4888916015625,
                   179.9, -0.0027458674740046263, 30.574893951416016)               # lower
    send_gripper(control, 'open')
    send_cartesian(control, -492.0802001953125, 629.2401733398438, 332.9703063964844,
                   179.9, -0.0027465890161693096, 30.574892044067383)               # lift

    # Start conveyor and wait for proximity sensor
    modbus.set_at_conveyor(True)
    send_conveyor(control, 'forward')
    modbus.set_conveyor_active(True)

    control.prox_triggered = False
    deadline = time.time() + 30.0
    while not control.prox_triggered and time.time() < deadline:
        time.sleep(0.05)
    if not control.prox_triggered:
        print("[conveyor] Timeout waiting for proximity sensor.")

    time.sleep(1.0)
    send_conveyor(control, 'stop')
    modbus.set_conveyor_active(False)
    modbus.set_conveyor_dice_ready(True)

    # Wait for Bill to confirm it received the dice, then go home
    modbus.wait_bill_conveyor_dice_ready(timeout=60.0)
    modbus.set_at_conveyor(False)
    modbus.set_conveyor_dice_ready(False)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)                            # home


def _return_r30(control):
    """Return die to table (held at r=30) and go home."""
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0,  30.0)
    send_gripper(control, 'open')
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)


def _return_r120(control):
    """Return die to table (held at r=120) and go home."""
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0, 120.0)
    send_gripper(control, 'open')
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)


# ─────────────────────────────────────────────────────────────────────────────
# Functions — call these with your coordinates
# ─────────────────────────────────────────────────────────────────────────────

def send_cartesian(control, x, y, z, w=200.0, p=200.0, r=200.0):
    """
    Send robot to a Cartesian position.

    Args:
        control:  the control node passed into run()
        x, y, z: position in mm
        w, p, r: Yaw/Pitch/Roll in degrees (-179.0 to 179.0).
                 Leave at default (200.0) to keep the current orientation.
    """
    control.cart_ac.wait_for_server()

    goal = CartPose.Goal()
    goal.x = float(x)
    goal.y = float(y)
    goal.z = float(z)
    goal.w = float(w)
    goal.p = float(p)
    goal.r = float(r)

    print(f"\n[cartesian] X={x}  Y={y}  Z={z}  W={w}  P={p}  R={r}")
    future = control.cart_ac.send_goal_async(goal, feedback_callback=control._feedback_cb)
    return control._wait(future)


def send_gripper(control, command):
    """
    Open or close the Schunk gripper.

    Args:
        control: the control node passed into run()
        command: 'open' or 'close'
    """
    control.schunk_ac.wait_for_server()

    goal = SchunkGripper.Goal()
    goal.command = command

    print(f"\n[gripper] {command}")
    future = control.schunk_ac.send_goal_async(goal)
    return control._wait(future)


def send_conveyor(control, command):
    """
    Start or stop the conveyor. Does not block — the conveyor runs until stopped.

    Args:
        control: the control node passed into run()
        command: 'forward', 'reverse', or 'stop'
    """
    control.conveyor_ac.wait_for_server()
    goal = Conveyor.Goal()
    goal.command = command
    print(f"\n[conveyor] {command}")
    control.conveyor_ac.send_goal_async(goal)


def send_joint(control, j1, j2, j3, j4, j5, j6):
    """
    Send robot to a Joint pose.

    Args:
        control: the control node passed into run()
        j1–j6:  joint angles in degrees (-179.0 to 179.0)
    """
    control.joint_ac.wait_for_server()

    goal = JointPose.Goal()
    goal.joint1 = float(j1)
    goal.joint2 = float(j2)
    goal.joint3 = float(j3)
    goal.joint4 = float(j4)
    goal.joint5 = float(j5)
    goal.joint6 = float(j6)

    print(f"\n[joint] J1={j1}  J2={j2}  J3={j3}  J4={j4}  J5={j5}  J6={j6}")
    future = control.joint_ac.send_goal_async(goal, feedback_callback=control._feedback_cb)
    return control._wait(future)


# ─────────────────────────────────────────────────────────────────────────────
# Internals — no need to edit below here
# ─────────────────────────────────────────────────────────────────────────────

class _ModbusInterface:
    """Synchronous Modbus client for signaling the camera system and reading pip counts."""

    def __init__(self, host, port):
        self._client = ModbusTcpClient(host, port=port)
        if not self._client.connect():
            raise ConnectionError(f"Could not connect to Modbus server at {host}:{port}")
        print(f"[modbus] Connected to {host}:{port}")

    def reset_coils(self):
        """Clear all coils and registers owned by this script before a new run."""
        self._client.write_coil(_SIGNALS["DJ_READY_FOR_PICTURE"],   False, device_id=1)
        self._client.write_coil(_SIGNALS["DJ_AT_CONVEYOR"],         False, device_id=1)
        self._client.write_coil(_SIGNALS["DJ_CONVEYOR_ACTIVE"],     False, device_id=1)
        self._client.write_coil(_SIGNALS["DJ_CONVEYOR_DICE_READY"], False, device_id=1)
        self._client.write_coil(_SIGNALS["CYCLE_ACTIVE"],           False, device_id=1)
        self._client.write_register(_REGISTERS["PIP_COUNT"], 0, device_id=1)
        print("[modbus] Coils reset.")

    def set_cycle_active(self):
        """Write CYCLE_ACTIVE coil to True."""
        self._client.write_coil(_SIGNALS["CYCLE_ACTIVE"], True, device_id=1)
        print("[modbus] CYCLE_ACTIVE = 1")

    def request_pip_count(self, timeout=30.0):
        """
        Assert DJ_READY_FOR PICTURE and poll until the camera system writes a
        nonzero value back into PIP_COUNT.

        Returns the pip count, or 0 on timeout.
        """
        self._client.write_register(_REGISTERS["PIP_COUNT"], 0, device_id=1)
        self._client.write_coil(_SIGNALS["DJ_READY_FOR_PICTURE"], True, device_id=1)
        print("[modbus] DJ_READY_FOR PICTURE = 1 — waiting for PIP_COUNT...")

        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._client.read_holding_registers(
                _REGISTERS["PIP_COUNT"], count=1, device_id=1
            )
            if not result.isError() and result.registers[0] != 0:
                pip = result.registers[0]
                self._client.write_coil(_SIGNALS["DJ_READY_FOR_PICTURE"], False, device_id=1)
                print(f"[modbus] PIP_COUNT = {pip}")
                return pip
            time.sleep(0.2)

        print("[modbus] Timeout waiting for PIP_COUNT.")
        self._client.write_coil(_SIGNALS["DJ_READY_FOR_PICTURE"], False, device_id=1)
        return 0

    def set_at_conveyor(self, value):
        self._client.write_coil(_SIGNALS["DJ_AT_CONVEYOR"], value, device_id=1)
        print(f"[modbus] DJ_AT_CONVEYOR = {int(value)}")

    def set_conveyor_active(self, value):
        self._client.write_coil(_SIGNALS["DJ_CONVEYOR_ACTIVE"], value, device_id=1)
        print(f"[modbus] DJ_CONVEYOR_ACTIVE = {int(value)}")

    def set_conveyor_dice_ready(self, value):
        self._client.write_coil(_SIGNALS["DJ_CONVEYOR_DICE_READY"], value, device_id=1)
        print(f"[modbus] DJ_CONVEYOR_DICE_READY = {int(value)}")

    def wait_bill_conveyor_dice_ready(self, timeout=60.0):
        """Poll until BILL_CONVEYOR_DICE_READY is True or timeout."""
        print("[modbus] Waiting for BILL_CONVEYOR_DICE_READY...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._client.read_coils(_SIGNALS["BILL_CONVEYOR_DICE_READY"], count=1, device_id=1)
            if not result.isError() and result.bits[0]:
                print("[modbus] BILL_CONVEYOR_DICE_READY = 1")
                return True
            time.sleep(0.2)
        print("[modbus] Timeout waiting for BILL_CONVEYOR_DICE_READY.")
        return False

    def close(self):
        self._client.close()


class _ControlNode(Node):
    """ROS2 node for robot arm motion."""

    prox_triggered = False

    def __init__(self, robot_name):
        super().__init__("control_node")
        self.cart_ac     = ActionClient(self, CartPose,      f"/{robot_name}/cartesian_pose")
        self.joint_ac    = ActionClient(self, JointPose,     f"/{robot_name}/joint_pose")
        self.schunk_ac   = ActionClient(self, SchunkGripper, f"/{robot_name}/schunk_gripper")
        self.conveyor_ac = ActionClient(self, Conveyor,      f"/{robot_name}/conveyor")
        self.create_subscription(ProxReadings, f"/{robot_name}/prox_readings",
                                 self._prox_cb, 10)

    def _prox_cb(self, msg):
        if msg.right:
            self.prox_triggered = True

    def _wait(self, send_future):
        while not send_future.done():
            time.sleep(0.05)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            print("  Goal REJECTED.")
            return False

        print("  Goal accepted — moving...")
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.05)

        success = result_future.result().result.success
        print("  Done." if success else "  Finished — server reported failure.")
        return success

    def _feedback_cb(self, feedback_msg):
        vals = [f"{v:.2f}" for v in feedback_msg.feedback.distance_left]
        print(f"  distance_left: [{', '.join(vals)}]", end="\r")


def main():
    parser = argparse.ArgumentParser(
        description="Pick up a die and find a target pip face, then return home."
    )
    parser.add_argument("target_pip", type=int, choices=range(1, 7),
                        metavar="target_pip",
                        help="Pip face to find (1–6)")
    parser.add_argument("--robot-name", default=ROBOT_NAME,
                        help=f"Robot namespace (default: {ROBOT_NAME!r})")
    parser.add_argument("--modbus-host", default=MODBUS_HOST,
                        help=f"Modbus server host (default: {MODBUS_HOST!r})")
    args = parser.parse_args()

    rclpy.init()
    control = _ControlNode(args.robot_name)
    modbus  = _ModbusInterface(args.modbus_host, MODBUS_PORT)

    executor = MultiThreadedExecutor()
    executor.add_node(control)
    threading.Thread(target=executor.spin, daemon=True).start()

    print(f"Robot namespace: /{args.robot_name}\n")
    try:
        run(control, modbus, args.target_pip)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        executor.shutdown()
        control.destroy_node()
        rclpy.shutdown()
        modbus.close()


if __name__ == "__main__":
    main()
