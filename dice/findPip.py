"""
findPip.py — DJ robot: find pips 1, 3, and 5 in sequence, handing each off to Bill.

Overall flow:
  1. Pick up die from table, find pip 1  →  send to back conveyor
     (Bill picks up, finds pip 2, sends die back via front conveyor)
  2. Wait for BILL_CONVEYOR_DICE_READY  →  pick up from front conveyor
     Find pip 3  →  send to back conveyor
     (Bill picks up, finds pip 4, sends die back)
  3. Wait for BILL_CONVEYOR_DICE_READY  →  pick up from front conveyor
     Find pip 5  →  send to back conveyor  (final handoff)

Usage:
    python3 dice/findPip.py [--robot-name <NAME>] [--modbus-host <HOST>]

Requires:
  - Robot nodes: ros2 launch start.launch.py robot_name:=<NAME> robot_ip:=<IP>
  - Modbus server (diceSlave.py) running
  - Bill's script running on the partner robot
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

from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper
from fanuc_interfaces.msg import ProxReadings

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

_REGISTERS = {
    "TARGET_NUMBER":        0,
    "PIP_COUNT":            1,
    "DJ_NUMBER_OF_TRIES":   2,
    "BILL_NUMBER_OF_TRIES": 3,
}

# ── Positions ──────────────────────────────────────────────────────────────────

# Table pickup (initial die, and temporary re-placement for Attempt 3)
_TABLE_APPROACH = (630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
_TABLE_PICKUP   = (630.0, -10.0,  65.0, 179.9, 0.0,  30.0)

# Camera positions
_CAMERA_POS_1    = (200.0, 1050.0, 195.0,           -90.0,  60.0,  0.0)
_CAMERA_POS_2    = (200.0, 1050.0, 339.0960388183594, 0.4133031368255615,
                    -1.2113802433013916, 60.2224006652832)
_CAMERA_ROLLBACK = (200.0,  505.0, 195.0,           -90.0,  60.0,  0.0)

# Back conveyor drop-off (DJ → Bill)
_BACK_CONV_INTERMEDIATE = (-52.70402908325195, 476.11212158203125, 550.0,
                            179.9, 1.0390314855612814e-05, 30.000024795532227)
_BACK_CONV_APPROACH     = (-492.0802001953125, 629.2401733398438, 332.9703063964844,
                            179.9, -0.0027465890161693096, 30.574892044067383)
_BACK_CONV_LOWER        = (-492.0802001953125, 629.2401733398438, 272.4888916015625,
                            179.9, -0.0027458674740046263, 30.574893951416016)

# Front conveyor pickup (die returned by Bill → DJ)
_FRONT_CONV_APPROACH = (-218.40814208984375, 671.5690307617188, 318.7359313964844,
                         179.9, -6.363753072946565e-06, 29.99998664855957)
_FRONT_CONV_PICKUP   = (-218.4081268310547,  671.5690307617188, 262.86444091796875,
                         179.9, -6.152979494800093e-06, 29.99998664855957)


# ─────────────────────────────────────────────────────────────────────────────
# Top-level sequence
# ─────────────────────────────────────────────────────────────────────────────

def run(control, modbus):
    modbus.reset_coils()
    modbus.set_cycle_active()
    # Initial register state: DJ is looking for pip 1, no attempts yet
    modbus.write_registers(target_number=1, pip_count=1, dj_tries=0, bill_tries=0)

    # ── Round 1: find pip 1 ───────────────────────────────────────────────────
    print("\n" + "═"*50)
    print("  ROUND 1 — finding pip 1")
    print("═"*50)
    _print_stats(modbus)
    _pickup_from_table(control)
    dj_tries = _find_pip(control, modbus, target=1, tries_offset=0)
    if dj_tries < 0:
        print("[findPip] pip 1 not found — aborting.")
        return
    # Tell Bill to look for pip 2; record how many tries DJ needed
    modbus.write_registers(target_number=2, pip_count=2, dj_tries=dj_tries, bill_tries=0)
    _send_to_conveyor(control, modbus)

    # ── Round 2: wait for Bill, find pip 3 ───────────────────────────────────
    print("\n" + "═"*50)
    print("  ROUND 2 — waiting for Bill, then finding pip 3")
    print("═"*50)
    if not modbus.wait_for_bill(timeout=300.0):
        print("[findPip] Timed out waiting for Bill — aborting.")
        return
    bill_tries = modbus.read_bill_tries()
    modbus.write_registers(target_number=3, pip_count=3, dj_tries=dj_tries, bill_tries=bill_tries)
    _print_stats(modbus)
    _pickup_from_front_conveyor(control)
    dj_tries = _find_pip(control, modbus, target=3, tries_offset=dj_tries)
    if dj_tries < 0:
        print("[findPip] pip 3 not found — aborting.")
        return
    # Tell Bill to look for pip 4
    modbus.write_registers(target_number=4, pip_count=4, dj_tries=dj_tries, bill_tries=bill_tries)
    _send_to_conveyor(control, modbus)

    # ── Round 3: wait for Bill, find pip 5 ───────────────────────────────────
    print("\n" + "═"*50)
    print("  ROUND 3 — waiting for Bill, then finding pip 5")
    print("═"*50)
    if not modbus.wait_for_bill(timeout=300.0):
        print("[findPip] Timed out waiting for Bill — aborting.")
        return
    bill_tries = modbus.read_bill_tries()
    modbus.write_registers(target_number=5, pip_count=5, dj_tries=dj_tries, bill_tries=bill_tries)
    _print_stats(modbus)
    _pickup_from_front_conveyor(control)
    dj_tries = _find_pip(control, modbus, target=5, tries_offset=dj_tries)
    if dj_tries < 0:
        print("[findPip] pip 5 not found — aborting.")
        return
    # Final send — pip 6 is the implied last pip for Bill
    modbus.write_registers(target_number=6, pip_count=6, dj_tries=dj_tries, bill_tries=bill_tries)
    _send_to_conveyor(control, modbus)

    print("\n[findPip] Cycle complete — waiting for Bill to clear CYCLE_ACTIVE...")
    modbus.wait_for_cycle_inactive()
    _print_stats(modbus)


# ─────────────────────────────────────────────────────────────────────────────
# Motion sequences
# ─────────────────────────────────────────────────────────────────────────────

def _pickup_from_table(control):
    """Initial die pickup from the table (gripper r=30)."""
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_gripper(control, 'open')
    time.sleep(2)
    send_cartesian(control, *_TABLE_APPROACH)
    send_cartesian(control, *_TABLE_PICKUP)
    send_gripper(control, 'close')
    time.sleep(2)
    send_cartesian(control, *_TABLE_APPROACH)


def _pickup_from_front_conveyor(control):
    """Pick up die returned by Bill from the front conveyor."""
    print("[findPip] Picking up die from front conveyor...")
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_gripper(control, 'open')
    send_cartesian(control, *_FRONT_CONV_APPROACH)
    send_cartesian(control, *_FRONT_CONV_PICKUP)
    send_gripper(control, 'close')
    time.sleep(2)
    send_cartesian(control, *_FRONT_CONV_APPROACH)


def _find_pip(control, modbus, target, tries_offset=0):
    """
    Try up to 3 camera attempts to locate the target pip (or its opposite face).

    Assumes die is already gripped.  On success, returns the cumulative DJ try count
    (tries_offset + attempts used).  Returns -1 if all attempts fail.

    Attempt 1: r=30 grip, camera position 1  — if target inferred on bottom, rotate & verify
    Attempt 2: same grip, wrist rotated to camera position 2  — accept inferred bottom as-is
    Attempt 3: place die, re-grab at r=120, camera rollback position
                  — if target inferred on bottom, rotate & verify
    """
    print(f"\n{'─'*44}")
    print(f"  Finding pip {target}")
    print(f"{'─'*44}")

    tries = tries_offset

    # ── Attempt 1 ────────────────────────────────────────────────────────────
    print("\n[Attempt 1]  r=30  |  camera pos 1")
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, *_CAMERA_POS_1)
    pip = modbus.request_pip_count(1)
    if pip == target:
        print(f"  TARGET pip {target} found on top face.")
        return tries
    if 7 - pip == target:
        print(f"  TARGET pip {target} inferred on bottom face — rotating to verify.")
        pip = _rotate_and_verify(control, modbus, 1)
        if pip == target:
            print(f"  TARGET pip {target} confirmed after rotation.")
            return tries
        print(f"  Verification failed (saw pip {pip}) — continuing to attempt 2.")
    else:
        print(f"  Neither pip {pip} nor {7 - pip} matches target {target}.")

    # ── Attempt 2 ────────────────────────────────────────────────────────────
    tries += 1
    modbus.update_dj_tries(tries)
    print("\n[Attempt 2]  r=30  |  camera pos 2 (rotated) — place as-is if bottom inferred")
    send_cartesian(control, *_CAMERA_POS_2)
    pip = modbus.request_pip_count(2)
    if pip == target:
        print(f"  TARGET pip {target} found on top face.")
        send_cartesian(control, *_CAMERA_POS_1)
        return tries
    if 7 - pip == target:
        print(f"  TARGET pip {target} inferred on bottom face — placing on conveyor as-is.")
        send_cartesian(control, *_CAMERA_POS_1)
        return tries
    print(f"  Neither pip {pip} nor {7 - pip} matches target {target}.")

    # ── Attempt 3: place, re-grab at r=120 ───────────────────────────────────
    tries += 1
    modbus.update_dj_tries(tries)
    print("\n[Attempt 3]  re-grabbing at r=120")
    send_cartesian(control, *_CAMERA_ROLLBACK)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)

    # Place die at table temporarily
    send_cartesian(control, *_TABLE_APPROACH)
    send_cartesian(control, *_TABLE_PICKUP)
    send_gripper(control, 'open')
    send_cartesian(control, *_TABLE_APPROACH)

    # Rotate to r=120 and re-grab
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0, 120.0)
    send_gripper(control, 'close')
    time.sleep(2)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, *_CAMERA_ROLLBACK)
    send_cartesian(control, *_CAMERA_POS_1)

    pip = modbus.request_pip_count(3)
    if pip == target:
        print(f"  TARGET pip {target} found on top face.")
        return tries
    if 7 - pip == target:
        print(f"  TARGET pip {target} inferred on bottom face — rotating to verify.")
        pip = _rotate_and_verify(control, modbus, 3)
        if pip == target:
            print(f"  TARGET pip {target} confirmed after rotation.")
            return tries
        print(f"  Verification failed after rotation.")
    else:
        print(f"  Neither pip {pip} nor {7 - pip} matches target {target}.")

    print(f"\n  WARNING: pip {target} not detected on any face.")
    return -1


def _rotate_and_verify(control, modbus, attempt_num):
    """
    Step wrist 180° in p to expose the inferred bottom face to the camera,
    request a pip count at the flipped position, then step back.
    Stepped waypoints avoid joint-limit errors.
    """
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,   60.0, 0.0)
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,  -30.0, 0.0)
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0, -120.0, 0.0)
    pip = modbus.request_pip_count(attempt_num)
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,  -30.0, 0.0)
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,   60.0, 0.0)
    return pip


def _print_stats(modbus):
    regs = modbus.read_all_registers()
    if regs:
        dj   = regs[_REGISTERS["DJ_NUMBER_OF_TRIES"]]
        bill = regs[_REGISTERS["BILL_NUMBER_OF_TRIES"]]
        print(f"\n  ── Try counts ──────────────────────────")
        print(f"  DJ tries:      {dj}")
        print(f"  Bill tries:    {bill}")
        print(f"  Overall tries: {dj + bill}")
        print(f"  ────────────────────────────────────────\n")


def _send_to_conveyor(control, modbus):
    """
    Drop die on the back conveyor and signal Bill it's ready.
    Works from any current position — starts by going to joint home.
    """
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, *_BACK_CONV_INTERMEDIATE)
    send_cartesian(control, *_BACK_CONV_APPROACH)
    send_cartesian(control, *_BACK_CONV_LOWER)
    send_gripper(control, 'open')
    send_cartesian(control, *_BACK_CONV_APPROACH)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)

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

    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    time.sleep(30.0)
    modbus.set_conveyor_dice_ready(False)


# ─────────────────────────────────────────────────────────────────────────────
# Motion helpers
# ─────────────────────────────────────────────────────────────────────────────

def send_cartesian(control, x, y, z, w=200.0, p=200.0, r=200.0):
    control.cart_ac.wait_for_server()
    goal = CartPose.Goal()
    goal.x, goal.y, goal.z = float(x), float(y), float(z)
    goal.w, goal.p, goal.r = float(w), float(p), float(r)
    print(f"\n[cart] X={x:.3f}  Y={y:.3f}  Z={z:.3f}  W={w:.3f}  P={p:.3f}  R={r:.3f}")
    future = control.cart_ac.send_goal_async(goal, feedback_callback=control._feedback_cb)
    return control._wait(future)


def send_gripper(control, command):
    control.schunk_ac.wait_for_server()
    goal = SchunkGripper.Goal()
    goal.command = command
    print(f"\n[gripper] {command}")
    future = control.schunk_ac.send_goal_async(goal)
    return control._wait(future)


def send_conveyor(control, command):
    control.conveyor_ac.wait_for_server()
    goal = Conveyor.Goal()
    goal.command = command
    print(f"\n[conveyor] {command}")
    control.conveyor_ac.send_goal_async(goal)


def send_joint(control, j1, j2, j3, j4, j5, j6):
    control.joint_ac.wait_for_server()
    goal = JointPose.Goal()
    goal.joint1, goal.joint2, goal.joint3 = float(j1), float(j2), float(j3)
    goal.joint4, goal.joint5, goal.joint6 = float(j4), float(j5), float(j6)
    print(f"\n[joint] J1={j1}  J2={j2}  J3={j3}  J4={j4}  J5={j5}  J6={j6}")
    future = control.joint_ac.send_goal_async(goal, feedback_callback=control._feedback_cb)
    return control._wait(future)


# ─────────────────────────────────────────────────────────────────────────────
# Modbus interface
# ─────────────────────────────────────────────────────────────────────────────

class _ModbusInterface:
    def __init__(self, host, port):
        self._client = ModbusTcpClient(host, port=port)
        if not self._client.connect():
            raise ConnectionError(f"Could not connect to Modbus server at {host}:{port}")
        print(f"[modbus] Connected to {host}:{port}")

    def reset_coils(self):
        for key in ("DJ_READY_FOR_PICTURE", "DJ_AT_CONVEYOR", "DJ_CONVEYOR_ACTIVE",
                    "DJ_CONVEYOR_DICE_READY", "CYCLE_ACTIVE"):
            self._client.write_coil(_SIGNALS[key], False, device_id=1)
        self._client.write_register(_REGISTERS["PIP_COUNT"], 0, device_id=1)
        print("[modbus] Coils reset.")

    def set_cycle_active(self):
        self._client.write_coil(_SIGNALS["CYCLE_ACTIVE"], True, device_id=1)
        print("[modbus] CYCLE_ACTIVE = 1")

    def clear_cycle_active(self):
        self._client.write_coil(_SIGNALS["CYCLE_ACTIVE"], False, device_id=1)
        print("[modbus] CYCLE_ACTIVE = 0")

    def request_pip_count(self, attempt_num, timeout=30.0):
        """Assert DJ_READY_FOR_PICTURE and poll until camera writes PIP_COUNT."""
        self._client.write_register(_REGISTERS["PIP_COUNT"], 0, device_id=1)
        self._client.write_coil(_SIGNALS["DJ_READY_FOR_PICTURE"], True, device_id=1)
        print(f"[modbus] DJ_READY_FOR_PICTURE = 1 — waiting for PIP_COUNT (attempt {attempt_num})...")

        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._client.read_holding_registers(
                _REGISTERS["PIP_COUNT"], count=1, device_id=1
            )
            if not result.isError() and result.registers[0] != 0:
                pip = result.registers[0]
                self._client.write_coil(_SIGNALS["DJ_READY_FOR_PICTURE"], False, device_id=1)
                print(f"[modbus] PIP_COUNT = {pip}  (opposite: {7 - pip})")
                return pip
            time.sleep(0.2)

        self._client.write_coil(_SIGNALS["DJ_READY_FOR_PICTURE"], False, device_id=1)
        print("[modbus] Timeout waiting for PIP_COUNT.")
        return 0

    def write_registers(self, target_number, pip_count, dj_tries, bill_tries):
        """Write all four holding registers in one call."""
        values = [target_number, pip_count, dj_tries, bill_tries]
        result = self._client.write_registers(address=0, values=values, device_id=1)
        if result.isError():
            print(f"[modbus] Error writing registers: {result}")
        else:
            print(f"[modbus] Registers — TARGET={target_number}  PIP_COUNT={pip_count}"
                  f"  DJ_TRIES={dj_tries}  BILL_TRIES={bill_tries}")

    def update_dj_tries(self, count):
        """Write only the DJ_NUMBER_OF_TRIES register."""
        result = self._client.write_register(
            _REGISTERS["DJ_NUMBER_OF_TRIES"], count, device_id=1
        )
        if not result.isError():
            print(f"[modbus] DJ_NUMBER_OF_TRIES = {count}")

    def read_bill_tries(self):
        """Read the current BILL_NUMBER_OF_TRIES register value."""
        result = self._client.read_holding_registers(
            _REGISTERS["BILL_NUMBER_OF_TRIES"], count=1, device_id=1
        )
        if not result.isError():
            return result.registers[0]
        print("[modbus] Error reading BILL_NUMBER_OF_TRIES.")
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

    def wait_for_bill(self, timeout=300.0):
        """
        Poll BILL_CONVEYOR_DICE_READY until True (Bill has sent die back).
        Clears DJ's conveyor signals once Bill confirms.
        """
        print("[modbus] Waiting for BILL_CONVEYOR_DICE_READY...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            result = self._client.read_coils(
                _SIGNALS["BILL_CONVEYOR_DICE_READY"], count=1, device_id=1
            )
            if not result.isError() and result.bits[0]:
                print("[modbus] BILL_CONVEYOR_DICE_READY = 1 — die ready on front conveyor.")
                self._client.write_coil(_SIGNALS["DJ_CONVEYOR_DICE_READY"], False, device_id=1)
                self._client.write_coil(_SIGNALS["DJ_AT_CONVEYOR"], False, device_id=1)
                return True
            time.sleep(0.2)
        print("[modbus] Timeout waiting for BILL_CONVEYOR_DICE_READY.")
        return False

    def read_all_registers(self):
        """Read all four holding registers (TARGET_NUMBER, PIP_COUNT, DJ_TRIES, BILL_TRIES)."""
        result = self._client.read_holding_registers(address=0, count=4, device_id=1)
        if not result.isError():
            return result.registers
        print("[modbus] Error reading registers.")
        return None

    def wait_for_cycle_inactive(self):
        """Block until CYCLE_ACTIVE coil reads 0 (set externally)."""
        print("[modbus] Waiting for CYCLE_ACTIVE to be cleared externally...")
        while True:
            result = self._client.read_coils(_SIGNALS["CYCLE_ACTIVE"], count=1, device_id=1)
            if not result.isError() and not result.bits[0]:
                print("[modbus] CYCLE_ACTIVE = 0 — ending program.")
                return
            time.sleep(0.5)

    def close(self):
        self._client.close()


# ─────────────────────────────────────────────────────────────────────────────
# ROS2 control node
# ─────────────────────────────────────────────────────────────────────────────

class _ControlNode(Node):
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
        description="DJ: find pips 1, 3, 5 in sequence and hand each off to Bill."
    )
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
        run(control, modbus)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        executor.shutdown()
        control.destroy_node()
        rclpy.shutdown()
        modbus.close()


if __name__ == "__main__":
    main()
