"""
findPip_solo.py — Standalone version of findPip.py that reads the camera
directly instead of relying on a partner camera ROS2 node / Modbus signaling.

No partner robot or Modbus server required.

Overall flow:
  Pick up die from table → find pip 1 → send to back conveyor

Usage:
    python3 dice/findPip_solo.py [--robot-name <NAME>]

Requires:
  - Robot nodes: ros2 launch start.launch.py robot_name:=<NAME> robot_ip:=<IP>
  - MindVision camera connected via USB
"""

import argparse
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper
from fanuc_interfaces.msg import ProxReadings

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from camera import grab_frame

_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
if os.path.isfile(_env_path):
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _k, _v = _line.split("=", 1)
                os.environ.setdefault(_k.strip(), _v.strip())

ROBOT_NAME = os.getenv("ROBOT_NAME", "")

# ── Positions ──────────────────────────────────────────────────────────────────

_TABLE_APPROACH = (630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
_TABLE_PICKUP   = (630.0, -10.0,  65.0, 179.9, 0.0,  30.0)

_CAMERA_POS_1    = (200.0, 1050.0, 195.0,            -90.0,  60.0,  0.0)
_CAMERA_POS_2    = (200.0, 1050.0, 339.0960388183594,  0.4133031368255615,
                    -1.2113802433013916, 60.2224006652832)
_CAMERA_ROLLBACK = (200.0,  505.0, 195.0,            -90.0,  60.0,  0.0)

_BACK_CONV_INTERMEDIATE = (-52.70402908325195,  476.11212158203125, 550.0,
                            179.9, 1.0390314855612814e-05, 30.000024795532227)
_BACK_CONV_APPROACH     = (-492.0802001953125,  629.2401733398438,  332.9703063964844,
                            179.9, -0.0027465890161693096, 30.574892044067383)
_BACK_CONV_LOWER        = (-492.0802001953125,  629.2401733398438,  272.4888916015625,
                            179.9, -0.0027458674740046263, 30.574893951416016)


# ─────────────────────────────────────────────────────────────────────────────
# Top-level sequence
# ─────────────────────────────────────────────────────────────────────────────

def run(control):
    print("\n" + "═"*50)
    print("  SOLO RUN — finding pip 1")
    print("═"*50)
    _pickup_from_table(control)
    tries = _find_pip(control, target=1)
    if tries < 0:
        print("[findPip] pip 1 not found — aborting.")
        return
    print(f"\n[findPip] pip 1 found after {tries + 1} attempt(s).")
    _send_to_conveyor(control)
    print("\n[findPip] Done.")


# ─────────────────────────────────────────────────────────────────────────────
# Camera
# ─────────────────────────────────────────────────────────────────────────────

def _capture_pip_count():
    """Grab a frame from the local camera and return the pip count."""
    frame = grab_frame()
    img = cv2.resize(frame, (640, 480))
    pip, annotated = _count_pips(img)
    print(f"[camera] Detected {pip} pips  (opposite face: {7 - pip})")
    return pip


def _count_pips(image):
    """
    Detect pips on a yellow die using Hough circle detection.
    Returns (pip_count, annotated_image).
    """
    pipcount = 0

    # Isolate yellow die face
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    yellow_mask = cv2.inRange(hsv, (15, 80, 80), (35, 255, 255))
    yellow_mask = cv2.dilate(yellow_mask, None, iterations=2)
    yellow_mask = cv2.erode(yellow_mask, None, iterations=2)

    contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("[camera] No die contour found in yellow mask.")
        return 0, image

    die_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(die_contour)
    crop = image[y:y+h, x:x+w].copy()

    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1, minDist=10,
                                param1=50, param2=15, minRadius=3, maxRadius=20)
    if circles is not None:
        for (cx, cy, r) in np.round(circles[0]).astype(int):
            cv2.circle(crop, (cx, cy), r, (0, 255, 0), 2)
            pipcount += 1

    image[y:y+h, x:x+w] = crop
    return pipcount, image


# ─────────────────────────────────────────────────────────────────────────────
# Motion sequences
# ─────────────────────────────────────────────────────────────────────────────

def _pickup_from_table(control):
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_gripper(control, 'open')
    time.sleep(2)
    send_cartesian(control, *_TABLE_APPROACH)
    send_cartesian(control, *_TABLE_PICKUP)
    send_gripper(control, 'close')
    time.sleep(2)
    send_cartesian(control, *_TABLE_APPROACH)


def _find_pip(control, target, tries_offset=0):
    """
    Try up to 3 camera attempts to locate the target pip (or its opposite face).

    Assumes die is already gripped.  On success returns the cumulative try count
    (tries_offset + attempts used).  Returns -1 if all attempts fail.

    Attempt 1: r=30 grip, camera pos 1  — if target inferred on bottom, rotate & verify
    Attempt 2: same grip, camera pos 2  — accept inferred bottom as-is
    Attempt 3: place die, re-grab at r=120, camera rollback pos
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
    pip = _capture_pip_count()
    if pip == target:
        print(f"  TARGET pip {target} found on top face.")
        return tries
    if 7 - pip == target:
        print(f"  TARGET pip {target} inferred on bottom face — rotating to verify.")
        pip = _rotate_and_verify(control)
        if pip == target:
            print(f"  TARGET pip {target} confirmed after rotation.")
            return tries
        print(f"  Verification failed (saw pip {pip}) — continuing to attempt 2.")
    else:
        print(f"  Neither pip {pip} nor {7 - pip} matches target {target}.")

    # ── Attempt 2 ────────────────────────────────────────────────────────────
    tries += 1
    print("\n[Attempt 2]  r=30  |  camera pos 2 — place as-is if bottom inferred")
    send_cartesian(control, *_CAMERA_POS_2)
    pip = _capture_pip_count()
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
    print("\n[Attempt 3]  re-grabbing at r=120")
    send_cartesian(control, *_CAMERA_ROLLBACK)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)

    send_cartesian(control, *_TABLE_APPROACH)
    send_cartesian(control, *_TABLE_PICKUP)
    send_gripper(control, 'open')
    send_cartesian(control, *_TABLE_APPROACH)

    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0, 120.0)
    send_gripper(control, 'close')
    time.sleep(2)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, *_CAMERA_ROLLBACK)
    send_cartesian(control, *_CAMERA_POS_1)

    pip = _capture_pip_count()
    if pip == target:
        print(f"  TARGET pip {target} found on top face.")
        return tries
    if 7 - pip == target:
        print(f"  TARGET pip {target} inferred on bottom face — rotating to verify.")
        pip = _rotate_and_verify(control)
        if pip == target:
            print(f"  TARGET pip {target} confirmed after rotation.")
            return tries
        print(f"  Verification failed after rotation.")
    else:
        print(f"  Neither pip {pip} nor {7 - pip} matches target {target}.")

    print(f"\n  WARNING: pip {target} not detected on any face.")
    return -1


def _rotate_and_verify(control):
    """
    Step wrist 180° in p to expose the inferred bottom face to the camera,
    capture a pip count at the flipped position, then step back.
    Stepped waypoints avoid joint-limit errors.
    """
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,   60.0, 0.0)
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,  -30.0, 0.0)
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0, -120.0, 0.0)
    pip = _capture_pip_count()
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,  -30.0, 0.0)
    send_cartesian(control, 200.0, 1050.0, 195.0, -90.0,   60.0, 0.0)
    return pip


def _send_to_conveyor(control):
    """Drop die on the back conveyor and wait for the prox sensor to confirm passage."""
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, *_BACK_CONV_INTERMEDIATE)
    send_cartesian(control, *_BACK_CONV_APPROACH)
    send_cartesian(control, *_BACK_CONV_LOWER)
    send_gripper(control, 'open')
    send_cartesian(control, *_BACK_CONV_APPROACH)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)

    send_conveyor(control, 'forward')
    control.prox_triggered = False
    deadline = time.time() + 30.0
    while not control.prox_triggered and time.time() < deadline:
        time.sleep(0.05)
    if not control.prox_triggered:
        print("[conveyor] Timeout waiting for proximity sensor.")
    time.sleep(1.0)
    send_conveyor(control, 'stop')


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
        description="DJ solo: find pip 1 using the local camera and send to back conveyor."
    )
    parser.add_argument("--robot-name", default=ROBOT_NAME,
                        help=f"Robot namespace (default: {ROBOT_NAME!r})")
    args = parser.parse_args()

    rclpy.init()
    control = _ControlNode(args.robot_name)

    executor = MultiThreadedExecutor()
    executor.add_node(control)
    threading.Thread(target=executor.spin, daemon=True).start()

    print(f"Robot namespace: /{args.robot_name}\n")
    try:
        run(control)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        executor.shutdown()
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
