"""
Send the FANUC robot to a position via ROS2 actions.

Edit the "USER SECTION" below and run:
    python3 tests/send_pose.py --robot-name <NAME>

Requires the robot nodes to already be running:
    ros2 launch start.launch.py robot_name:=<NAME> robot_ip:=<IP>

ROS2 topics used by this script:
    /{robot_name}/camera_trigger  (std_msgs/Bool)           control → camera
    /{robot_name}/pip_counts      (std_msgs/Int32MultiArray) camera → control
"""

import argparse
import os
import platform
import sys
import threading
import time

import cv2 as cv
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, Int32MultiArray

from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import mvsdk

# Load .env from the tests/ directory if present
_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
if os.path.isfile(_env_path):
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _k, _v = _line.split("=", 1)
                os.environ.setdefault(_k.strip(), _v.strip())

# ── Camera tuning constants ───────────────────────────────────────────────────
HSV_LOW  = (10, 140, 40)
HSV_HIGH = (45, 255, 255)

PIP_AREA_MIN = 30    # pip contours smaller than this are ignored (noise)
PIP_AREA_MAX = 90    # pip contours larger than this are ignored (dice border etc.)
PIP_AREA_MAX_SINGLE = 160   # relaxed max area used only for single-pip fallback
PIP_CIRCULARITY_MIN = 0.35  # 1.0 is a perfect circle
PIP_SOLIDITY_MIN = 0.70     # reject fragmented/noisy shapes
PIP_EDGE_MARGIN = 0.06      # reject border artifacts near crop edges
PIP_SINGLE_CENTER_TOL = 0.32  # fallback candidate must be near die center
# ─────────────────────────────────────────────────────────────────────────────


# ─────────────────────────────────────────────────────────────────────────────
# USER SECTION — edit here
# ─────────────────────────────────────────────────────────────────────────────

ROBOT_NAME = os.getenv("ROBOT_NAME", "")   # set in tests/.env or via --robot-name
NUM_RUNS   = 3      # number of dice to pick up and photograph


def run(control):
    """
    Pick up NUM_RUNS dice in sequence, photograph each one, and print a
    pip-count summary at the end.

    Available calls:
        counts = capture_dice(control)                      # trigger camera, wait for pip counts
        send_cartesian(control, x, y, z)                    # keep current W/P/R
        send_cartesian(control, x, y, z, w, p, r)           # full pose
        send_joint(control, j1, j2, j3, j4, j5, j6)        # all joints in degrees
        send_gripper(control, 'open')
        send_gripper(control, 'close')
    """
    pickup_dice(control)


def pickup_dice(control):
    """
    Pick up NUM_RUNS dice in sequence, photograph each at the viewing position,
    and report total and per-die pip counts.
    """
    results = []   # list of (label, pip_count) — one entry per die

    # ── Die 1 — gripper at r=30 ───────────────────────────────────────────────
    _run_header(1)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_gripper(control, 'open')
    time.sleep(2)

    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)   # approach
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0,  30.0)   # lower
    send_gripper(control, 'close')
    time.sleep(2)

    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)   # lift
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)              # carry
    send_cartesian(control, 200.0, 505.0, 195.0, -90.0, 60.0, 0.0)    # photo pos

    results.append(("Die 1", _photo_and_record(control, 1)))

    # Return die 1, transition to die 2 pickup angle
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0,  30.0)
    send_gripper(control, 'open')
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)   # rotate

    # ── Die 2 — gripper at r=120 ──────────────────────────────────────────────
    _run_header(2)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0, 120.0)   # lower
    send_gripper(control, 'close')
    time.sleep(2)

    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)   # lift
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)              # carry
    send_cartesian(control, 200.0, 505.0, 195.0, -90.0, 60.0, 0.0)    # photo pos

    results.append(("Die 2", _photo_and_record(control, 2)))

    # Return die 2, transition to die 3 pickup angle
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0,  30.0)
    send_gripper(control, 'open')
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)   # rotate

    # ── Die 3 — gripper at r=120 ──────────────────────────────────────────────
    _run_header(3)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0, 120.0)   # lower
    send_gripper(control, 'close')
    time.sleep(2)

    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0, 120.0)   # lift
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)              # carry
    send_cartesian(control, 200.0, 505.0, 195.0, -90.0, 60.0, 0.0)    # photo pos

    results.append(("Die 3", _photo_and_record(control, 3)))

    # Return die 3 and go home
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_cartesian(control, 630.0, -10.0,  65.0, 179.9, 0.0,  30.0)
    send_gripper(control, 'open')
    send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)
    send_joint(control, 0.0, 0.0, 0.0, 0.0, -90.0, 30.0)

    _print_summary(results)


def _run_header(n):
    print(f"\n{'═'*44}")
    print(f"  RUN {n} / {NUM_RUNS}  —  picking up die")
    print(f"{'═'*44}")


def _photo_and_record(control, run_num):
    """Trigger the camera node, wait for pip counts, print per-run result."""
    counts = capture_dice(control)
    total = sum(counts) if counts else 0
    pip_word = "pip" if total == 1 else "pips"
    print(f"\n  [Run {run_num}]  detected {total} {pip_word}  {counts}")
    return total


def _print_summary(results):
    total = sum(count for _, count in results)
    w = 42
    print(f"\n{'═'*w}")
    print(f"{'PIP COUNT SUMMARY':^{w}}")
    print(f"{'─'*w}")
    for label, count in results:
        pip_word = "pip" if count == 1 else "pips"
        print(f"  {label:<8}  →  {count} {pip_word}")
    print(f"{'─'*w}")
    pip_word = "pip" if total == 1 else "pips"
    print(f"  {'TOTAL':<8}  →  {total} {pip_word} across {len(results)} dice")
    print(f"{'═'*w}\n")


# ─────────────────────────────────────────────────────────────────────────────
# Functions — call these with your coordinates
# ─────────────────────────────────────────────────────────────────────────────

def capture_dice(control, timeout=15.0):
    """
    Publish a trigger to the camera node and wait for it to publish pip counts.

    Args:
        control: the control node passed into run()
        timeout: seconds to wait for a response (default 15)

    Returns:
        list of pip counts, one per detected die (e.g. [3, 5, 1])
    """
    return control.request_pip_count(timeout=timeout)


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

class _CameraNode(Node):
    """
    Camera node — subscribes to /{robot_name}/camera_trigger (std_msgs/Bool).
    On trigger: grabs a frame from the MindVision camera, detects pip counts,
    and publishes results to /{robot_name}/pip_counts (std_msgs/Int32MultiArray).
    """

    def __init__(self, robot_name):
        super().__init__("camera_node")

        self._pub = self.create_publisher(
            Int32MultiArray, f"/{robot_name}/pip_counts", 10
        )
        self.create_subscription(
            Bool, f"/{robot_name}/camera_trigger", self._trigger_cb, 10
        )
        self.get_logger().info(
            f"Camera node ready — listening on /{robot_name}/camera_trigger"
        )

    def _trigger_cb(self, msg):
        self.get_logger().info("Trigger received — capturing frame...")
        frame = self._grab_frame()
        counts = self._detect_pips(frame) if frame is not None else []

        out = Int32MultiArray()
        out.data = counts
        self._pub.publish(out)
        self.get_logger().info(f"Published pip counts: {counts}")

    # ── Camera capture ────────────────────────────────────────────────────────

    def _grab_frame(self):
        DevList = mvsdk.CameraEnumerateDevice()
        if len(DevList) < 1:
            self.get_logger().error("No camera found.")
            return None

        DevInfo = DevList[0]
        self.get_logger().info(f"Camera: {DevInfo.GetFriendlyName()}")
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        cap = mvsdk.CameraGetCapability(hCamera)
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

        fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if monoCamera else mvsdk.CAMERA_MEDIA_TYPE_BGR8
        mvsdk.CameraSetIspOutFormat(hCamera, fmt)
        mvsdk.CameraSetTriggerMode(hCamera, 0)
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 60 * 1000)
        mvsdk.CameraPlay(hCamera)

        buf_size = (cap.sResolutionRange.iWidthMax *
                    cap.sResolutionRange.iHeightMax *
                    (1 if monoCamera else 3))
        pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            channels = 1 if monoCamera else 3
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (FrameHead.iHeight, FrameHead.iWidth, channels)
            ).copy()
            return frame
        finally:
            mvsdk.CameraUnInit(hCamera)
            mvsdk.CameraAlignFree(pFrameBuffer)

    # ── Pip detection ─────────────────────────────────────────────────────────

    def _detect_pips(self, img):
        img = cv.resize(img, (640, 480))

        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
        mask = cv.inRange(hsv, HSV_LOW, HSV_HIGH)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)
        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)

        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        result = img.copy()

        self.get_logger().info(
            f"Dice contours: {len(contours)}, "
            f"areas: {[round(cv.contourArea(c)) for c in contours]}"
        )

        pip_counts = []
        for i, c in enumerate(contours):
            if cv.contourArea(c) < 500:
                continue

            x, y, w, h = cv.boundingRect(c)
            x = max(x, 0);  y = max(y, 0)
            dice_crop = result[y:y+h, x:x+w]
            orig_crop = img[y:y+h, x:x+w]

            gray = cv.cvtColor(orig_crop, cv.COLOR_BGR2GRAY)
            _, thresh = cv.threshold(gray, 0, 255, cv.THRESH_BINARY_INV + cv.THRESH_OTSU)

            dot_contours, strict_pips, relaxed_single = self._pip_candidates(
                thresh, PIP_AREA_MIN, PIP_AREA_MAX
            )
            kept, mode = self._select_final_pips(strict_pips, relaxed_single)

            dot_areas = sorted([cv.contourArea(d) for d in dot_contours], reverse=True)
            self.get_logger().info(
                f"  Die {i} — pip areas: {[round(a) for a in dot_areas[:15]]}  mode: {mode}"
            )

            dot_count = 0
            for d in kept:
                dot_count += 1
                px, py, pw, ph = cv.boundingRect(d)
                cv.rectangle(dice_crop, (px, py), (px + pw, py + ph), (0, 0, 255), 2)

            self.get_logger().info(f"  Pips counted: {dot_count}")
            pip_counts.append(dot_count)

        return pip_counts

    def _pip_candidates(self, thresh, area_min, area_max):
        h, w = thresh.shape[:2]
        edge_x = int(w * PIP_EDGE_MARGIN)
        edge_y = int(h * PIP_EDGE_MARGIN)
        cx = w * 0.5
        cy = h * 0.5
        center_r2 = (min(w, h) * PIP_SINGLE_CENTER_TOL) ** 2

        dot_contours, _ = cv.findContours(thresh, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        kept = []
        relaxed_single = []

        for d in dot_contours:
            area = cv.contourArea(d)
            x, y, ww, hh = cv.boundingRect(d)
            if x <= edge_x or y <= edge_y or (x + ww) >= (w - edge_x) or (y + hh) >= (h - edge_y):
                continue

            bx = x + (ww * 0.5)
            by = y + (hh * 0.5)
            dist2 = (bx - cx) ** 2 + (by - cy) ** 2

            per = cv.arcLength(d, True)
            if per <= 0:
                continue
            circularity = (4.0 * np.pi * area) / (per * per)

            hull = cv.convexHull(d)
            hull_area = cv.contourArea(hull)
            if hull_area <= 0:
                continue
            solidity = area / hull_area

            if (area_min < area < area_max
                    and circularity >= PIP_CIRCULARITY_MIN
                    and solidity >= PIP_SOLIDITY_MIN):
                kept.append(d)

            if (area_max <= area < PIP_AREA_MAX_SINGLE
                    and circularity >= PIP_CIRCULARITY_MIN * 0.85
                    and solidity >= PIP_SOLIDITY_MIN * 0.85
                    and dist2 <= center_r2):
                relaxed_single.append(d)

        return dot_contours, kept, relaxed_single

    def _select_final_pips(self, strict_pips, relaxed_single):
        if strict_pips:
            return strict_pips, "strict"
        if len(relaxed_single) == 1:
            return [relaxed_single[0]], "single-fallback"
        return [], "none"


class _ControlNode(Node):
    """
    Control node — ROS2 node for robot arm motion.
    Publishes to /{robot_name}/camera_trigger to request a pip count, and
    subscribes to /{robot_name}/pip_counts to receive the result.
    """

    def __init__(self, robot_name):
        super().__init__("control_node")

        # Robot action clients
        self.cart_ac   = ActionClient(self, CartPose,      f"/{robot_name}/cartesian_pose")
        self.joint_ac  = ActionClient(self, JointPose,     f"/{robot_name}/joint_pose")
        self.schunk_ac = ActionClient(self, SchunkGripper, f"/{robot_name}/schunk_gripper")

        # Camera pub/sub
        self._trigger_pub = self.create_publisher(
            Bool, f"/{robot_name}/camera_trigger", 10
        )
        self._pip_result = None
        self._pip_event  = threading.Event()
        self.create_subscription(
            Int32MultiArray, f"/{robot_name}/pip_counts", self._pip_cb, 10
        )

    def request_pip_count(self, timeout=15.0):
        """Publish a camera trigger and block until pip counts are received."""
        self._pip_result = None
        self._pip_event.clear()

        msg = Bool()
        msg.data = True
        self._trigger_pub.publish(msg)
        self.get_logger().info("Camera trigger published — waiting for pip counts...")

        if self._pip_event.wait(timeout=timeout):
            return self._pip_result
        else:
            self.get_logger().warning("Timeout waiting for pip counts.")
            return []

    def _pip_cb(self, msg):
        self._pip_result = list(msg.data)
        self._pip_event.set()

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
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot-name", default=ROBOT_NAME,
                        help=f"Robot namespace (default: {ROBOT_NAME})")
    args = parser.parse_args()

    rclpy.init()
    control = _ControlNode(args.robot_name)
    camera  = _CameraNode(args.robot_name)

    executor = MultiThreadedExecutor()
    executor.add_node(control)
    executor.add_node(camera)
    threading.Thread(target=executor.spin, daemon=True).start()

    print(f"Robot namespace: /{args.robot_name}\n")
    try:
        run(control)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        executor.shutdown()
        control.destroy_node()
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
