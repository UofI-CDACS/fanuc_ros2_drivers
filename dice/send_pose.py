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
    


def _run_header(n):
    print(f"\n{'═'*44}")
    print(f"  RUN {n} / {NUM_RUNS}  —  picking up die")
    print(f"{'═'*44}")



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
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
