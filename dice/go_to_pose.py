"""
Send the FANUC robot to a single Cartesian position.

Usage:
    python3 dice/go_to_pose.py --robot-name <NAME>

Requires the robot nodes to already be running:
    ros2 launch start.launch.py robot_name:=<NAME> robot_ip:=<IP>
"""

import argparse
import os
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from fanuc_interfaces.action import CartPose

_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
if os.path.isfile(_env_path):
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _k, _v = _line.split("=", 1)
                os.environ.setdefault(_k.strip(), _v.strip())

ROBOT_NAME = os.getenv("ROBOT_NAME", "")


# ─────────────────────────────────────────────────────────────────────────────
# USER SECTION — edit target pose here
# ─────────────────────────────────────────────────────────────────────────────

CAMERA_X =  200.0
CAMERA_Y =  505.0
CAMERA_Z =  195.0
CAMERA_W =  -90.0
CAMERA_P =   60.0
CAMERA_R =    0.0

POSE2_X =  196.3519287109375
POSE2_Y =  1050.0159301757812
POSE2_Z =  339.0960388183594
POSE2_W =    0.4133031368255615
POSE2_P =   -1.2113802433013916
POSE2_R =   60.2224006652832

 

# ─────────────────────────────────────────────────────────────────────────────


def run(control):
    # send_cartesian(control, CAMERA_X, CAMERA_Y, CAMERA_Z, CAMERA_W, CAMERA_P, CAMERA_R)
    # send_cartesian(control, POSE2_X,  POSE2_Y,  POSE2_Z,  POSE2_W,  POSE2_P,  POSE2_R)
    #send_cartesian(control, 630.0, -10.0, 110.0, 179.9, 0.0,  30.0)  # approach
    # send_cartesian(control, -52.70402908325195, 476.11212158203125, 550.0,
    # #                179.0, 1.0390314855612814e-05, 30.000024795532227) 
    # send_cartesian(control, -490.0802001953125, 629.2401733398438, 332.9703063964844,
    #                 179.9, -0.0027465890161693096, 30.574892044067383)               # approach
    send_cartesian(control, 200.0, 1050.0, 195.0,-90.0,  60.0,  0.0 )
    send_cartesian(control, 200.0, 1050.0, 195.0,-90.0,  -30.0,  0.0 )
    send_cartesian(control, 200.0, 1050.0, 195.0,-90.0,  -120.0,  0.0 )
    send_cartesian(control, 200.0, 1050.0, 195.0,-90.0,  -30.0,  0.0 )
    send_cartesian(control, 200.0, 1050.0, 195.0,-90.0,  60.0,  0.0 )

def send_cartesian(control, x, y, z, w=200.0, p=200.0, r=200.0):
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


class _ControlNode(Node):
    def __init__(self, robot_name):
        super().__init__("control_node")
        self.cart_ac = ActionClient(self, CartPose, f"/{robot_name}/cartesian_pose")

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
    parser = argparse.ArgumentParser(description="Send robot to a single Cartesian pose.")
    parser.add_argument("--robot-name", default=ROBOT_NAME,
                        help=f"Robot namespace (default: {ROBOT_NAME!r})")
    args = parser.parse_args()

    rclpy.init()
    control = _ControlNode(args.robot_name)

    executor = SingleThreadedExecutor()
    executor.add_node(control)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"Robot namespace: /{args.robot_name}\n")
    try:
        run(control)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        executor.shutdown(wait_for_completion=False)
        spin_thread.join(timeout=2.0)
        control.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
