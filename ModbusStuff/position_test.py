"""
Interactive position test — pick up dice, present to camera, read pips.

Steps through each move one at a time, waiting for Enter before
proceeding so you can verify each position.

A live OpenCV window pops up at each camera step showing what the camera
sees with the pip count overlaid.  Press q / Esc / Enter to close it.

Usage:
    python3 position_test.py

Make sure the ROS2 action servers and camera node are running first:
    ros2 launch launch/start.launch.py robot_name:=dj robot_ip:=<IP>
    ros2 run dice_pipeline camera_node
"""

import sys
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper
from fanuc_interfaces.srv import CountPips
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

NAMESPACE = 'dj'

# ── Positions ─────────────────────────────────────────────────────────────────
HOME_JOINTS  = [0.0, 0.0, 0.0, 0.0, -90.0, 30.0]

# All pick positions as joint moves — captured live 2026-04-24, J4≈0 (NOFLIP).
# PICK_APPROACH: safe height above dice, grabbing from above — captured live 2026-04-24.
PICK_APPROACH_JOINTS = [141.164, 19.121, -10.195, 2.351, -81.139, -25.044]
# PICK_GRASP: dice grab position.    Cartesian: x=631.32, y=-11.40, z=61.46
PICK_GRASP_JOINTS    = [12.708, 23.675, -52.565, 0.0, -37.435, 16.618]

# Camera read positions.
# CAMERA_POS_1: cartesian — robot moves here after picking up die (front face to camera).
CAMERA_POS_1    = dict(x=183.91, y=638.96, z=747.54, w=91.89, p=-62.45, r=174.91)

# CAMERA_JOINTS_2: second camera position — captured live 2026-04-24.
CAMERA_JOINTS_2 = [82.0, 52.0, 0.0, 15.0, 90.0, 15.0]

# CONVEYOR_ABOVE_JOINTS: above conveyor drop position — captured live 2026-04-24.
CONVEYOR_ABOVE_JOINTS = [162.634, 21.584, -44.831, -77.074, 78.240, 73.204]

# CONVEYOR_PLACE_JOINTS: set die on conveyor — captured live 2026-04-24.
CONVEYOR_PLACE_JOINTS = [166.122, 39.897, -67.909, -76.828, 83.029, 54.237]
# ─────────────────────────────────────────────────────────────────────────────


class PositionTest(Node):

    def __init__(self):
        super().__init__('position_test')

        # Action clients
        self.cart_ac   = ActionClient(self, CartPose,      f'/{NAMESPACE}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose,     f'/{NAMESPACE}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{NAMESPACE}/schunk_gripper')

        # Camera service
        self.pip_client = self.create_client(CountPips, '/camera/count_pips')

        # Live image subscription
        self._bridge       = CvBridge()
        self._latest_frame = None
        self._frame_lock   = threading.Lock()
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)

    # ── Image callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().warn(f'Image decode error: {e}')

    def _get_frame(self):
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    # ── Blocking helpers (executor spins in background thread) ───────────────

    def _wait(self, fut):
        while not fut.done():
            time.sleep(0.02)

    def move_joints(self, joints: list) -> bool:
        self.joints_ac.wait_for_server()
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = joints[0], joints[1], joints[2]
        goal.joint4, goal.joint5, goal.joint6 = joints[3], joints[4], joints[5]
        send = self.joints_ac.send_goal_async(goal)
        self._wait(send)
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error('Joint goal rejected'); return False
        self._wait(gh.get_result_async())
        return True

    def move_cart(self, pose: dict) -> bool:
        self.cart_ac.wait_for_server()
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = pose['x'], pose['y'], pose['z']
        goal.w, goal.p, goal.r = pose['w'], pose['p'], pose['r']
        send = self.cart_ac.send_goal_async(goal)
        self._wait(send)
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error('Cart goal rejected'); return False
        self._wait(gh.get_result_async())
        return True

    def gripper(self, command: str) -> bool:
        self.schunk_ac.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command
        send = self.schunk_ac.send_goal_async(goal)
        self._wait(send)
        gh = send.result()
        if not gh or not gh.accepted:
            self.get_logger().error(f'Gripper rejected ({command})'); return False
        self._wait(gh.get_result_async())
        return True

    # ── Camera reading with live window ──────────────────────────────────────

    def read_camera(self) -> int | None:
        """Call count_pips, then show a live camera window with the result overlaid.

        Returns pip_count (int), or None if the camera node is unavailable.
        Press q / Esc / Enter inside the window to close it and continue.
        """
        # Check service is up
        if not self.pip_client.wait_for_service(timeout_sec=3.0):
            print('  Camera node not reachable — is dice_pipeline running?')
            return None

        # Wait for a live frame before calling the service
        print('  Waiting for camera image ...')
        deadline = time.time() + 5.0
        while time.time() < deadline:
            if self._get_frame() is not None:
                break
            time.sleep(0.1)
        else:
            print('  No image received on /camera/image_raw — check camera node')
            return None

        # Call count_pips
        print('  Calling /camera/count_pips ...')
        fut = self.pip_client.call_async(CountPips.Request())
        self._wait(fut)
        res = fut.result()

        pip_count = res.pip_count if (res and res.success) else -1
        status    = res.message   if res else 'no response'
        colour    = (0, 220, 0)   if pip_count >= 0 else (0, 0, 220)

        if pip_count >= 0:
            print(f'  Pip count: {pip_count}  ({status})')
        else:
            print(f'  Detection failed: {status}')

        # ── Live window ───────────────────────────────────────────────────────
        label  = f'Pips: {pip_count}' if pip_count >= 0 else f'Failed: {status}'
        WINDOW = 'Camera View  —  q / Esc / Enter to close'
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 900, 700)
        print('  Window open — press  q / Esc / Enter  to close and continue')

        while True:
            frame = self._get_frame()
            if frame is not None:
                # Pip count banner
                cv2.putText(frame, label,
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX,
                            1.8, colour, 3, cv2.LINE_AA)
                # Small "re-read" hint at bottom
                cv2.putText(frame, 'Press SPACE to re-read pips',
                            (10, frame.shape[0] - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (200, 200, 200), 1)
                cv2.imshow(WINDOW, frame)

            key = cv2.waitKey(30) & 0xFF

            if key == ord(' '):          # re-read without moving
                print('  Re-reading pips ...')
                fut2 = self.pip_client.call_async(CountPips.Request())
                self._wait(fut2)
                res2 = fut2.result()
                pip_count = res2.pip_count if (res2 and res2.success) else -1
                status    = res2.message   if res2 else 'no response'
                colour    = (0, 220, 0)   if pip_count >= 0 else (0, 0, 220)
                label     = f'Pips: {pip_count}' if pip_count >= 0 else f'Failed: {status}'
                print(f'  → {pip_count}  ({status})')

            elif key in (ord('q'), 27, 13):   # q, Esc, Enter
                break

        cv2.destroyAllWindows()
        return pip_count

    # ── Main sequence ─────────────────────────────────────────────────────────

    def run(self):
        steps = [
            ('HOME',                          lambda: self.move_joints(HOME_JOINTS)),
            ('GRIPPER OPEN',                  lambda: self.gripper('open')),
            ('PICK APPROACH — joint move above dice',  lambda: self.move_joints(PICK_APPROACH_JOINTS)),
            ('PICK GRASP    — joint move to grab',     lambda: self.move_joints(PICK_GRASP_JOINTS)),
            ('GRIPPER CLOSE — grab dice',              lambda: self.gripper('close')),
            ('PICK APPROACH — joint move back up',     lambda: self.move_joints(PICK_APPROACH_JOINTS)),
            ('CAMERA POS 1 — front face (cartesian)',        lambda: self.move_cart(CAMERA_POS_1)),
            ('READ CAMERA  (face 1)',                         lambda: self.read_camera()),
            ('CAMERA JOINTS 2 — second face (joint move)',    lambda: self.move_joints(CAMERA_JOINTS_2)),
            ('READ CAMERA  (face 2)',                         lambda: self.read_camera()),
            ('CONVEYOR ABOVE — move above conveyor',          lambda: self.move_joints(CONVEYOR_ABOVE_JOINTS)),
            ('CONVEYOR PLACE — set die on conveyor',          lambda: self.move_joints(CONVEYOR_PLACE_JOINTS)),
            ('GRIPPER OPEN  — release die',                   lambda: self.gripper('open')),
        ]

        print(f"\n{'='*58}")
        print(f"  Position test  —  namespace: /{NAMESPACE}")
        print(f"{'='*58}")
        print("  Enter to execute each step.  'q' to quit at any point.\n")

        for label, action in steps:
            try:
                ans = input(f'  >>> {label}\n      Enter / q: ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                print('\n  Stopped.'); return

            if ans == 'q':
                print('  Aborted.'); return

            result = action()
            if result is False:
                print('  Step failed — stopping.'); return

            print('  Done.\n')

        print(f"{'='*58}")
        print('  All steps complete.')
        print(f"{'='*58}\n")


def main():
    rclpy.init()
    node = PositionTest()

    # Spin the executor in a background thread so image callbacks and
    # service responses are processed while the main thread is blocking
    # on user input or polling futures.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
