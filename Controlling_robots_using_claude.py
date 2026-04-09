"""
Made by: Colin
With the help of Claude Code

Controlling_robots_using_claude.py

Autonomous FANUC dice inspection routine:
  1. Pick up the dice with the Schunk gripper
  2. Present it to the overhead camera
  3. Capture an image and count pips using HSV colour masking
  4. Repeat for NUM_ROLLS total dice
  5. Report results and return home

Prerequisites:
  - Robot driver running:
      ros2 launch launch/start.launch.py robot_name:=bunsen robot_ip:=<ip>
  - OpenCV installed:
      sudo apt install python3-opencv

Run from the repo root (so fanuc_interfaces is on the path):
  source install/setup.bash
  python3 Controlling_robots_using_claude.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper

import cv2
import numpy as np
import time
from collections import Counter
from camera import Camera

# ─── CONFIGURE THESE BEFORE RUNNING ──────────────────────────────────────────

ROBOT_NAMESPACE  = 'Beaker'       # Must match robot_name used in your launch command
NUM_ROLLS        = 3              # Total number of dice picks
CAMERA_IP        = 'Camera_IP'  # Camera IP, or None to auto-detect

# Robot positions
PICK_ABOVE  = dict(x=470.0, y=-15.0, z=-18.0,   w=179.9, p=0.0,   r=30.0)
PICK_DOWN   = dict(x=470.0, y=-15.0, z=-185.0,  w=179.9, p=0.0,   r=30.0)
PICK_LIFT   = dict(x=470.0, y=-15.0, z=-18.0,   w=179.9, p=0.0,   r=30.0)
CAMERA_POSE = dict(x=490.0, y=890.0, z=881.0,   w=73.0,  p=-66.0, r=-170.0)
DROP_POSE   = dict(x=470.0, y=-15.0, z=-185.0,  w=179.9, p=0.0,   r=120)

# HSV pip detection tuning
YELLOW_LO    = (18, 180, 150)
YELLOW_HI    = (24, 255, 255)
BLACK_V_MAX  = 60
PIP_AREA_MIN = 30
PIP_AREA_MAX = 2000

# ─────────────────────────────────────────────────────────────────────────────


class DiceInspector(Node):

    def __init__(self):
        super().__init__('dice_inspector')

        self.cart_client    = ActionClient(self, CartPose,      f'/{ROBOT_NAMESPACE}/cartesian_pose')
        self.joint_client   = ActionClient(self, JointPose,     f'/{ROBOT_NAMESPACE}/joint_pose')
        self.gripper_client = ActionClient(self, SchunkGripper, f'/{ROBOT_NAMESPACE}/schunk_gripper')
        self.camera         = Camera(camera_ip=CAMERA_IP)

    # ── Action helpers ────────────────────────────────────────────────────────

    def _send_cart(self, x, y, z, w=200.0, p=200.0, r=200.0) -> bool:
        self.cart_client.wait_for_server()
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = float(x), float(y), float(z)
        goal.w, goal.p, goal.r = float(w), float(p), float(r)

        future = self.cart_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Cartesian goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def _send_gripper(self, command: str) -> bool:
        self.gripper_client.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command

        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'Gripper goal rejected: {command}')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def _send_joint(self, j1, j2, j3, j4, j5, j6) -> bool:
        self.joint_client.wait_for_server()
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = float(j1), float(j2), float(j3)
        goal.joint4, goal.joint5, goal.joint6 = float(j4), float(j5), float(j6)

        future = self.joint_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Joint goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    # ── Robot moves ───────────────────────────────────────────────────────────

    def go_home(self):
        self.get_logger().info('Moving to home position...')
        self._send_joint(1.1, 1.5, -2.0, -1.7, -88.6, -30.0)

    def pick_dice(self):
        self.get_logger().info('Opening gripper...')
        self._send_gripper('open')

        self.get_logger().info('Moving above dice...')
        self._send_cart(**PICK_ABOVE)

        self.get_logger().info('Moving down to dice...')
        self._send_cart(**PICK_DOWN)

        self.get_logger().info('Closing gripper (grasping)...')
        self._send_gripper('close')

        self.get_logger().info('Lifting dice...')
        self._send_cart(**PICK_LIFT)

    def present_to_camera(self):
        self.get_logger().info('Moving to camera position...')
        self._send_cart(**CAMERA_POSE)
        time.sleep(0.5)

    def release_dice(self, extra_r: float = 0.0):
        self.get_logger().info(f'Moving to drop position (r offset: {extra_r:+.0f}°)...')
        self._send_cart(**{**DROP_POSE, 'r': DROP_POSE['r'] + extra_r})
        self.get_logger().info('Opening gripper (releasing)...')
        self._send_gripper('open')

    # ── Vision ────────────────────────────────────────────────────────────────

    def capture_image(self):
        return self.camera.getFrame()

    def count_pips(self, image, roll_num: int) -> int:
        """
        Count black pips on a yellow dice face using HSV colour masking.
        Saves an annotated debug image to ~/Desktop.
        Returns pip count (1–6), or 0 if the dice face is not found.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # ── Step 1: isolate the yellow dice face ──────────────────────────────
        yellow_mask = cv2.inRange(hsv, np.array(YELLOW_LO), np.array(YELLOW_HI))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN,  kernel)

        # Fill solid so pip holes don't block the AND
        y_cnts, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not y_cnts:
            self.get_logger().warn('No yellow region found — check YELLOW_LO/HI tuning')
            return 0
        filled_mask = np.zeros_like(yellow_mask)
        cv2.drawContours(filled_mask, [max(y_cnts, key=cv2.contourArea)], -1, 255, cv2.FILLED)
        yellow_mask = filled_mask

        # ── Step 2: find dark pips inside the dice face ───────────────────────
        black_mask = cv2.inRange(hsv, np.array([0, 0, 0]),
                                      np.array([180, 255, BLACK_V_MAX]))
        pip_mask = cv2.bitwise_and(black_mask, black_mask, mask=yellow_mask)
        pip_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        pip_mask = cv2.morphologyEx(pip_mask, cv2.MORPH_OPEN, pip_kernel)

        # ── Step 3: count contours ────────────────────────────────────────────
        contours, _ = cv2.findContours(pip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pip_contours = [c for c in contours
                        if PIP_AREA_MIN < cv2.contourArea(c) < PIP_AREA_MAX]
        pip_count = min(len(pip_contours), 6)

        # ── Step 4: save annotated debug image ────────────────────────────────
        annotated = image.copy()
        for c in pip_contours:
            (cx, cy), r = cv2.minEnclosingCircle(c)
            cv2.circle(annotated, (int(cx), int(cy)), max(int(r), 4), (0, 255, 0), 2)
        cv2.putText(annotated, f'Pips: {pip_count}', (10, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 255), 2)

        save_path = f'/home/colin/Desktop/dice_roll_{roll_num}.png'
        cv2.imwrite(save_path, annotated)
        self.get_logger().info(f'Image saved: {save_path}  |  Pips detected: {pip_count}')

        return pip_count

    # ── Main routine ──────────────────────────────────────────────────────────

    def run(self):
        self.get_logger().info('Waiting for action servers...')
        self.cart_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action servers ready. Starting dice inspection.')

        self.go_home()

        results = []

        for roll_num in range(1, NUM_ROLLS + 1):
            print(f'\n{"="*45}')
            print(f'  ROLL {roll_num} of {NUM_ROLLS}')
            print(f'{"="*45}')

            self.pick_dice()
            self.present_to_camera()

            image     = self.capture_image()
            pip_count = self.count_pips(image, roll_num)
            results.append(pip_count)
            self.get_logger().info(f'Roll {roll_num}: {pip_count} pip(s)')

            self.release_dice()   

        self.go_home()

        # ── Results report ────────────────────────────────────────────────────
        face_counts = Counter(results)

        print(f'\n{"="*45}')
        print('         DICE INSPECTION RESULTS')
        print(f'{"="*45}')
        print('  Per-roll results:')
        for i, pips in enumerate(results, 1):
            print(f'    Roll {i}: {pips} pip(s)')
        print(f'  {"─"*35}')
        print('  Individual face counts:')
        for face in sorted(face_counts):
            print(f'    Face {face}: appeared {face_counts[face]} time(s)')
        print(f'  {"─"*35}')
        print(f'  Total pip count: {sum(results)}')
        print(f'{"="*45}\n')


def main():
    rclpy.init()
    node = DiceInspector()
    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
    finally:
        node.camera.disable()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
