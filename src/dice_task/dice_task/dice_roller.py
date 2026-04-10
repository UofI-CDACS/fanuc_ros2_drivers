#!/usr/bin/env python3
"""
dice_roller.py
--------------
Picks up three dice one at a time, presents each to the overhead MindVision
camera, counts the pips on the top face using OpenCV, and reports a running
total and final sum.

Sequence per die:
  1. Open gripper
  2. Move to approach height above die
  3. Descend to pick position
  4. Close gripper
  5. Lift die to approach height
  6. Move to camera present position
  7. Capture image from MindVision camera
  8. Count pips with OpenCV (HoughCircles)
  9. Log pip count and running total
 10. Open gripper

Requirements:
  - fanuc_ros2_drivers action servers must be running
  - rosbridge / MindVision ROS2 camera driver must be running
  - pip install opencv-python cv_bridge
"""

# ── stdlib ────────────────────────────────────────────────────────────────────────────────
import os

# ── ROS2 ─────────────────────────────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

# ── FANUC interfaces ──────────────────────────────────────────────────────────────────────────────
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper

# ── Camera ────────────────────────────────────────────────────────────────────────────────
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

# ── Vision ────────────────────────────────────────────────────────────────────────────────
import cv2
import numpy as np


# ══════════════════════════════════════════════════════════════════════════════
#  Configuration  —  replace dummy values with real ones before running
# ══════════════════════════════════════════════════════════════════════════════

ROBOT_NAME = os.environ.get('ROBOT_NAME', 'my_robot')
ROBOT_IP   = os.environ.get('ROBOT_IP',   '0.0.0.0')   # unused here, set at launch

# Pick positions for each of the three dice [x, y, z, w, p, r] in mm / degrees
# w=180, p=0, r=0 orients the gripper straight down — adjust to match your setup
DICE_WIDTH = 80.0  # mm — center-to-center spacing between dice in the +x direction

# Height (mm) the robot rises above a pick position before/after gripping
APPROACH_OFFSET_Z = 2 * DICE_WIDTH

_d1 = {'x': 469.0, 'y': -15.4, 'z': -178.5, 'w': 179.9, 'p': 0.0, 'r': 30.0}
DICE_PICK_POSITIONS = [
    {**_d1, 'x': _d1['x'] + i * DICE_WIDTH} for i in range(3)
]

# Joint position where the robot presents the die to the camera (same for all three dice)
CAMERA_PRESENT_JOINTS = {'j1': 60.0, 'j2': 33.0, 'j3': 20.0, 'j4': -57.0, 'j5': -34.0, 'j6': 80.0}

# ROS2 topic published by the MindVision camera driver
CAMERA_TOPIC = '/mv_camera/image_raw'

# Directory where raw and annotated images are saved for debugging
IMAGE_SAVE_DIR = '/tmp/'

# HSV crop parameters — tune with 'just calibrate-hsv' to isolate the yellow die face
# OpenCV HSV scale: H 0-179, S 0-255, V 0-255
HSV_H_LOW  = 20    # hue lower bound
HSV_S_LOW  = 100   # saturation lower bound
HSV_V_LOW  = 100   # value lower bound
HSV_H_HIGH = 35    # hue upper bound
HSV_S_HIGH = 255   # saturation upper bound
HSV_V_HIGH = 255   # value upper bound

# HoughCircles parameters — tune these to match your camera/die/lighting setup
HOUGH_DP        = 1.2   # inverse ratio of accumulator resolution to image resolution
HOUGH_MIN_DIST  = 20    # minimum distance between detected pip centers (px)
HOUGH_PARAM1    = 50    # upper Canny edge threshold
HOUGH_PARAM2    = 25    # accumulator threshold — lower catches more circles
HOUGH_MIN_R     = 0     # minimum pip radius (px)
HOUGH_MAX_R     = 20    # maximum pip radius (px)


# ══════════════════════════════════════════════════════════════════════════════
#  Node
# ══════════════════════════════════════════════════════════════════════════════

class DiceRollerNode(Node):

    def __init__(self):
        super().__init__('dice_roller')

        ns = ROBOT_NAME

        # ── Action clients ────────────────────────────────────────────────────────────────────
        self.cart_ac   = ActionClient(self, CartPose,      f'/{ns}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose,     f'/{ns}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{ns}/schunk_gripper')

        # ── Camera ──────────────────────────────────────────────────────────────────────────────
        self.bridge       = CvBridge()
        self.latest_image = None
        self.create_subscription(Image, CAMERA_TOPIC, self._image_cb, 10)
        self.capture_client = self.create_client(Trigger, '/mv_camera/capture')
        self.get_logger().info('Waiting for camera capture service...')
        self.capture_client.wait_for_service()

        self.get_logger().info('Waiting for action servers...')
        self.cart_ac.wait_for_server()
        self.joints_ac.wait_for_server()
        self.schunk_ac.wait_for_server()
        self.get_logger().info('Action servers ready.')

    # ── Camera callback ─────────────────────────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        """Store the most recent camera frame."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ── Generic blocking goal sender ───────────────────────────────────────────────────────────────────
    def _send_goal(self, client: ActionClient, goal):
        """
        Send an action goal and block until it completes.
        Returns the result object, or None if the goal was rejected.
        """
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server.')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result

    # ── Motion helpers ──────────────────────────────────────────────────────────────────────────────
    def _move_cart(self, x, y, z, w=200.0, p=200.0, r=200.0) -> bool:
        """
        Move to a Cartesian position.
        Leaving w/p/r at 200.0 (default) keeps the current orientation.
        """
        goal = CartPose.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        goal.w = float(w)
        goal.p = float(p)
        goal.r = float(r)

        result = self._send_goal(self.cart_ac, goal)
        if result is None or not result.success:
            self.get_logger().error(f'Move failed: x={x} y={y} z={z}')
            return False
        return True

    def _move_joints(self, j1, j2, j3, j4, j5, j6) -> bool:
        """Move to a joint position (degrees)."""
        goal = JointPose.Goal()
        goal.joint1 = float(j1)
        goal.joint2 = float(j2)
        goal.joint3 = float(j3)
        goal.joint4 = float(j4)
        goal.joint5 = float(j5)
        goal.joint6 = float(j6)

        result = self._send_goal(self.joints_ac, goal)
        if result is None or not result.success:
            self.get_logger().error(f'Joint move failed: {j1},{j2},{j3},{j4},{j5},{j6}')
            return False
        return True

    def go_home(self):
        """Move to home joint position (0,0,0,0,-90,30) and open the gripper."""
        self.get_logger().info('Moving to home position...')
        self._move_joints(0, 0, 0, 0, -90, 30)
        self.get_logger().info('Opening gripper...')
        self._schunk('open')

    # ── Gripper helper ──────────────────────────────────────────────────────────────────────────────
    def _schunk(self, command: str) -> bool:
        """Open or close the Schunk gripper. command must be 'open' or 'close'."""
        goal = SchunkGripper.Goal()
        goal.command = command

        result = self._send_goal(self.schunk_ac, goal)
        if result is None or not result.success:
            self.get_logger().error(f'Schunk gripper command failed: {command}')
            return False
        return True

    # ── Camera callback ─────────────────────────────────────────────────────────────────────────────
    def _image_cb(self, msg: Image):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ── Camera capture ──────────────────────────────────────────────────────────────────────────────
    def _capture_image(self) -> np.ndarray:
        """
        Ask the camera node to grab a fresh frame, then receive it via subscription.
        Returns the image array, or None on failure.
        """
        self.latest_image = None

        # Tell the camera node to capture now
        future = self.capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error('Camera capture service returned failure.')
            return None

        # Wait for the published frame to arrive
        deadline_ns = self.get_clock().now().nanoseconds + int(3.0 * 1e9)
        while self.latest_image is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > deadline_ns:
                self.get_logger().error('Timed out waiting for camera frame after service call.')
                return None

        return self.latest_image.copy()

    # ── HSV crop ─────────────────────────────────────────────────────────────────────────────────
    def _crop_die(self, image: np.ndarray) -> np.ndarray:
        """
        Crop the image to the bounding box of the yellow die face using HSV masking.
        Returns the cropped region, or the full image if no yellow region is found.
        Tune HSV_* constants at the top of this file with 'just calibrate-hsv'.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = np.array([HSV_H_LOW,  HSV_S_LOW,  HSV_V_LOW])
        upper = np.array([HSV_H_HIGH, HSV_S_HIGH, HSV_V_HIGH])
        mask = cv2.inRange(hsv, lower, upper)

        # Morphological clean-up: close small holes, remove small noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn('HSV crop: no yellow region found — using full image.')
            return image

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))

        # Add padding so we don't clip pip edges
        pad = 10
        x1 = max(0, x - pad)
        y1 = max(0, y - pad)
        x2 = min(image.shape[1], x + w + pad)
        y2 = min(image.shape[0], y + h + pad)

        return image[y1:y2, x1:x2]

    # ── Pip counter ────────────────────────────────────────────────────────────────────────────────
    def _count_pips(self, image: np.ndarray) -> tuple:
        """
        Count the pips (dots) on the visible die face.

        Steps:
          1. Convert to grayscale.
          2. Gaussian blur to reduce noise.
          3. HoughCircles to detect circular pips.
          4. Draw detections on a debug image.

        Returns (pip_count: int, debug_image: np.ndarray).
        Tune HOUGH_* constants at the top of this file if detection is off.
        """
        gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (11, 11), 2)

        circles = cv2.HoughCircles(
            blurred,
            cv2.HOUGH_GRADIENT,
            dp       = HOUGH_DP,
            minDist  = HOUGH_MIN_DIST,
            param1   = HOUGH_PARAM1,
            param2   = HOUGH_PARAM2,
            minRadius= HOUGH_MIN_R,
            maxRadius= HOUGH_MAX_R,
        )

        debug = image.copy()

        if circles is None:
            cv2.putText(debug, 'No pips detected', (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            return 0, debug

        pip_count = len(circles[0])

        # Draw each detected circle on the debug image
        for (cx, cy, r) in np.round(circles[0]).astype(int):
            cv2.circle(debug, (cx, cy), r, (0, 255, 0), 2)   # pip outline
            cv2.circle(debug, (cx, cy), 3, (0, 0, 255), -1)  # center dot

        cv2.putText(debug, f'Pips: {pip_count}', (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        return pip_count, debug

    # ── Test sequence ──────────────────────────────────────────────────────────────────────────────
    def test_run(self):
        """
        Test: home → open gripper → approach die 1 → pick → lift → camera present → stop.
        """
        self.go_home()

        pos = DICE_PICK_POSITIONS[0]

        self.get_logger().info('Opening gripper...')
        self._schunk('open')

        self.get_logger().info('Moving to approach position...')
        self._move_cart(
            pos['x'], pos['y'], pos['z'] + DICE_WIDTH,
            pos['w'], pos['p'], pos['r']
        )

        self.get_logger().info('Descending to pick position...')
        self._move_cart(
            pos['x'], pos['y'], pos['z'],
            pos['w'], pos['p'], pos['r']
        )

        self.get_logger().info('Closing gripper...')
        self._schunk('close')

        self.get_logger().info('Lifting die...')
        self._move_cart(
            pos['x'], pos['y'], pos['z'] + DICE_WIDTH,
            pos['w'], pos['p'], pos['r']
        )

        self.get_logger().info('Moving to camera present position...')
        cam = CAMERA_PRESENT_JOINTS
        self._move_joints(cam['j1'], cam['j2'], cam['j3'],
                          cam['j4'], cam['j5'], cam['j6'])

        self.get_logger().info('Test complete — stopped at camera present.')

    # ── Main sequence ──────────────────────────────────────────────────────────────────────────────
    def run(self):
        """
        Main task loop — iterate over three dice, pick each one up,
        present it to the camera, count pips, and accumulate a total.
        """
        self.go_home()

        total_pips = 0

        for i, pos in enumerate(DICE_PICK_POSITIONS):
            die_num = i + 1
            self.get_logger().info(f'')
            self.get_logger().info(f'──────────────────────────────')
            self.get_logger().info(f'  Die {die_num} of {len(DICE_PICK_POSITIONS)}')
            self.get_logger().info(f'──────────────────────────────')

            # 1. Open gripper before approaching
            self.get_logger().info('Opening gripper...')
            self._schunk('open')

            # 2. Move to approach height (above the die)
            self.get_logger().info('Moving to approach position...')
            self._move_cart(
                pos['x'], pos['y'], pos['z'] + APPROACH_OFFSET_Z,
                pos['w'], pos['p'], pos['r']
            )

            # 3. Descend to pick position
            self.get_logger().info('Descending to pick position...')
            self._move_cart(
                pos['x'], pos['y'], pos['z'],
                pos['w'], pos['p'], pos['r']
            )

            # 4. Close gripper to grasp die
            self.get_logger().info('Closing gripper...')
            self._schunk('close')

            # 5. Lift die back up to approach height
            self.get_logger().info('Lifting die...')
            self._move_cart(
                pos['x'], pos['y'], pos['z'] + APPROACH_OFFSET_Z,
                pos['w'], pos['p'], pos['r']
            )

            # 6. Move to camera present position
            self.get_logger().info('Moving to camera present position...')
            cam = CAMERA_PRESENT_JOINTS
            self._move_joints(cam['j1'], cam['j2'], cam['j3'],
                              cam['j4'], cam['j5'], cam['j6'])

            # 7. Capture a fresh frame from the MindVision camera
            self.get_logger().info('Capturing image...')
            image = self._capture_image()

            if image is None:
                self.get_logger().warn(f'Die {die_num}: no image captured — skipping pip count.')
                self._schunk('open')
                continue

            # 8. Crop to die face, then count pips
            image = self._crop_die(image)
            pip_count, debug_img = self._count_pips(image)
            total_pips += pip_count

            self.get_logger().info(
                f'Die {die_num}: {pip_count} pip(s) detected  |  Running total: {total_pips}'
            )

            # Save raw and annotated images for debugging / tuning
            raw_path   = f'{IMAGE_SAVE_DIR}die_{die_num}_raw.png'
            debug_path = f'{IMAGE_SAVE_DIR}die_{die_num}_debug.png'
            cv2.imwrite(raw_path,   image)
            cv2.imwrite(debug_path, debug_img)
            self.get_logger().info(f'Images saved: {raw_path}, {debug_path}')

            # 9. Return die to its starting position
            self.get_logger().info('Returning die to starting position...')
            self._move_cart(
                pos['x'], pos['y'], pos['z'] + APPROACH_OFFSET_Z,
                pos['w'], pos['p'], pos['r']
            )
            self._move_cart(
                pos['x'], pos['y'], pos['z'],
                pos['w'], pos['p'], pos['r']
            )

            # 10. Release die
            self.get_logger().info('Releasing die...')
            self._schunk('open')

            # 11. Lift away before moving to next die
            self._move_cart(
                pos['x'], pos['y'], pos['z'] + APPROACH_OFFSET_Z,
                pos['w'], pos['p'], pos['r']
            )

        # ── Final result ──────────────────────────────────────────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('══════════════════════════════')
        self.get_logger().info(f'  FINAL TOTAL: {total_pips} pip(s)')
        self.get_logger().info('══════════════════════════════')

        self.go_home()


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = DiceRollerNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
