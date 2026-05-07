#!/usr/bin/env python3
"""
dj_cam_test.py  —  DJ camera presentation + pip search test

Ask user for target pip (1-6), then loop:
  - Normal pickup → present → set down, up to 3 attempts
  - If not found after 3, do one twist pickup → present → set down
  - Continue normal pickup → present → set down until found
When target pip found: set down, re-pick vertically, place on conveyor, go home.

Run:
    python3 tests/dj_cam_test.py
"""

import sys
import time
import threading

import cv2
import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from fanuc_interfaces.action import JointPose, SchunkGripper

sys.path.insert(0, 'src/dice_game/dice_game')
import mvsdk

# ── Joint positions ───────────────────────────────────────────────────────────
CAM_PICKUP           = [71.35, 42.61, -25.51, -0.91, -66.07, 44.90]   # TODO
CAM_PICKUP_UP        = [71.35, 39.17, -15.94, -0.86, -75.64, 44.75]   # TODO
CAM_PICKUP_TWIST     = [69.43, 40.24, -27.88, -1.85, -61.39, -41.50]   # TODO
CAM_PICKUP_TWIST_UP  = [69.43, 36.39, -17.89, -1.71, -71.37, -41.84]   # TODO
CAM_PRESENT          = [65.87, 20.85, -27.87, -135.54, -37.71, 168.74]   # TODO
SETDOWN              = [34.03, 53.90, -75.45, -122.12, -83.09, 194.72]   # TODO
SETDOWN_UP           = [34.30, 42.04, -65.02, -120.87, -77.55, 185.76]   # TODO
PRE_SET_ON_CONVEYOR  = [128.42, 36.69, -0.81, -1.22, -89.58, -9.54]   # TODO
SET_ON_CONVEYOR      = [128.42, 38.38, -13.94, -1.26, -76.45, -9.25]   # TODO
PRE_PICK_DJ_CONVEYOR      = [114.37, 22.08, -11.75, -1.73, -78.81, 0.68]   # TODO
PICK_DJ_CONVEYOR          = [114.36, 25.1, -23.46, -1.84, -67.11, 1.06]   # TODO
INTERMEDIATE_DJ_CONVEYOR  = [88.60, 26.60, -7.94, -0.69, -83.14, 30.37]   # TODO
HOME                 = [0.0, 0.0, 0.0, 0.0, -90.0, 30.0]   # TODO

# ── Gripper ───────────────────────────────────────────────────────────────────
# Schunk gripper uses 'open' / 'close' commands (no width/force)

MOTION_COMPLETE_TIMEOUT = 60.0


class DjCamTest(Node):
    def __init__(self):
        super().__init__('dj_cam_test')
        cb = ReentrantCallbackGroup()
        self._joints  = ActionClient(self, JointPose,     'DJ/joint_pose',      callback_group=cb)
        self._gripper = ActionClient(self, SchunkGripper, 'DJ/schunk_gripper',  callback_group=cb)
        self._annotated = None

    # ── ROS2 helpers ──────────────────────────────────────────────────────────
    def _wait(self, future, timeout=30.0):
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                raise TimeoutError('Future timed out')
            time.sleep(0.02)
        return future.result()

    def _move(self, joints: list, speed: int = 0):
        self.get_logger().info(f'Move → {joints}' + (f' @ {speed}mm/s' if speed else ''))
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = joints[0], joints[1], joints[2]
        goal.joint4, goal.joint5, goal.joint6 = joints[3], joints[4], joints[5]
        goal.speed = speed
        self._joints.wait_for_server()
        gh = self._wait(self._joints.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError('Joint move rejected')
        self._wait(gh.get_result_async(), timeout=MOTION_COMPLETE_TIMEOUT)

    def _set_gripper(self, command: str):
        """command: 'open' or 'close'"""
        goal = SchunkGripper.Goal()
        goal.command = command
        self._gripper.wait_for_server()
        gh = self._wait(self._gripper.send_goal_async(goal))
        if not gh.accepted:
            raise RuntimeError('Gripper goal rejected')
        self._wait(gh.get_result_async())
        self.get_logger().info(f'Gripper {command.upper()}')
        time.sleep(2.0)

    # ── Camera ────────────────────────────────────────────────────────────────
    def _capture_image(self) -> np.ndarray:
        devs = mvsdk.CameraEnumerateDevice()
        if len(devs) < 1:
            raise RuntimeError('No MindVision camera found. Check ethernet cable.')

        hCamera = None
        for attempt in range(5):
            try:
                hCamera = mvsdk.CameraInit(devs[0], -1, -1)
                break
            except Exception as e:
                self.get_logger().warn(f'CameraInit attempt {attempt+1} failed: {e}')
                time.sleep(2.0)
        if hCamera is None:
            raise RuntimeError('CameraInit failed after 5 attempts')

        cap = mvsdk.CameraGetCapability(hCamera)
        mono = (cap.sIspCapacity.bMonoSensor != 0)
        fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
        mvsdk.CameraSetIspOutFormat(hCamera, fmt)
        mvsdk.CameraSetTriggerMode(hCamera, 0)
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
        mvsdk.CameraPlay(hCamera)

        channels = 1 if mono else 3
        buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * channels
        pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (FrameHead.iHeight, FrameHead.iWidth, channels))
            return frame.copy()
        finally:
            mvsdk.CameraUnInit(hCamera)
            mvsdk.CameraAlignFree(pFrameBuffer)

    def _count_pips(self, image: np.ndarray) -> int:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([15, 60, 40]), np.array([45, 255, 255]))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn('No yellow die detected in image')
            self._annotated = image.copy()
            return -1

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        margin = 8
        x = max(0, x - margin);  y = max(0, y - margin)
        w = min(image.shape[1] - x, w + 2 * margin)
        h = min(image.shape[0] - y, h + 2 * margin)

        roi = image[y:y+h, x:x+w].copy()
        roi_bright = cv2.convertScaleAbs(roi, alpha=3.5, beta=50)
        gray = cv2.cvtColor(roi_bright, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)

        _, dark_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_adapt = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 8)
        dark = cv2.bitwise_or(dark_otsu, dark_adapt)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

        pip_contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        border_x = int(w * 0.12);  border_y = int(h * 0.12)
        pip_count = 0
        annotated = image.copy()
        cv2.rectangle(annotated, (x, y), (x+w, y+h), (255, 0, 0), 2)

        for c in pip_contours:
            area = cv2.contourArea(c)
            if area < 20 or area > 6000:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0 or (4 * np.pi * area / (perimeter ** 2)) <= 0.55:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00']) + x
            cy = int(M['m01'] / M['m00']) + y
            if (cx - x) < border_x or (cx - x) > w - border_x:
                continue
            if (cy - y) < border_y or (cy - y) > h - border_y:
                continue
            pip_count += 1
            cv2.drawContours(annotated, [c + np.array([[[x, y]]])], -1, (0, 255, 0), 2)
            cv2.circle(annotated, (cx, cy), 4, (0, 0, 255), -1)

        cv2.putText(annotated, f'pips: {pip_count}', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
        self._annotated = annotated
        return pip_count

    def _capture_and_show(self, label: str, target: int = 0) -> int:
        """Capture, count, show annotated pop-up for 2 seconds, return pip count."""
        self.get_logger().info(f'Capturing ({label})...')
        frame = self._capture_image()
        count = self._count_pips(frame)
        self.get_logger().info(f'Pip count: {count}')
        cv2.imwrite('dice_capture.jpg', frame)
        cv2.imwrite('dice_annotated.jpg', self._annotated)
        rgb = cv2.cvtColor(self._annotated, cv2.COLOR_BGR2RGB)
        fig, ax = plt.subplots()
        ax.imshow(rgb)
        ax.set_title(f'Pips: {count}  (target: {target})')
        ax.axis('off')
        plt.tight_layout()
        plt.show(block=False)
        plt.pause(2.0)
        plt.close(fig)
        return count

    # ── Movement sub-routines ─────────────────────────────────────────────────
    def _pickup_from_bill_conveyor_present(self, label: str, target: int) -> int:
        """Pick die off DJ's conveyor, move to cam_present, capture. Still holding die."""
        self._move(HOME, speed=300)
        self._set_gripper('open')
        self._move(PRE_PICK_DJ_CONVEYOR)
        self._move(PICK_DJ_CONVEYOR)
        self._set_gripper('close')
        self._move(PRE_PICK_DJ_CONVEYOR)
        self._move(INTERMEDIATE_DJ_CONVEYOR)
        self._move(CAM_PICKUP_UP)
        self._move(CAM_PRESENT, speed=50)
        return self._capture_and_show(label, target)

    def _normal_pickup_present(self, label: str, target: int) -> int:
        """Pick up normally, move to cam_present, capture. Returns pip count. Still holding die."""
        self._move(CAM_PICKUP_UP, speed=300)
        self._move(CAM_PICKUP)
        self._set_gripper('close')
        self._move(CAM_PICKUP_UP)
        self._move(CAM_PRESENT, speed=50)
        return self._capture_and_show(label, target)

    def _set_down(self):
        """Set die back down on table from cam_present position."""
        self._move(SETDOWN_UP, speed = 300)
        self._move(SETDOWN)
        self._set_gripper('open')
        self._move(SETDOWN_UP)

    def _twist_pickup_present(self, label: str, target: int) -> int:
        """Pick up with twist grip, move to cam_present, capture. Returns pip count. Still holding die."""
        self._move(CAM_PICKUP_TWIST_UP, speed=300)
        self._move(CAM_PICKUP_TWIST)
        self._set_gripper('close')
        self._move(CAM_PICKUP_TWIST_UP)
        self._move(CAM_PICKUP_UP)
        self._move(CAM_PRESENT, speed=50)
        return self._capture_and_show(label, target)

    def _place_on_conveyor(self):
        """Set die down normally, re-pick vertically, then place on conveyor."""
        self._set_down()
        self._move(CAM_PICKUP_TWIST_UP, speed=300)
        self._move(CAM_PICKUP_TWIST)
        self._set_gripper('close')
        self._move(CAM_PICKUP_TWIST_UP)
        self._move(INTERMEDIATE_DJ_CONVEYOR)
        self._move(PRE_SET_ON_CONVEYOR)
        self._move(SET_ON_CONVEYOR)
        self._set_gripper('open')
        self._move(PRE_SET_ON_CONVEYOR)

    # ── Main sequence ─────────────────────────────────────────────────────────
    def run(self):
        while True:
            try:
                target = int(input('Enter target pip count (1-6): '))
                if 1 <= target <= 6:
                    break
                print('Please enter a number between 1 and 6.')
            except ValueError:
                print('Invalid input. Please enter a number.')

        self.get_logger().info(f'=== Searching for pip {target} ===')

        self._set_gripper('open')

        attempt = 0
        twist_done = False
        capture_num = 0

        # First pick is from DJ's conveyor
        capture_num += 1
        count = self._pickup_from_bill_conveyor_present(f'bill_conveyor_{capture_num}', target)
        if count == target:
            self.get_logger().info(f'Target pip {target} found on first pick! Placing on conveyor.')
            self._place_on_conveyor()
            self._move(HOME)
            self.get_logger().info('=== Done — die placed on conveyor, robot at home ===')
            return
        self._set_down()
        attempt += 1

        while True:
            # After 3 failed normal attempts, do one twist if not already done
            if attempt == 4 and not twist_done:
                self.get_logger().info('3 normal attempts failed — trying twist grip')
                capture_num += 1
                count = self._twist_pickup_present(f'twist_{capture_num}', target)
                if count == target:
                    self.get_logger().info(f'Target pip {target} found on twist! Placing on conveyor.')
                    self._place_on_conveyor()
                    break
                self._set_down()
                twist_done = True

            # Normal pickup
            capture_num += 1
            count = self._normal_pickup_present(f'normal_{capture_num}', target)
            if count == target:
                self.get_logger().info(f'Target pip {target} found! Placing on conveyor.')
                self._place_on_conveyor()
                break
            self._set_down()
            attempt += 1

        self._move(HOME)
        self.get_logger().info('=== Done — die placed on conveyor, robot at home ===')


def main(args=None):
    rclpy.init(args=args)
    node = DjCamTest()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
