#!/usr/bin/env python3
"""
claude_assignment.py

Executes the full dice inspection assignment:
  - Pick up die with vertical arm
  - Rotate to horizontal present position
  - Capture image and count pips
  - Set die back down
  - Repeat 3 times total
  - Track individual and running pip counts
  - Return to home

Usage:
  ros2 run dice_inspection claude_assignment
"""

import os
import sys
import time
import subprocess
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from fanuc_interfaces.action import CartPose, OnRobotGripper
from fanuc_interfaces.msg import IsMoving, CurCartesian
from fanuc_interfaces.srv import SetSpeed

# Import pip counting utilities from pip_counter.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pip_counter import capture_image, find_die_bbox, count_pips

# -----------------------------------------------------------------------
# Positions
# -----------------------------------------------------------------------
HOME          = dict(x=540.0,    y=-150.0,   z=550.0,    w=-179.9, p=0.0,   r=0.0)

PICK          = dict(x=134,  y=-845, z=250,   w=-179, p=0, r=45)
PRE_PICK      = dict(x=134,  y=-845, z=350,   w=-179, p=0, r=45)

PRE_PRESENT   = dict(x=133,  y=-672, z=379,  w=91,   p=46,  r=0.12)
PRESENT       = dict(x=134,  y=-845, z=350,  w=90,   p=45,  r=1)
PRE_SETDOWN   = dict(x=140,  y=-573, z=100,  w=89,   p=45,  r=1)
SETDOWN       = dict(x=125,  y=-582, z=-29,  w=89.1, p=45,  r=1)

# -----------------------------------------------------------------------
# Gripper parameters
# -----------------------------------------------------------------------
GRIPPER_OPEN_WIDTH  = 100
GRIPPER_OPEN_FORCE  = 20
GRIPPER_CLOSE_WIDTH = 55
GRIPPER_CLOSE_FORCE = 40

NUM_ROUNDS = 3


class ClaudeAssignment(Node):
    def __init__(self):
        super().__init__('claude_assignment')

        self.declare_parameter('robot_name', 'BILL')
        robot_name = self.get_parameter('robot_name').value

        self._cart_client = ActionClient(self, CartPose,
                                         f'/{robot_name}/cartesian_pose')
        self._grip_client = ActionClient(self, OnRobotGripper,
                                         f'{robot_name}/onrobot_gripper')

        self._is_moving = False
        self.create_subscription(IsMoving, f'{robot_name}/is_moving',
                                 self._moving_callback, 10)

        self._current_pose = None
        self.create_subscription(CurCartesian, f'{robot_name}/cur_cartesian',
                                 self._pose_callback, 10)

        self._speed_client = self.create_client(SetSpeed, f'{robot_name}/set_speed')

        self.get_logger().info(f'Claude Assignment node ready (robot: {robot_name})')

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------
    def _moving_callback(self, msg):
        self._is_moving = msg.moving

    def _pose_callback(self, msg):
        self._current_pose = msg.pose  # [x, y, z, w, p, r]

    # -----------------------------------------------------------------------
    # Speed
    # -----------------------------------------------------------------------
    def _set_speed(self, speed_mm_s: int):
        self._speed_client.wait_for_service()
        req = SetSpeed.Request()
        req.speed = speed_mm_s
        future = self._speed_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result().success:
            self.get_logger().info(f'Speed set to {speed_mm_s} mm/s')
        else:
            self.get_logger().warn(f'Failed to set speed to {speed_mm_s} mm/s')

    # -----------------------------------------------------------------------
    # Motion helpers
    # -----------------------------------------------------------------------
    def _send_cart(self, pos: dict) -> bool:
        self._cart_client.wait_for_server()
        goal = CartPose.Goal()
        goal.x = float(pos['x'])
        goal.y = float(pos['y'])
        goal.z = float(pos['z'])
        goal.w = float(pos['w'])
        goal.p = float(pos['p'])
        goal.r = float(pos['r'])
        self.get_logger().info(
            f'Moving → x={goal.x:.1f}, y={goal.y:.1f}, z={goal.z:.1f}'
        )
        future = self._cart_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Cart goal rejected!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.success
        if not success:
            self.get_logger().error('Cart move failed.')
        return success

    def _send_gripper(self, width: int, force: int) -> bool:
        self._grip_client.wait_for_server()
        goal = OnRobotGripper.Goal()
        goal.width = width
        goal.force = force
        label = 'OPEN' if width >= GRIPPER_OPEN_WIDTH else 'CLOSE'
        self.get_logger().info(f'Gripper {label} (width={width}mm, force={force}N)')
        future = self._grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def _wait_until_still(self, move_start_timeout=5.0, move_end_timeout=30.0):
        """
        Wait until the robot has moved and come to a stop.
        - If the robot is already still (already at position), return immediately.
        - If the robot hasn't started moving within move_start_timeout seconds, abort.
        - If the robot hasn't stopped within move_end_timeout seconds, abort.
        """
        # Phase 1: wait for robot to START moving
        self.get_logger().info('Waiting for robot to start moving...')
        start = time.time()
        while not self._is_moving:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > move_start_timeout:
                self.get_logger().warn(
                    'Robot did not start moving — already at position, continuing.'
                )
                return

        self.get_logger().info('Robot is moving...')

        # Phase 2: wait for robot to STOP moving
        start = time.time()
        while self._is_moving:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > move_end_timeout:
                self.get_logger().error(
                    f'Robot did not stop within {move_end_timeout}s — aborting.'
                )
                self.destroy_node()
                rclpy.shutdown()
                sys.exit(1)

        self.get_logger().info('Robot stopped.')

    # -----------------------------------------------------------------------
    # Pip capture and count
    # -----------------------------------------------------------------------
    def _capture_and_count(self, round_num: int) -> int:
        self.get_logger().info('Capturing image...')
        try:
            frame = capture_image()

            img_name = f'die_image_round{round_num}.jpg'
            proc_name = f'die_processed_round{round_num}.jpg'
            cv2.imwrite(img_name, frame)
            self.get_logger().info(f'Raw image saved: {img_name}')

            bbox, _ = find_die_bbox(frame)
            if bbox is None:
                self.get_logger().error('No yellow die detected in image!')
                return 0

            pip_count, annotated_roi = count_pips(frame, bbox)
            x, y, w, h = bbox
            output = frame.copy()
            output[y:y+h, x:x+w] = annotated_roi
            cv2.rectangle(output, (x, y), (x+w, y+h), (0, 255, 255), 3)
            cv2.putText(output, f'Round {round_num} - Pips: {pip_count}',
                        (x, max(y - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 255), 2)
            cv2.imwrite(proc_name, output)
            self.get_logger().info(f'Processed image saved: {proc_name}')
            subprocess.Popen(['eog', '--new-instance', proc_name])

            return pip_count

        except Exception as e:
            self.get_logger().error(f'Capture/count error: {e}')
            return 0

    # -----------------------------------------------------------------------
    # Main sequence
    # -----------------------------------------------------------------------
    def run(self):
        counts = []
        running_total = 0

        # Set safe speed to avoid SYST-320 contact force errors
        self.get_logger().info('=== Setting speed ===')
        self._set_speed(300)

        # HOME
        self.get_logger().info('=== HOME ===')
        self._send_cart(HOME)
        self._wait_until_still()

        # Open gripper before starting
        self.get_logger().info('=== Open gripper ===')
        self._send_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_OPEN_FORCE)

        for i in range(NUM_ROUNDS):
            round_num = i + 1
            self.get_logger().info(f'{"="*50}')
            self.get_logger().info(f'  ROUND {round_num} of {NUM_ROUNDS}')
            self.get_logger().info(f'{"="*50}')

            # PRE_PICK
            self.get_logger().info('--- PRE_PICK ---')
            self._send_cart(PRE_PICK)
            self._wait_until_still()

            # PICK
            self.get_logger().info('--- PICK ---')
            self._send_cart(PICK)
            self._wait_until_still()

            # Close gripper
            self.get_logger().info('--- Close gripper ---')
            self._send_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_CLOSE_FORCE)

            # PRE_PICK
            self.get_logger().info('--- PRE_PICK ---')
            self._send_cart(PRE_PICK)
            self._wait_until_still()

            # HOME (config transition: FUT → NUT)
            self.get_logger().info('--- HOME (transition) ---')
            self._send_cart(HOME)
            self._wait_until_still()

            # # PRE_PRESENT
            # self.get_logger().info('--- PRE_PRESENT ---')
            # self._send_cart(PRE_PRESENT)
            # self._wait_until_still()

            # PRESENT
            self.get_logger().info('--- PRESENT ---')
            self._send_cart(PRESENT)
            self._wait_until_still()

            # Capture image, count pips, print results
            self.get_logger().info('--- Capture and count ---')
            pip_count = self._capture_and_count(round_num)
            counts.append(pip_count)
            running_total += pip_count
            print(f'\n  Round {round_num} pip count : {pip_count}')
            print(f'  Running total             : {running_total}\n')
            self.get_logger().info(f'Round {round_num}: {pip_count} pip(s) | Running total: {running_total}')

            # PRE_SETDOWN
            self.get_logger().info('--- PRE_SETDOWN ---')
            self._send_cart(PRE_SETDOWN)
            self._wait_until_still()

            # SETDOWN
            self.get_logger().info('--- SETDOWN ---')
            self._send_cart(SETDOWN)
            self._wait_until_still()

            # Open gripper
            self.get_logger().info('--- Open gripper ---')
            self._send_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_OPEN_FORCE)

            # PRE_SETDOWN
            self.get_logger().info('--- PRE_SETDOWN ---')
            self._send_cart(PRE_SETDOWN)
            self._wait_until_still()

            # HOME (config transition: NUT → FUT)
            self.get_logger().info('--- HOME (transition) ---')
            self._send_cart(HOME)
            self._wait_until_still()

        # HOME
        self.get_logger().info('=== HOME ===')
        self._send_cart(HOME)
        self._wait_until_still()

        # Final report
        self._report(counts, running_total)

    def _report(self, counts: list, total: int):
        line = '=' * 45
        print(f'\n{line}')
        print('       FINAL DICE INSPECTION RESULTS')
        print(line)
        for i, c in enumerate(counts, start=1):
            print(f'  Round {i}: {c} pip(s)')
        print(f'  {"─"*35}')
        print(f'  Total across {NUM_ROUNDS} rounds: {total} pip(s)')
        print(f'{line}\n')

        self.get_logger().info(line)
        self.get_logger().info('FINAL RESULTS')
        for i, c in enumerate(counts, start=1):
            self.get_logger().info(f'  Round {i}: {c} pip(s)')
        self.get_logger().info(f'  Total: {total} pip(s)')
        self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = ClaudeAssignment()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
