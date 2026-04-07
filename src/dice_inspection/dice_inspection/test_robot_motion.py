#!/usr/bin/env python3
"""
test_robot_motion.py

Tests basic robot motion and gripper via ROS2:
  1. Read current cartesian position
  2. Move UP 300 mm (Z + 300)
  3. Close gripper
  4. Wait 2 seconds
  5. Open gripper
  6. Wait 2 seconds
  7. Move DOWN 300 mm (Z - 300, back to start)

Usage:
  ros2 run dice_inspection test_robot_motion
  or
  python3 test_robot_motion.py  (with ROS2 sourced)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from fanuc_interfaces.action import CartPose, OnRobotGripper
from fanuc_interfaces.msg import CurCartesian

GRIPPER_OPEN_WIDTH  = 100
GRIPPER_OPEN_FORCE  = 20
GRIPPER_CLOSE_WIDTH = 55
GRIPPER_CLOSE_FORCE = 40


class RobotMotionTest(Node):
    def __init__(self):
        super().__init__('robot_motion_test')

        self.declare_parameter('robot_name', 'BILL')
        robot_name = self.get_parameter('robot_name').value

        self._cart_client = ActionClient(self, CartPose,
                                         f'/{robot_name}/cartesian_pose')
        self._grip_client = ActionClient(self, OnRobotGripper,
                                         f'{robot_name}/onrobot_gripper')

        # Subscribe to current cartesian position
        self._current_pose = None
        self.create_subscription(
            CurCartesian,
            f'{robot_name}/cur_cartesian',
            self._pose_callback,
            10
        )

        # Subscribe to is_moving topic for completion checks
        from fanuc_interfaces.msg import IsMoving
        self._is_moving = True
        self.create_subscription(
            IsMoving,
            f'{robot_name}/is_moving',
            self._moving_callback,
            10
        )

        self.get_logger().info(f'Robot motion test node ready (robot: {robot_name})')

    def _pose_callback(self, msg):
        self._current_pose = list(msg.pose)

    def _moving_callback(self, msg):
        self._is_moving = msg.moving

    def _wait_until_still(self, timeout=15.0):
        """Block until is_moving is False — confirms the robot has fully stopped."""
        self.get_logger().info('Waiting for robot to stop moving...')
        # Give the robot a moment to actually start moving before we check
        time.sleep(0.5)
        start = time.time()
        while self._is_moving:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().warn('Timeout waiting for robot to stop — continuing anyway.')
                return
        self.get_logger().info('Robot confirmed still.')

    def _wait_for_pose(self, timeout=5.0):
        """Block until we receive a current pose from the publisher."""
        self.get_logger().info('Waiting for current cartesian position...')
        start = time.time()
        while self._current_pose is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                raise TimeoutError('Timed out waiting for current cart position.')
        return self._current_pose

    def _send_cart(self, x, y, z, w, p, r) -> bool:
        self._cart_client.wait_for_server()
        goal = CartPose.Goal()
        goal.x = float(x)
        goal.y = float(y)
        goal.z = float(z)
        goal.w = float(w)
        goal.p = float(p)
        goal.r = float(r)
        self.get_logger().info(f'Moving to Cart: x={x:.1f}, y={y:.1f}, z={z:.1f}')
        future = self._cart_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Cart goal rejected!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        success = result_future.result().result.success
        if success:
            self.get_logger().info('Move complete.')
        else:
            self.get_logger().error('Move failed.')
        return success

    def _send_gripper(self, width, force) -> bool:
        self._grip_client.wait_for_server()
        goal = OnRobotGripper.Goal()
        goal.width = width
        goal.force = force
        action = 'OPEN' if width >= GRIPPER_OPEN_WIDTH else 'CLOSE'
        self.get_logger().info(f'Gripper {action} (width={width}mm, force={force}N)')
        future = self._grip_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Gripper goal rejected!')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.success

    def run(self):
        # 1. Read current position
        pose = self._wait_for_pose()
        x, y, z, w, p, r = pose
        self.get_logger().info(
            f'Current position: x={x:.1f}, y={y:.1f}, z={z:.1f}, '
            f'w={w:.1f}, p={p:.1f}, r={r:.1f}'
        )

        # 2. Move UP 300 mm
        self.get_logger().info('--- Moving UP 300 mm ---')
        self._send_cart(x, y, z + 300.0, w, p, r)
        self._wait_until_still()

        # 3. Close gripper
        self.get_logger().info('--- Closing gripper ---')
        self._send_gripper(GRIPPER_CLOSE_WIDTH, GRIPPER_CLOSE_FORCE)

        # 4. Wait 2 seconds
        self.get_logger().info('Waiting 2 seconds...')
        time.sleep(2.0)

        # 5. Open gripper
        self.get_logger().info('--- Opening gripper ---')
        self._send_gripper(GRIPPER_OPEN_WIDTH, GRIPPER_OPEN_FORCE)

        # 6. Wait 2 seconds
        self.get_logger().info('Waiting 2 seconds...')
        time.sleep(2.0)

        # 7. Move DOWN 300 mm (back to start)
        self.get_logger().info('--- Moving DOWN 300 mm (returning to start) ---')
        self._send_cart(x, y, z, w, p, r)
        self._wait_until_still()

        self.get_logger().info('Motion test complete.')


def main(args=None):
    rclpy.init(args=args)
    node = RobotMotionTest()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
