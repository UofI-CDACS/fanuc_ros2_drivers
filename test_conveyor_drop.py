#!/usr/bin/env python3
"""
test_conveyor_drop.py — move Beaker to rear conveyor drop position and run belt.

Sequence:
  1. Move to CONV_REAR_ABV  (above drop position)
  2. Move to CONV_REAR_DRP  (drop position)
  3. Open gripper
  4. Move back to CONV_REAR_ABV
  5. Run conveyor forward for RUN_SECONDS
  6. Stop conveyor

Usage:
    python3 test_conveyor_drop.py

Requires the FANUC driver running in another terminal:
    ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=10.8.4.16
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import CartPose, SchunkGripper, Conveyor

# ── Edit these ────────────────────────────────────────────────────────────────
ROBOT_NAME  = 'Beaker'
RUN_SECONDS = 9.9

CONV_REAR_ABV = dict(x=-194.112, y=617.369, z=200.840, w=179.9, p=0.0, r=120.0)
CONV_REAR_DRP = dict(x=-194.112, y=617.369, z=8.840,   w=179.9, p=0.0, r=120.0)
# ─────────────────────────────────────────────────────────────────────────────


class ConveyorDropTest(Node):

    def __init__(self, robot_name: str):
        super().__init__('conveyor_drop_test')
        self._cart    = ActionClient(self, CartPose,      f'/{robot_name}/cartesian_pose')
        self._gripper = ActionClient(self, SchunkGripper, f'/{robot_name}/schunk_gripper')
        self._conv    = ActionClient(self, Conveyor,      f'/{robot_name}/conveyor')

    def _send_cart(self, **kwargs) -> bool:
        self._cart.wait_for_server()
        goal = CartPose.Goal()
        for k, v in kwargs.items():
            setattr(goal, k, float(v))
        fut = self._cart.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error('Cartesian goal rejected')
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _send_gripper(self, command: str) -> bool:
        self._gripper.wait_for_server()
        goal = SchunkGripper.Goal()
        goal.command = command
        fut = self._gripper.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error(f'Gripper goal rejected: {command}')
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def _send_conveyor(self, command: str) -> bool:
        self._conv.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        fut = self._conv.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error(f'Conveyor goal rejected: {command}')
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def run(self, seconds: float):
        self.get_logger().info('Waiting for action servers...')
        if not self._cart.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Cartesian action server not found after 10s.')
            return
        if not self._conv.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Conveyor action server not found after 10s.')
            return

        self.get_logger().info('Moving above conveyor drop position...')
        self._send_cart(**CONV_REAR_ABV)

        self.get_logger().info('Moving to drop position...')
        self._send_cart(**CONV_REAR_DRP)

        self.get_logger().info('Opening gripper...')
        self._send_gripper('open')

        self.get_logger().info('Moving back above conveyor...')
        self._send_cart(**CONV_REAR_ABV)

        self.get_logger().info(f'Running conveyor forward for {seconds}s...')
        self._send_conveyor('forward')
        time.sleep(seconds)
        self._send_conveyor('stop')

        self.get_logger().info('Done.')


def main():
    rclpy.init()

    seconds    = float(sys.argv[1]) if len(sys.argv) > 1 else RUN_SECONDS
    robot_name = sys.argv[2]        if len(sys.argv) > 2 else ROBOT_NAME

    node = ConveyorDropTest(robot_name)
    try:
        node.run(seconds)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — stopping conveyor.')
        node._send_conveyor('stop')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
