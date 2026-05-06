#!/usr/bin/env python3
"""
test_conveyor.py — quick conveyor timing test

Connects to Beaker's conveyor action server and runs the belt
forward for a configurable duration, then stops.

Usage:
    python3 test_conveyor.py                  # uses defaults below
    python3 test_conveyor.py 8.0              # run for 8 seconds
    python3 test_conveyor.py 8.0 Beaker       # run for 8 seconds, robot name Beaker

Requires ROS2 and the FANUC driver nodes to be running:
    ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=10.8.4.16
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import Conveyor

# ── Edit these to tune timing ─────────────────────────────────────────────────
RUN_SECONDS = 9.9   # how long to run the belt forward
ROBOT_NAME  = 'Beaker'
# ─────────────────────────────────────────────────────────────────────────────


class ConveyorTest(Node):

    def __init__(self, robot_name: str):
        super().__init__('conveyor_test')
        self._conv = ActionClient(self, Conveyor, f'/{robot_name}/conveyor')

    def _send(self, command: str):
        self.get_logger().info(f'Sending: {command}')
        self._conv.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        fut = self._conv.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut)
        gh = fut.result()
        if not gh.accepted:
            self.get_logger().error(f'Goal rejected for command: {command}')
            return False
        res = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res)
        return res.result().result.success

    def run(self, seconds: float):
        self.get_logger().info(f'Waiting for conveyor action server...')
        if not self._conv.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                'Conveyor action server not found after 10s. '
                'Is the driver running?  '
                'ros2 launch launch/start.launch.py robot_name:=Beaker robot_ip:=10.8.4.16'
            )
            return
        self.get_logger().info(f'Server found. Running belt FORWARD for {seconds}s...')

        self._send('forward')
        time.sleep(seconds)
        self._send('stop')

        self.get_logger().info('Done. Belt stopped.')


def main():
    rclpy.init()

    seconds    = float(sys.argv[1]) if len(sys.argv) > 1 else RUN_SECONDS
    robot_name = sys.argv[2]        if len(sys.argv) > 2 else ROBOT_NAME

    node = ConveyorTest(robot_name)
    try:
        node.run(seconds)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted — stopping belt.')
        node._send('stop')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
