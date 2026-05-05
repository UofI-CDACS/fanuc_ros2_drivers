#!/usr/bin/env python3
"""
test_conveyor.py
----------------
Conveyor belt test.

  just conveyer         — forward → stop → reverse → stop
  just conveyer-front   — forward only (runs until Ctrl+C, then stops)
  just conveyer-back    — reverse only (runs until Ctrl+C, then stops)
"""
import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import Conveyor

PAUSE_SEC  = 2.0   # how long to run in each direction (full test only)
MODE       = sys.argv[1] if len(sys.argv) > 1 else 'test'  # 'front' | 'back' | 'test'
# Conveyor namespace: pass robot name as second arg, or fall back to env vars
if len(sys.argv) > 2:
    ROBOT_NAME = sys.argv[2]
elif MODE == 'back':
    ROBOT_NAME = os.environ.get('ROBOT_1_NAME', 'robot1')
else:
    ROBOT_NAME = os.environ.get('ROBOT_2_NAME', 'robot2')


class ConveyorTestNode(Node):

    def __init__(self):
        super().__init__('conveyor_test')
        self.ac = ActionClient(self, Conveyor, f'/{ROBOT_NAME}/conveyor')
        self.get_logger().info(f'Waiting for /{ROBOT_NAME}/conveyor action server...')
        self.ac.wait_for_server()
        self.get_logger().info('Ready.')

    def _send(self, command):
        goal = Conveyor.Goal()
        goal.command = command
        self.get_logger().info(f'Sending: {command}')
        future = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error(f'Goal "{command}" rejected.')
            return
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        self.get_logger().info(f'"{command}" done — success={rf.result().result.success}')

    def _pause(self, sec=PAUSE_SEC):
        import time
        time.sleep(sec)

    def run(self):
        if MODE == 'front':
            self._send('forward')
            self.get_logger().info('Running forward — press Ctrl+C to stop.')
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                pass
            self._send('stop')
        elif MODE == 'back':
            self._send('reverse')
            self.get_logger().info('Running reverse — press Ctrl+C to stop.')
            try:
                rclpy.spin(self)
            except KeyboardInterrupt:
                pass
            self._send('stop')
        else:
            self._send('forward');  self._pause()
            self._send('stop');     self._pause(0.5)
            self._send('reverse');  self._pause()
            self._send('stop')
            self.get_logger().info('Conveyor test complete.')


def main():
    rclpy.init()
    node = ConveyorTestNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
