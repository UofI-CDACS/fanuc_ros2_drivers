#!/usr/bin/env python3
"""
test_gripper.py
---------------
Test Schunk or OnRobot gripper commands.

Usage (via just):
    just open-schunk        — open Robot 1 Schunk gripper
    just close-schunk       — close Robot 1 Schunk gripper
    just open-onrobot       — open Robot 2 OnRobot gripper
    just close-onrobot      — close Robot 2 OnRobot gripper

Args: <gripper> <command> <robot_name>
  gripper  : schunk | onrobot
  command  : open | close
  robot_name: robot name (from env)
"""
import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from fanuc_interfaces.action import SchunkGripper, OnRobotGripper

if len(sys.argv) < 4:
    print('Usage: test_gripper.py <schunk|onrobot> <open|close> <robot_name>')
    sys.exit(1)

GRIPPER     = sys.argv[1]   # 'schunk' or 'onrobot'
COMMAND     = sys.argv[2]   # 'open' or 'close'
ROBOT_NAME  = sys.argv[3]

# OnRobot open/close widths (mm) and force (N)
ONROBOT_OPEN_WIDTH  = 100
ONROBOT_CLOSE_WIDTH = 5
ONROBOT_FORCE       = 40


class GripperTestNode(Node):

    def __init__(self):
        super().__init__('gripper_test')

        if GRIPPER == 'schunk':
            topic = f'/{ROBOT_NAME}/schunk_gripper'
            self.ac = ActionClient(self, SchunkGripper, topic)
            self.get_logger().info(f'Waiting for {topic}...')
            self.ac.wait_for_server()
        elif GRIPPER == 'onrobot':
            topic = f'/{ROBOT_NAME}/onrobot_gripper'
            self.ac = ActionClient(self, OnRobotGripper, topic)
            self.get_logger().info(f'Waiting for {topic}...')
            self.ac.wait_for_server()
        else:
            self.get_logger().error(f'Unknown gripper: {GRIPPER}')
            raise SystemExit(1)

        self.get_logger().info('Ready.')

    def run(self):
        if GRIPPER == 'schunk':
            goal = SchunkGripper.Goal()
            goal.command = COMMAND
        else:
            goal = OnRobotGripper.Goal()
            goal.width = ONROBOT_OPEN_WIDTH if COMMAND == 'open' else ONROBOT_CLOSE_WIDTH
            goal.force = ONROBOT_FORCE

        self.get_logger().info(f'{GRIPPER} {COMMAND}...')
        future = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        gh = future.result()
        if not gh.accepted:
            self.get_logger().error('Goal rejected.')
            return
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf)
        self.get_logger().info(f'Done — success={rf.result().result.success}')


def main():
    rclpy.init()
    node = GripperTestNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
