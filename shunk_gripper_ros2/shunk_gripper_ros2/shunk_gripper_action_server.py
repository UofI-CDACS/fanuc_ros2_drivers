#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("../../dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_ros2_interfaces.action import ShunkGripper
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

#drive_path = '129.101.98.214'
# Robot IP is passed as command line argument 1
robot_ip = sys.argv[1]


class shunk_gripper_server(Node):
    def __init__(self):
        super().__init__('shunk_gripper_server')

        self.goal = ShunkGripper.Goal()
        self.bot = robot(robot_ip)

        self._action_server = ActionServer(self, ShunkGripper, 'ShunkGripper', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback)

    def goal_callback(self, goal_request):
        # accepts or Rejects client request to begin Action
        self.goal = goal_request
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        command = self.goal.command

        # Goal stuff
        self.bot.shunk_gripper(command)

        goal_handle.succeed()
        result = ShunkGripper.Result()
        result.success = True
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    shunk_gripper_action_server = shunk_gripper_server()

    rclpy.spin(shunk_gripper_action_server)

    shunk_gripper_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
