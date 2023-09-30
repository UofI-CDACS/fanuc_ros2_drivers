#!/usr/bin/env python3
import sys
import os
import rclpy
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_ros2_interfaces.action import WriteJointOffset
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

drive_path = '129.101.98.214'


class write_joint_offset_server(Node):
    def __init__(self):
        super().__init__('write_joint_offset_server')

        self.goal = WriteJointOffset.Goal()
        self.bot = robot(drive_path)

        self._action_server = ActionServer(self, WriteJointOffset, 'WriteJointOffset', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback)

    def goal_callback(self, goal_request):
        # accepts or Rejects client request to begin Action
        self.goal = goal_request
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        joint = self.goal.joint
        value = self.goal.value

        print('Joint: ', joint)
        print('How far in mm: ', value)

        # Goal stuff
        self.bot.write_joint_offset(joint, value)
        self.bot.start_robot()

        goal_handle.succeed()
        result = WriteJointOffset.Result()
        result.success = True
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    write_joint_offset_action_server = write_joint_offset_server()

    rclpy.spin(write_joint_offset_action_server)

    write_joint_offset_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
