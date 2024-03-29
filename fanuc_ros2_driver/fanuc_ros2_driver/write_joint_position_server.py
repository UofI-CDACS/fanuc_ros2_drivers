#!/usr/bin/env python3
import sys
import os
import rclpy
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_ros2_interfaces.action import WriteJointPosition
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

#drive_path = '129.101.98.215'
# Robot IP is passed as command line argument 1
robot_ip = sys.argv[1]


class write_joint_position_server(Node):
    def __init__(self):
        super().__init__('write_joint_position_server')

        self.goal = WriteJointPosition.Goal()
        self.bot = robot(robot_ip)

        self._action_server = ActionServer(self, WriteJointPosition, 'WriteJointPosition', 
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
        print('Set to angle: ', value)

        # Goal stuff
        self.bot.write_joint_position(joint, value)
        self.bot.start_robot()

        goal_handle.succeed()
        result = WriteJointPosition.Result()
        result.success = True
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    write_joint_position_action_server = write_joint_position_server()

    rclpy.spin(write_joint_position_action_server)

    write_joint_position_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
