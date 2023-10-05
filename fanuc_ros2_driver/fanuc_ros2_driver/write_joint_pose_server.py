#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("../../dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_ros2_interfaces.action import WriteJointPose
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

#drive_path = '129.101.98.214'
# Robot IP is passed as command line argument 1
robot_ip = sys.argv[1]


class write_joint_pose_server(Node):
    def __init__(self):
        super().__init__('write_joint_pose_server')

        self.goal = WriteJointOffset.Goal()
        self.bot = robot(robot_ip)

        self._action_server = ActionServer(self, WriteJointPose, 'WriteJointPose', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback)

    def goal_callback(self, goal_request):
        # accepts or Rejects client request to begin Action
        self.goal = goal_request
        return GoalResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        joint_position_array = self.goal.joint_position_array

        # Goal stuff
        self.bot.write_joint_pose(joint_position_array)
        self.bot.start_robot()

        goal_handle.succeed()
        result = WriteJointPose.Result()
        result.success = True
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    write_joint_pose_action_server = write_joint_pose_server()

    rclpy.spin(write_joint_pose_action_server)

    write_joint_pose_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
