#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("src/dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_interfaces.action import OnRobotGripper
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

# Robot IP is passed as command line argument 1
robot_ip = sys.argv[1]
name = sys.argv[2]

# # Quick and dirty
# if robot_ip == '172.29.208.124':
# 	name = "beaker"
# elif robot_ip == '172.29.208.123':
#      name = "bunsen"
# else:
# 	name = "rogue"

class onrobot_gripper_server(Node):
    def __init__(self):
        super().__init__('onrobot_gripper_server')

        self.goal = OnRobotGripper.Goal()
        self.bot = robot(robot_ip)

        self._action_server = ActionServer(self, OnRobotGripper, f'{name}/onrobot_gripper', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        
        # Check that it recieved a valid goal
        if self.goal.width > 160 | self.goal.width < 0:
            self.get_logger().info(f'Width should be between [0,160], got: {self.goal.width}')
            return GoalResponse.REJECT
        if self.goal.force > 120 | self.goal.force < 0:
            self.get_logger().info(f'Force should be between [0,120], got: {self.goal.force}')
            return GoalResponse.REJECT
        else:
            return GoalResponse.ACCEPT
                
    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        if self.goal == None:
            self.get_logger().info('No goal to cancel...')
            return CancelResponse.REJECT
        else:
            self.get_logger().info('Received cancel request')
            goal_handle.canceled()
            return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        # WIP: Add Try/Except to catch possible error
        self.bot.onrobot_gripper(self.goal.width, self.goal.force)
        
        goal_handle.succeed()
        result = OnRobotGripper.Result()
        result.success = True
        self.goal = OnRobotGripper.Goal() # Reset
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    onrobot_gripper_action_server = onrobot_gripper_server()

    rclpy.spin(onrobot_gripper_action_server)

    onrobot_gripper_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

