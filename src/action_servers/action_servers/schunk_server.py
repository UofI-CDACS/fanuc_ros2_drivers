#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("src/dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_interfaces.action import SchunkGripper
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class schunk_gripper_server(Node):
    def __init__(self):
        super().__init__('schunk_gripper_server')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.goal = SchunkGripper.Goal()
        self.bot = robot(self.get_parameter('robot_ip').value)

        self._action_server = ActionServer(self, SchunkGripper, f"{self.get_parameter('robot_name').value}/schunk_gripper", 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        
        # Check that it recieved a valid goal
        if self.goal.command == "open" or self.goal.command == "close":
            self.get_logger().info('Schunk goal recieved: '+ str(self.goal))
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info(f'Invalid request, got: {self.goal.command} type: {type(self.goal.command)}')
            return GoalResponse.REJECT
                
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
        self.bot.schunk_gripper(self.goal.command)

        goal_handle.succeed()
        result = SchunkGripper.Result()
        result.success = True
        self.goal = SchunkGripper.Goal() # Reset
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    schunk_gripper_action_server = schunk_gripper_server()

    rclpy.spin(schunk_gripper_action_server)

    schunk_gripper_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

