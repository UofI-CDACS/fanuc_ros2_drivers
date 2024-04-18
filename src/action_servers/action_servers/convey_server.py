#!/usr/bin/env python3
import sys
import os
import rclpy

import dependencies.FANUCethernetipDriver as FANUCethernetipDriver

from dependencies.robot_controller import robot
from fanuc_interfaces.action import Conveyor
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class convey_server(Node):
    def __init__(self):
        super().__init__('convey_server')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.goal = Conveyor.Goal()
        self.bot = robot(self.get_parameter('robot_ip').value)

        self._action_server = ActionServer(self, Conveyor, f"/{self.get_parameter('robot_name').value}/conveyor", 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        
        # Check that it recieved a valid goal
        if (self.goal.command == 'forward' or
            self.goal.command == 'reverse' or 
            self.goal.command == 'stop'):
            self.get_logger().info('Convey goal recieved: '+ str(self.goal))
            return GoalResponse.ACCEPT
        else:
            self.get_logger().info('Invalid request')
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
        self.bot.conveyor(self.goal.command)

        goal_handle.succeed()
        result = Conveyor.Result()
        result.success = True
        self.goal = Conveyor.Goal() # Reset
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    convey_action_server = convey_server()

    rclpy.spin(convey_action_server)

    convey_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

