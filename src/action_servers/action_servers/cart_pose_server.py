#!/usr/bin/env python3
import sys
import os
import rclpy

import dependencies.FANUCethernetipDriver as FANUCethernetipDriver

from dependencies.robot_controller import robot
from fanuc_interfaces.action import CartPose
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

class cart_pose_server(Node):
    def __init__(self):
        super().__init__('cart_pose_server')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),] # custom, default
        )

        self.goal = CartPose.Goal()
        self.bot = robot(self.get_parameter('robot_ip').value)

        self._action_server = ActionServer(self, CartPose, f'/cartesian_pose', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        
        # Check that it recieved a valid goal
        if self.goal.w > 179.9 or self.goal.w < -179.9:
            if self.goal.w == 200.0:
                self.goal.w = self.bot.read_current_cartesian_pose()[3]
            else:
                self.get_logger().info('Invalid request, W needs to be in the range of [-179.9,179.9]')
                return GoalResponse.REJECT
        
        if self.goal.p > 179.9 or self.goal.p < -179.9:
            if self.goal.p == 200.0:
                self.goal.p = self.bot.read_current_cartesian_pose()[4]
            else:
                self.get_logger().info('Invalid request, P needs to be in the range of [-179.9,179.9]')
                return GoalResponse.REJECT
        
        if self.goal.r > 179.9 or self.goal.r < -179.9:
            if self.goal.r == 200.0:
                self.goal.r = self.bot.read_current_cartesian_pose()[5]
            else:
                self.get_logger().info('Invalid request, R needs to be in the range of [-179.9,179.9]')
                return GoalResponse.REJECT
        
        # If here, all values are acceptable
        self.get_logger().info('Cart goal recieved: '+ str(self.goal))
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

    def execute_callback(self, goal_handle):
        try:
            # Create base for feedback
            feedback_msg = CartPose.Feedback()
            feedback_msg.distance_left = self.bot.read_current_cartesian_pose() # starting pose

            self.bot.write_cartesian_position([self.goal.x,
                                            self.goal.y,
                                            self.goal.z,
                                            self.goal.w, 
                                            self.goal.p,
                                            self.goal.r],
                                            blocking=False)

            while self.bot.is_moving():
                # Calculate distance left
                feedback_msg.distance_left[0] -= self.goal.x
                feedback_msg.distance_left[1] -= self.goal.y
                feedback_msg.distance_left[2] -= self.goal.z
                feedback_msg.distance_left[3] -= self.goal.w
                feedback_msg.distance_left[4] -= self.goal.p
                feedback_msg.distance_left[5] -= self.goal.r
                goal_handle.publish_feedback(feedback_msg) # Send value

                feedback_msg.distance_left = self.bot.read_current_cartesian_pose() # Update cur pos

            
            goal_handle.succeed()
            result = CartPose.Result()
            result.success = True
        except:
            goal_handle.canceled()
            result = CartPose.Result()
            result.success = False
        self.goal = CartPose.Goal() #Reset
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    cart_pose_action_server = cart_pose_server()

    rclpy.spin(cart_pose_action_server)

    cart_pose_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

