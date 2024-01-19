#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("src/dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_interfaces.action import CartPose
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

# Robot IP is passed as command line argument 1
robot_ip = sys.argv[1]

# Quick and dirty
if robot_ip == '172.29.208.124':
	name = "beaker"
elif robot_ip == '172.29.208.123':
     name = "bunsen"
else:
	name = "rogue"

class cart_pose_server(Node):
    def __init__(self):
        super().__init__('cart_pose_server')

        self.goal = CartPose.Goal()
        self.bot = robot(robot_ip)

        self._action_server = ActionServer(self, CartPose, f'{name}/cartesian_pose', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        
        # Check that it recieved a valid goal
        if self.goal.w > 179.9 or self.goal.w < -179.9:
            if self.goal.w == 200:
                self.goal.w = self.bot.read_current_cartesian_pose()[5]
            else:
                self.get_logger().info('Invalid request, W needs to be in the range of [-179.9,179.9]')
                return GoalResponse.REJECT
        
        if self.goal.p > 179.9 or self.goal.p < -179.9:
            if self.goal.p == 200:
                self.goal.p = self.bot.read_current_cartesian_pose()[6]
            else:
                self.get_logger().info('Invalid request, P needs to be in the range of [-179.9,179.9]')
                return GoalResponse.REJECT
        
        if self.goal.r > 179.9 or self.goal.r < -179.9:
            if self.goal.r == 200:
                self.goal.r = self.bot.read_current_cartesian_pose()[7]
            else:
                self.get_logger().info('Invalid request, R needs to be in the range of [-179.9,179.9]')
                return GoalResponse.REJECT
        
        # If here, all values are acceptable
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
        # WIP: Add Try/Except to catch possible errors
        # Create base for feedback
        feedback_msg = CartPose.Feedback()
        feedback_msg.distance_left = self.bot.read_current_cartesian_pose()[2:8] # starting pose

        self.bot.write_cartesian_position(self.goal.x,
                                          self.goal.y,
                                          self.goal.z,
                                          self.goal.w, 
                                          self.goal.p,
                                          self.goal.r)
        self.bot.start_robot(blocking=False)

        while self.bot.is_moving():
            # Calculate distance left
            feedback_msg.distance_left[0] -= self.goal.x
            feedback_msg.distance_left[1] -= self.goal.y
            feedback_msg.distance_left[2] -= self.goal.z
            feedback_msg.distance_left[3] -= self.goal.w
            feedback_msg.distance_left[4] -= self.goal.p
            feedback_msg.distance_left[5] -= self.goal.r
            goal_handle.publish_feedback(feedback_msg) # Send value

            feedback_msg.distance_left = self.bot.read_current_cartesian_pose()[2:8] # Update cur pos

        goal_handle.succeed()
        result = CartPose.Result()
        result.success = True
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
