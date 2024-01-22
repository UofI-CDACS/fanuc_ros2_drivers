#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("src/dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_interfaces.action import SJointPose
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

# Robot IP is passed as command line argument 1
# robot_ip = sys.argv[1]

# # Quick and dirty
# if robot_ip == '172.29.208.124':
# 	name = "beaker"
# elif robot_ip == '172.29.208.123':
#      name = "bunsen"
# else:
# 	name = "rogue"

class sjoint_pose_server(Node):
    def __init__(self):
        super().__init__('sjoint_pose_server')

        self.declare_parameter('robot_ip', rclpy.Parameter.Type.STRING) 
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.goal = SJointPose.Goal()
        self.bot = robot(robot_ip)

        self._action_server = ActionServer(self, SJointPose, f'/single_joint_pose', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        
        # Check that it recieved a valid goal
        if self.goal.angle > 179.9 or self.goal.angle < -179.9:
            self.get_logger().info('Invalid request')
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
        feedback_msg = SJointPose.Feedback()
        temp = self.bot.read_current_joint_position()
        feedback_msg.distance_left = temp[self.goal.joint + 1] # starting pose
        
        self.bot.write_joint_position(self.goal.joint, self.goal.angle)
        self.bot.start_robot(blocking=False)

        while self.bot.is_moving():
            # Calculate distance left
            feedback_msg.distance_left -= self.goal.angle
            goal_handle.publish_feedback(feedback_msg) # Send value

            temp = self.bot.read_current_joint_position()
            feedback_msg.distance_left = temp[self.goal.joint + 1] # Update cur pos


        goal_handle.succeed()
        result = SJointPose.Result()
        result.success = True
        self.goal = SJointPose.Goal()
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    sjoint_pose_action_server = sjoint_pose_server()

    rclpy.spin(sjoint_pose_action_server)

    sjoint_pose_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
