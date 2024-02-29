#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("src/dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_interfaces.action import JointPose
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

class joint_pose_server(Node):
    def __init__(self):
        super().__init__('joint_pose_server')

        self.goal = JointPose.Goal()
        self.bot = robot(robot_ip)

        self._action_server = ActionServer(self, JointPose, f'{name}/joint_pose', 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        # FIX!! This is ugly.. Put into a list.any()? Switch is also faster
        # Check that it recieved a valid goal
        if self.goal.joint1 > 179.9 or self.goal.joint1 < -179.9:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint2 > 179.9 or self.goal.joint2 < -179.9:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint3 > 179.9 or self.goal.joint3 < -179.9:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint4 > 179.9 or self.goal.joint4 < -179.9:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint5 > 179.9 or self.goal.joint5 < -179.9:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint6 > 179.9 or self.goal.joint6 < -179.9:
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
        try:
            feedback_msg = JointPose.Feedback()
            feedback_msg.distance_left = self.bot.read_current_joint_position() # starting pose

            list = [self.goal.joint1,
                    self.goal.joint2,
                    self.goal.joint3,
                    self.goal.joint4, 
                    self.goal.joint5,
                    self.goal.joint6]
            
            self.bot.write_joint_pose(list, blocking=False)

            while self.bot.is_moving():
                # Calculate distance left
                feedback_msg.distance_left[0] -= self.goal.joint1
                feedback_msg.distance_left[1] -= self.goal.joint2
                feedback_msg.distance_left[2] -= self.goal.joint3
                feedback_msg.distance_left[3] -= self.goal.joint4
                feedback_msg.distance_left[4] -= self.goal.joint5
                feedback_msg.distance_left[5] -= self.goal.joint6
                goal_handle.publish_feedback(feedback_msg) # Send value

                feedback_msg.distance_left = self.bot.read_current_joint_position() # Update cur pos

            goal_handle.succeed()
            result = JointPose.Result()
            result.success = True
        except:
            goal_handle.canceled()
            result = JointPose.Result()
            result.success = False
        self.goal = JointPose.Goal() # Reset
        return result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()


def main(args=None):
    rclpy.init()

    joint_pose_action_server = joint_pose_server()

    rclpy.spin(joint_pose_action_server)

    joint_pose_action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
