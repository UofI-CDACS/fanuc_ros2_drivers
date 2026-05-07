#!/usr/bin/env python3
import sys
import os
import time
import rclpy

import dependencies.FANUCethernetipDriver as FANUCethernetipDriver

from dependencies.robot_controller import robot
from fanuc_interfaces.action import JointPose
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class joint_pose_server(Node):
    def __init__(self):
        super().__init__('joint_pose_server')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.goal = JointPose.Goal()
        self.bot = robot(self.get_parameter('robot_ip').value)
        self.bot.set_speed(300)  # reset to full speed on startup

        self._action_server = ActionServer(self, JointPose, f"{self.get_parameter('robot_name').value}/joint_pose", 
                                        execute_callback = self.execute_callback, 
                                        goal_callback = self.goal_callback,
                                        cancel_callback = self.cancel_callback)

    def goal_callback(self, goal_request):
        """ Accepts or Rejects client request to begin Action """
        self.goal = goal_request 
        # FIX!! This is ugly.. Put into a list.any()? Switch is also faster
        # Check that it recieved a valid goal
        if self.goal.joint1 > 270.0 or self.goal.joint1 < -270.0:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint2 > 270.0 or self.goal.joint2 < -270.0:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint3 > 270.0 or self.goal.joint3 < -270.0:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint4 > 270.0 or self.goal.joint4 < -270.0:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint5 > 270.0 or self.goal.joint5 < -270.0:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        
        elif self.goal.joint6 > 270.0 or self.goal.joint6 < -270.0:
            self.get_logger().info('Invalid request')
            return GoalResponse.REJECT
        else:
            self.get_logger().info('Joint goal recieved: '+ str(self.goal))
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

            if self.goal.speed > 0:
                self.bot.set_speed(min(self.goal.speed, 300))
            else:
                self.bot.set_speed(300)

            self.bot.write_joint_pose(list, blocking=False)

            # Give the robot controller time to set the moving flag before polling.
            # Without this, is_moving() can return False before motion starts.
            time.sleep(0.3)

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

            # Verify robot has reached target before returning result.
            # is_moving() can return False before the robot settles, so poll
            # joint positions until all joints are within tolerance.
            TOLERANCE_DEG = 2.0
            deadline = time.time() + 30.0
            while True:
                current = self.bot.read_current_joint_position()
                if all(abs(current[i] - list[i]) < TOLERANCE_DEG for i in range(6)):
                    break
                if time.time() > deadline:
                    self.get_logger().warn('Position verification timed out — proceeding anyway')
                    break
                time.sleep(0.1)

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
