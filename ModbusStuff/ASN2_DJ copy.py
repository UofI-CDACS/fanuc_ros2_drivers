
# ROS packages
import random
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
import sys
sys.path.append("../src/dependencies/")
from time import sleep
# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper, SJointPose

# The position where the two robots meet to handoff the object
handoff_middle_pos = [250,850,100,-90.0,-30.0,0.0]
# the handoff position that is far away safe from bumping into Bill
handoff_stay_away_pos =[250,500,100,-90.0,-30.0,0.0]


def find_random_position():
   """Find a random position near the handoff position"""
   new_pos=handoff_middle_pos.copy()
   x=handoff_middle_pos[0]
   y=handoff_middle_pos[1]
   z=handoff_middle_pos[2]
   new_pos[0]=random.uniform(x-150,x+150)
   new_pos[1]=random.uniform(y-150,y+150)
   new_pos[2]=random.uniform(z-125,z+125)
   return new_pos


namespace = 'dj'
class FanucActions(Node):
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')

    def run_test(self):
		# Cartesian
        print("Running Joint test")
        self.home_position()
        self.move_cartesian(find_random_position())
        
    def move_cartesian(self, cartesian_pose):
        print("Moving to new Cartesian Position")
        self.cart_ac.wait_for_server() # Wait till its ready
        cart_goal = CartPose.Goal() # Make goal
        cart_goal.x = cartesian_pose[0]
        cart_goal.y = cartesian_pose[1]
        cart_goal.z = cartesian_pose[2]
        cart_goal.w = cartesian_pose[3]
        cart_goal.p = cartesian_pose[4]
        cart_goal.r = cartesian_pose[5]
        future = self.cart_ac.send_goal_async(cart_goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or not future.result().accepted:
            self.get_logger().info('Goal rejected')
            return
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        future.add_done_callback(self.goal_response_callback)
    def move_joints(self, pose):
        print("Moving to new Joint Position")
        self.joints_ac.wait_for_server() # Wait till its ready
        joints_goal = JointPose.Goal() # Make goal
        joints_goal.joint1 = pose[0]
        joints_goal.joint2 = pose[1]
        joints_goal.joint3 = pose[2]
        joints_goal.joint4 = pose[3]
        joints_goal.joint5 = pose[4]
        joints_goal.joint6 = pose[5]
        future = self.joints_ac.send_goal_async(joints_goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or not future.result().accepted:
            self.get_logger().info('Goal rejected')
            return
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        future.add_done_callback(self.goal_response_callback)

    def home_position(self):
        print("Moving to Home Position")
        self.move_joints([0.0, 0.0, 0.0, 0.0, -90.0, 30.0])

    def shunk_gripper(self, command):
        print("Sending command to Schunk Gripper")
        self.schunk_ac.wait_for_server() # Wait till its ready
        schunk_goal = SchunkGripper.Goal() # Make goal
        schunk_goal.command = command
        future = self.schunk_ac.send_goal_async(schunk_goal, feedback_callback=self.feedback_callback)
        rclpy.spin_until_future_complete(self, future)
        if not future.result() or not future.result().accepted:
            self.get_logger().info('Goal rejected')
            return
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        future.add_done_callback(self.goal_response_callback)

#------- Helper functions -------------
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.distance_left))



if __name__ == '__main__':
    rclpy.init()
    fanuc = FanucActions(namespace)
    fanuc.run_test()
    rclpy.spin(fanuc)
    rclpy.shutdown()
