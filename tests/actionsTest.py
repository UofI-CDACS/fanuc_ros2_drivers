"""
This is a test of all the actions and (if added) their feedbacks
"""
# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper, SJointPose

namespace = 'beaker'

class FanucActions(Node):
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.convey_ac = ActionClient(self, Conveyor, f'/{namespace}/conveyor')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.sin_joint_ac = ActionClient(self, SJointPose, f'/{namespace}/single_joint_pose')
		
    def run_test(self):
		# Cartesian
        self.cart_ac.wait_for_server() # Wait till its ready
        cart_goal = CartPose.Goal() # Make goal
		# Add all coordinates 
        cart_goal.x = 110.77
        cart_goal.y = 672
        cart_goal.z = -102.75
        cart_goal.w = 170
        cart_goal.p = 0
        cart_goal.r = 30
        future = self.cart_ac.send_goal_async(cart_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback) 
        # New goal with no change to WPR values
        self.cart_ac.wait_for_server() # Wait till its ready
        cart_goal_2 = CartPose.Goal()
        cart_goal_2.x = 100
        cart_goal_2.y = 600
        cart_goal_2.z = -90
        self.cart_ac.send_goal(cart_goal_2)
		

        # Conveyor
        self.convey_ac.wait_for_server()
        convey_goal = Conveyor.Goal()
        convey_goal.command = 'forward'
        self.convey_ac.send_goal(convey_goal)
        convey_goal = Conveyor.Goal()
        convey_goal.command = 'stop'
        self.convey_ac.send_goal(convey_goal)


		# Joints
        self.joints_ac.wait_for_server()
        joint_goal = JointPose.Goal()
        # Add all joints
        joint_goal.joint1 = 90
        joint_goal.joint2 = 18
        joint_goal.joint3 = -41
        joint_goal.joint4 = -2
        joint_goal.joint5 = -48
        joint_goal.joint6 = -148
        future = self.cart_ac.send_goal_async(joint_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)


        # Schunk (both because unknown current state)
        self.schunk_ac.wait_for_server()
        schunk_goal = SchunkGripper.Goal()
        schunk_goal.command = "open"
        self.schunk_ac.send_goal(schunk_goal)
        schunk_goal = SchunkGripper.Goal()
        schunk_goal.command = "closed"
        self.schunk_ac.send_goal(schunk_goal)


        # Single Joints
        self.sin_joint_ac.wait_for_server()
        sjoint_goal = SJointPose.Goal()
        sjoint_goal.joint = 1
        sjoint_goal.angle = 45
        future = self.cart_ac.send_goal_async(sjoint_goal, feedback_callback=self.feedback_callback)
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
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.distance_left)



if __name__ == '__main__':
    rclpy.init()
	
    fanuc = FanucActions(namespace)
	
    rclpy.spin_until_future_complete(fanuc, fanuc.run_test())
    rclpy.shutdown()