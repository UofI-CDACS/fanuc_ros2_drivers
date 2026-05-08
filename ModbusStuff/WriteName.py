
# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import sys
sys.path.append("../src/dependencies/")
from time import sleep
# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper, SJointPose

namespace = 'Bill'
class FanucActions(Node):
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
    def run_test(self):
		# Cartesian
        print("Running Joint test")
        """self.joints_ac.wait_for_server()
        joint_goal = JointPose.Goal()
        # Add all joints
        joint_goal.joint1 = 0.0
        joint_goal.joint2 = 0.0
        joint_goal.joint3 = 0.0
        joint_goal.joint4 = 0.0
        joint_goal.joint5 = -90.0
        joint_goal.joint6 = 30.0
        future = self.joints_ac.send_goal_async(joint_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)"""
        sleep(5)
        print("Writing M")
        self.move_cartesian(509.8601989746094,
-188.23312377929688,
220.414794921875,
-95.12461853027344,
-46.882389068603516,
-88.9289779663086)
        self.move_cartesian(509.06463623046875,
 -100.65288543701172,
 500.7088623046875,
 -79.46234130859375,
 -43.21661376953125,
 -92.05157470703125)
        self.move_cartesian(535.1044311523438,
 44.19460678100586,
 320.60137939453125,
 -79.46733093261719,
 -43.25926971435547,
 -96.7047119140625)
        self.move_cartesian(480.05517578125,
 173.3386688232422,
 500.17169189453125,
 -74.74259948730469,
 -42.186676025390625,
 -102.65570068359375)
        self.move_cartesian( 480.64813232421875,
 177.55661010742188,
220.414794921875,
 -91.8639907836914,
 -48.86869812011719,
 -90.92040252685547)
        print("Writing H")
        self.move_cartesian(509.8601989746094,
-188.23312377929688,
220.414794921875,
-95.12461853027344,
-46.882389068603516,
-88.9289779663086)
        self.move_cartesian(509.06463623046875,
 -188.65288543701172,
 500.7088623046875,
 -79.46234130859375,
 -43.21661376953125,
 -92.05157470703125)
        self.move_cartesian(509.8601989746094,
-188.23312377929688,
375.414794921875,
-95.12461853027344,
-46.882389068603516,
-88.9289779663086)
        
        self.move_cartesian(509.8601989746094,
 173.3386688232422,
375.17169189453125,
 -74.74259948730469,
 -42.186676025390625,
 -102.65570068359375)
        self.move_cartesian(509.8601989746094,
 173.3386688232422,
 500.17169189453125,
 -74.74259948730469,
 -42.186676025390625,
 -102.65570068359375)
        self.move_cartesian( 509.8601989746094,
 173.3386688232422,
220.414794921875,
 -91.8639907836914,
 -48.86869812011719,
 -90.92040252685547)
    def move_cartesian(self, x, y, z, w, p, r):
        print("Moving to new Cartesian Position")
        self.cart_ac.wait_for_server() # Wait till its ready
        cart_goal = CartPose.Goal() # Make goal
        cart_goal.x = x
        cart_goal.y = y
        cart_goal.z = z
        cart_goal.w = w
        cart_goal.p = p
        cart_goal.r = r
        future = self.cart_ac.send_goal_async(cart_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        sleep(5)
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
