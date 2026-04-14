import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
from std_srvs.srv import Trigger

import time
import sys
import pip_count

import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, OnRobotGripper, SJointPose, SchunkGripper

namespace = "beaker"

# ── Joint positions ───────────────────────────────────────────────────────────


HOME_JOINTS = {
    "joint1": 18.446,   
    "joint2": -7.714,   
    "joint3": -12.393,   
    "joint4": .285,   
    "joint5": -76.969,   
    "joint6": 99.095,   
}

DICE_ABOVE = {
    "joint1": 18.164,   
    "joint2": 9.377,   
    "joint3": -51.754, 
    "joint4": -.386,  
    "joint5": -39.782,  
    "joint6": 99.863,   
}

DICE_PICKUP = {
    "joint1": 18.165,   
    "joint2": 14.144,   
    "joint3": -58.408,   
    "joint4": -.388,   
    "joint5": -35.008,   
    "joint6": 99.857,   
}

CAMERA_JOINTS = {
    "joint1": 61.241,   
    "joint2": 20.961,   
    "joint3": 2.768,   
    "joint4": -84.885,   
    "joint5": -29.431,   
    "joint6": 109.099,   
}
# ─────────────────────────────────────────────────────────────────────────────


class ExampleNode(Node):
    def __init__(self, namespace):
        super().__init__(node_name="robot")

        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.grip_ac = ActionClient(self, OnRobotGripper, f'/{namespace}/onrobot_gripper')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.camera_sc = self.create_client(Trigger, 'take_picture')

        self.run_program()

    def run_program(self):
        dicePips = 0
        totalPips = 0

        iterations = 1

        self.schunk_ac.wait_for_server()
        schunk_goal = SchunkGripper.Goal()
        schunk_goal.command = 'open'

        future = self.schunk_ac.send_goal_async(
            goal=schunk_goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        print("Gripper opened")
        time.sleep(1)

        print("Starting program")
        while iterations < 4:
            # 1. Move to home position
            print("Moving to home position...")
            self._send_joint_goal(HOME_JOINTS)

            # 2. Move to above dice
            print("Moving to above dice position...")
            self._send_joint_goal(DICE_ABOVE)

            # 3. Move to dice pickup and grab dice
            print("Moving to dice pickup position...")
            self._send_joint_goal(DICE_PICKUP)

            self.schunk_ac.wait_for_server()
            schunk_goal = SchunkGripper.Goal()
            schunk_goal.command = 'close'

            future = self.schunk_ac.send_goal_async(
                goal=schunk_goal,
                feedback_callback=self.feedback_callback
            )
            future.add_done_callback(self.goal_response_callback)
            print("Gripper opened")
            time.sleep(1)

            # 4. Move in front of camera
            print("Moving to camera position...")
            self._send_joint_goal(CAMERA_JOINTS)

            # 5. Take picture
            print("Taking picture...")
            while not self.camera_sc.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Waiting for camera service...')
            future = self.camera_sc.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result.success:
                self.get_logger().info(f'Picture taken: {result.message}')
            else:
                self.get_logger().error(f'Failed to take picture: {result.message}')

            # 6. Count the pips on the dice
            dicePips = pip_count.pipCount()
            totalPips += dicePips
            print(" ")
            print("======================================================")
            print("The dice has " + str(dicePips) + " pips on it!")
            print("That is a total of " + str(totalPips) + " pips")
            print("This is iteration #" + str(iterations))
            iterations = iterations + 1
            print("======================================================")
            print(" ")

            # 7. Go back to drop off dice
            print("Moving to home position...")
            self._send_joint_goal(HOME_JOINTS)

            print("Moving to dice pickup position...")
            self._send_joint_goal(DICE_PICKUP)
            
            self.schunk_ac.wait_for_server()
            schunk_goal = SchunkGripper.Goal()
            schunk_goal.command = 'open'

            future = self.schunk_ac.send_goal_async(
                goal=schunk_goal,
                feedback_callback=self.feedback_callback
            )
            future.add_done_callback(self.goal_response_callback)
            print("Gripper opened")
            time.sleep(1)



        print("Program complete")

    def _send_joint_goal(self, joints: dict):
        self.joints_ac.wait_for_server()
        goal = JointPose.Goal()
        goal.joint1 = joints["joint1"]
        goal.joint2 = joints["joint2"]
        goal.joint3 = joints["joint3"]
        goal.joint4 = joints["joint4"]
        goal.joint5 = joints["joint5"]
        goal.joint6 = joints["joint6"]

        # Wait for goal to be accepted
        send_future = self.joints_ac.send_goal_async(
            goal=goal, feedback_callback=self.feedback_callback
        )
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return

        self.get_logger().info("Goal accepted, waiting for robot to finish moving...")

        # Wait for the robot to finish moving
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"Move complete, success: {result.success}")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.get_logger().info("received feedback: {0}".format(feedback.distance_left))

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info(message = "goal rejected")
            return
        #self.get_logger().info(message = "goal accepted")
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        #self.get_logger().info(message = f"result: {0}".format(result.success))

if __name__ == "__main__":
    rclpy.init()
    bot = ExampleNode(namespace)
    rclpy.spin(bot)
