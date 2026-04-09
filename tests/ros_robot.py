import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
import asyncio
import sys
sys.path.append("../src/dependencies/")

from time import sleep

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, OnRobotGripper

namespace = "beaker"
class FanucRosNode(Node):
    def __init__(self, namespace):
        super().__init__("robot")
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.onrobot_ac = ActionClient(self, OnRobotGripper, f'/{namespace}/onrobot_gripper')

    #Movement Functions
    async def move_cartesian(self, pose): #can send xyzwpr or just xyz
        self.cart_ac.wait_for_server()

        cart_goal = CartPose.Goal()
        cart_goal.x = float(pose[0])
        cart_goal.y = float(pose[1])
        cart_goal.z = float(pose[2])
        if len(pose) > 3:
            cart_goal.w = float(pose[3])
            cart_goal.p = float(pose[4])
            cart_goal.r = float(pose[5])

        self.get_logger().info("Moving cartesian to " + str(pose))
        return await self.__send_action_and_wait(self.cart_ac, cart_goal)

    async def move_joints(self, pose):
        self.joints_ac.wait_for_server()

        joint_goal = JointPose.Goal()
        joint_goal.joint1 = pose[0]
        joint_goal.joint2 = pose[1]
        joint_goal.joint3 = pose[2]
        joint_goal.joint4 = pose[3]
        joint_goal.joint5 = pose[4]
        joint_goal.joint6 = pose[5]

        self.get_logger().info("Moving joints to " + str(pose))
        return await self.__send_action_and_wait(self.joints_ac, joint_goal)

    async def open_gripper_schunk(self, open): #send str 'open' or 'close'
        self.schunk_ac.wait_for_server()

        schunk_goal = SchunkGripper.Goal()
        schunk_goal.command = open

        self.get_logger().info("Gripping with Schunk gripper" + (" opening" if open == 'open' else " closing"))
        return await self.__send_action_and_wait(self.schunk_ac, schunk_goal)

    async def open_gripper_onrobot(self, open): #send True || False
        self.onrobot_ac.wait_for_server()

        onrobot_goal = OnRobotGripper.Goal()
        if open:
            onrobot_goal.width = 150
        else:
            onrobot_goal.width = 73
        onrobot_goal.force = 40

        self.get_logger().info("Gripping with OnRobot gripper" + (" opening" if open else " closing"))
        return await self.__send_action_and_wait(self.onrobot_ac, onrobot_goal)

    #Helper functions for ros2 actions
    async def __send_action_and_wait(self, action_client, goal_msg):
        action_client.wait_for_server()

        goal_future = action_client.send_goal_async(goal_msg)
        goal_handle = await self.__wait_for_future(goal_future)

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return False

        #self.get_logger().info("Goal accepted")

        # Wait for result
        result_future = goal_handle.get_result_async()
        result = await self.__wait_for_future(result_future)

        success = result.result.success
        #self.get_logger().info(f"Result: {success}")

        return success

    async def __wait_for_future(self, rclpy_future):
        loop = asyncio.get_running_loop()
        asyncio_future = loop.create_future()

        def done_callback(fut):
            loop.call_soon_threadsafe(asyncio_future.set_result, fut.result())

        rclpy_future.add_done_callback(done_callback)

        return await asyncio_future

async def spin_robot(executor):
        try:
            while rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
                await asyncio.sleep(0.01)
        except rclpy.executors.ExternalShutdownException:
            pass