import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
import asyncio
import sys
sys.path.append("../src/dependencies/")

from time import sleep

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, JointPose, SchunkGripper, OnRobotGripper, Conveyor
from fanuc_interfaces.msg import ProxReadings

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class FanucRosNode(Node):
    def __init__(self, namespace):
        super().__init__(f'{namespace}_robot_node')
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.onrobot_ac = ActionClient(self, OnRobotGripper, f'/{namespace}/onrobot_gripper')
        self.conveyor_ac = ActionClient(self, Conveyor, f'/{namespace}/conveyor')

        self.namespace = namespace

        # self.camera_capture_client = self.create_client(Trigger, 'camera/capture')
        self.camera_pip_client = self.create_client(Trigger, 'camera/count_pips')

        self.cube_identify_client = self.create_client(Trigger, 'dice/identify_dice_location')
        self.cube_find_face_client = self.create_client(Trigger, 'dice/find_face_with_pip')
        self.dice_params_client = self.create_client(SetParameters, '/dice_node/set_parameters')

        other_bot = 'bunsun' if namespace == 'beaker' else 'beaker'
        self.dice_ready_client = self.create_client(Trigger, f'/{other_bot}/dice_ready')
        self.create_service(Trigger, f'/{namespace}/dice_ready', self.__dice_ready)
        self.dice_ready = False

        self._bridge = CvBridge()

    #Camera functions
    # async def get_overhead_camera_frame(self):
    #     self.get_logger().info("Requesting camera capture...")
    #     self.camera_capture_client.wait_for_service()
    #     frame_future = asyncio.get_running_loop().create_future()
    #     sub = self.create_subscription(Image, 'camera/image_raw',
    #         lambda msg: frame_future.set_result(self._bridge.imgmsg_to_cv2(msg, 'bgr8')), 10)
    #     await self.__wait_for_future(self.camera_capture_client.call_async(Trigger.Request()))
    #     frame = await frame_future
    #     self.destroy_subscription(sub)
    #     return frame

    async def get_dice_pip_count(self):
        self.get_logger().info("Requesting pip count from camera node...")
        self.camera_pip_client.wait_for_service()
        result = await self.__wait_for_future(self.camera_pip_client.call_async(Trigger.Request()))
        return int(result.message) if result.success else None

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
        succ = await self.__send_action_and_wait(self.onrobot_ac, onrobot_goal)
        await asyncio.sleep(5) #give it a moment to grip before moving again
        return succ

    #Conveyor functions
    async def move_conveyor(self, direction):# 'forward', 'reverse' or 'stop'
        self.conveyor_ac.wait_for_server()

        conveyor_goal = Conveyor.Goal()
        conveyor_goal.command = direction

        self.get_logger().info("Moving conveyor " + direction)
        return await self.__send_action_and_wait(self.conveyor_ac, conveyor_goal)
    
    async def read_conveyor_sensor(self, side = 'left'): #'left' or 'right'
        reading_future = asyncio.get_running_loop().create_future()
        sub = self.create_subscription(ProxReadings, f'{self.namespace}/prox_readings',
            lambda msg: reading_future.set_result(msg), 10)
        reading = await reading_future
        self.destroy_subscription(sub)
        return reading.left if side == 'left' else reading.right
    
    #Dice functions
    async def identify_dice_location(self, top, side, clockwise):
        self.dice_params_client.wait_for_service()
        await self.__set_dice_params(top=top, side=side, clockwise=clockwise)
        self.cube_identify_client.wait_for_service()
        return await self.__wait_for_future(self.cube_identify_client.call_async(Trigger.Request()))

    async def find_face_with_pip(self, pip, prefer_clockwise):
        self.dice_params_client.wait_for_service()
        await self.__set_dice_params(pip=pip, prefer_clockwise=prefer_clockwise)
        self.cube_find_face_client.wait_for_service()
        result = await self.__wait_for_future(self.cube_find_face_client.call_async(Trigger.Request()))
        return [int(m) for m in result.message.split(',') if m] if result.success else []

    async def __set_dice_params(self, **kwargs):
        req = SetParameters.Request()
        for name, value in kwargs.items():
            pv = ParameterValue()
            if isinstance(value, bool):
                pv.type = ParameterType.PARAMETER_BOOL
                pv.bool_value = value
            else:
                pv.type = ParameterType.PARAMETER_INTEGER
                pv.integer_value = int(value)
            req.parameters.append(Parameter(name=name, value=pv))
        await self.__wait_for_future(self.dice_params_client.call_async(req))

    #status functiions
    def __dice_ready(self, request, response):
        try:
            self.get_logger().info(f'Dice ready status requested, current status: {self.dice_ready}')
            response.success = True
            response.message = str(self.dice_ready)
            self.dice_ready = False if self.dice_ready else self.dice_ready
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response
    
    def set_dice_ready(self, ready):
        self.dice_ready = ready
        self.get_logger().info(f'Dice ready status set to: {self.dice_ready}')

    async def request_dice_ready(self):
        self.get_logger().info("Requesting dice ready status...")
        self.dice_ready_client.wait_for_service()
        result = await self.__wait_for_future(self.dice_ready_client.call_async(Trigger.Request()))
        self.get_logger().info(f"Received dice ready response: {result.message}")
        return (True if result.message == "True" else False) if result.success else None
    
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