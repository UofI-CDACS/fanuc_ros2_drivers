#!/usr/bin/env python3
"""
bunsen_dice_node_final.py

ROS2 node for the Bunsen FANUC robot. Full dice-inspection pipeline with
Modbus coordination. Uses async action helper for background-thread safety.

Algorithm:
  1.  HOME  (joint)
  2.  BACK_CONV_ABOVE  (cart)
  3.  Gripper OPEN
  4.  BACK_CONV  (cart)          — pick die from back conveyor
  5.  Gripper CLOSE
  6.  BACK_CONV_ABOVE  (cart)
  7.  ABOVE_TABLE  (cart)
  8.  ON_TABLE  (cart)           — place die at camera station
  9.  Gripper OPEN               — release die
  10. ABOVE_TABLE  (cart)
  11. SHOW_CAMERA_ABOVE_TABLE  (cart)
  12. SHOW_CAMERA_TABLE  (cart)  — grip die at camera
  13. Gripper CLOSE
  14. SHOW_CAMERA  (cart)        — present die to camera
  15. Capture image -> count pips
  16. Branch on pip_count vs target_pip / opp_pip

Run (after sourcing ROS2 and building):
    python3 bunsen_dice_node_final.py --ros-args \
        -p robot_name:=fanuc \
        -p target_pip:=2 \
        -p opp_pip:=5

Setup:
    source /opt/ros/jazzy/setup.bash
    source ~/Desktop/fanuc_ros2_drivers/install/setup.bash
    cd ~/Desktop/fanuc_ros2_drivers && colcon build
"""

import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from pymodbus.client import ModbusTcpClient

import grab
import pip_counter

from fanuc_interfaces.action import CartPose, Conveyor, JointPose, OnRobotGripper
from fanuc_interfaces.msg import ProxReadings
from fanuc_interfaces.srv import SetSpeed

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

MODBUS_HOST     = "10.8.4.19"
MODBUS_PORT     = 5020
COORD_REGISTER  = 0   # 0=idle, 1=robot1 working, 2=robot2 working
CYCLE_REGISTER  = 1   # current cycle (1-6), advances on each handoff


# ── Pose constants ────────────────────────────────────────────────────────────

HOME_JOINT       = [0.00, -0.0, 0.004, -0.001, -90.0, 0.0]

BACK_CONV_ABOVE  = [-223.411376953125, -792.5889892578125,  162.81547546386719, -178.99896240234375, -0.004184994380921125, -1.7302083506365307e-05]
BACK_CONV        = [-223.411376953125, -792.5889892578125,   62.81547546386719, -178.99896240234375, -0.004184994380921125, -1.7302083506365307e-05]

ABOVE_TABLE      = [ 715.5281372070312, -573.9559326171875,  192.56549835205078,  176.95286560058594,   4.302637100219727,  -89.71446228027344]
ON_TABLE         = [ 715.5281372070312, -573.9559326171875,  -71.56549835205078,  176.95286560058594,   4.302637100219727,  -89.71446228027344]
X_Y              = [ 715.5281372070312, -573.9559326171875,  192.56549835205078,  176.95286560058594,   4.302637100219727,    3.71446228027344]
X_Y_TABLE        = [ 715.5281372070312, -573.9559326171875,  -71.56549835205078,  176.95286560058594,   4.302637100219727,    3.71446228027344]

SHOW_CAMERA                  = [ 723.1131591796875,  -861.2346801757812,   624.1238403320312,   117.47567749023438,  -70.04679870605469,  -35.097389221191406]
SHOW_CAMERA_ABOVE_TABLE      = [ 715.622802734375,   -506.1666259765625,   135.1529083251953,   145.5094757080078,   -81.20520782470703,  -55.31917953491211]
SHOW_CAMERA_TABLE            = [ 715.528076171875,   -506.75445556640625, -153.93357849121094,  145.50318908691406,  -81.20399475097656,  -55.31047821044922]
SHOW_CAMERA_INT_ROTATE       = [ 719.5548706054688,  -862.0361328125,      632.5980834960938,    99.14537811279297,    7.818133354187012,   -7.793117046356201]
SHOW_CAMERA_ROTATE           = [ 729.0421142578125,  -864.2291259765625,   636.82421875,        -124.54747772216797,   73.88070678710938,  137.4707794189453]
SHOW_CAMERA_ROTATE_ABOVE_TABLE = [715.528076171875,  -506.75445556640625,  100.93357849,        -146.2725830078125,    83.26343536376953,  115.06522369384766]
SHOW_CAMERA_ROT_TABLE        = [ 715.528076171875,   -506.75445556640625, -153.93357849121094,  -146.2725830078125,    83.26343536376953,  115.06522369384766]

FRONT_CONV_ABOVE = [140.0443572998047, -795.18115234375, 148.31916809082031, -178.4838104248047,  0.3290110230445862, -5.910837650299072]
FRONT_CONV       = [140.0443572998047, -795.18115234375,  48.31916809082031, -178.4838104248047,  0.3290110230445862, -5.910837650299072]

R2_ABOVE_TABLE   = [ 458.764, -743.341, 192.319, -178.999, -0.004, -92.796]
R2_ON_TABLE      = [ 458.764, -743.341,  -64.9,  -178.999, -0.004, -92.796]
R2_X_Y           = [ 458.764, -743.341, 192.319, -179.012,  0.273,   3.986]
R2_Y_X           = [ 458.764, -743.341, 192.319, -179.012,  0.273, -179.243]


class BunsenDiceNode(Node):

    def __init__(self):
        super().__init__('bunsen_dice_node')

        self.declare_parameter('robot_name',    'bunsen')
        self.declare_parameter('move_speed',    300)
        self.declare_parameter('target_pip',    2)
        self.declare_parameter('opp_pip',       5)
        self.declare_parameter('exposure_ms',   500.0)
        self.declare_parameter('grip_open_mm',  120)
        self.declare_parameter('grip_close_mm', 70)
        self.declare_parameter('grip_force_n',  50)

        self._cb = ReentrantCallbackGroup()
        robot = self.get_parameter('robot_name').value

        self.joints_ac = ActionClient(self, JointPose,      f'{robot}/joint_pose',      callback_group=self._cb)
        self.cart_ac   = ActionClient(self, CartPose,       f'{robot}/cartesian_pose',  callback_group=self._cb)
        self.grip_ac   = ActionClient(self, OnRobotGripper, f'{robot}/onrobot_gripper', callback_group=self._cb)
        self._convey_ac    = ActionClient(self, Conveyor,       f'{robot}/conveyor',        callback_group=self._cb)
        self._speed_client = self.create_client(SetSpeed, f'{robot}/set_speed',             callback_group=self._cb)

        self._sensor_triggered = False
        self._prox_sub = self.create_subscription(
            ProxReadings, f'/{robot}/prox_readings', self._prox_callback, 10)

        self._modbus_connect()

        threading.Thread(target=self._run, daemon=True).start()
        self.get_logger().info('bunsen_dice_node started')

    # ── Motion helpers ────────────────────────────────────────────────────

    def _set_speed(self, speed: int):
        if self._speed_client.wait_for_service(timeout_sec=5.0):
            req = SetSpeed.Request()
            req.speed = speed
            self._speed_client.call(req)

    # ── Conveyor / sensor ─────────────────────────────────────────────────

    def _send_conveyor_goal(self, command: str):
        self._convey_ac.wait_for_server()
        goal = Conveyor.Goal()
        goal.command = command
        send_future = self._convey_ac.send_goal_async(
            goal=goal, feedback_callback=self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Conveyor goal rejected')
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

    def _prox_callback(self, msg: ProxReadings):
        if msg.left:
            self._sensor_triggered = True

    def _wait_for_sensor(self):
        self._sensor_triggered = False
        while not self._sensor_triggered:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._send_conveyor_goal('stop')

    # ── Modbus ────────────────────────────────────────────────────────────

    def _modbus_connect(self):
        self._mb_client = ModbusTcpClient(MODBUS_HOST, port=MODBUS_PORT)
        self.get_logger().info('Waiting for Modbus server...')
        while not self._mb_client.connect():
            self.get_logger().warn('Modbus connection failed, retrying in 1s...')
            time.sleep(1)
        self.get_logger().info('Connected to Modbus server')

    def _modbus_write(self, address, value):
        try:
            self._mb_client.write_register(address, value)
            self.get_logger().info(f'Modbus register {address} = {value}')
        except Exception as e:
            self.get_logger().error(f'Modbus write error: {e}')

    def _modbus_read(self, address):
        try:
            result = self._mb_client.read_holding_registers(address, count=1)
            return result.registers[0]
        except Exception as e:
            self.get_logger().error(f'Modbus read error: {e}')
            return -1

    def _wait_for_robot1(self):
        self.get_logger().info('Waiting for robot1 to signal ready (register 0 = 2)...')
        while self._modbus_read(COORD_REGISTER) != 2:
            time.sleep(0.5)
        self.get_logger().info('Robot1 signaled ready, starting pickup')

    # ── Camera / pip detection ────────────────────────────────────────────

    def _inspect_dice(self) -> int:
        grab.main()
        dicePips = pip_counter.pipCount()

        print("Pips: " + str(dicePips))

        return dicePips

    # ── Main algorithm (full inspection pipeline) ─────────────────────────

    def _run(self):
        # self._send_conveyor_goal('reverse')
        # time.sleep(8)
        # self._send_conveyor_goal('stop')

        time.sleep(1.0)   # wait for executor to start spinning before sending goals
        self._set_speed(self.get_parameter('move_speed').value)

        target = self.get_parameter('target_pip').value
        opp    = self.get_parameter('opp_pip').value

        try:
            self._pipeline(target, opp)
        except RuntimeError as e:
            self.get_logger().error(f'Pipeline aborted at: {e}')

    def _pipeline(self, target: int, opp: int):
        # ── Pick die from back conveyor ───────────────────────────────────

        while(True):
            self._wait_for_robot1()

            beaker_target = self._modbus_read(CYCLE_REGISTER)
            print("Beaker target is " + str(beaker_target))
            # Bunsen's camera is 180° opposite Beaker's. When the die is correctly
            # oriented for Beaker (beaker_target face toward Beaker), Bunsen sees
            # the opposite face (7 - beaker_target). Swap target/opp accordingly.
            target = beaker_target
            opp    = 7 - target

            self.get_logger().info('Step 1: Home')
            self.joints_ac.wait_for_server()
            joint_goal = JointPose.Goal()
            joint_goal.joint1 = float(HOME_JOINT[0])
            joint_goal.joint2 = float(HOME_JOINT[1])
            joint_goal.joint3 = float(HOME_JOINT[2])
            joint_goal.joint4 = float(HOME_JOINT[3])
            joint_goal.joint5 = float(HOME_JOINT[4])
            joint_goal.joint6 = float(HOME_JOINT[5])
            future = self.joints_ac.send_goal_async(goal=joint_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            time.sleep(0.5)
            self.get_logger().info('Step 2: Back conveyor above')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = BACK_CONV_ABOVE[0]
            cart_goal.y = BACK_CONV_ABOVE[1]
            cart_goal.z = BACK_CONV_ABOVE[2]
            cart_goal.w = BACK_CONV_ABOVE[3]
            cart_goal.p = BACK_CONV_ABOVE[4]
            cart_goal.r = BACK_CONV_ABOVE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.get_logger().info('Step 3: Gripper open')
            self.grip_ac.wait_for_server()
            grip_goal = OnRobotGripper.Goal()
            grip_goal.width = 120
            grip_goal.force = 40
            future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(0.5)

            self.get_logger().info('Step 4: Descend to back conveyor')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = BACK_CONV[0]
            cart_goal.y = BACK_CONV[1]
            cart_goal.z = BACK_CONV[2]
            cart_goal.w = BACK_CONV[3]
            cart_goal.p = BACK_CONV[4]
            cart_goal.r = BACK_CONV[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)
            time.sleep(0.5)

            self.get_logger().info('Step 5: Gripper close')
            self.grip_ac.wait_for_server()
            grip_goal = OnRobotGripper.Goal()
            grip_goal.width = 73
            grip_goal.force = 40
            future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(0.5)

            self.get_logger().info('Step 6: Retract from conveyor')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = BACK_CONV_ABOVE[0]
            cart_goal.y = BACK_CONV_ABOVE[1]
            cart_goal.z = BACK_CONV_ABOVE[2]
            cart_goal.w = BACK_CONV_ABOVE[3]
            cart_goal.p = BACK_CONV_ABOVE[4]
            cart_goal.r = BACK_CONV_ABOVE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.get_logger().info('Step 7: Above table')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = ABOVE_TABLE[0]
            cart_goal.y = ABOVE_TABLE[1]
            cart_goal.z = ABOVE_TABLE[2]
            cart_goal.w = ABOVE_TABLE[3]
            cart_goal.p = ABOVE_TABLE[4]
            cart_goal.r = ABOVE_TABLE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.get_logger().info('Step 8: On table — place die')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = ON_TABLE[0]
            cart_goal.y = ON_TABLE[1]
            cart_goal.z = ON_TABLE[2]
            cart_goal.w = ON_TABLE[3]
            cart_goal.p = ON_TABLE[4]
            cart_goal.r = ON_TABLE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.get_logger().info('Step 9: Gripper open — release die')
            self.grip_ac.wait_for_server()
            grip_goal = OnRobotGripper.Goal()
            grip_goal.width = 120
            grip_goal.force = 40
            future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(0.5)

            self.get_logger().info('Step 10: Above table')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = ABOVE_TABLE[0]
            cart_goal.y = ABOVE_TABLE[1]
            cart_goal.z = ABOVE_TABLE[2]
            cart_goal.w = ABOVE_TABLE[3]
            cart_goal.p = ABOVE_TABLE[4]
            cart_goal.r = ABOVE_TABLE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.get_logger().info('Step 11: Show camera above table')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = SHOW_CAMERA_ABOVE_TABLE[0]
            cart_goal.y = SHOW_CAMERA_ABOVE_TABLE[1]
            cart_goal.z = SHOW_CAMERA_ABOVE_TABLE[2]
            cart_goal.w = SHOW_CAMERA_ABOVE_TABLE[3]
            cart_goal.p = SHOW_CAMERA_ABOVE_TABLE[4]
            cart_goal.r = SHOW_CAMERA_ABOVE_TABLE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.get_logger().info('Step 12: Show camera table — grip die')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = SHOW_CAMERA_TABLE[0]
            cart_goal.y = SHOW_CAMERA_TABLE[1]
            cart_goal.z = SHOW_CAMERA_TABLE[2]
            cart_goal.w = SHOW_CAMERA_TABLE[3]
            cart_goal.p = SHOW_CAMERA_TABLE[4]
            cart_goal.r = SHOW_CAMERA_TABLE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.get_logger().info('Step 13: Gripper close')
            self.grip_ac.wait_for_server()
            grip_goal = OnRobotGripper.Goal()
            grip_goal.width = 73
            grip_goal.force = 40
            future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(0.5)

            self.get_logger().info('Step 14: Show camera')
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = SHOW_CAMERA[0]
            cart_goal.y = SHOW_CAMERA[1]
            cart_goal.z = SHOW_CAMERA[2]
            cart_goal.w = SHOW_CAMERA[3]
            cart_goal.p = SHOW_CAMERA[4]
            cart_goal.r = SHOW_CAMERA[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(9)

            # ── Branch on pip count ───────────────────────────────────────────
            pip_count = self._inspect_dice()  # hardcoded to match move_Bunsen.py
            self.get_logger().info(f'pip_count={pip_count}  target={target}  opp={opp}')
            time.sleep(2)

            if pip_count == target:
                print("Target found")
                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_TABLE[0]
                cart_goal.y = SHOW_CAMERA_TABLE[1]
                cart_goal.z = SHOW_CAMERA_TABLE[2]
                cart_goal.w = SHOW_CAMERA_TABLE[3]
                cart_goal.p = SHOW_CAMERA_TABLE[4]
                cart_goal.r = SHOW_CAMERA_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ABOVE_TABLE[0]
                cart_goal.y = ABOVE_TABLE[1]
                cart_goal.z = ABOVE_TABLE[2]
                cart_goal.w = ABOVE_TABLE[3]
                cart_goal.p = ABOVE_TABLE[4]
                cart_goal.r = ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                print("letting go")
                self.grip_ac.wait_for_server()
                grip_goal = OnRobotGripper.Goal()
                grip_goal.width = 120
                grip_goal.force = 40
                future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(0.5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ON_TABLE[0]
                cart_goal.y = ON_TABLE[1]
                cart_goal.z = ON_TABLE[2]
                cart_goal.w = ON_TABLE[3]
                cart_goal.p = ON_TABLE[4]
                cart_goal.r = ON_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.grip_ac.wait_for_server()
                grip_goal = OnRobotGripper.Goal()
                grip_goal.width = 73
                grip_goal.force = 40
                future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(0.5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ABOVE_TABLE[0]
                cart_goal.y = ABOVE_TABLE[1]
                cart_goal.z = ABOVE_TABLE[2]
                cart_goal.w = ABOVE_TABLE[3]
                cart_goal.p = ABOVE_TABLE[4]
                cart_goal.r = ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

            elif pip_count == opp:
                print("opp found")
                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_INT_ROTATE[0]
                cart_goal.y = SHOW_CAMERA_INT_ROTATE[1]
                cart_goal.z = SHOW_CAMERA_INT_ROTATE[2]
                cart_goal.w = SHOW_CAMERA_INT_ROTATE[3]
                cart_goal.p = SHOW_CAMERA_INT_ROTATE[4]
                cart_goal.r = SHOW_CAMERA_INT_ROTATE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_ROTATE[0]
                cart_goal.y = SHOW_CAMERA_ROTATE[1]
                cart_goal.z = SHOW_CAMERA_ROTATE[2]
                cart_goal.w = SHOW_CAMERA_ROTATE[3]
                cart_goal.p = SHOW_CAMERA_ROTATE[4]
                cart_goal.r = SHOW_CAMERA_ROTATE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_ROTATE_ABOVE_TABLE[0]
                cart_goal.y = SHOW_CAMERA_ROTATE_ABOVE_TABLE[1]
                cart_goal.z = SHOW_CAMERA_ROTATE_ABOVE_TABLE[2]
                cart_goal.w = SHOW_CAMERA_ROTATE_ABOVE_TABLE[3]
                cart_goal.p = SHOW_CAMERA_ROTATE_ABOVE_TABLE[4]
                cart_goal.r = SHOW_CAMERA_ROTATE_ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_ROT_TABLE[0]
                cart_goal.y = SHOW_CAMERA_ROT_TABLE[1]
                cart_goal.z = SHOW_CAMERA_ROT_TABLE[2]
                cart_goal.w = SHOW_CAMERA_ROT_TABLE[3]
                cart_goal.p = SHOW_CAMERA_ROT_TABLE[4]
                cart_goal.r = SHOW_CAMERA_ROT_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.grip_ac.wait_for_server()
                grip_goal = OnRobotGripper.Goal()
                grip_goal.width = 120
                grip_goal.force = 40
                future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(0.5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_ROTATE_ABOVE_TABLE[0]
                cart_goal.y = SHOW_CAMERA_ROTATE_ABOVE_TABLE[1]
                cart_goal.z = SHOW_CAMERA_ROTATE_ABOVE_TABLE[2]
                cart_goal.w = SHOW_CAMERA_ROTATE_ABOVE_TABLE[3]
                cart_goal.p = SHOW_CAMERA_ROTATE_ABOVE_TABLE[4]
                cart_goal.r = SHOW_CAMERA_ROTATE_ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ABOVE_TABLE[0]
                cart_goal.y = ABOVE_TABLE[1]
                cart_goal.z = ABOVE_TABLE[2]
                cart_goal.w = ABOVE_TABLE[3]
                cart_goal.p = ABOVE_TABLE[4]
                cart_goal.r = ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ON_TABLE[0]
                cart_goal.y = ON_TABLE[1]
                cart_goal.z = ON_TABLE[2]
                cart_goal.w = ON_TABLE[3]
                cart_goal.p = ON_TABLE[4]
                cart_goal.r = ON_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.grip_ac.wait_for_server()
                grip_goal = OnRobotGripper.Goal()
                grip_goal.width = 73
                grip_goal.force = 40
                future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(0.5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ABOVE_TABLE[0]
                cart_goal.y = ABOVE_TABLE[1]
                cart_goal.z = ABOVE_TABLE[2]
                cart_goal.w = ABOVE_TABLE[3]
                cart_goal.p = ABOVE_TABLE[4]
                cart_goal.r = ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

            else:

                print("PUTTING ABOVE TABLE")
                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_ABOVE_TABLE[0]
                cart_goal.y = SHOW_CAMERA_ABOVE_TABLE[1]
                cart_goal.z = SHOW_CAMERA_ABOVE_TABLE[2]
                cart_goal.w = SHOW_CAMERA_ABOVE_TABLE[3]
                cart_goal.p = SHOW_CAMERA_ABOVE_TABLE[4]
                cart_goal.r = SHOW_CAMERA_ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)     

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_TABLE[0]
                cart_goal.y = SHOW_CAMERA_TABLE[1]
                cart_goal.z = SHOW_CAMERA_TABLE[2]
                cart_goal.w = SHOW_CAMERA_TABLE[3]
                cart_goal.p = SHOW_CAMERA_TABLE[4]
                cart_goal.r = SHOW_CAMERA_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                print("letting go")
                self.grip_ac.wait_for_server()
                grip_goal = OnRobotGripper.Goal()
                grip_goal.width = 120
                grip_goal.force = 40
                future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(0.5)

                

                print("PUTTING ABOVE TABLE")
                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA_ABOVE_TABLE[0]
                cart_goal.y = SHOW_CAMERA_ABOVE_TABLE[1]
                cart_goal.z = SHOW_CAMERA_ABOVE_TABLE[2]
                cart_goal.w = SHOW_CAMERA_ABOVE_TABLE[3]
                cart_goal.p = SHOW_CAMERA_ABOVE_TABLE[4]
                cart_goal.r = SHOW_CAMERA_ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5) 

                print("PUTTING ON TABLE")
                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ABOVE_TABLE[0]
                cart_goal.y = ABOVE_TABLE[1]
                cart_goal.z = ABOVE_TABLE[2]
                cart_goal.w = ABOVE_TABLE[3]
                cart_goal.p = ABOVE_TABLE[4]
                cart_goal.r = ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ON_TABLE[0]
                cart_goal.y = ON_TABLE[1]
                cart_goal.z = ON_TABLE[2]
                cart_goal.w = ON_TABLE[3]
                cart_goal.p = ON_TABLE[4]
                cart_goal.r = ON_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.grip_ac.wait_for_server()
                grip_goal = OnRobotGripper.Goal()
                grip_goal.width = 72
                grip_goal.force = 50
                future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(2)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = ABOVE_TABLE[0]
                cart_goal.y = ABOVE_TABLE[1]
                cart_goal.z = ABOVE_TABLE[2]
                cart_goal.w = ABOVE_TABLE[3]
                cart_goal.p = ABOVE_TABLE[4]
                cart_goal.r = ABOVE_TABLE[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                self.cart_ac.wait_for_server()
                cart_goal = CartPose.Goal()
                cart_goal.x = SHOW_CAMERA[0]
                cart_goal.y = SHOW_CAMERA[1]
                cart_goal.z = SHOW_CAMERA[2]
                cart_goal.w = SHOW_CAMERA[3]
                cart_goal.p = SHOW_CAMERA[4]
                cart_goal.r = SHOW_CAMERA[5]
                future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                future.add_done_callback(callback=self._goal_response_callback)
                time.sleep(5)

                pip_count = self._inspect_dice()
                self.get_logger().info(f'Re-check pip count: {pip_count}')

                if pip_count == target:
                    print("found target after miss")
                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_ABOVE_TABLE[0]
                    cart_goal.y = SHOW_CAMERA_ABOVE_TABLE[1]
                    cart_goal.z = SHOW_CAMERA_ABOVE_TABLE[2]
                    cart_goal.w = SHOW_CAMERA_ABOVE_TABLE[3]
                    cart_goal.p = SHOW_CAMERA_ABOVE_TABLE[4]
                    cart_goal.r = SHOW_CAMERA_ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_TABLE[0]
                    cart_goal.y = SHOW_CAMERA_TABLE[1]
                    cart_goal.z = SHOW_CAMERA_TABLE[2]
                    cart_goal.w = SHOW_CAMERA_TABLE[3]
                    cart_goal.p = SHOW_CAMERA_TABLE[4]
                    cart_goal.r = SHOW_CAMERA_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.grip_ac.wait_for_server()
                    grip_goal = OnRobotGripper.Goal()
                    grip_goal.width = 120
                    grip_goal.force = 40
                    future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(2)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_ABOVE_TABLE[0]
                    cart_goal.y = SHOW_CAMERA_ABOVE_TABLE[1]
                    cart_goal.z = SHOW_CAMERA_ABOVE_TABLE[2]
                    cart_goal.w = SHOW_CAMERA_ABOVE_TABLE[3]
                    cart_goal.p = SHOW_CAMERA_ABOVE_TABLE[4]
                    cart_goal.r = SHOW_CAMERA_ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ABOVE_TABLE[0]
                    cart_goal.y = ABOVE_TABLE[1]
                    cart_goal.z = ABOVE_TABLE[2]
                    cart_goal.w = ABOVE_TABLE[3]
                    cart_goal.p = ABOVE_TABLE[4]
                    cart_goal.r = ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ON_TABLE[0]
                    cart_goal.y = ON_TABLE[1]
                    cart_goal.z = ON_TABLE[2]
                    cart_goal.w = ON_TABLE[3]
                    cart_goal.p = ON_TABLE[4]
                    cart_goal.r = ON_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.grip_ac.wait_for_server()
                    grip_goal = OnRobotGripper.Goal()
                    grip_goal.width = 73
                    grip_goal.force = 40
                    future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(0.5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ABOVE_TABLE[0]
                    cart_goal.y = ABOVE_TABLE[1]
                    cart_goal.z = ABOVE_TABLE[2]
                    cart_goal.w = ABOVE_TABLE[3]
                    cart_goal.p = ABOVE_TABLE[4]
                    cart_goal.r = ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                elif pip_count == opp:
                    print("found opp after miss")
                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_INT_ROTATE[0]
                    cart_goal.y = SHOW_CAMERA_INT_ROTATE[1]
                    cart_goal.z = SHOW_CAMERA_INT_ROTATE[2]
                    cart_goal.w = SHOW_CAMERA_INT_ROTATE[3]
                    cart_goal.p = SHOW_CAMERA_INT_ROTATE[4]
                    cart_goal.r = SHOW_CAMERA_INT_ROTATE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_ROTATE[0]
                    cart_goal.y = SHOW_CAMERA_ROTATE[1]
                    cart_goal.z = SHOW_CAMERA_ROTATE[2]
                    cart_goal.w = SHOW_CAMERA_ROTATE[3]
                    cart_goal.p = SHOW_CAMERA_ROTATE[4]
                    cart_goal.r = SHOW_CAMERA_ROTATE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_ROTATE_ABOVE_TABLE[0]
                    cart_goal.y = SHOW_CAMERA_ROTATE_ABOVE_TABLE[1]
                    cart_goal.z = SHOW_CAMERA_ROTATE_ABOVE_TABLE[2]
                    cart_goal.w = SHOW_CAMERA_ROTATE_ABOVE_TABLE[3]
                    cart_goal.p = SHOW_CAMERA_ROTATE_ABOVE_TABLE[4]
                    cart_goal.r = SHOW_CAMERA_ROTATE_ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_ROT_TABLE[0]
                    cart_goal.y = SHOW_CAMERA_ROT_TABLE[1]
                    cart_goal.z = SHOW_CAMERA_ROT_TABLE[2]
                    cart_goal.w = SHOW_CAMERA_ROT_TABLE[3]
                    cart_goal.p = SHOW_CAMERA_ROT_TABLE[4]
                    cart_goal.r = SHOW_CAMERA_ROT_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(1)

                    self.grip_ac.wait_for_server()
                    grip_goal = OnRobotGripper.Goal()
                    grip_goal.width = 120
                    grip_goal.force = 40
                    future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(2)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA_ROTATE_ABOVE_TABLE[0]
                    cart_goal.y = SHOW_CAMERA_ROTATE_ABOVE_TABLE[1]
                    cart_goal.z = SHOW_CAMERA_ROTATE_ABOVE_TABLE[2]
                    cart_goal.w = SHOW_CAMERA_ROTATE_ABOVE_TABLE[3]
                    cart_goal.p = SHOW_CAMERA_ROTATE_ABOVE_TABLE[4]
                    cart_goal.r = SHOW_CAMERA_ROTATE_ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ABOVE_TABLE[0]
                    cart_goal.y = ABOVE_TABLE[1]
                    cart_goal.z = ABOVE_TABLE[2]
                    cart_goal.w = ABOVE_TABLE[3]
                    cart_goal.p = ABOVE_TABLE[4]
                    cart_goal.r = ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ON_TABLE[0]
                    cart_goal.y = ON_TABLE[1]
                    cart_goal.z = ON_TABLE[2]
                    cart_goal.w = ON_TABLE[3]
                    cart_goal.p = ON_TABLE[4]
                    cart_goal.r = ON_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.grip_ac.wait_for_server()
                    grip_goal = OnRobotGripper.Goal()
                    grip_goal.width = 73
                    grip_goal.force = 40
                    future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(1)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ABOVE_TABLE[0]
                    cart_goal.y = ABOVE_TABLE[1]
                    cart_goal.z = ABOVE_TABLE[2]
                    cart_goal.w = ABOVE_TABLE[3]
                    cart_goal.p = ABOVE_TABLE[4]
                    cart_goal.r = ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                else:
                    print("still searching")
                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ABOVE_TABLE[0]
                    cart_goal.y = ABOVE_TABLE[1]
                    cart_goal.z = ABOVE_TABLE[2]
                    cart_goal.w = ABOVE_TABLE[3]
                    cart_goal.p = ABOVE_TABLE[4]
                    cart_goal.r = ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = X_Y[0]
                    cart_goal.y = X_Y[1]
                    cart_goal.z = X_Y[2]
                    cart_goal.w = X_Y[3]
                    cart_goal.p = X_Y[4]
                    cart_goal.r = X_Y[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = X_Y_TABLE[0]
                    cart_goal.y = X_Y_TABLE[1]
                    cart_goal.z = X_Y_TABLE[2]
                    cart_goal.w = X_Y_TABLE[3]
                    cart_goal.p = X_Y_TABLE[4]
                    cart_goal.r = X_Y_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.grip_ac.wait_for_server()
                    grip_goal = OnRobotGripper.Goal()
                    grip_goal.width = 120
                    grip_goal.force = 40
                    future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(1)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = X_Y_TABLE[0]
                    cart_goal.y = X_Y_TABLE[1]
                    cart_goal.z = X_Y_TABLE[2]
                    cart_goal.w = X_Y_TABLE[3]
                    cart_goal.p = X_Y_TABLE[4]
                    cart_goal.r = X_Y_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = X_Y[0]
                    cart_goal.y = X_Y[1]
                    cart_goal.z = X_Y[2]
                    cart_goal.w = X_Y[3]
                    cart_goal.p = X_Y[4]
                    cart_goal.r = X_Y[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ABOVE_TABLE[0]
                    cart_goal.y = ABOVE_TABLE[1]
                    cart_goal.z = ABOVE_TABLE[2]
                    cart_goal.w = ABOVE_TABLE[3]
                    cart_goal.p = ABOVE_TABLE[4]
                    cart_goal.r = ABOVE_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = ON_TABLE[0]
                    cart_goal.y = ON_TABLE[1]
                    cart_goal.z = ON_TABLE[2]
                    cart_goal.w = ON_TABLE[3]
                    cart_goal.p = ON_TABLE[4]
                    cart_goal.r = ON_TABLE[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    self.grip_ac.wait_for_server()
                    grip_goal = OnRobotGripper.Goal()
                    grip_goal.width = 73
                    grip_goal.force = 40
                    future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(1)

                    self.cart_ac.wait_for_server()
                    cart_goal = CartPose.Goal()
                    cart_goal.x = SHOW_CAMERA[0]
                    cart_goal.y = SHOW_CAMERA[1]
                    cart_goal.z = SHOW_CAMERA[2]
                    cart_goal.w = SHOW_CAMERA[3]
                    cart_goal.p = SHOW_CAMERA[4]
                    cart_goal.r = SHOW_CAMERA[5]
                    future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                    future.add_done_callback(callback=self._goal_response_callback)
                    time.sleep(5)

                    pip_count = self._inspect_dice()
                    self.get_logger().info(f'Sub-check pip count: {pip_count}')

                    if pip_count == target:
                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = SHOW_CAMERA_TABLE[0]
                        cart_goal.y = SHOW_CAMERA_TABLE[1]
                        cart_goal.z = SHOW_CAMERA_TABLE[2]
                        cart_goal.w = SHOW_CAMERA_TABLE[3]
                        cart_goal.p = SHOW_CAMERA_TABLE[4]
                        cart_goal.r = SHOW_CAMERA_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.grip_ac.wait_for_server()
                        grip_goal = OnRobotGripper.Goal()
                        grip_goal.width = 120
                        grip_goal.force = 40
                        future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(1)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = ABOVE_TABLE[0]
                        cart_goal.y = ABOVE_TABLE[1]
                        cart_goal.z = ABOVE_TABLE[2]
                        cart_goal.w = ABOVE_TABLE[3]
                        cart_goal.p = ABOVE_TABLE[4]
                        cart_goal.r = ABOVE_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = ON_TABLE[0]
                        cart_goal.y = ON_TABLE[1]
                        cart_goal.z = ON_TABLE[2]
                        cart_goal.w = ON_TABLE[3]
                        cart_goal.p = ON_TABLE[4]
                        cart_goal.r = ON_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.grip_ac.wait_for_server()
                        grip_goal = OnRobotGripper.Goal()
                        grip_goal.width = 73
                        grip_goal.force = 40
                        future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(1)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = ABOVE_TABLE[0]
                        cart_goal.y = ABOVE_TABLE[1]
                        cart_goal.z = ABOVE_TABLE[2]
                        cart_goal.w = ABOVE_TABLE[3]
                        cart_goal.p = ABOVE_TABLE[4]
                        cart_goal.r = ABOVE_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                    elif pip_count == opp:
                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = SHOW_CAMERA_INT_ROTATE[0]
                        cart_goal.y = SHOW_CAMERA_INT_ROTATE[1]
                        cart_goal.z = SHOW_CAMERA_INT_ROTATE[2]
                        cart_goal.w = SHOW_CAMERA_INT_ROTATE[3]
                        cart_goal.p = SHOW_CAMERA_INT_ROTATE[4]
                        cart_goal.r = SHOW_CAMERA_INT_ROTATE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = SHOW_CAMERA_ROTATE[0]
                        cart_goal.y = SHOW_CAMERA_ROTATE[1]
                        cart_goal.z = SHOW_CAMERA_ROTATE[2]
                        cart_goal.w = SHOW_CAMERA_ROTATE[3]
                        cart_goal.p = SHOW_CAMERA_ROTATE[4]
                        cart_goal.r = SHOW_CAMERA_ROTATE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = SHOW_CAMERA_ROT_TABLE[0]
                        cart_goal.y = SHOW_CAMERA_ROT_TABLE[1]
                        cart_goal.z = SHOW_CAMERA_ROT_TABLE[2]
                        cart_goal.w = SHOW_CAMERA_ROT_TABLE[3]
                        cart_goal.p = SHOW_CAMERA_ROT_TABLE[4]
                        cart_goal.r = SHOW_CAMERA_ROT_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.grip_ac.wait_for_server()
                        grip_goal = OnRobotGripper.Goal()
                        grip_goal.width = 120
                        grip_goal.force = 40
                        future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(1)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = ABOVE_TABLE[0]
                        cart_goal.y = ABOVE_TABLE[1]
                        cart_goal.z = ABOVE_TABLE[2]
                        cart_goal.w = ABOVE_TABLE[3]
                        cart_goal.p = ABOVE_TABLE[4]
                        cart_goal.r = ABOVE_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = ON_TABLE[0]
                        cart_goal.y = ON_TABLE[1]
                        cart_goal.z = ON_TABLE[2]
                        cart_goal.w = ON_TABLE[3]
                        cart_goal.p = ON_TABLE[4]
                        cart_goal.r = ON_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)

                        self.grip_ac.wait_for_server()
                        grip_goal = OnRobotGripper.Goal()
                        grip_goal.width = 73
                        grip_goal.force = 40
                        future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(1)

                        self.cart_ac.wait_for_server()
                        cart_goal = CartPose.Goal()
                        cart_goal.x = ABOVE_TABLE[0]
                        cart_goal.y = ABOVE_TABLE[1]
                        cart_goal.z = ABOVE_TABLE[2]
                        cart_goal.w = ABOVE_TABLE[3]
                        cart_goal.p = ABOVE_TABLE[4]
                        cart_goal.r = ABOVE_TABLE[5]
                        future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
                        future.add_done_callback(callback=self._goal_response_callback)
                        time.sleep(5)
            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = FRONT_CONV_ABOVE[0]
            cart_goal.y = FRONT_CONV_ABOVE[1]
            cart_goal.z = FRONT_CONV_ABOVE[2]
            cart_goal.w = FRONT_CONV_ABOVE[3]
            cart_goal.p = FRONT_CONV_ABOVE[4]
            cart_goal.r = FRONT_CONV_ABOVE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = FRONT_CONV[0]
            cart_goal.y = FRONT_CONV[1]
            cart_goal.z = FRONT_CONV[2]
            cart_goal.w = FRONT_CONV[3]
            cart_goal.p = FRONT_CONV[4]
            cart_goal.r = FRONT_CONV[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            print("letting go")
            self.grip_ac.wait_for_server()
            grip_goal = OnRobotGripper.Goal()
            grip_goal.width = 120
            grip_goal.force = 40
            future = self.grip_ac.send_goal_async(goal=grip_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(.5)

            self.cart_ac.wait_for_server()
            cart_goal = CartPose.Goal()
            cart_goal.x = FRONT_CONV_ABOVE[0]
            cart_goal.y = FRONT_CONV_ABOVE[1]
            cart_goal.z = FRONT_CONV_ABOVE[2]
            cart_goal.w = FRONT_CONV_ABOVE[3]
            cart_goal.p = FRONT_CONV_ABOVE[4]
            cart_goal.r = FRONT_CONV_ABOVE[5]
            future = self.cart_ac.send_goal_async(goal=cart_goal, feedback_callback=self._feedback_callback)
            future.add_done_callback(callback=self._goal_response_callback)
            time.sleep(5)

            self._modbus_write(CYCLE_REGISTER, target + 1)
            self._modbus_write(COORD_REGISTER, 1)

        self.get_logger().info('Pipeline complete.')



    # ── Callbacks ─────────────────────────────────────────────────────────

    def _feedback_callback(self, feedback_msg):
        pass

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        future.result().result


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node     = BunsenDiceNode()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
