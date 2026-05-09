#!/usr/bin/env python3
"""
move_beaker_node.py

ROS2 node for the Beaker FANUC robot. Combines the full dice-inspection
pipeline and the simpler pick-and-place conveyor handoff workflow.

Full pipeline (_run):
  1.  home  (joint)
  2.  Gripper OPEN
  3.  pick_dice  (joint)
  4.  Gripper CLOSE
  5.  home  (joint)
  6.  above_table  (cart)
  7.  on_table  (cart)  — place die under camera
  8.  Gripper OPEN
  9.  home  (joint)
  10. Capture image -> count pips
  11. Branch on pip_count vs target_pip / opp_pip

Run (after sourcing ROS2 and building):
    python3 move_beaker_node.py --ros-args \
        -p robot_name:=beaker \
        -p target_pip:=3 \
        -p opp_pip:=4
"""

import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from pymodbus.server import StartTcpServer
from pymodbus.datastore import ModbusDeviceContext, ModbusServerContext, ModbusSequentialDataBlock

from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper
from fanuc_interfaces.msg import ProxReadings
from fanuc_interfaces.srv import SetSpeed

import pip_counter
import grab

MODBUS_HOST = "10.8.4.19"
MODBUS_PORT = 5020
COORD_REGISTER = 0  # 0=idle, 1=robot1 working, 2=robot2 working
CYCLE_REGISTER = 1  # current cycle (1-6), advances on each handoff


# ── Pose constants (full pipeline, from move_beaker_node.py) ──────────────────

home      = [2.7272884845733643, -11.803813934326172, -12.07469367980957,
             -1.368260654999176e-05, -77.92526245117188, 32.81810760498047]   # joint
pick_dice = [16.961986541748047, 13.305452346801758, -58.92708969116211,
             -1.7103257050621323e-05, -31.072879791259766, 18.583459854125977] # joint

on_table    = [626.997802734375, 422.9151611328125, -115.12930297851562,
               -178.65402221679688, 0.14127972722053528, 118.52584838867188]
above_table = [626.997802734375, 422.9151611328125,  193.12930297851562,
               -178.65402221679688, 0.14127972722053528, 118.52584838867188]
above_table_rotate = [626.9978637695312, 422.9152526855469, 193.1291961669922,
                      -178.65411376953125, 0.14162315428256989, 21.915035247802734]
on_table_rotate    = [626.9978637695312, 422.9152526855469, -115.1291961669922,
                      -178.65411376953125, 0.14162315428256989, 21.915035247802734]

conv_front       = [148.23, 828.19, 25.667, 179.435, 1.301, -155.589]#[142.5789337158203, 617.3692626953125,   11.0885009765625,
                    #179.8, -5.218559817876667e-05, 123.86540222167969]
above_conv_front = [148.23, 828.19, 75.667, 179.435, 1.301, -155.589]#[142.5789337158203, 617.369140625,       221.2885284423828,
                    #179.8, -4.86212557007093e-05,  120.8293685913086]
back_conv        = [121.27876281738281, 15.291391372680664, -29.437347412109375,
                    -0.11526226997375488, -66.22425079345703, -9.508495330810547]  # joint
above_conv_back  = [122.16696166992188,  8.901558876037598, -21.29300880432129,
                     0.2479698657989502, -68.73829650878906,  -2.3875930309295654]

show_camera          = [572.6586303710938, 986.0700073242188, 625.0792236328125,
                        100.44342803955078, -66.16981506347656, 156.8412628173828]
on_table_show_camera = [626.997802734375,  395.9151611328125,  -140.5203094482422,
                         95.7508544921875, -65.40574645996094,  172.04368591308594]
above_table_show_camera_before = [626.997802734375,  395.9151611328125, -50.5203094482422,
                            95.7508544921875, -65.40574645996094, 172.04368591308594]

above_table_show_camera = [640.376220703125, 411.58001708984375, 193.5203094482422,
                            95.7508544921875, -65.40574645996094, 172.04368591308594]

show_camera_int    = [572.6586303710938, 986.070068359375, 625.0792846679688,
                      -94.7546615600586, -27.92806053161621, -11.357534408569336]
show_camera_rotate = [572.6586303710938, 986.070068359375, 625.0792846679688,
                      -99.36951446533203, 63.266075134277344, -21.97146224975586]
on_table_show_rotate    = [626.9977416992188, 395.9151611328125, -128.01463317871094,
                           -85.2046890258789, 59.14896011352539, 0.24636583030223846]
above_table_show_rotate = [626.9977416992188, 422.9151611328125,  193.01463317871094,
                           -85.2046890258789, 59.14896011352539, 0.24636583030223846]

#

# ── Pose constants (pick-and-place workflow, from robot1.py) ──────────────────

R1_HOME        = [18.446,  -7.714, -12.393,  0.285, -76.969,  99.095]
R1_DICE_ABOVE  = [18.164,   9.377, -51.754, -0.386, -39.782,  99.863]
R1_DICE_PICKUP = [18.165,  14.144, -58.408, -0.388, -35.008,  99.857]
R1_CAMERA      = [61.241,  20.961,   2.768, -84.885, -29.431, 109.099]

R1_BACK_CONV_ABOVE = [-190.955, 642.114,  87.894, 179.45,  -0.217, -153.744]
R1_BACK_CONV       = [-190.955, 642.114,  17.0,   179.45,  -0.217, -153.744]
R1_BEFORE_CONV     = [  48.82,  474.92,  203.408, 174.564, -6.589, -166.053]


class MoveBeakerNode(Node):

    def __init__(self):
        super().__init__('move_beaker_node')

        self.declare_parameter('robot_name',  'beaker')
        self.declare_parameter('move_speed',   300)
        self.declare_parameter('target_pip',     3)
        self.declare_parameter('opp_pip',        4)
        self.declare_parameter('exposure_ms', 500.0)

        self._cb = ReentrantCallbackGroup()
        robot = self.get_parameter('robot_name').value

        self._joint_client = ActionClient(self, JointPose,     f'{robot}/joint_pose',     callback_group=self._cb)
        self._cart_client  = ActionClient(self, CartPose,      f'{robot}/cartesian_pose', callback_group=self._cb)
        self._grip_client  = ActionClient(self, SchunkGripper, f'{robot}/schunk_gripper', callback_group=self._cb)
        self._convey_ac    = ActionClient(self, Conveyor,      f'{robot}/conveyor',       callback_group=self._cb)
        self._speed_client = self.create_client(SetSpeed, f'{robot}/set_speed',           callback_group=self._cb)

        self._sensor_triggered = False
        self._prox_sub = self.create_subscription(
            ProxReadings, f'/{robot}/prox_readings', self._prox_callback, 10)

        self._start_modbus_server()

        threading.Thread(target=self._run, daemon=True).start()
        self.get_logger().info('move_beaker_node started')

    # ── Motion helpers ────────────────────────────────────────────────────

    def _call_action(self, client, goal, timeout_sec=60.0) -> bool:
        """Send an action goal from a background thread and block until done."""
        done = threading.Event()
        result_box = [None]

        def on_goal(future):
            gh = future.result()
            if not gh.accepted:
                result_box[0] = False
                done.set()
                return
            gh.get_result_async().add_done_callback(on_result)

        def on_result(future):
            result_box[0] = future.result().result.success
            done.set()

        client.send_goal_async(goal).add_done_callback(on_goal)
        done.wait(timeout=timeout_sec)
        return bool(result_box[0])

    def _joint(self, pose: list) -> bool:
        if not self._joint_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('JointPose server not available')
            return False
        goal = JointPose.Goal()
        goal.joint1, goal.joint2, goal.joint3 = float(pose[0]), float(pose[1]), float(pose[2])
        goal.joint4, goal.joint5, goal.joint6 = float(pose[3]), float(pose[4]), float(pose[5])
        return self._call_action(self._joint_client, goal)

    def _cart(self, pose: list) -> bool:
        if not self._cart_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('CartPose server not available')
            return False
        goal = CartPose.Goal()
        goal.x, goal.y, goal.z = float(pose[0]), float(pose[1]), float(pose[2])
        goal.w, goal.p, goal.r = float(pose[3]), float(pose[4]), float(pose[5])
        return self._call_action(self._cart_client, goal)

    def _grip(self, command: str) -> bool:
        if not self._grip_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('SchunkGripper server not available')
            return False
        goal = SchunkGripper.Goal()
        goal.command = command
        return self._call_action(self._grip_client, goal)

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
        if msg.right:
            self._sensor_triggered = True

    def _wait_for_sensor(self):
        self._sensor_triggered = False
        while not self._sensor_triggered:
            rclpy.spin_once(self, timeout_sec=0.05)
        self._send_conveyor_goal('stop')

    # ── Modbus ────────────────────────────────────────────────────────────

    def _start_modbus_server(self):
        store = ModbusDeviceContext(hr=ModbusSequentialDataBlock(0, [0] * 10))
        self._mb_context = ModbusServerContext(devices=store, single=True)
        server_thread = threading.Thread(
            target=StartTcpServer,
            kwargs={'context': self._mb_context, 'address': (MODBUS_HOST, MODBUS_PORT)},
            daemon=True,
        )
        server_thread.start()
        time.sleep(0.5)
        self.get_logger().info(f'Modbus server started on {MODBUS_HOST}:{MODBUS_PORT}')

    def _modbus_write(self, address, value):
        try:
            self._mb_context[0x00].setValues(3, address, [value])
            self.get_logger().info(f'Modbus register {address} = {value}')
        except Exception as e:
            self.get_logger().error(f'Modbus write error: {e}')

    def _modbus_read(self, address):
        try:
            return self._mb_context[0x00].getValues(3, address, 1)[0]
        except Exception as e:
            self.get_logger().error(f'Modbus read error: {e}')
            return -1

    def _wait_for_robot2(self):
        self.get_logger().info('Waiting for robot1 to signal ready (register 0 = 2)...')
        while self._modbus_read(COORD_REGISTER) != 1:
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
        self._set_speed(self.get_parameter('move_speed').value)

        
        #target = self.get_parameter('target_pip').value
        #opp    = self.get_parameter('opp_pip').value
        Target_pip = 1
        opp_pip    = 7 - Target_pip  # opposite face on a standard die always sums to 7

        self._modbus_write(COORD_REGISTER, 1)
        self._modbus_write(CYCLE_REGISTER, 1)

        

        self.get_logger().info('home')
        self._joint(home)

        self.get_logger().info('Gripper open')
        self._grip('open')

        self.get_logger().info('pick_dice')
        self._joint(pick_dice)

        self.get_logger().info('Gripper close')
        self._grip('close')

        self.get_logger().info('home')
        self._joint(home)

        self.get_logger().info('above_table')
        self._cart(above_table)

        self.get_logger().info('on_table — place die')
        self._cart(on_table)

        self.get_logger().info('Gripper open — release die')
        self._grip('open')

        self.get_logger().info('above_table')
        self._cart(above_table)

        self.get_logger().info('above_table_show_camera')
        self._cart(above_table_show_camera)

        self.get_logger().info('on_table_show_camera — pick die for inspection')
        self._cart(on_table_show_camera)

        self.get_logger().info('Gripper close')
        self._grip('close')

        self.get_logger().info('show_camera')
        self._cart(show_camera)


        # ── Inspect die ───────────────────────────────────────────────────
        self.get_logger().info('Capturing image and counting pips')
        pip_count = self._inspect_dice()
        self.get_logger().info(
            f'pip_count={pip_count}  target={Target_pip}  opp={opp_pip}')

        # ── Branch on result ──────────────────────────────────────────────
        if pip_count == Target_pip:
            self.get_logger().info('Branch: target pip')
            self._cart(above_table_show_camera)
            self._cart(on_table_show_camera)
            self._grip('open')
            self._cart(above_table_show_camera)
            self._cart(above_table)
            self._cart(on_table)
            self._grip('close')
            self._cart(above_table)

        elif pip_count == opp_pip:
            self.get_logger().info('Branch: opposite pip — rotate die')
            self._cart(show_camera_int)
            self._cart(show_camera_rotate)
            self._cart(above_table_show_rotate)
            self._cart(on_table_show_rotate)
            self._grip('open')
            self._joint(home)
            self._cart(above_table)
            self._cart(on_table)
            self._grip('close')
            self._cart(above_table)

        else:
            self.get_logger().info('Branch: wrong pip — reposition and re-check')
            self._cart(above_table_show_camera)
            self._cart(on_table_show_camera)
            self._grip('open')
            self._cart(above_table_show_camera)
            
            self._cart(above_table)
            self._cart(on_table)
            self._grip('close')
            self._cart(above_table)
            self._cart(show_camera)

            pip_count = self._inspect_dice()
            self.get_logger().info(f'Re-check pip_count={pip_count}')

            if pip_count == Target_pip:
                print("Found correct pip after wrong pip")
                self._cart(on_table_show_camera)
                self._grip('open')
                #self._joint(home)
                print("Moving up")
                self._cart(above_table_show_camera_before)
                print("Getting ready to grab dice")
                self._cart(above_table_show_camera)
                self._joint(home)
                self._cart(above_table)
                self._cart(on_table)
                self._grip('close')
                self._cart(above_table)

            elif pip_count == opp_pip:
                print("Opp found after recheck")
                self._cart(show_camera_int)
                self._cart(show_camera_rotate)
                self._cart(above_table_show_rotate)
                self._cart(on_table_show_rotate)
                self._grip('open')
                self._joint(home)
                self._cart(above_table)
                self._cart(on_table)
                self._grip('close')
                self._cart(above_table)

            else:
                self.get_logger().info('Sub-branch: rotate on table and re-check')

                print("going above table")
                self._cart(above_table)
                print("rotating above table")
                self._cart(above_table_rotate)
                print("rotating on table")
                self._cart(on_table_rotate)
                self._grip('open')
                self._cart(above_table_rotate)

                self._cart(above_table)
                self._cart(on_table)
                self._grip('close')
                self._cart(show_camera)

                pip_count = self._inspect_dice()
                self.get_logger().info(f'Sub-branch re-check pip_count={pip_count}')

                if pip_count == Target_pip:
                    self._cart(above_table_show_camera)
                    self._cart(on_table_show_camera)
                    self._grip('open')
                    self._cart(above_table_show_camera)
                    self._cart(above_table)
                    self._cart(on_table)
                    self._grip('close')
                    self._cart(above_table)

                elif pip_count == opp_pip:
                    self._cart(show_camera_int)
                    self._cart(show_camera_rotate)
                    self._cart(above_table_show_rotate)
                    self._cart(on_table_show_rotate)
                    self._grip('open')
                    self._joint(home)
                    self._cart(above_table)
                    self._cart(on_table)
                    self._grip('close')
                    self._cart(above_table)


        self._run_pick_and_place()

        Target_pip = Target_pip + 1

        print("Turning on conveyor belt")
        self._send_conveyor_goal('forward')
        self._wait_for_sensor()
        next_cycle = (self._modbus_read(CYCLE_REGISTER) % 6) + 1
        self._modbus_write(CYCLE_REGISTER, next_cycle)
        self._modbus_write(COORD_REGISTER, 2)

        while True:
            self._wait_for_robot2()

            if(self._modbus_read(CYCLE_REGISTER) == 6):
                break

            self._cart(above_conv_front)
            self._cart(conv_front)

            self._grip('close')

            self._joint(home)
            self.get_logger().info('show_camera')
            self._cart(show_camera)


            # ── Inspect die ───────────────────────────────────────────────────
            self.get_logger().info('Capturing image and counting pips')
            pip_count = self._inspect_dice()
            self.get_logger().info(
                f'pip_count={pip_count}  target={Target_pip}  opp={opp_pip}')

            # ── Branch on result ──────────────────────────────────────────────
            if pip_count == Target_pip:
                self.get_logger().info('Branch: target pip')
                self._cart(above_table_show_camera)
                self._cart(on_table_show_camera)
                self._grip('open')
                self._cart(above_table_show_camera)
                self._cart(above_table)
                self._cart(on_table)
                self._grip('close')
                self._cart(above_table)

            elif pip_count == opp_pip:
                self.get_logger().info('Branch: opposite pip — rotate die')
                self._cart(show_camera_int)
                self._cart(show_camera_rotate)
                self._cart(above_table_show_rotate)
                self._cart(on_table_show_rotate)
                self._grip('open')
                self._joint(home)
                self._cart(above_table)
                self._cart(on_table)
                self._grip('close')
                self._cart(above_table)

            else:
                self.get_logger().info('Branch: wrong pip — reposition and re-check')
                self._cart(above_table_show_camera)
                self._cart(on_table_show_camera)
                self._grip('open')
                self._cart(above_table_show_camera)
                
                self._cart(above_table)
                self._cart(on_table)
                self._grip('close')
                self._cart(above_table)
                self._cart(show_camera)

                pip_count = self._inspect_dice()
                self.get_logger().info(f'Re-check pip_count={pip_count}')

                if pip_count == Target_pip:
                    print("Found correct pip after wrong pip")
                    self._cart(on_table_show_camera)
                    self._grip('open')
                    #self._joint(home)
                    print("Moving up")
                    self._cart(above_table_show_camera_before)
                    print("Getting ready to grab dice")
                    self._cart(above_table_show_camera)
                    self._joint(home)
                    self._cart(above_table)
                    self._cart(on_table)
                    self._grip('close')
                    self._cart(above_table)

                elif pip_count == opp_pip:
                    print("Opp found after recheck")
                    self._cart(show_camera_int)
                    self._cart(show_camera_rotate)
                    self._cart(above_table_show_rotate)
                    self._cart(on_table_show_rotate)
                    self._grip('open')
                    self._joint(home)
                    self._cart(above_table)
                    self._cart(on_table)
                    self._grip('close')
                    self._cart(above_table)

                else:
                    self.get_logger().info('Sub-branch: rotate on table and re-check')
                    print("going above table")
                    self._cart(above_table)
                    print("rotating above table")
                    self._cart(above_table_rotate)
                    print("rotating on table")
                    self._cart(on_table_rotate)
                    self._grip('open')
                    self._cart(above_table_rotate)

                    self._cart(above_table)
                    self._cart(on_table)
                    self._grip('close')
                    self._cart(show_camera)

                    pip_count = self._inspect_dice()
                    self.get_logger().info(f'Sub-branch re-check pip_count={pip_count}')

                    if pip_count == Target_pip:
                        self._cart(above_table_show_camera)
                        self._cart(on_table_show_camera)
                        self._grip('open')
                        self._cart(above_table_show_camera)
                        self._cart(above_table)
                        self._cart(on_table)
                        self._grip('close')
                        self._cart(above_table)

                    elif pip_count == opp_pip:
                        self._cart(show_camera_int)
                        self._cart(show_camera_rotate)
                        self._cart(above_table_show_rotate)
                        self._cart(on_table_show_rotate)
                        self._grip('open')
                        self._joint(home)
                        self._cart(above_table)
                        self._cart(on_table)
                        self._grip('close')
                        self._cart(above_table)
            self._run_pick_and_place()

            Target_pip = Target_pip + 1

            print("Turning on conveyor belt")
            self._send_conveyor_goal('forward')
            self._wait_for_sensor()
            next_cycle = (self._modbus_read(CYCLE_REGISTER) % 6) + 1
            self._modbus_write(CYCLE_REGISTER, next_cycle)
            self._modbus_write(COORD_REGISTER, 2)
        print("finished!!!")


        

    def _run_pick_and_place(self):
        """Pick die and drop on back conveyor, then signal robot2 (from robot1.py)."""
        

        self.get_logger().info('moving towards conveyor')
        self._cart(R1_BEFORE_CONV)
        self.get_logger().info('moving to the conveyor')
        self._cart(R1_BACK_CONV_ABOVE)
        self._cart(R1_BACK_CONV)

        self._grip('open')

        self._cart(R1_BACK_CONV_ABOVE)
        self._cart(R1_BEFORE_CONV)

        

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
        result = future.result().result


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)

    executor = MultiThreadedExecutor()
    node     = MoveBeakerNode()
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
