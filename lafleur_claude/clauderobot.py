##
# Manufacturing Line
# Tyler LaFleur

# Tasks:
#   Working with your partner and using assistance from Claude, we will write ROS2 code to perform a few tasks:

#     Pick up the Dice from in front of robot 1 (team decides robot 1 and robot 2)
#     Robot 1 presents it to the camera and captures an image (overhead for FANUC’s)
#     Run process to count the pips in the dice, make sure to save this somewhere in your program and communicate it to the other robot. You will need to do this until you see pip count of 1. That marks the “start” state.
#         NOTE: Only one camera server node is allowed. You decide who will spin that up, then you will both have client nodes for each robot when camera is needed.
#     You will then pass the dice to robot 2 using the conveyor
#         If pip count is even, use the "front" (closest) conveyor

#         If pip count is odd, use the "back" (furthest) conveyor
#     With this in mind, each robot controls one conveyor so you must communicate in order to use both conveyors.

# Process will be continued until all pips have been counted SEQUENTIALLY (start at 1, finish on 6), keeping track of retries (how many times robot takes to get the correct number of pips - TOTAL COUNT for each robot + together). So you will have total retries for robot 1 and 2, as well as both combined, presented neatly.

#     Robot who has pip 6 at the end is in charge of placing the dice down in front of itself (this will be robot 2).
# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import sys
sys.path.append("../src/dependencies/")

import time
from time import sleep

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper, SJointPose, OnRobotGripper
from fanuc_interfaces.msg import ProxReadings 

# Modbus packages
import asyncio
import os
from dotenv import load_dotenv
from pymodbus.client import AsyncModbusTcpClient

load_dotenv(os.path.join(os.path.dirname(__file__), '.env'))
MODBUS_IP = os.environ["MODBUS_IP"]
MODBUS_PORT = int(os.environ["MODBUS_PORT"])

namespace = 'Bill'
positions = [
    [0.0, 0.0, 0.0, 0.0, -90.0, -50.0], # home pos (joint)

    [-100.0, 0.0, 0.0, 0.0, -90.0, -50.0], # convstandby
    [-470.0, -700.0, 300.0, 179.0, 0.0, 130.0], # bconvup
    [-470.0, -700.0, 240.0, 179.0, 0.0, 130.0], # bconvdown
    [-70.0, 0.0, 0.0, 0.0, -90.0, -50.0], # camstandby
    [-70.0, 15.0, 0.0, 0.0, -5.0, -50.0], # camat

    [-70.0, 15.0, 0.0, 0.0, -5.0, 40.0], # flip1
    [630.0, -380.0, 160.0, -179.0, 0.0, -135.0], # fix1up
    [630.0, -380.0, 110.0, -179.0, 0.0, -135.0], # fix1down
    [630.0, -380.0, 160.0, -179.0, 0.0, -45.0], # fix1rotateup
    [630.0, -380.0, 100.0, -179.0, 0.0, -45.0], # fix1rotatedown

    [0.0, 10.0, 0.0, 0.0, -85.0, 45.0], # fix2standby
    [650.0, -125.0, -40.0, 95.0, 45.0, 0.0], # fix2up cart
    [650.0, -125.0, -130.0, 95.0, 45.0, 0.0], # fix2down
    [650.0, 0.0, 120.0, 179.0, 0.0, -50.0], # placeup
    [650.0, 0.0, 50.0, 179.0, 0.0, -50.0], # placedown

    [-70.0, 15.0, 0.0, 0.0, -5.0, 130.0], # flip2
    [-165.0, -840.0, 300.0, 179.0, 0.0, 130.0], # fconvup
    [-165.0, -840.0, 240.0, 179.0, 0.0, 130.0], # fconvdown
            ]

SIGNALS = {
    # DJ
    "DJ_GRIPPER_CLOSED":        0,
    "DJ_HAS_DICE":              1,
    "DJ_READY_FOR_PICTURE":     2,
    "DJ_AT_CONVEYOR":           3, 
    "DJ_CONVEYOR_ACTIVE":       4,
    "DJ_CONVEYOR_DICE_READY":   5,
    # Bill
    "BILL_GRIPPER_CLOSED":      10,
    "BILL_HAS_DICE":            11,
    "BILL_READY_FOR_PICTURE":   12,
    "BILL_AT_CONVEYOR":         13, 
    "BILL_CONVEYOR_ACTIVE":     14,
    "BILL_CONVEYOR_DICE_READY": 15,
    # Camera
    "CAMERA_READY":     20,
    "CAMERA_DONE":      21,
    # Shared / Safety
    "FAULT":            29,
    "RESET":            30,
    "CYCLE_ACTIVE":     31,
}

# Holding Registers
# 0:    Target Number
# 1:    Pip Count
# 2:    DJ Number of Tries
# 3:    Bill Number of Tries

class FanucActions(Node):
    current_state = 1
    attempt_count = 0
    registers = []
    prox_triggered = False 

    def __init__(self, namespace):
        super().__init__("robot")

        # Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.convey_ac = ActionClient(self, Conveyor, f'/{namespace}/conveyor')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.sin_joint_ac = ActionClient(self, SJointPose, f'/{namespace}/single_joint_pose')
        self.onRobotGripper_ac = ActionClient(self, OnRobotGripper, f'/{namespace}/onrobot_gripper')

        self.create_subscription(ProxReadings, f'/{namespace}/prox_readings', self.prox_callback, 10) 

    def prox_callback(self, msg):
        if msg.left:
            self.prox_triggered = True 

    def state_test(self):
        while(1):
            match(self.current_state):
                case 1: # Reset to home position and standby for start signal
                    print("In HOME state")

                    # Reset all coils
                    asyncio.run(write_coil(SIGNALS["BILL_HAS_DICE"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_READY_FOR_PICTURE"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_AT_CONVEYOR"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_ACTIVE"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_DICE_READY"], False))

                    # Reset position
                    self.joint_move(positions[0]) # home
                    self.open_gripper()

                    # Start up the sequence when DJ is ready
                    Cycle_Start = asyncio.run(read_coil(SIGNALS["CYCLE_ACTIVE"]))
                    while Cycle_Start != True:
                        sleep(1)
                        print("Waiting for cycle start...")
                        Cycle_Start = asyncio.run(read_coil(SIGNALS["CYCLE_ACTIVE"]))

                    self.current_state = 2

                case 2: # Wait for dice to arrive at the conveyor
                    print("Picking up dice at conveyor.")
                    # Move to (back) conveyor
                    self.joint_move(positions[1])

                    # Wait for the dice to arrive at the conveyor
                    Dice_ready = asyncio.run(read_coil(SIGNALS["DJ_CONVEYOR_DICE_READY"]))
                    while Dice_ready != True:
                        sleep(1)
                        print("Waiting for dice to arrive...")
                        Dice_ready = asyncio.run(read_coil(SIGNALS["DJ_CONVEYOR_DICE_READY"]))

                    # Pick up dice
                    self.cart_move(positions[2])
                    self.cart_move(positions[3])
                    self.close_gripper()
                    self.cart_move(positions[2])
                    asyncio.run(write_coil(SIGNALS["BILL_HAS_DICE"], True))
                    self.joint_move(positions[4])

                    self.current_state = 3

                case 3: # Show to camera
                    print("Showing to camera")
                    # Go to camera standby
                    self.joint_move(positions[5])
                    asyncio.run(write_coil(SIGNALS["BILL_READY_FOR_PICTURE"], True))

                    # Wait for the camera to analyze
                    Camera_Done = asyncio.run(read_coil(SIGNALS["CAMERA_DONE"]))
                    while Camera_Done != True:
                        sleep(1)
                        print("Waiting for camera...")
                        Camera_Done = asyncio.run(read_coil(SIGNALS["CAMERA_DONE"]))
                        
                    asyncio.run(write_coil(SIGNALS["BILL_READY_FOR_PICTURE"], False))
                    self.registers = asyncio.run(read_registers())
                    # If we found our target number
                    if self.registers[1] == self.registers[0]:
                        # If we are at pip 6
                        if self.registers[1] >= 6:
                            self.current_state = 7
                        else:
                            asyncio.run(write_registers(
                                self.registers[0]+1,
                                self.registers[1],
                                self.registers[2],
                                self.registers[3]
                            ))
                            self.current_state = 6
                    # If target number is on opposite side
                    elif self.registers[1] + self.registers[0] == 7:
                        self.attempt_count = self.attempt_count + 1
                        asyncio.run(write_registers(
                                self.registers[0],
                                self.registers[1],
                                self.registers[2],
                                self.attempt_count
                            ))
                        self.current_state = 4
                    # If target is still unknown
                    else:
                        self.attempt_count = self.attempt_count + 1
                        asyncio.run(write_registers(
                                self.registers[0],
                                self.registers[1],
                                self.registers[2],
                                self.attempt_count
                            ))
                        self.current_state = 5
                
                case 4: # Flip the dice around
                    print("Target pip on opposite side; Flipping dice")
                    self.joint_move(positions[6])
                    self.joint_move(positions[16])
                    self.current_state = 3
                    asyncio.run(write_coil(SIGNALS["BILL_READY_FOR_PICTURE"], True))
                    
                    # Wait for the camera to analyze
                    Camera_Done = asyncio.run(read_coil(SIGNALS["CAMERA_DONE"]))
                    while Camera_Done != True:
                        sleep(1)
                        print("Waiting for camera...")
                        Camera_Done = asyncio.run(read_coil(SIGNALS["CAMERA_DONE"]))
                        
                    asyncio.run(write_coil(SIGNALS["BILL_READY_FOR_PICTURE"], False))
                    self.registers = asyncio.run(read_registers())
                    if self.registers[1] == self.registers[0]:
                        # If we are at pip 6
                        if self.registers[1] >= 6:
                            self.current_state = 7
                        else:
                            asyncio.run(write_registers(
                                self.registers[0]+1,
                                self.registers[1],
                                self.registers[2],
                                self.registers[3]
                            ))
                            self.current_state = 6
                    


                case 5: # Go to new dice pip pair
                    print("Target pip location unknown; Checking new dice face")
                    self.joint_move(positions[4])
                    if self.attempt_count % 2 == 1:
                        # rotate hand 90 (perpendicular to conveyor to parallel to conveyor)
                        self.cart_move(positions[7])
                        self.cart_move(positions[8])
                        self.open_gripper()
                        self.cart_move(positions[7])
                        self.cart_move(positions[9])
                        self.cart_move(positions[10])
                        self.close_gripper()
                        self.cart_move(positions[9])
                        self.joint_move(positions[4])
                    else: 
                        # rotate hand 90 (perpendicular to table to parallel to table)
                        self.joint_move(positions[11])
                        self.cart_move(positions[12])
                        self.cart_move(positions[13])
                        self.open_gripper()
                        self.cart_move(positions[12])
                        self.joint_move(positions[11])
                        self.cart_move(positions[7])
                        self.cart_move(positions[8])
                        self.close_gripper()
                        self.joint_move(positions[4])
                    self.current_state = 3

                case 6: # Send dice back to DJ via conveyor
                    print("Target found, moving dice to conveyor")
                    self.joint_move(positions[4])
                    self.joint_move(positions[1])
                    self.cart_move(positions[17])
                    self.cart_move(positions[18])
                    self.open_gripper()
                    self.cart_move(positions[17])
                    
                    asyncio.run(write_coil(SIGNALS["BILL_HAS_DICE"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_AT_CONVEYOR"], True))

                    # Conveyor
                    self.prox_triggered = False
                    self.start_conveyor()
                    deadline = time.time() + 30.0
                    while not self.prox_triggered and time.time() < deadline:
                        rclpy.spin_once(self, timeout_sec=0.01)
                    sleep(2)
                    self.stop_conveyor()

                    asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_DICE_READY"], True))
                    
                    # When light sensor is reset (b/c DJ picked up dice)
                    sleep(10)
                    asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_DICE_READY"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_AT_CONVEYOR"], False))
                    self.current_state = 2

                case 7: # Wrap up then go back home
                    print("Cycle complete, wrapping up.")
                    self.joint_move(positions[4])
                    self.cart_move(positions[14])
                    self.cart_move(positions[15])
                    self.open_gripper()
                    asyncio.run(write_coil(SIGNALS["BILL_HAS_DICE"], False))
                    self.cart_move(positions[14])
                    self.joint_move(positions[0])
                    asyncio.run(write_coil(SIGNALS["CYCLE_ACTIVE"], False))
                    self.current_state = 8

                case 8: # Print out the run information
                    self.registers = asyncio.run(read_registers())
                    djAttempts = self.registers[2]
                    totalAttempts = djAttempts + self.attempt_count
                    djAttempts = djAttempts - 1
                    print(f"===================")
                    print(f"Total Attempts: {totalAttempts}")
                    print(f"___________________")
                    print(f"DJ Retries:    {djAttempts}")
                    print(f"Bill Retries:  {self.attempt_count}")
                    print(f"===================")
                    exit()

                case 0: # FAULT
                    print("In FAULT state")
                    asyncio.run(write_coil(SIGNALS["BILL_HAS_DICE"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_READY_FOR_PICTURE"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_AT_CONVEYOR"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_ACTIVE"], False))
                    asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_DICE_READY"], False))

                    self.current_state = 1
                
                case _:
                    self.current_state = 0


    def start_conveyor(self):
        print("Starting Conveyor")
        self.convey_ac.wait_for_server()
        convey_goal = Conveyor.Goal()
        convey_goal.command = 'reverse'
        future = self.convey_ac.send_goal_async(convey_goal)
        future.add_done_callback(self.goal_response_callback)
        asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_ACTIVE"], True))

    def stop_conveyor(self):
        print("Stopping Conveyor")
        self.convey_ac.wait_for_server()
        convey_goal = Conveyor.Goal()
        convey_goal.command = 'stop'
        future = self.convey_ac.send_goal_async(convey_goal)
        future.add_done_callback(self.goal_response_callback)
        asyncio.run(write_coil(SIGNALS["BILL_CONVEYOR_ACTIVE"], False))

    def open_gripper(self):
        print("Opening gripper")
        self.onRobotGripper_ac.wait_for_server()
        gripper_goal = OnRobotGripper.Goal()
        gripper_goal.width = 100
        gripper_goal.force = 100
        future = self.onRobotGripper_ac.send_goal_async(gripper_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        sleep(2)
        asyncio.run(write_coil(SIGNALS["BILL_GRIPPER_CLOSED"], False))

    def close_gripper(self):
        print("Closing gripper")
        self.onRobotGripper_ac.wait_for_server()
        gripper_goal = OnRobotGripper.Goal()
        gripper_goal.width = 80
        gripper_goal.force = 50
        future = self.onRobotGripper_ac.send_goal_async(gripper_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        sleep(2)
        asyncio.run(write_coil(SIGNALS["BILL_GRIPPER_CLOSED"], True))

    def joint_move(self, joints):
        # Joints
        print("Moving (Joint)")
        self.joints_ac.wait_for_server()
        joint_goal = JointPose.Goal()
        # Add all joints
        joint_goal.joint1 = joints[0]
        joint_goal.joint2 = joints[1]
        joint_goal.joint3 = joints[2]
        joint_goal.joint4 = joints[3]
        joint_goal.joint5 = joints[4]
        joint_goal.joint6 = joints[5]
        future = self.joints_ac.send_goal_async(joint_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        sleep(5)

    def cart_move(self, carts):
        # Joints
        print("Moving (Cart)")
        self.cart_ac.wait_for_server() # Wait till its ready
        cart_goal = CartPose.Goal() # Make goal
        # Add all coordinates 
        cart_goal.x = carts[0]
        cart_goal.y = carts[1]
        cart_goal.z = carts[2]
        cart_goal.w = carts[3]
        cart_goal.p = carts[4]
        cart_goal.r = carts[5]
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

async def write_coil(address, value):
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    result = await client.write_coil(address, value)
    if result.isError():
        print(f"Error writing coil {address}: {result}")
    else:
        print(f"Set coil ({address}) = {value}")
    client.close()

async def read_coil(address):
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    result = await client.read_coils(address, count=1)
    client.close()
    if result.isError():
        print(f"Error reading coil {address}: {result}")
        return None
    return result.bits[0]

async def read_pose():
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    await read_pose_registers(client)
    client.close()

async def read_pose_registers(client):
    pose_result = await client.read_holding_registers(address=0, count=3, device_id=1)
    if not pose_result.isError():
        pose_regs = pose_result.registers
        print(f"Read Pose Registers: X={pose_regs[0]}, Y={pose_regs[1]}, Z={pose_regs[2]}")
        positions[2][0] = float(pose_regs[0])
        positions[2][1] = float(pose_regs[1])
        positions[2][2] = float(pose_regs[2])
        return pose_regs
    else:
        print("Error reading pose registers!")
        return None

async def write_signal(client, signal_name, value):
    """Write a single signal/coil to the server."""
    address = SIGNALS[signal_name]
    result = await client.write_coil(address, value, device_id=1)
    if result.isError():
        print(f"Error writing {signal_name}: {result}")
    else:
        print(f"Wrote {signal_name} = {value}")


async def write_registers(target_number, pip_count, dj_tries, bill_tries):
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    values = [target_number, pip_count, dj_tries, bill_tries]
    result = await client.write_registers(address=0, values=values, device_id=1)
    if result.isError():
        print(f"Error writing registers: {result}")
    else:
        print(f"Wrote TARGET_NUMBER={target_number}, PIP_COUNT={pip_count}, "
              f"DJ_NUMBER_OF_TRIES={dj_tries}, BILL_NUMBER_OF_TRIES={bill_tries}")
    client.close()


async def read_registers():
    client = AsyncModbusTcpClient(MODBUS_IP, port=MODBUS_PORT)
    await client.connect()
    result = await client.read_holding_registers(address=0, count=4, device_id=1)
    client.close()
    if not result.isError():
        regs = result.registers
        print(f"Read Registers - Target: {regs[0]}, Pips: {regs[1]}, "
              f"DJ Tries: {regs[2]}, BILL Tries: {regs[3]}")
        return regs
    else:
        print("Error reading registers!")
        return None

def translate_coords(coords):
    translated_coords = coords
    translated_coords[0] += 40.0
    translated_coords[1] = -700.0 - (900.0 - coords[1])
    translated_coords[2] = float(coords[2])
    return translated_coords

if __name__ == '__main__':
    rclpy.init()
    fanuc = FanucActions(namespace)

    fanuc.state_test()
    rclpy.spin(fanuc)
        
    rclpy.shutdown()