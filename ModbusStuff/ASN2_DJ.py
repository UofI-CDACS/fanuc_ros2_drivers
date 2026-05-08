
# ROS packages
import random
from xmlrpc import client
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
import sys
sys.path.append("../src/dependencies/")
# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper, SJointPose
# Modbus packages
import asyncio
from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadDecoder
from pymodbus.constants import Endian
import time
COIL_REGISTER_MAP = {
    # Bill
    "Ready_To_Pass": 0, 
    "Holding_Dice": 1,
    "In_Hand_Off_Position": 2,
    #DJ
    "Pose_valid": 3,
    "Ready_To_Receive": 4,
    "In_Recieve_Position": 5,
    "Gripper_Closed": 6,
    "Has_Dice": 7,
    #Shared
    "Fault": 8,
    "Reset": 9,
    "Cycle_Active": 10,
}
HOLDING_REGISTER_MAP = {
    # Bill
    "X": 0,
    "Y": 2,
    "Z": 4,
}

# The position where the two robots meet to handoff the object
handoff_middle_pos = [270.0,870.0,125.0,-90.0,-30.0,0.0]
# the handoff position that is far away safe from bumping into Bill
handoff_stay_away_pos =[270.0,500.0,125.0,-90.0,-30.0,0.0]
dice_place_pos = [ 630.00,0.0,70.000,179.000,0.0000,30.00]
# offsets to convert
x_offset = -30
y_offset = 1770
z_offset = 0

namespace = 'dj'
# Timeout before DJ declares an error state if we don't hear from Bill
timeout_time = 15

class FanucActions(Node):

    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        #Stuff to update modbus registers
        self.Pose_valid = False
        self.Ready_To_Receive = False
        self.In_Recieve_Position = False
        self.Gripper_Closed = False
        self.Has_Dice = False
        # Bill's state (read from modbus)
        self.bill_ready_to_pass = False
        self.bill_holding_dice = False
        self.bill_in_handoff_position = False
        self.fault = False
        self.reset = False
        self.cycle_active = False
        self.Bill_X = 0.0
        self.Bill_Y = 0.0
        self.Bill_Z = 0.0
        #self.create_timer(1.0, self.read_modbus_registers) # Update modbus registers every second
        #self.create_timer(1.0, self.update_modbus_registers) # Update modbus registers every second
    def run_test(self):
		# Cartesian
        #self.move_cartesian(dice_place_pos)
        # If we detect a reset in the beginning of the program return home and reset state
        if self.reset:
            self.get_logger().info("\033[33m Reset signal received, moving to home position and resetting state]\033[0m")
            self.home_position()
            self.Pose_valid = False
            self.Ready_To_Receive = False
            self.In_Recieve_Position = False
            self.Gripper_Closed = False
            self.Has_Dice = False
            return
        # If we detect a fault in the beginning of the program return home and reset state
        if self.fault:
            return # Do nothing if there's a fault, wait for reset
        # Start in a known home position with gripper open
        self.home_position()
        self.shunk_gripper("open")
        # Initialize modbus registers 
        self.Ready_To_Receive = False
        self.Pose_valid = False
        # Start a timer
        start_time = time.time()
        while not self.bill_holding_dice:
            self.get_logger().info("\033[33m Waiting for Bill to be holding dice...]\033[0m")
            #rclpy.spin_once(self, timeout_sec=1.0)  # Process timers while waiting
            self.read_modbus_registers() # Manually read registers to update state
            time.sleep(1)
            # Check for timeout to avoid waiting forever if something goes wrong on Bill's side
            if time.time() - start_time > timeout_time:
                self.get_logger().error("Timeout waiting for Bill to be holding dice!")
                self.fault = True
                self.update_modbus_registers() # Update modbus registers with new state
        # Once Bill is holding the dice, we can start the handoff process by moving to the stay away position and then the handoff position
        self.update_modbus_registers() # Update modbus registers with new state
        # Start a timer
        start_time = time.time()
        while not self.bill_ready_to_pass:
            self.get_logger().info("\033[33m Waiting for Bill to be ready to pass...]\033[0m")
            #rclpy.spin_once(self, timeout_sec=1.0)  # Process timers while waiting
            self.read_modbus_registers() # Manually read registers to update state
            time.sleep(1)
            # Check for timeout to avoid waiting forever if something goes wrong on Bill's side
            if time.time() - start_time > timeout_time:
                self.get_logger().error("Timeout waiting for Bill to be ready to pass!")
                self.fault = True
                self.update_modbus_registers() # Update modbus registers with new state
        # Convert Bill's coordinates to DJ's coordinate system and move to the handoff position
        X=self.Bill_X + x_offset
        Y=self.Bill_Y + y_offset
        Z=self.Bill_Z + z_offset
        # Move to the handoff position and update modbus registers with new state
        while not self.Pose_valid:
            self.read_modbus_registers() # Manually read registers to update state
            self.move_cartesian(handoff_stay_away_pos)
            self.Pose_valid = self.move_cartesian([X,Y,Z,handoff_middle_pos[3],handoff_middle_pos[4],handoff_middle_pos[5]])
            if self.Pose_valid:
                self.get_logger().info("\033[32m Pose is valid, moving to handoff position]\033[0m")
                self.Ready_To_Receive = True
                self.move_cartesian(handoff_stay_away_pos)
                self.update_modbus_registers() # Update modbus registers with new state
            else:
                self.get_logger().error("Error failed moving to handoff position")
                self.fault = True
                self.update_modbus_registers() # Update modbus registers with new state
                return
        # Start a timer
        start_time = time.time()
        # Wait for Bill to be in the handoff position before closing the gripper, updating modbus registers with new state, and checking for timeout
        while not self.bill_in_handoff_position:
            self.get_logger().info("\033[33m Waiting for Bill to be in handoff position...]\033[0m")
            self.read_modbus_registers() # Manually read registers to update state
            time.sleep(1)
            if time.time() - start_time > timeout_time:
                self.get_logger().error("Timeout waiting for Bill to be in handoff position!")
                self.fault = True
                self.update_modbus_registers() # Update modbus registers with new state
                return
        # Once Bill is in the handoff position, close the gripper to grab the dice, update modbus registers with new state, and check for timeout while waiting for Bill to release the dice (i.e. stop holding the dice)
        self.Pose_valid = self.move_cartesian([X,Y,Z,handoff_middle_pos[3],handoff_middle_pos[4],handoff_middle_pos[5]])
        self.shunk_gripper("close")
        self.Gripper_Closed = True
        self.update_modbus_registers() # Update modbus registers with new state
        start_time = time.time()
        while self.bill_holding_dice:
            self.get_logger().info("\033[33m Waiting for Bill to be holding the dice...]\033[0m")
            self.read_modbus_registers() # Manually read registers to update state
            time.sleep(1)
            if time.time() - start_time > timeout_time:
                self.get_logger().error("Timeout waiting for Bill to be holding the dice!")
                self.fault = True
                self.update_modbus_registers() # Update modbus registers with new state
        # Once Bill releases the dice, we can assume the handoff is complete, so we update our state and modbus registers, move to the stay away position, and then move to a new random position to finish the test
        self.Has_Dice = True
        self.update_modbus_registers() # Update modbus registers with new state
        self.move_cartesian(handoff_stay_away_pos)
        self.get_logger().info("\033[32m Handoff complete, moved to safe position]\033[0m")
        self.home_position()
        self.move_cartesian(dice_place_pos)
        self.shunk_gripper("open")
        self.home_position()
        self.get_logger().info("\033[32m Test complete, moved to home and opened gripper]\033[0m")
        # Reset Cycle register for next run
        self.cycle_active = False
        self.update_modbus_registers() # Update modbus registers with new state

    
    def move_cartesian(self, cartesian_pose):
        self.read_modbus_registers() # Update state before moving
        if self.fault:
            self.get_logger().error("Fault detected, cannot move!")
            while self.reset == False:
                self.read_modbus_registers() # Wait for reset
                time.sleep(1)
            self.get_logger().info("Reset detected, moving to home position and resetting state")
            self.home_position()
            self.shunk_gripper("open")
            self.Pose_valid = False
            self.Ready_To_Receive = False
            self.In_Recieve_Position = False
            self.Gripper_Closed = False
            self.Has_Dice = False
            self.update_modbus_registers() # Update modbus registers with new state
            exit(1)
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
            return False
        goal_handle = future.result()
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        future.add_done_callback(self.goal_response_callback)
        return True
    
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
            return False
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

    def update_modbus_registers(self):
        """Synchronous modbus update - runs every 1 second via ROS2 timer."""
        client = ModbusTcpClient("localhost", port=5020)
        client.connect()
        try:
            # Write DJ's coils
            client.write_coil(COIL_REGISTER_MAP["Pose_valid"], self.Pose_valid, slave=1)
            client.write_coil(COIL_REGISTER_MAP["Ready_To_Receive"], self.Ready_To_Receive, slave=1)
            client.write_coil(COIL_REGISTER_MAP["In_Recieve_Position"], self.In_Recieve_Position, slave=1)
            client.write_coil(COIL_REGISTER_MAP["Gripper_Closed"], self.Gripper_Closed, slave=1)
            client.write_coil(COIL_REGISTER_MAP["Has_Dice"], self.Has_Dice, slave=1)
            client.write_coil(COIL_REGISTER_MAP["Cycle_Active"], self.cycle_active, slave=1)
            client.write_coil(COIL_REGISTER_MAP["Fault"], self.fault, slave=1)
            client.write_coil(COIL_REGISTER_MAP["Reset"], self.reset, slave=1)
            # Write DJ's holding registers (X, Y, Z)
        except Exception as e:
            self.get_logger().error(f'Modbus error: {e}')
        finally:
            client.close()
    
    def read_modbus_registers(self):
        """Synchronous modbus read - can be called as needed."""
        client = ModbusTcpClient("localhost", port=5020)
        client.connect()
        try:
            # Read Bill's coils
            bill_ready_to_pass = client.read_coils(address=COIL_REGISTER_MAP["Ready_To_Pass"], count=1, slave=1).bits[0]
            bill_holding_dice = client.read_coils(address=COIL_REGISTER_MAP["Holding_Dice"], count=1, slave=1).bits[0]
            bill_in_handoff_position = client.read_coils(address=COIL_REGISTER_MAP["In_Hand_Off_Position"], count=1, slave=1).bits[0]
            error = client.read_coils(address=COIL_REGISTER_MAP["Fault"], count=1, slave=1).bits[0]
            reset = client.read_coils(address=COIL_REGISTER_MAP["Reset"], count=1, slave=1).bits[0]
            cycle_active = client.read_coils(address=COIL_REGISTER_MAP["Cycle_Active"], count=1, slave=1).bits[0]
            self.bill_ready_to_pass = bill_ready_to_pass
            self.bill_holding_dice = bill_holding_dice
            self.bill_in_handoff_position = bill_in_handoff_position
            result_x = client.read_holding_registers(address=HOLDING_REGISTER_MAP["X"], count=2, slave=1)
            decoder = BinaryPayloadDecoder.fromRegisters(result_x.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
            value_x = decoder.decode_32bit_float()
            result_y = client.read_holding_registers(address=HOLDING_REGISTER_MAP["Y"], count=2, slave=1)
            decoder = BinaryPayloadDecoder.fromRegisters(result_y.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
            value_y = decoder.decode_32bit_float()
            result_z = client.read_holding_registers(address=HOLDING_REGISTER_MAP["Z"], count=2, slave=1)
            decoder = BinaryPayloadDecoder.fromRegisters(result_z.registers, byteorder=Endian.BIG, wordorder=Endian.BIG)
            value_z = decoder.decode_32bit_float()
            self.Bill_X = value_x
            self.Bill_Y = value_y
            self.Bill_Z = value_z
            self.cycle_active = cycle_active
            if error:
                self.get_logger().error("Bill reported a fault!")
                # Send an interrupt or something to stop the robot
            self.fault = error
            if reset:
                self.get_logger().info("Bill reported a reset!")
                # Send an interrupt or something to stop the robot
            self.reset = reset
        except Exception as e:
            self.get_logger().error(f'Modbus error: {e}')
        finally:
            client.close()
    

if __name__ == '__main__':
    rclpy.init()
    fanuc = FanucActions(namespace)
    fanuc.run_test()
    rclpy.spin(fanuc)
    rclpy.shutdown()
