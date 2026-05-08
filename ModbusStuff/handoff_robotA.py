# ROS packages
import os
import random
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
import sys
sys.path.append("../src/dependencies/")
from time import sleep
# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, SchunkGripper, SJointPose, OnRobotGripper
from pymodbus.client import ModbusTcpClient
from pymodbus.payload import BinaryPayloadBuilder
from pymodbus.constants import Endian
import numpy  as np
import time

### ROBOT VARIABLES
TIMEOUT_LENGTH = 10
LONGER_TIMEOUT_LENGTH = 20
namespace = 'bill'
safe_offset = 300
# Gripper specs (for OnRobot gripper)
open_width = 100
close_width = 80
force = 40
# yaw, pitch, roll = 145,-90,-55
yaw, pitch, roll = -179.9, 0.0, -45.0
# CARTESIAN positions
home_pos_cart = [540.0, -150.0, 550.0, yaw, pitch, roll]
on_dice_start = [640.0, 0.0, 50.0, yaw, pitch, roll]
above_dice_start = [640.0, 0.0, 150.0, yaw, pitch, roll]
handoff_yaw = -90.0
handoff_pitch = -45.0
handoff_roll = 179.9
handoff_middle_pos = [300.0,-900.0,125.0,handoff_yaw, handoff_pitch,handoff_roll]   # could be 88 instead of 90
safe_pose = [250.0,-600.0,125.0,145.0,-90.0,-55.0] 

def generate_random_pos(rand_x_y_offset = 150, rand_z_offset = 125):
    """generate a random position in the reachable area of both robots

    Args:
        rand_x_y_offset (int, optional): max offset in the x and y. Defaults to 150.
        rand_z_offset (int, optional): max offset in the z. Defaults to 125.

    Returns:
        list: x,y,z coordinate
    """
    new_pos = []
    x = handoff_middle_pos[0]
    offset = random.randint((rand_x_y_offset*-1), rand_x_y_offset)
    x+=offset
    new_pos.append(x)
    y = handoff_middle_pos[1]
    offset = random.randint((rand_x_y_offset*-1), rand_x_y_offset)
    y+=offset
    new_pos.append(y)
    z = handoff_middle_pos[2]
    offset = random.randint((rand_z_offset*-1), rand_z_offset)
    z+=offset
    new_pos.append(z)
    new_pos.append(handoff_yaw)
    new_pos.append(handoff_pitch)
    new_pos.append(handoff_roll)
    return new_pos

def test_handoff_limits(bill,rand_x_y_offset = 150, rand_z_offset = 125):
    """used to test the max random handoff limits, not used in main code

    Args:
        bill (robot): fanuc robot object
        rand_x_y_offset (int, optional): max x,y offset. Defaults to 150.
        rand_z_offset (int, optional): max z offset. Defaults to 125.
    """
    print("***** -X, -Y, -Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]-=rand_x_y_offset
    p1[1]-=rand_x_y_offset
    p1[2]-=rand_z_offset
    bill.move_cartesian(p1)
    
    print("***** -X, -Y, Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]-=rand_x_y_offset
    p1[1]-=rand_x_y_offset
    p1[2]+=rand_z_offset
    bill.move_cartesian(p1)
    
    print("***** -X, Y, -Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]-=rand_x_y_offset
    p1[1]+=rand_x_y_offset
    p1[2]-=rand_z_offset
    bill.move_cartesian(p1)
    
    print("***** -X, Y, Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]-=rand_x_y_offset
    p1[1]+=rand_x_y_offset
    p1[2]+=rand_z_offset
    bill.move_cartesian(p1)
    
    print("***** X, -Y, -Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]+=rand_x_y_offset
    p1[1]-=rand_x_y_offset
    p1[2]-=rand_z_offset
    bill.move_cartesian(p1)
    
    print("***** X, -Y, Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]+=rand_x_y_offset
    p1[1]-=rand_x_y_offset
    p1[2]+=rand_z_offset
    bill.move_cartesian(p1)
    
    print("***** X, Y, -Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]+=rand_x_y_offset
    p1[1]+=rand_x_y_offset
    p1[2]-=rand_z_offset
    bill.move_cartesian(p1)
    
    print("***** X, Y, Z LIMIT *****")
    p1 = handoff_middle_pos.copy()
    p1[0]+=rand_x_y_offset
    p1[1]+=rand_x_y_offset
    p1[2]+=rand_z_offset
    bill.move_cartesian(p1)
    
def timeout_occurred(node, bill):
    """sets system to fault state after a timeout

    Args:
        node: modbus node object
        bill: fanuc robot object, passed to check_fault()
    """
    print("\033[31m******** Timeout Occurred! ********")
    node.write_coil(COIL_REGISTER_MAP["Fault"],1)
    print("BILL set Fault = 1")
    check_fault(node, bill)

def check_fault(node, bill):
    """checks if fault occurred. if it did, wait for reset bit to be set, else do nothing

    Args:
        node: modbus node object
        bill: fanuc robot object, passed to reset_cycle
    """
    if node.read_coil(COIL_REGISTER_MAP["Fault"]) == 1:
        print("\033[31m******** Fault Occurred! ********")
        print("Stopping all motion...")
        print("Waiting for Reset to = 1...\033[0m")
        # No timeout here because the only way out of fault state is reset
        while node.read_coil(COIL_REGISTER_MAP["Reset"]) == 0:
            sleep(0.2)
        reset_cycle(node, bill)
        
def reset_cycle(node, bill):
    """reset the cycle by resetting all bits, going home, opening gripper, exiting
            used when the reset bit is set after a fault state

    Args:
        node (modbus): modbus object
        bill (robot): fanuc robot object
    """
    bill.on_robot_gripper("open")
    sleep(2)
    node.reset_all_modbus_bits()
    bill.home_position()
    node.write_coil(COIL_REGISTER_MAP["Fault"],0)
    print("BILL set Fault = 0")
    print("\033[32mSuccessfully reset. Exiting cycle.\033[0m")
    sys.exit()
    
class FanucActions(Node):
    """defines all fanuc robot actions
    """
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        #self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.on_robot_ac = ActionClient(self,OnRobotGripper,f'/{namespace}/onrobot_gripper')
    #Basically your main function
    def main(self): # will have to pass client later
        self.on_robot_gripper("open")
        sleep(2)
        # bill.home_position()
        # self.move_cartesian(home_pos_cart)
        self.home_position()
        self.move_cartesian(above_dice_start)
        self.move_cartesian(on_dice_start)
        self.on_robot_gripper("close")
        sleep(2)
        # self.move_cartesian(home_pos_cart)
        self.home_position()
        self.move_cartesian(handoff_middle_pos)
        
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
        self.move_joints([0.0, 0.0, 0.0, 0.0, -90.0, -45.0])

    def schunk_gripper(self, command):
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

    def on_robot_gripper(self, command):
        # Gripper specs (for OnRobot gripper)
        open_width = 100
        close_width = 80
        force = 40
        if command == "close":
            print("closing gripper...")
            command = close_width
        elif command == "open":
            print("opening gripper...")
            command = open_width
        else:
            print("invalid gripper operation.")
            return
        print("Sending command to On Robot Gripper")
        self.on_robot_ac.wait_for_server() # Wait till its ready
        on_robot_goal = OnRobotGripper.Goal() # Make goal
        on_robot_goal.force = force
        on_robot_goal.width = command
        future = self.on_robot_ac.send_goal_async(on_robot_goal, feedback_callback=self.feedback_callback)
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

### MODBUS REGISTER MAPS
COIL_REGISTER_MAP = {
    # Bill - Robot A
    "Ready_To_Pass": 0,             # Bill has set position registers and is ready to pass
    "Holding_Dice": 1,              # Bill is holding die
    "In_Hand_Off_Position": 2,      # Bill holding die and in hand off position
    # DJ - Robot B
    "Pose_valid": 3,                # Pose Bill sent is valid
    "Ready_To_Receive": 4,          # DJ is out of the way, ready to recieve die
    "In_Recieve_Position": 5,       # DJ in die handoff location
    "Gripper_Closed": 6,            # DJ gripper is closed on die
    "Has_Dice": 7,                  # Bill has let go, DJ is holding die
    # Shared
    "Fault": 8,                     # Fault bit
    "Reset": 9,                     # Reset bit
    "Cycle_Active": 10,             # Clycle Active bit
}
HOLDING_REGISTER_MAP = {
    "x": 0,                         # X coordinate for handoff (Bill, DJ calculates offset on robot B side)
    "y": 2,                         # Y coordinate for handoff (Bill, DJ calculates offset on robot B side)
    "z": 4,                         # Z coordinate for handoff (Bill, DJ calculates offset on robot B side)
}

class ModbusMonitor(Node):
    """Modbus functionality for inter-robot communication
    """
    def __init__(self):
        super().__init__('modbus_monitor')

        self.client = ModbusTcpClient(
            os.environ.get('PARTNER_IP', 'localhost'), port=5020,
        )
        self.client.connect()

        # Poll every 100ms (10 Hz)
        self.timer = self.create_timer(0.1, self.poll_fault)

    def poll_fault(self):
        result = self.client.read_holding_registers(0, 1, slave=1)
        if result.isError():
            self.get_logger().error("Modbus read failed")
            return
        fault_state = result.registers[COIL_REGISTER_MAP["Fault"]]
        if fault_state:
            self.get_logger().warn(f"FAULT ACTIVE: {fault_state}")
            print("ERROR STATE!!!")
            
    def write_coord(self, register, value):
        test_x = np.float32(value)  
        builder = BinaryPayloadBuilder(byteorder=Endian.BIG, wordorder=Endian.BIG)
        builder.add_32bit_float(test_x)
        payload = builder.to_registers()
        self.client.write_registers(register, values=payload)
    
    def write_coil(self, coil, value):
        self.client.write_coil(coil, value)
    
    def read_coil(self, coil):
        raw_val = self.client.read_coils(coil, 1)
        return raw_val.bits[0]

    def destroy_node(self):
        self.client.close()
        super().destroy_node()    
    
    def reset_all_modbus_bits(self):
        print("\033[36mResetting all Modbus bits...\033[0m")
        for coil in COIL_REGISTER_MAP.values():
            self.write_coil(coil,0)
        for reg in HOLDING_REGISTER_MAP.values():
            self.write_coord(reg, 0)

if __name__ == '__main__':
    try:
        # init
        rclpy.init()
        node = ModbusMonitor()
        bill = FanucActions(namespace)
        
        # check for fault state
        check_fault(node,bill)
        
        # set cycle bit
        node.write_coil(COIL_REGISTER_MAP["Cycle_Active"],1)
        print("\033[36mBILL set Cycle_Active = 1\033[0m")
        
        # Pick up die from start location and go to home position
        bill.on_robot_gripper("open")
        sleep(2)
        bill.home_position()
        bill.move_cartesian(above_dice_start)
        bill.move_cartesian(on_dice_start)
        bill.on_robot_gripper("close")
        sleep(2)
        node.write_coil(COIL_REGISTER_MAP["Holding_Dice"],1)
        print("\033[36mBILL Holding_Dice = 1\033[0m")
        bill.home_position()
        
        # generate and publish random handoff position
        rand_handoff_pos = generate_random_pos()
        # send via Modbus
        node.write_coord(HOLDING_REGISTER_MAP["x"], rand_handoff_pos[0])
        node.write_coord(HOLDING_REGISTER_MAP["y"], rand_handoff_pos[1])
        node.write_coord(HOLDING_REGISTER_MAP["z"], rand_handoff_pos[2])
        node.write_coil(COIL_REGISTER_MAP["Ready_To_Pass"], 1)
        print(f"\033[36mBILL sent random pose: {rand_handoff_pos}")
        print("BILL Ready_To_Pass = 1\033[0m")
        
        # check for fault state
        check_fault(node,bill)
        
        # WAIT for robot B to be ready to accept the location
        print("\033[33mWaiting for DJ Ready_To_Receive to = 1...\033[0m")
        start_time = time.time()
        elapsed_time = 0
        while node.read_coil(COIL_REGISTER_MAP["Ready_To_Receive"]) == 0:
            sleep(1)
            elapsed_time = time.time() - start_time
            if(elapsed_time>=LONGER_TIMEOUT_LENGTH):
                timeout_occurred(node, bill)
        sleep(0.2)
        
        # check fault state
        check_fault(node,bill)
        
        # wait for robot B to validate and accept location
        print("\033[33mChecking DJ Pose_valid...\033[0m")
        while node.read_coil(COIL_REGISTER_MAP["Pose_valid"]) == 0:
            # Try again while Robot B rejects the location
            print("Invalid pose sent. Trying again.")
            node.write_coil(COIL_REGISTER_MAP["Ready_To_Pass"], 0)
            print("\033[36mBILL Ready_To_Pass = 0\033[0m")
            rand_handoff_pos = generate_random_pos()
            # send via Modbus
            node.write_coord(HOLDING_REGISTER_MAP["x"], rand_handoff_pos[0])
            node.write_coord(HOLDING_REGISTER_MAP["y"], rand_handoff_pos[1])
            node.write_coord(HOLDING_REGISTER_MAP["z"], rand_handoff_pos[2])
            node.write_coil(COIL_REGISTER_MAP["Ready_To_Pass"], 1)
            print(f"\033[36mBILL sent random pose: {rand_handoff_pos}")
            print("BILL Ready_To_Pass = 1\033[0m")
            # Wait for robot B to be ready to accept new location
            print("\033[33mWaiting for DJ Ready_To_Receive to = 1...\033[0m")
            start_time = time.time()
            elapsed_time = 0
            while node.read_coil(COIL_REGISTER_MAP["Ready_To_Receive"]) == 0:
                sleep(1)
                elapsed_time = time.time() - start_time
                if(elapsed_time>=TIMEOUT_LENGTH):
                    timeout_occurred(node, bill)
          
        # check fault state
        check_fault(node,bill)
          
        # go to handoff location
        print("\033[32mPosition Accepted!\033[0m Going to Handoff spot...")
        check_fault(node,bill)
        bill.move_cartesian(rand_handoff_pos)
        node.write_coil(COIL_REGISTER_MAP["In_Hand_Off_Position"],1)
        print("\033[36mBILL In_Hand_Off_Position = 1\033[0m")
        
        # wait for robot B to grab die
        print("\033[33mWaiting for DJ Gripper_Closed to = 1...\033[0m")
        start_time = time.time()
        elapsed_time = 0
        while node.read_coil(COIL_REGISTER_MAP["Gripper_Closed"]) == 0:
            sleep(0.2)
            elapsed_time = time.time() - start_time
            if(elapsed_time>=TIMEOUT_LENGTH):
                timeout_occurred(node, bill)
                    
        # check fault state
        check_fault(node,bill)
        
        # let go of die and tell robot B 
        bill.on_robot_gripper("open")
        sleep(1)
        node.write_coil(COIL_REGISTER_MAP["Holding_Dice"],0)
        print("\033[36mBILL Holding_Dice = 0\033[0m")
        
        # ensure robot B has the die
        print("\033[33mWaiting for DJ Has_Dice to = 0...\033[0m")
        start_time = time.time()
        elapsed_time = 0
        while node.read_coil(COIL_REGISTER_MAP["Has_Dice"]) == 0:            
            sleep(0.2)
            elapsed_time = time.time() - start_time
            if(elapsed_time>=TIMEOUT_LENGTH):
                    timeout_occurred(node, bill)
        print("\033[32mHandoff successful! Going home.\033[0m")
        
        # check fault state
        check_fault(node,bill)
        
        # back up from handoff location
        rand_handoff_pos[1]+=200
        bill.move_cartesian(rand_handoff_pos)
        
        # go home and check fault
        bill.home_position()
        check_fault(node,bill)
        
        # wait for cycle to be over
        print("\033[33mWaiting for DJ to set Cycle_Active to 0...\033[0m")
        start_time = time.time()
        elapsed_time = 0
        while node.read_coil(COIL_REGISTER_MAP["Cycle_Active"]) == 1:
            sleep(0.2)
            elapsed_time = time.time() - start_time
            if(elapsed_time>=TIMEOUT_LENGTH):
                    timeout_occurred(node, bill)
        
        # check fault state
        check_fault(node,bill)
        
        # reset all bits
        node.reset_all_modbus_bits()
        print("\033[32mCycle Complete.\033[0m")
        
        # spin ros2 actions
        rclpy.spin(bill)
    except KeyboardInterrupt:
        print("\nProgram Stopped.")
    finally:
        # Do not reset all bits because a premature exit could have meant a fault that I want to ensure robot B handles
        node.destroy_node()
        sys.exit()