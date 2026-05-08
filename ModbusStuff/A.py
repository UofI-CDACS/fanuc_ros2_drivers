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


states = {
    "IDLE" : 0,
    "GENERATE_RANDOM_POSE" : 1,
    "MOVE_TO_POSE" : 2,
    "WAIT_FOR_B_READY" : 3,
    "WAIT_FOR_B_IN_POSITION" : 4,
    "RELEASE_DICE" : 5,
    "WAIT_FOR_B_CONFIRM" : 6,
    "DONE" : 7,
    "FAULT" : 8,
}

namespace = 'dj'
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
handoff_middle_pos = [250.0,-900.0,125.0,-90.0,-45.0,179.9]   # could be 88 instead of 90
safe_pose = [250.0,-600.0,125.0,145.0,-90.0,-55.0] 

class FanucActions(Node):
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.on_robot_ac = ActionClient(self,OnRobotGripper,f'/{namespace}/onrobot_gripper')
    #Basically your main function
    def main(self): # will have to pass client later
        #self.on_robot_gripper("open")
        # bill.home_position()
        # self.move_cartesian(home_pos_cart)
        self.home_position()
        self.move_cartesian(above_dice_start)
        self.move_cartesian(on_dice_start)
        #self.on_robot_gripper("close")
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

# Modbus Stuff
FXN_CODES = {
    "coil": 1,
    "discrete_input": 2,
 	"holding_register": 3,
	"input_register": 4,
}
COIL_REGISTER_MAP = {
    # Bill
    "Ready_To_Pass": 0,
    "Holding_Dice": 1,
    "In_Hand_Off_Position": 2,
    # DJ
    "Pose_valid": 3,
    "Ready_To_Receive": 4,
    "In_Recieve_Position": 5,
    "Gripper_Closed": 6,
    "Has_Dice": 7,
    # Shared
    "Fault": 8,
    "Reset": 9,
    "Cycle_Active": 10,
}
HOLDING_REGISTER_MAP = {
    "x": 0,
    "y": 2,
    "z": 4,
}
class ModbusMonitor(Node):
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
    

if __name__ == '__main__':
    rclpy.init()
    # modbus node
    # node = ModbusMonitor()
    # try:
    bill = FanucActions(namespace)
    bill.main()

    # node.write_coord(HOLDING_REGISTER_MAP["x"], 123.456)
    # node.write_coil(COIL_REGISTER_MAP["Ready_To_Pass"], 1)
    # val = node.read_coil(COIL_REGISTER_MAP["Pose_valid"])

    rclpy.spin(bill)
    # except KeyboardInterrupt:
    #     print("\nStopped.")
    # finally:
    #     # node.destroy_node()
    rclpy.shutdown()

    
