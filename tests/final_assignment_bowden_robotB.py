# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient
import sys
sys.path.append("../src/dependencies/")

# Miscellaneous packages
from time import sleep
from datetime import datetime
import cv2
import numpy as np
import mvsdk

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, OnRobotGripper, SJointPose
from fanuc_interfaces.msg import ProxReadings
from std_msgs.msg import Int32

# Modbus stuff
from pymodbus.client import ModbusTcpClient
HOST = "MODBUS_HOST"
PORT = 0  # set to your Modbus TCP port
COIL_REGISTER_MAP = {
   "DJ_Has_Dice":     0,
   "Bill_Has_Dice":   1,
   "Ready_For_Pickup":2,
   "Cycle_Active":    3,
}
HOLDING_REGISTER_MAP = {
   "Total_Pip_Count": 0,
   "Total_Retries":   1,
   "Bill_Retries":    2,
   "DJ_Retries":      3,
   "Last_Known_Pip":  4,
}
# Reverse: {0: "DJ_Has_Dice", 1: "Bill_Has_Dice", ...}                        
COIL_NAME_BY_VALUE = {v: k for k, v in COIL_REGISTER_MAP.items()}
REGISTER_NAME_BY_VALUE = {v: k for k, v in HOLDING_REGISTER_MAP.items()}

namespace =  'bill' # 'bunsen'

class FanucActions(Node):
   def __init__(self, namespace):
      super().__init__("robot")
   
   # Actions
      self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
      self.convey_ac = ActionClient(self, Conveyor, f'/{namespace}/conveyor')
      self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
      self.onrobot_ac = ActionClient(self, OnRobotGripper, f'/{namespace}/onrobot_gripper')
      self.sin_joint_ac = ActionClient(self, SJointPose, f'/{namespace}/single_joint_pose')

   # Proximity sensor state
      self.prox_left = False
      self.prox_right = False
      self.create_subscription(ProxReadings, f'/{namespace}/prox_readings', self._prox_callback, 10)

   # DJ's pip count feed
      self.pip_count = None
      self.create_subscription(Int32, '/camera/pip_count', self._pip_count_cb, 10)

   def _prox_callback(self, msg):
      self.prox_left = msg.left
      self.prox_right = msg.right

   def _pip_count_cb(self, msg):
      self.pip_count = msg.data

   def get_pip_count(self):
      self.pip_count = None
      while rclpy.ok() and self.pip_count is None:
         rclpy.spin_once(self, timeout_sec=0.1)
      return self.pip_count

   def poll_pip_count(self):
      print("Polling pip count (Ctrl+C to stop)...")
      while rclpy.ok():
         print(f"  pip count: {self.get_pip_count()}")

   def conveyor(self, command='stop'):
      print(f"Conveyor: {command}")
      self.convey_ac.wait_for_server()
      goal = Conveyor.Goal()
      goal.command = command
      future = self.convey_ac.send_goal_async(goal)
      rclpy.spin_until_future_complete(self, future)
      if not future.result() or not future.result().accepted:
         self.get_logger().info('Conveyor goal rejected')
         return False
      goal_handle = future.result()
      result_future = goal_handle.get_result_async()
      rclpy.spin_until_future_complete(self, result_future)
      return result_future.result().result.success

   def wait_for_prox(self, side='left', state=True):
      print(f"Waiting for {side} proximity sensor to be {state}...")
      while rclpy.ok():
         rclpy.spin_once(self, timeout_sec=0.1)
         current = self.prox_left if side == 'left' else self.prox_right
         if current == state:
            print(f"{side} sensor is {state}")
            return

   def poll_prox(self):
      print("Polling proximity sensors (Ctrl+C to stop)...")
      while rclpy.ok():
         rclpy.spin_once(self, timeout_sec=0.1)
         print(f"  left: {self.prox_left}  right: {self.prox_right}")
         sleep(0.5)

   def cart_move(self, x=540.0, y=-150.0, z=550.0, w=179.9, p=0.0, r=-45.0):
      print(f"Cartesian move to  [{x}, {y}, {z}, {w}, {p}, {r}]")
      self.cart_ac.wait_for_server() # Wait till its ready
      cart_goal = CartPose.Goal() # Make goal
      # Add all coordinates 
      cart_goal.x = x
      cart_goal.y = y
      cart_goal.z = z
      cart_goal.w = w
      cart_goal.p = p
      cart_goal.r = r
      future = self.cart_ac.send_goal_async(cart_goal, feedback_callback=self.feedback_callback)
      rclpy.spin_until_future_complete(self, future)
      if not future.result() or not future.result().accepted:
         self.get_logger().info('Goal rejected')
         return
      goal_handle = future.result()
      result_future = goal_handle.get_result_async()
      rclpy.spin_until_future_complete(self, result_future)
      future.add_done_callback(self.goal_response_callback)
   
   def joint_move(self, j1=0.0, j2=0.0, j3=0.0, j4=0.0, j5=-90.0, j6=-45.0):
      # Joints
      print("Running Joint test")
      self.joints_ac.wait_for_server()
      joint_goal = JointPose.Goal()
      # Add all joints
      joint_goal.joint1 = j1
      joint_goal.joint2 = j2
      joint_goal.joint3 = j3
      joint_goal.joint4 = j4
      joint_goal.joint5 = j5
      joint_goal.joint6 = j6
      future = self.joints_ac.send_goal_async(joint_goal, feedback_callback=self.feedback_callback)
      rclpy.spin_until_future_complete(self, future)
      if not future.result() or not future.result().accepted:
         self.get_logger().info('Goal rejected')
         return
      goal_handle = future.result()
      result_future = goal_handle.get_result_async()
      rclpy.spin_until_future_complete(self, result_future)
      future.add_done_callback(self.goal_response_callback)
      
   def open_gripper(self, width=100, force=40):
      print(f"Open gripper — width: {width}, force: {force}")
      self.onrobot_ac.wait_for_server()
      gripper_goal = OnRobotGripper.Goal()
      gripper_goal.width = width
      gripper_goal.force = force
      future = self.onrobot_ac.send_goal_async(gripper_goal, feedback_callback=self.feedback_callback)
      rclpy.spin_until_future_complete(self, future)
      
      if not future.result() or not future.result().accepted:
         self.get_logger().info('Goal rejected')
         return
      goal_handle = future.result()
      result_future = goal_handle.get_result_async()
      rclpy.spin_until_future_complete(self, result_future)
      future.add_done_callback(self.goal_response_callback)

   
   def close_gripper(self, width=78, force=40):
      print(f"Close gripper — width: {width}, force: {force}")
      self.onrobot_ac.wait_for_server()
      gripper_goal = OnRobotGripper.Goal()
      gripper_goal.width = width
      gripper_goal.force = force
      future = self.onrobot_ac.send_goal_async(gripper_goal, feedback_callback=self.feedback_callback)
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

#------- Modbus -------------
class ModbusMonitor:
   def __init__(self, host=HOST, port=PORT):
      self.client = ModbusTcpClient(host, port=port)
      self.client.connect()

   def write_register(self, register, value):
      print(f"Writing register {REGISTER_NAME_BY_VALUE[register]} = {value}")
      self.client.write_register(register, int(value), slave=1)

   def write_coil(self, coil, value):
      print(f"Writing coil {COIL_NAME_BY_VALUE[coil]} = {value}")
      self.client.write_coil(coil, value, slave=1)

   def read_coil(self, coil):
      return self.client.read_coils(coil, 1, slave=1).bits[0]

   def read_register(self, register):
      result = self.client.read_holding_registers(register, 1, slave=1)
      if result.isError():
         return None
      return result.registers[0]

   def wait_for_coil(self, name, state):
      print(f"Waiting for {name} = {state}...")
      while self.read_coil(COIL_REGISTER_MAP[name]) != state:
         sleep(0.2)
      print(f"{name} is {state}")

   def reset_all_modbus_bits(self):
      print("\033[36mResetting all Modbus bits...\033[0m")
      for coil in COIL_REGISTER_MAP.values():
         self.write_coil(coil, False)
      for reg in HOLDING_REGISTER_MAP.values():
         self.write_register(reg, 0)

   def close(self):
      self.client.close()

def pick_up_die_test(fanuc):
   fanuc.joint_move()
   #     above die home
   fanuc.cart_move(x=640.224, y=-0.623, z=172.0)
   #     on die home
   fanuc.cart_move(x=640.224, y=-0.623, z=49.5)
   fanuc.close_gripper()
   sleep(2)
   #     above die home
   fanuc.cart_move(x=640.224, y=-0.623, z=172.0)

def place_back_die_test(fanuc):
   fanuc.joint_move()
   #     above die home
   fanuc.cart_move(x=640.224, y=-0.623, z=172.0)
   #     on die home
   fanuc.cart_move(x=640.224, y=-0.623, z=49.5)
   fanuc.open_gripper()
   sleep(2)
   #     above die home
   fanuc.cart_move(x=640.224, y=-0.623, z=172.0)

def rotate_die(fanuc):   
   # home
   fanuc.joint_move()
   # above table
   fanuc.cart_move(x=503.904, y=-396.527, z=216.544)
   # on table
   fanuc.cart_move(x=503.904, y=-396.527, z=114.016)
   fanuc.open_gripper()
   sleep(2)
   # back above table
   fanuc.cart_move(x=503.904, y=-396.527, z=216.544)
   # rotate
   fanuc.cart_move(x=503.904, y=-396.527, z=216.544, r=45.0)
   # orient
   fanuc.cart_move(x=503.904, y=-130.0, z=216.544, w=90.0, p=45.0, r=0.0)
   # on die
   fanuc.cart_move(x=503.904, y=-130.0, z=-152.599, w=90.0, p=45.0, r=0.0)
   fanuc.close_gripper()
   sleep(2)
   # pick up
   fanuc.cart_move(x=503.904, y=-130.0, z=216.544, w=90.0, p=45.0, r=0.0)
   # straighten out
   fanuc.cart_move(x=503.904, y=-396.527, z=216.544)

def _read_face(fanuc, side=1):
   if side == 1:
      fanuc.joint_move(j1=-70.0)
      fanuc.joint_move(j1=-70.0, j5=-5.0)
      fanuc.joint_move(j1=-76.793, j2=19.282, j3=10.591, j4=-23.778, j5=-16.982, j6=-21.557)
   elif side ==-1:
      pass # don't flip, just read
   else:
      # side 2
      fanuc.joint_move(j1=-72.309, j2=51.509, j3=2.131, j4=1.809, j5=88.756, j6=17.372)
   sleep(2) # wait for image count to update
   return fanuc.get_pip_count()

def find_pip_count(fanuc, target_pips):
   total_pip_count = 0
   if target_pips not in range(1, 7):
      print(f"{target_pips} is not a valid pip count (1-6)")
      return None

   known_pips = set()
   print(f"Looking for side {target_pips}")

   tries = 0

   # Rotate until we physically land on the target
   for attempt in range(6):
      # read first side
      face = _read_face(fanuc, side=1)
      if face is not None:
         known_pips |= {face, 7 - face}
         tries+=1
         print(f"Found {face} pips on attempt {attempt}")
         total_pip_count+=face
         # found it
         if face == target_pips:
            print(f"Found {target_pips} pips after {tries} tries")
            return face, tries, total_pip_count
         # target is on opposite side
         elif ((7 - face) == target_pips):
            print("The one I want is on the bottom. Rotating...")
            tries +=1
            # FLIP OVER
            fanuc.joint_move(j1=-76.793, j2=19.282, j3=10.591, j4=-23.778, j5=-16.982, j6=152.625)
            face =_read_face(fanuc, side = -1)
            print(f"Found side {face}")
            total_pip_count+=face
            print(f"   SUCCESS!!!\nFound {target_pips} pips on the opposite of side 1 in {tries} tries")
            return face, tries, total_pip_count
      # check bottom of die in this pos
      face = _read_face(fanuc, side=2)
      if face is not None:
         tries += 1
         known_pips |= {face}
         print(f"Found {face} pips on bottom side")
         total_pip_count+=face
         if face == target_pips:
            print(f"   SUCCESS!!!\nFound {target_pips} pips on bottom side in {tries} tries")
            return face, tries, total_pip_count
      print(f"Known faces after {tries} views: {sorted(known_pips)} — rotating to find {target_pips}")
      # rotate to try looking again
      rotate_die(fanuc)

   print(f"Could not find {target_pips} pips")
   return None, tries, total_pip_count

if __name__ == '__main__':
   rclpy.init()

   fanuc = FanucActions(namespace)
   modbus = ModbusMonitor()
   cycle_done = False
   
   # go home
   fanuc.joint_move()
   fanuc.open_gripper()
   sleep(2)
   while not cycle_done:
      modbus.wait_for_coil("DJ_Has_Dice", False)
      # move above DJ's conveyor
      #  rotate
      fanuc.joint_move(j1=-70.0)
      #  above DJ conveyor
      fanuc.cart_move(x=-467.673, y=-705.499, z=370.0, w=179.9, p=0.0, r=-135.295)
      
      modbus.wait_for_coil("Ready_For_Pickup", True)
      
      modbus.write_coil(COIL_REGISTER_MAP["Bill_Has_Dice"], True)
      modbus.write_coil(COIL_REGISTER_MAP["Ready_For_Pickup"], False)
      
      # pick up die
      #  down to on DJ conveyor
      fanuc.cart_move(x=-467.673, y=-705.499, z=241.177, w=179.9, p=0.0, r=-135.295)

      fanuc.close_gripper()
      sleep(2)
      #  above DJ conveyor
      fanuc.cart_move(x=-467.673, y=-705.499, z=370.0, w=179.9, p=0.0, r=-135.295)

      #  rotate back
      fanuc.joint_move(j1=-70.0)
      
      pip_num = modbus.read_register(HOLDING_REGISTER_MAP["Last_Known_Pip"])
      print(f"OLD Pip num = {pip_num}")
      
      pip_num+=1
      num_tries = 0
      print(f"NEW Pip num = {pip_num}")
      
      # look for the pip num
      _, num_tries, this_iter_pip_count = find_pip_count(fanuc, target_pips=pip_num)
      
      # write latest pip num
      modbus.write_register(HOLDING_REGISTER_MAP["Last_Known_Pip"], pip_num)
      
      # Write updated number of tries (Bill and Total)
      bill_retries = modbus.read_register(HOLDING_REGISTER_MAP["Bill_Retries"])
      bill_retries+=num_tries 
      modbus.write_register(HOLDING_REGISTER_MAP["Bill_Retries"], bill_retries)   
      total_retries = modbus.read_register(HOLDING_REGISTER_MAP["Total_Retries"])
      total_retries+=num_tries 
      modbus.write_register(HOLDING_REGISTER_MAP["Total_Retries"], total_retries)
      
      # increment total pip count   
      total_pip_count = modbus.read_register(HOLDING_REGISTER_MAP["Total_Pip_Count"])
      total_pip_count+=this_iter_pip_count 
      modbus.write_register(HOLDING_REGISTER_MAP["Total_Pip_Count"], total_pip_count)
      
      if pip_num==6:
         # done with cycle
         cycle_done=True
         # go home
         fanuc.joint_move()
      else:
         # place die on conveyor
            # rotate
         fanuc.joint_move(j1=-70.0)
            # above conveyor
         fanuc.joint_move(j1=-90.677, j2=12.250, j3=-15.150, j4=-1.232, j5=-75.713, j6=-44.920)
            # on conveyor
         fanuc.joint_move(j1=-90.677, j2=17.002, j3=-30.506, j4=-1.373, j5=-60.360, j6=-44.544)
         fanuc.open_gripper()
         sleep(2)
            # above conveyor
         fanuc.joint_move(j1=-90.677, j2=12.250, j3=-15.150, j4=-1.232, j5=-75.713, j6=-44.920)
         modbus.write_coil(COIL_REGISTER_MAP["Bill_Has_Dice"], False)
            
         fanuc.conveyor('reverse')
         fanuc.wait_for_prox('left')
         fanuc.wait_for_prox('left', False) # wait for die to clear prox before stopping   
         fanuc.conveyor('stop')
         modbus.write_coil(COIL_REGISTER_MAP["Ready_For_Pickup"], True)
         # start loop over again

   # place die at start
   #     above die home
   fanuc.cart_move(x=640.224, y=-0.623, z=172.0)
   #     on die home
   fanuc.cart_move(x=640.224, y=-0.623, z=49.5)
   #     let go
   fanuc.open_gripper()
   sleep(2)
   #     above die home
   fanuc.cart_move(x=640.224, y=-0.623, z=172.0)
   # home and end cycle
   fanuc.joint_move()
   modbus.write_coil(COIL_REGISTER_MAP["Cycle_Active"], False)

   # Summarize and print all collected info
   print("\n\n\n\033[36mThe Cycle has been completed Successfully!\n\n\033[0m")
   bill_retries = modbus.read_register(HOLDING_REGISTER_MAP["Bill_Retries"])
   dj_retries = modbus.read_register(HOLDING_REGISTER_MAP["DJ_Retries"])
   total_retries = modbus.read_register(HOLDING_REGISTER_MAP["Total_Retries"])
   total_pip_count = modbus.read_register(HOLDING_REGISTER_MAP["Total_Pip_Count"])
   print(f"Number of Total Tries: {total_retries}")
   print(f"   Bill's Tries: {bill_retries}     DJ's Tries: {dj_retries}")
   print(f"Total Pip Count: {total_pip_count}")
   modbus.reset_all_modbus_bits()
   
   #TESTCODE
   # for i in range(1,7):
   #    pick_up_die_test(fanuc)
   #    print(f"Looking for pip {i}")
   #    _, tries = find_pip_count(fanuc, target_pips=i)
   #    print(f"Took {tries} tries to find the pip.")
   #    place_back_die_test(fanuc)
   
   # PLACE ON CONV TEST
   # pick_up_die_test(fanuc)
   # # place die on conveyor
   #    # rotate
   # fanuc.joint_move(j1=-70.0)
   #    # above conveyor
   # fanuc.joint_move(j1=-90.677, j2=12.250, j3=-15.150, j4=-1.232, j5=-75.713, j6=-44.920)
   #    # on conveyor
   # fanuc.joint_move(j1=-90.677, j2=17.002, j3=-30.506, j4=-1.373, j5=-60.360, j6=-44.544)
   # fanuc.open_gripper()
   # sleep(2)
   #    # above conveyor
   # fanuc.joint_move(j1=-90.677, j2=12.250, j3=-15.150, j4=-1.232, j5=-75.713, j6=-44.920)
      
   # fanuc.conveyor('reverse')
   # fanuc.wait_for_prox('left')
   # fanuc.wait_for_prox('left', False) # wait for die to clear prox before stopping   
   # fanuc.conveyor('stop')
   
   
   # #PICKUP FROM CONV TEST
   # fanuc.joint_move(j1=-70.0)
   # #  above DJ conveyor
   # fanuc.cart_move(x=-467.673, y=-696.499, z=370.0, w=179.9, p=0.0, r=-135.295)
      
   # # pick up die
   # #  down to on DJ conveyor
   # fanuc.cart_move(x=-467.673, y=-696.499, z=241.177, w=179.9, p=0.0, r=-135.295)
   # fanuc.close_gripper()
   # sleep(2)
   # #  above DJ conveyor
   # fanuc.cart_move(x=-467.673, y=-696.499, z=370.0, w=179.9, p=0.0, r=-135.295)
   # #  rotate back
   # fanuc.joint_move(j1=-70.0)
   
   
      
   rclpy.spin(fanuc)
   modbus.close()
   rclpy.shutdown()

   
