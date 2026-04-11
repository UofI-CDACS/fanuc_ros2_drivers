"""
This is a test of all the actions and (if added) their feedbacks
"""
# ROS packages
import rclpy
from rclpy.node import Node
from rclpy.action.client import ActionClient

import sys
sys.path.append("../src/dependencies/")

from time import sleep
from datetime import datetime
import cv2
import numpy as np
import mvsdk

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.action import CartPose, Conveyor, JointPose, OnRobotGripper, SJointPose

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



#------- Camera / vision functions -------------
# DIE_SIZE_UPPER = 4000
DIE_SIZE_LOWER = 5000
PIP_SIZE_UPPER = 1000
PIP_SIZE_LOWER = 200
HSV_lower = [7, 141, 53]
HSV_upper = [18, 255, 135]
def show_img(img, window='image', text=None):
   cv2.namedWindow(window, cv2.WINDOW_NORMAL)
   cv2.resizeWindow(window, 900, 720)
   display = img.copy()
   if text:
      cv2.putText(display, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
   cv2.imshow(window, display)
   cv2.waitKey(0)
   cv2.destroyAllWindows()

def take_photo(save_path=None):
   print("Taking photo...")
   DevList = mvsdk.CameraEnumerateDevice()
   if len(DevList) < 1:
      print("No camera found!")
      return None

   hCamera = 0
   last_err = None
   for dev in DevList:
      try:
         hCamera = mvsdk.CameraInit(dev, -1, -1)
         last_err = None
         break
      except mvsdk.CameraException as e:
         last_err = e
         continue
   if last_err is not None:
      print(f"CameraInit failed({last_err.error_code}): {last_err.message}")
      return None

   cap = mvsdk.CameraGetCapability(hCamera)
   monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
   mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8 if monoCamera else mvsdk.CAMERA_MEDIA_TYPE_BGR8)
   mvsdk.CameraSetTriggerMode(hCamera, 0)
   mvsdk.CameraSetAeState(hCamera, 0)
   mvsdk.CameraSetExposureTime(hCamera, 50 * 1000)
   mvsdk.CameraPlay(hCamera)

   FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
   pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

   img = None
   try:
      pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
      mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
      mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
      frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
      frame = np.frombuffer(frame_data, dtype=np.uint8)
      frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth,
                             1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
      img = frame.copy()
   except mvsdk.CameraException as e:
      print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")
   finally:
      mvsdk.CameraUnInit(hCamera)
      mvsdk.CameraAlignFree(pFrameBuffer)

   if img is not None:
      if save_path is None:
         return img
         save_path = datetime.now().strftime("photo_%Y%m%d_%H%M%S.png")
      cv2.imwrite(save_path, img)
      print(f"Photo saved to {save_path}")

   return img

def count_dice(img):
   """Detect yellow dice in img. Returns (num_dice, bounding_boxes).
   bounding_boxes is a list of (x, y, w, h) for each die found."""
   hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv, np.array(HSV_lower), np.array(HSV_upper))
   contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
   dice = []
   for contour in contours:
      if cv2.contourArea(contour) > DIE_SIZE_LOWER:
         x, y, w, h = cv2.boundingRect(contour)
         dice.append((x, y, w, h))
   print(f"Found {len(dice)} die/dice")
   return len(dice), dice

def count_pips_per_die(img):
   """Count pips on each die in img. Returns a list of pip counts, one per die."""
   hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv, np.array(HSV_lower), np.array(HSV_upper))
   contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

   pip_counts = []
   die_num = 0
   for contour in contours:
      if cv2.contourArea(contour) > DIE_SIZE_LOWER:
         die_num += 1
         x, y, w, h = cv2.boundingRect(contour)
         die_face = mask[y:y + h, x:x + w].copy()
         pip_contours, _ = cv2.findContours(die_face, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
         num_pips = 0
         for pip in pip_contours:
            area = cv2.contourArea(pip)
            if PIP_SIZE_LOWER < area < PIP_SIZE_UPPER:
               num_pips += 1
         print(f"Die {die_num} has {num_pips} pips")
         pip_counts.append(num_pips)
   return pip_counts

def draw_detections(img, dice_boxes, pip_counts):
   """Draw bounding boxes on a copy of img using pre-computed detection results.
   dice_boxes: list of (x,y,w,h) from count_dice()
   pip_counts: list of pip counts from count_pips_per_die()
   Returns annotated image."""
   out = img.copy()
   hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
   mask = cv2.inRange(hsv, np.array(HSV_lower), np.array(HSV_upper))

   for i, (x, y, w, h) in enumerate(dice_boxes):
      # green box + label around each die
      num_pips = pip_counts[i] if i < len(pip_counts) else '?'
      cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 2)
      cv2.putText(out, f"die{i+1}: {num_pips} pips", (x, y - 8),
                  cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

      # red boxes around each pip inside this die's crop
      die_face = mask[y:y + h, x:x + w].copy()
      pip_contours, _ = cv2.findContours(die_face, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
      for pip in pip_contours:
         area = cv2.contourArea(pip)
         if PIP_SIZE_LOWER < area < PIP_SIZE_UPPER:
            px, py, pw, ph = cv2.boundingRect(pip)
            cv2.rectangle(out, (x + px, y + py), (x + px + pw, y + py + ph), (0, 0, 255), 2)
            cv2.putText(out, f"{area:.0f}", (x + px, y + py - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (0, 0, 255), 2)

   return out

def tune_hsv(img):
   """Interactive HSV tuning tool. Drag sliders to adjust bounds and see the
   mask update live. Press any key to exit and print the final values."""
   window = 'HSV Tuner'
   cv2.namedWindow(window, cv2.WINDOW_NORMAL)
   cv2.resizeWindow(window, 1400, 500)

   cv2.createTrackbar('H low',  window, 15,  179, lambda _: None)
   cv2.createTrackbar('H high', window, 25,  179, lambda _: None)
   cv2.createTrackbar('S low',  window, 40,  255, lambda _: None)
   cv2.createTrackbar('S high', window, 255, 255, lambda _: None)
   cv2.createTrackbar('V low',  window, 40,  255, lambda _: None)
   cv2.createTrackbar('V high', window, 255, 255, lambda _: None)

   hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

   while True:
      h_low  = cv2.getTrackbarPos('H low',  window)
      h_high = cv2.getTrackbarPos('H high', window)
      s_low  = cv2.getTrackbarPos('S low',  window)
      s_high = cv2.getTrackbarPos('S high', window)
      v_low  = cv2.getTrackbarPos('V low',  window)
      v_high = cv2.getTrackbarPos('V high', window)

      mask   = cv2.inRange(hsv, np.array([h_low, s_low, v_low]), np.array([h_high, s_high, v_high]))
      masked = cv2.bitwise_and(img, img, mask=mask)
      display = np.hstack([img, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), masked])
      cv2.imshow(window, display)

      if cv2.waitKey(30) != -1:
         break

   cv2.destroyWindow(window)
   print(f"HSV lower: [{h_low}, {s_low}, {v_low}]")
   print(f"HSV upper: [{h_high}, {s_high}, {v_high}]")


if __name__ == '__main__':
   loop_count = 3
   overall_pip_counts = []
   rclpy.init()

   fanuc = FanucActions(namespace)
   
   # go home
   fanuc.joint_move()
   
   for i in range(loop_count):
      fanuc.open_gripper()
      sleep(2)
      # pick up die
      #     above die home
      fanuc.cart_move(x=640.224, y=-0.623, z=172.0)
      #     on die home
      fanuc.cart_move(x=640.224, y=-0.623, z=49.5)
      #     grab
      fanuc.close_gripper()
      sleep(2)
      #     above die home
      fanuc.cart_move(x=640.224, y=-0.623, z=172.0)
      
      # home
      fanuc.joint_move()
      # Present die to camera
      fanuc.joint_move(j1=-70.0)
      fanuc.joint_move(j1=-70.0, j5=-5.0)
      fanuc.joint_move(j1=-76.793, j2=19.282, j3=10.591, j4=-23.778, j5=-16.982, j6=-21.557)
      
      # count dice and pips
      img = take_photo()
      # tune_hsv(img)
      num_dice, dice_boxes = count_dice(img)                                                                                                      
      pip_counts = count_pips_per_die(img)
      print(f"Found {num_dice} dice,   with {pip_counts} pips")

      current_die = [f"Die number {i}"]
      current_die.append(pip_counts[0])
      overall_pip_counts.append(current_die)
      annotated = draw_detections(img, dice_boxes, pip_counts)                                                                                    
      show_img(annotated)
      
      # home
      fanuc.joint_move()
      
      # go to place back
      #     above die home
      fanuc.cart_move(x=640.224, y=-0.623, z=172.0)
      #     above die home (rotate)
      fanuc.cart_move(x=640.224, y=-0.623, z=172.0, w=179.9, p=0.0, r=45.0)
      #     on die home
      fanuc.cart_move(x=640.224, y=-0.623, z=49.5, w=179.9, p=0.0, r=45.0)
      #     open gripper
      fanuc.open_gripper()
      sleep(2)
      #     back up above
      fanuc.cart_move(x=640.224, y=-0.623, z=172.0, w=179.9, p=0.0, r=45.0)
      #     above die home (un-rotate) will be at start of next
      
   # home
   fanuc.joint_move()
   
   # print all pip counts
   print()
   print()
   print(f"Pip counts: {overall_pip_counts}\n\nGoodbye!\n\n\n\n\n")
   
   rclpy.spin(fanuc)
   rclpy.shutdown()
