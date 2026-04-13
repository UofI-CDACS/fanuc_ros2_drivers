##
# Robotics with Claude
# Tyler LaFleur

# Tasks:
    # Pick up the Dice
    # Present it to the camera and capture an image (overhead if FANUC, attached camera if SBOT)
    # Run process to count the pips in the dice
    #     Process will be completed a total of 3 times, keeping track of pip count (overall and individual). IE total pip count at the end, as well as individual count per dice number.
    #     You choose how to present this information.
    # At a minimum, you will need to make:
    #     Camera node
    #     Master/Control node for robot operations

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

# Machine Vision
import mvsdk
import cv2 as cv
import numpy as np

namespace = 'Bill'
positions = [
    [0.0, 0.0, 0.0, 0.0, -90.0, -50.0], # home pos (joint)
    [640.0, 0.0, 100.0, -178.0, 0.0, -50.0], # at dice
    [640.0, 0.0, 40.0, -178.0, 0.0, -50.0], # on dice
    [-75.0, 0.0, 0.0, 0.0, -45.0, -50.0], # in between step from home to camera
    [-75.0, 10.0, -5.0, 0.0, -5.0, -50.0], # at camera
    [-75.0, 10.0, -5.0, 0.0, -5.0, 40.0], # in between step for rotate dice
    [-75.0, 10.0, -5.0, 0.0, -5.0, 130.0], # at camera
    [-75.0, 60.0, 0.0, 5.0, 85.0, -50.0], # at camera
    # [13.55, 21.5, -47.8, 0.0, -42.2, -63.5], # above dice (after)
    # [0.0, 0.0, 0.0, 0.0, -90.0, -50.0], # on dice (after)
            ]

class FanucActions(Node):
    current_state = 1 ## CHANGE ME BACK TO 1
    count = 0
    Pip_total = 0
    Pip_current = 0

    def __init__(self, namespace):
        super().__init__("robot")
		
		# Actions
        self.cart_ac = ActionClient(self, CartPose, f'/{namespace}/cartesian_pose')
        self.convey_ac = ActionClient(self, Conveyor, f'/{namespace}/conveyor')
        self.joints_ac = ActionClient(self, JointPose, f'/{namespace}/joint_pose')
        self.schunk_ac = ActionClient(self, SchunkGripper, f'/{namespace}/schunk_gripper')
        self.sin_joint_ac = ActionClient(self, SJointPose, f'/{namespace}/single_joint_pose')
        self.onRobotGripper_ac = ActionClient(self, OnRobotGripper, f'/{namespace}/onrobot_gripper')
		
    def state_test(self):
        

        # initialize camera
        DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
            print("No camera was found!")
            return
        for i, DevInfo in enumerate(DevList):
            print("{}: {} {}".format(i, DevInfo.GetFriendlyName(),
            DevInfo.GetPortType()))
        i = 0 if nDev == 1 else int(input("Select camera: "))
        DevInfo = DevList[i]
        print(DevInfo)
        hCamera = 0
        try:
            hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
            exit()
        cap = mvsdk.CameraGetCapability(hCamera)
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
        if monoCamera:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
        mvsdk.CameraSetTriggerMode(hCamera, 0)
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 50 * 1000)
        mvsdk.CameraPlay(hCamera)
        FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
        pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

        while(1):
            
            match(self.current_state):
                case 1: # HOME
                    # Reset to home position and standby for start signal
                    print("At home. Resetting for next rep.")

                    self.joint_move(positions[0]) # home
                    self.close_gripper()
                    self.open_gripper()
                    # self.joint_move(positions[6])

                    self.current_state = 2

                case 2: # Pick up dice
                    print("Picking up dice")
                    self.cart_move(positions[1])
                    self.cart_move(positions[2])
                    self.close_gripper()
                    self.cart_move(positions[1])
                    self.joint_move(positions[0])
                    self.joint_move(positions[3])
                    self.current_state = 3

                case 3: # Show to camera
                    print("Bringing dice to camera")
                    if self.count == 0:
                        self.joint_move(positions[4])
                    elif self.count == 1:
                        self.joint_move(positions[5])
                        self.joint_move(positions[6])
                    elif self.count == 2:
                        self.joint_move(positions[5])
                        self.joint_move(positions[4])
                        self.joint_move(positions[7])
                        sleep(10)
                    self.current_state = 4

                case 4: # Take picture and analyze pips
                    print("Capturing image and analyzing")
                    
                    # dice analyzing
                    try:
                        pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
                        mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
                        mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
                        frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
                        frame = np.frombuffer(frame_data, dtype=np.uint8)
                        frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )
                        frame = cv.resize(frame, (640,480), interpolation = cv.INTER_LINEAR)
                        # DO STUFF
                        self.Pip_current, image = self.countPips(frame)

                        # Debugging
                        cv.imshow('Display Window', image)
                        cv.waitKey(0)
                        cv.destroyAllWindows()
                    except mvsdk.CameraException as e:
                        if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                            print("CameraGetImageBuffer failed({}):{}".format(e.error_code, e.message) )
                    self.count = self.count + 1

                    self.Pip_total = self.Pip_total + self.Pip_current
                    print(f"Iteration {self.count}:")
                    print(f"This dice has {self.Pip_current} pip(s).")
                    print(f"Pip total is at {self.Pip_total} pip(s) total!")
                    self.Pip_current = 0

                    if self.count < 3:
                        self.current_state = 3
                    elif self.count >= 3:
                        self.current_state = 5

                case 5: # Return dice
                    print("Returning dice")
                    self.joint_move(positions[4])
                    sleep(10)
                    self.joint_move(positions[3])
                    self.joint_move(positions[0])
                    self.cart_move(positions[1])
                    self.cart_move(positions[2])
                    self.open_gripper()
                    self.cart_move(positions[1])
                    self.joint_move(positions[0])
                    self.current_state = 6

                case 6: # Final admin
                    print(f"Cycle complete!")
                    print(f"Pip total is {self.Pip_total} pips!")
                    self.Pip_total = 0
                    self.count = self.count + 1
                    self.current_state = 1

                case 0: # FAULT
                    print("In FAULT state")
                    self.current_state = 1
                
                case _:
                    self.current_state = 0

            if self.count >= 4:
                exit()



    def open_gripper(self):
        print("Opening gripper")
        self.onRobotGripper_ac.wait_for_server()
        gripper_goal = OnRobotGripper.Goal()
        gripper_goal.width = 100
        gripper_goal.force = 100
        future = self.onRobotGripper_ac.send_goal_async(gripper_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        sleep(2)

    def close_gripper(self):
        print("Closing gripper")
        self.onRobotGripper_ac.wait_for_server()
        gripper_goal = OnRobotGripper.Goal()
        gripper_goal.width = 80
        gripper_goal.force = 50
        future = self.onRobotGripper_ac.send_goal_async(gripper_goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        sleep(2)

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

    def countPips(self, image):
        pipcount = 0

        # Crop to bottom-left quadrant to reduce noise
        image = image[240:480, 0:320]

        # Isolate yellow die face
        hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
        yellow_mask = cv.inRange(hsv, (15, 80, 80), (35, 255, 255))
        yellow_mask = cv.dilate(yellow_mask, None, iterations=2)
        yellow_mask = cv.erode(yellow_mask, None, iterations=2)

        # Find die contour from yellow mask
        contours, _ = cv.findContours(yellow_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        if not contours:
            return 0, image

        die_contour = max(contours, key=cv.contourArea)
        x, y, w, h = cv.boundingRect(die_contour)
        crop = image[y:y+h, x:x+w]

        # --- Approach 1: Contour + circularity filter ---
        # gray = cv.cvtColor(crop, cv.COLOR_BGR2GRAY)
        # _, pip_mask = cv.threshold(gray, 50, 255, cv.THRESH_BINARY_INV)
        # pip_mask = cv.erode(pip_mask, None, iterations=1)
        # pip_mask = cv.dilate(pip_mask, None, iterations=1)
        # pip_contours, _ = cv.findContours(pip_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        # for pip in pip_contours:
        #     area = cv.contourArea(pip)
        #     if not (25 < area < 500):
        #         continue
        #     perimeter = cv.arcLength(pip, True)
        #     if perimeter == 0:
        #         continue
        #     circularity = 4 * np.pi * area / (perimeter ** 2)
        #     if circularity > 0.5:
        #         px, py, pw, ph = cv.boundingRect(pip)
        #         cv.rectangle(crop, (px, py), (px + pw, py + ph), (0, 255, 0), 1)
        #         pipcount += 1

        # --- Approach 2: Hough circle detection ---
        # Tuning knobs: param2 (accumulator threshold — lower += more circles, higher = fewer)                               
        #               minRadius/maxRadius — adjust to match +pip size in frame   
        gray = cv.cvtColor(crop, cv.COLOR_BGR2GRAY)
        gray = cv.GaussianBlur(gray, (5, 5), 0)
        circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, dp=1, minDist=10,
                                  param1=50, param2=15, minRadius=3, maxRadius=20)
        if circles is not None:
            for (cx, cy, r) in np.round(circles[0]).astype(int):
                cv.rectangle(crop, (cx - r, cy - r), (cx + r, cy + r), (0, 255, 0), 1)
                pipcount += 1

        return pipcount, image

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

if __name__ == '__main__':
    rclpy.init()
    fanuc = FanucActions(namespace)

    fanuc.state_test()
    rclpy.spin(fanuc)
        
    rclpy.shutdown()