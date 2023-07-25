"""! @brief Defines the robot controller class."""

##
# @file robot_controller.py
#
# @brief Defines the roboot controller class.
#
# @section description_robot_controller Description
# Defines the base and end user class for controlling robot using FANUC-EthernetIP library
#
# @section todo_robot_controller todo_robot_controller
# - Add more useful functions
# -
# 
# @section author_robot_controller Authors(s)
# - Original Code by John Shovic
# - Modified by James Lasso on 6/12/2023

# Imports
import sys
sys.path.append('./pycomm3/pycomm3')
import struct
import random
import time
import math
import FANUCethernetipDriver

FANUCethernetipDriver.DEBUG = False

class robot:
    def __init__(self, robotIP):
        self.robot_IP = robotIP
        self.CurJointPosList = FANUCethernetipDriver.returnJointCurrentPosition(self.robot_IP)
        self.CurCartesianPosList = FANUCethernetipDriver.returnCartesianCurrentPostion(self.robot_IP)
        self.PRNumber = 1 # This is the position register for holding coordinates
        self.start_register = 1
        self.sync_register = 2
        self.sync_value = 1
        self.speed_register = 5

    ##
    #
    # Joint movement functions
    #
    ##

    # read CURPOS from Robot
    def read_current_joint_position(self):
        self.CurJointPosList = FANUCethernetipDriver.returnJointCurrentPosition(self.robot_IP)

    # read PR[1] Joint Coordinates
    def read_joint_position_register(self):
        PR_1_Value = FANUCethernetipDriver.readJointPositionRegister(self.robot_IP, self.PRNumber)
        print("PR[%d]"% self.PRNumber)
        print("list=", PR_1_Value)

    # write PR[1] offset
    def write_joint_offset(self, joint, value):
        print("***********************************************")
        print(f" Write Joint Offset Value:[{value}] to Joint:[{joint}] ")
        print("***********************************************")
        joint += 1

        newPosition = self.CurJointPosList[joint] + value
        print(f"New Position value: {newPosition}\n")

        self.CurJointPosList[joint] = newPosition
        print(self.CurJointPosList[joint])

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    # write PR[1] Joint value
    def write_joint_position(self, joint, value):
        print("--------------------------------")
        print("| write PR[1] Joint Coordinate |")
        print("--------------------------------")
        joint = joint + 1

        newPosition = value

        self.CurJointPosList[joint] = newPosition

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    # Set pose of robot by passing an array of joint positions
    # ie pose = [1, 2, 3, 4, 5, 6]
    # set_pose(pose)
    def set_pose(self, joint_position_array):
        joint_number = 1
        for joint in joint_position_array:
            self.CurJointPosList[joint_number + 1] = joint_position_array[joint_number - 1]
            joint_number += 1

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)
        

    # Put robot in home position
    def set_joints_to_home_position(self):
        print("*************************************************")
        print("* Setting Joint Positions to Home Configuration *")
        print("*************************************************")

        # Set positions DO NOT USE 0
        # joint coordinates start at list item 2, ie: Joint2 = CurPosList[3]
        self.CurJointPosList[2] = 1.0 # J1
        self.CurJointPosList[3] = 1.0 # J2
        self.CurJointPosList[4] = 1.0 # J3
        self.CurJointPosList[5] = 1.0 # J4
        self.CurJointPosList[6] = 1.0 # J5 PB50IB does not like this join, OK on CRX10
        self.CurJointPosList[7] = 1.0 # J6

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    ##
    #
    # Cartesian Movement Functions
    #
    ##

    # read current cartesian position from Robot
    def get_coords(self):
        print("--------------------------")
        print("| read CURPOS from Robot |")
        print("--------------------------")
        CurPosList = FANUCethernetipDriver.returnCartesianCurrentPostion(self.robot_IP)

        print("CURPOS=", CurPosList)

    # read PR[1] Cartesian Coordinates
    def read_cartesian_position_register(self):
        print("-----------------------------------")
        print("| read PR[1] Cartesian Coordinate |")
        print("-----------------------------------")

        PR_1_Value = FANUCethernetipDriver.readCartesianPositionRegister(self.robot_IP, self.PRNumber)
        print("PR[%d]"% self.PRNumber)
        print("list=", PR_1_Value)

    # write PR[1] Cartesian Coordinates
    def send_coords(self, newX, newY, newZ):
        self.CurCartesianPosList[2] = newX
        self.CurCartesianPosList[3] = newY
        self.CurCartesianPosList[4] = newZ
        
        print("------------------------")
        print(" write PR[1] Cartesian Coordinate")
        print("------------------------")

        newPositionList = self.CurCartesianPosList

        FANUCethernetipDriver.writeCartesianPositionRegister(self.robot_IP, self.PRNumber, newPositionList)

    ##
    #
    # Radian Movement
    #
    ##

    # Get the radians of all joints
    # Returns: list: A float list of radians
    def get_radians(self):
        self.read_current_joint_position()
        indices = range(2,8)
        radian_list = [self.CurJointPosList[i] for i in indices]

        for i in range(len(radian_list)):
            radian_list[i] = math.radians((radian_list[i]))

        return radian_list


    # Send the radians of all joints to robot
    def send_radians(self, radians):
        pass

    ##
    #
    # Utility Functions
    #
    ##

    # write R[5] to set Speed in mm/sec
    def set_speed(self, value):
        # print("------------------------------")
        # print(f"| Speed set to {value}mm/sec |")
        # print("------------------------------")
        FANUCethernetipDriver.writeR_Register(self.robot_IP, self.speed_register, value)

    # get current speed
    def get_speed(self):
        return FANUCethernetipDriver.readR_Register(self.robot_IP, self.speed_register)
        

    # Starts robot movement and checks to see when it has completed
    def start_robot(self):  
        # Write to start register to begin movement
        FANUCethernetipDriver.writeR_Register(self.robot_IP, self.start_register, 1)

        # Wait till robot is done moving
        moving = self.read_robot_start_register()
        while(moving):
            moving = self.read_robot_start_register()

        # Update position List
        self.read_current_joint_position()

        # Signal end of move action
        print("********************************************")
        print("* Moving Joint(s) to Position(s): COMPLETE *")
        print("********************************************")

    # Detect if the robot is moving
    def is_moving(self):
        start_register = read_robot_start_register()
        if start_register == 1:
            return 1

        elif start_register == 0:
            return 0

        else:
            return -1


    # Put CRX10 in a mount position to change end tooling
    # !!! -- THIS JOIN CONFIGURATION IS FOR THE CRX10 ROBOT -- !!!
    def set_joints_to_mount_position(self):
        print("**************************************************")
        print("* Setting Joint Positions to Mount Configuration *")
        print("**************************************************")

        # Set positions DO NOT USE 0
        # joint coordinates start at list item 2, ie: Joint2 = CurPosList[3]
        self.CurJointPosList[2] = 1.0 # J1
        self.CurJointPosList[3] = 58.0 # J2
        self.CurJointPosList[4] = -12.0 # J3
        self.CurJointPosList[5] = -2.0 # J4
        self.CurJointPosList[6] = 11.0 # J5 PB50IB does not like this join, OK on CRX10
        self.CurJointPosList[7] = -6.0 # J6

        myList = self.CurJointPosList

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, myList)

    # This function reads register 1(sync bit for position register)
    def read_robot_start_register(self):
        start_register = FANUCethernetipDriver.readR_Register(self.robot_IP, self.start_register)
        return start_register

    # Toggle gripper open and close
    def gripper(self, command):
        # !! Registers 20 and 23 need to be toggled for opening and closing !!

        if command == 'open':
            print("Opening Gripper...\n")
            # set bits to toggle 20 off and 23 on
            FANUCethernetipDriver.writeR_Register(self.robot_IP, 20, 0)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, 23, 1)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, self.sync_register, 1)

        elif command == 'close':
            print("Closing Gripper...\n")
            FANUCethernetipDriver.writeR_Register(self.robot_IP, 20, 1)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, 23, 0)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, self.sync_register, 1)

        else:
            print("Invalid command.")

    # Read conveyor belt sensor: returns value of 1 or 0
    def conveyor_proximity_sensor(self, sensor):
        right_sensor_register = 31
        left_sensor_register = 30

        if sensor == "right":
            return FANUCethernetipDriver.readR_Register(self.robot_IP, right_sensor_register)
        elif sensor == "left":
            return FANUCethernetipDriver.readR_Register(self.robot_IP, left_sensor_register)
        else:
            print("Invalid Sensor, Try 'right' or 'left'\n")

    # Control conveyor belt
    def conveyor(self, command):

        #R21 is forward
        #R22 is reverse
        forward_register = 21
        reverse_register = 22
        on = 1
        off = 0
        sync_register = 2
        sync_value = 1

        if command == 'forward':
            # Make sure belt is not moving
            FANUCethernetipDriver.writeR_Register(self.robot_IP, reverse_register, off)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, forward_register, off)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, sync_register, sync_value)

            FANUCethernetipDriver.writeR_Register(self.robot_IP, forward_register, on)
            ## Set sync bit to update
            FANUCethernetipDriver.writeR_Register(self.robot_IP, sync_register, sync_value)
        elif command == 'reverse':
            FANUCethernetipDriver.writeR_Register(self.robot_IP, reverse_register, off)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, forward_register, off)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, sync_register, sync_value)

            FANUCethernetipDriver.writeR_Register(self.robot_IP, reverse_register, on)
            ## Set sync bit to update
            FANUCethernetipDriver.writeR_Register(self.robot_IP, sync_register, sync_value)
        elif command == 'stop':
            FANUCethernetipDriver.writeR_Register(self.robot_IP, reverse_register, off)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, forward_register, off)
            ## Set sync bit to update
            FANUCethernetipDriver.writeR_Register(self.robot_IP, sync_register, sync_value)
        else:
            FANUCethernetipDriver.writeR_Register(self.robot_IP, reverse_register, off)
            FANUCethernetipDriver.writeR_Register(self.robot_IP, forward_register, off)
            ## Set sync bit to update
            FANUCethernetipDriver.writeR_Register(self.robot_IP, sync_register, sync_value)









