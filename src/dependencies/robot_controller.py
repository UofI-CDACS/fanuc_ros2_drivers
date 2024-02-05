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
# - Modified by Kris Olds on 2/1/2024
#

# Imports
import math
import typing
import FANUCethernetipDriver

## The mode of operation; 

## Robot Class
# @param self, robotIP
class robot:
    def __init__(self, robotIP: str, DEBUG: bool=False):
        """! Initializes the robot.
        @param robotIP      IP address of robot
        """
        self.robot_IP = robotIP
        self.CurJointPosList = FANUCethernetipDriver.returnJointCurrentPosition(self.robot_IP)
        self.CurCartesianPosList = FANUCethernetipDriver.returnCartesianCurrentPostion(self.robot_IP)
        self.PRNumber = 1 # This is the position register for holding coordinates
        self.start_register = 1
        self.sync_register = 2
        self.sync_value = 1
        self.speed_register = 5

        self.DEBUG = DEBUG
        FANUCethernetipDriver.DEBUG = DEBUG


    # set debug on or off
    def set_debug(state:bool):
        FANUCethernetipDriver.DEBUG = state

    # Joint movement functions

    def read_current_joint_position(self) -> list:
        """! Returns list of angles at each joint. [0] -> joint 1
        """
        self.CurJointPosList = FANUCethernetipDriver.returnJointCurrentPosition(self.robot_IP)
        return self.CurJointPosList[2:8]

    # read PR[1] Joint Coordinates
    def read_joint_position_register(self) -> float:
        """! Reads joint position register(PR1) and prints the value and prints list.
        """
        PR_1_Value = FANUCethernetipDriver.readJointPositionRegister(self.robot_IP, self.PRNumber)
        #print("PR[%d]"% self.PRNumber)
        #print("list=", PR_1_Value)
        return PR_1_Value

    # write PR[1] offset
    def write_joint_offset(self, joint:int, value:float, blocking:bool=True):
        """! Offsets current joint position by value given in degrees.
        @param joint        which joint to move
        @param value        degrees you want to move, negative or positive direction
        """
        print("***********************************************")
        print(f" Write Joint Offset Value:[{value}] to Joint:[{joint}] ")
        print("***********************************************")
        joint += 1

        self.CurJointPosList[joint] = self.CurJointPosList[joint] + value
        #print(self.CurJointPosList[joint])

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, self.CurJointPosList)
        self.start_robot(blocking=blocking)

    # write PR[1] Joint value
    def write_joint_position(self, joint:int, value:float, blocking:bool=True):
        """! Sets a specified joint to specific angle based on value
        @param joint        which joint to move
        @param value        angle to set joint to from -180 to 180
        """
        print("--------------------------------")
        print("|   write Joint Coordinate     |")
        print("--------------------------------")
        if value > 179.9 or value < -179.9:
            raise Warning(f"Angle should be in the range of [-179.9, 179.9], got {value}")
        
        if joint > 6 or joint < 1:
            raise Warning(f"Joint should be in the range of [1,6], got {joint}")
        
        joint = joint + 1
        self.CurJointPosList[joint] = value

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, self.CurJointPosList)
        self.start_robot(blocking=blocking)

    # Set pose of robot by passing an array of joint positions
    def write_joint_pose(self, joint_position_array:list[float], blocking:bool=True):
        """! Set a pose(all joint positions) for robot
        @param joint_position_array         a list of joint angles 
        """
        joint_number = 1
        for joint in joint_position_array:
            self.CurJointPosList[joint_number + 1] = joint_position_array[joint_number - 1]
            joint_number += 1

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, self.CurJointPosList)
        self.start_robot(blocking=blocking)
        

    # Put robot in home position
    def set_joints_to_home_position(self, blocking:bool=True):
        """! This used to set a CRX10 into a 'home' position. Not useful for other machines probably.
        Will be removed in the future!
        """
        print("Deperication Warning: This function causes the robot to get stuck in a 'singilarity', will be removed in future update; currently will do nothing. ")
        return 
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

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, self.CurJointPosList)
        self.start_robot(blocking=blocking)

    # Cartesian Movement Functions

    # read current cartesian position from Robot
    def read_current_cartesian_pose(self) -> list[float]:
        """! Print current cartesian coordinates from robot. Returns [X, Y, Z, W, P, R]
        """
        # print("--------------------------")
        # print("| read CURPOS from Robot |")
        # print("--------------------------")
        CurPosList = FANUCethernetipDriver.returnCartesianCurrentPostion(self.robot_IP)

        #print("CURPOS=", CurPosList)
        return CurPosList[2:8]

    # write PR[1] Cartesian Coordinates
    # Takes x, y, z, w, p, r coords.
    # WPR are the orientation of the end effector, DEFAULT to current orientation
    # SOON TO BE CHANGED!! Will take a list of coordinates instead of individual arguements. Keeping for now for compatability
    def write_cartesian_position(self, X: float, Y:float, Z:float, W:float | None, P:float | None, R:float | None, blocking:bool=True):
        """! Send cartesian coordinates to robot using X, Y, Z, W, P, R system. 
        These coordinates usually correlate to the tool end effectors position.
        @param X            X cartesian coordinate
        @param Y            Y cartesian coordinate
        @param Z            Z cartesian coordinate
        @param W            Yaw
        @param P            Pitch
        @param R            Roll
        """
        print("Deperication Warning: Passing arguments in this style will soon be depricated! In the future, pass arguments as a list.")
        if W > 179.9 or W < -179.9:
            raise Warning(f"W, P and R should be in the range of [-179.9, 179.9], got {W}")
        if P > 179.9 or P < -179.9:
            raise Warning(f"W, P and R should be in the range of [-179.9, 179.9], got {P}")
        if R > 179.9 or R < -179.9:
            raise Warning(f"W, P and R should be in the range of [-179.9, 179.9], got {R}")
        
        self.CurCartesianPosList[2] = X
        self.CurCartesianPosList[3] = Y
        self.CurCartesianPosList[4] = Z
        self.CurCartesianPosList[5] = W if W is not None else self.CurCartesianPosList[5]
        self.CurCartesianPosList[6] = P if P is not None else self.CurCartesianPosList[6]
        self.CurCartesianPosList[7] = R if R is not None else self.CurCartesianPosList[7]

        FANUCethernetipDriver.writeCartesianPositionRegister(self.robot_IP, self.PRNumber,  self.CurCartesianPosList)
        self.start_robot(blocking=blocking)

    # write PR[1] Cartesian Coordinates
    # Takes x, y, z, w, p, r coords.
    # WPR are the orientation of the end effector, DEFAULT to current orientation
    def write_cartesian_position(self, coords:list[float], blocking:bool=True):
        """! Send cartesian coordinates to robot using X, Y, Z, W, P, R system. 
        These coordinates usually correlate to the tool end effectors position.
        @param coords[X, Y, Z, W, P, R]  OR  coords[[X,Y,Z,W,P,R], [X,Y,Z,...], ...]
            - X cartesian coordinate
            - Y cartesian coordinate
            - Z cartesian coordinate
            - Yaw (optional)
            - Pitch (optional)
            - Roll (optional)
        """
        if isinstance(coords[0], list):
            # If the first element is a list (making coords a list of lists of coordinates to all be run one after another)
            # Check that all elements in the list are also lists
            if all(isinstance(x, list) for x in coords):
                for coord in coords:
                    self.write_cartesian_position(coord, blocking=blocking) # Loop through all coords in list and run them.
            else:
                raise Warning("If passing a list of lists, all elements must be lists!")
            
        # If here, coords is NOT a list of lists (single list)
        elif len(coords) == 6:
            # This means we got all 6 coordinates
            # First check that the W,P, R are vaild moves
            if coords[3] > 179.9 or coords[3] < -179.9:
                raise Warning(f"W, P and R should be in the range of [-179.9, 179.9], got {coords[3]}")
            if coords[4] > 179.9 or coords[4] < -179.9:
                raise Warning(f"W, P and R should be in the range of [-179.9, 179.9], got {coords[4]}")
            if coords[5] > 179.9 or coords[5] < -179.9:
                raise Warning(f"W, P and R should be in the range of [-179.9, 179.9], got {coords[5]}")
            
            self.CurCartesianPosList[2] = coords[0]
            self.CurCartesianPosList[3] = coords[1]
            self.CurCartesianPosList[4] = coords[2]
            self.CurCartesianPosList[5] = coords[3]
            self.CurCartesianPosList[6] = coords[4]
            self.CurCartesianPosList[7] = coords[5]
            FANUCethernetipDriver.writeCartesianPositionRegister(self.robot_IP, self.PRNumber,  self.CurCartesianPosList)
            self.start_robot(blocking=blocking)

        elif len(coords) == 3:
            # This means we got X,Y,Z
            self.CurCartesianPosList[2] = coords[0]
            self.CurCartesianPosList[3] = coords[1]
            self.CurCartesianPosList[4] = coords[2]

            FANUCethernetipDriver.writeCartesianPositionRegister(self.robot_IP, self.PRNumber,  self.CurCartesianPosList)
            self.start_robot(blocking=blocking)         
        else:
            # Error
            raise Warning("Not enough values passed!")
        

    # Utility Functions
    # write R[5] to set Speed in mm/sec
    def set_speed(self, value: int):
        """! Set movement speed of robot in mm/s
        @param value        speed in mm/s
        """
        print("------------------------------")
        print(f"| Speed set to {value}mm/sec |")
        print("------------------------------")
        if value > 300 or value < 0:
            raise Warning(f"Speed should be in the range of [0, 300], got {value}")
        
        FANUCethernetipDriver.writeR_Register(self.robot_IP, self.speed_register, value)

    # get current speed
    def get_speed(self) -> int:
        """! Returns current set speed of robot
        @return             speed in mm/s
        """
        return FANUCethernetipDriver.readR_Register(self.robot_IP, self.speed_register)
        
    # Starts robot movement and checks to see when it has completed
    # Default to blocking 
    # Function will block until move action is complete
    def start_robot(self, blocking: bool=True):  
        """! starts robot movement by setting the sync register to 1 on the TP program executing commands.
        @param blocking     True/False program will wait to continue till move is finished. Default=True
        """
        # Write to start register to begin movement
        FANUCethernetipDriver.writeR_Register(self.robot_IP, self.start_register, 1)

        # Wait till robot is done moving
        if blocking == True:
            moving = self.is_moving()
            while(moving):
                moving = self.is_moving()
            if self.read_robot_start_register() == 1:
                # If the start register is still 1 but the robot is no longer moving, then there is an error
                raise TimeoutError("Error has occurred on robot, check TP for further diagnois")

            # Signal end of move action
            print("********************************************")
            print("* Moving Joint(s) to Position(s): COMPLETE *")
            print("********************************************")
        elif blocking == False:
            pass # If an error happens here, it 'dies quietly' 

    # Detect if the robot is moving
    def is_moving(self) -> bool:
        """! checks to see if robot is moving based on the value of the sync register 1=moving 0=not moving
        """
        pose1 = self.read_current_cartesian_pose()
        pose2 = self.read_current_cartesian_pose()
        diff = list(map(lambda a, b: a - b, pose1, pose2))
        #print("Difference: ", diff)

        if all([ value == 0.0 for value in diff ]): # If there is no difference
            return 0 # Not moving
        else:
            return 1 


    # Put CRX10 in a mount position to change end tooling
    # !!! -- THIS CAN BE MOVED INTO OWN FILE -- !!!
    # !!! -- THIS JOIN CONFIGURATION IS FOR THE CRX10 ROBOT -- !!!
    def set_joints_to_mount_position(self, blocking:bool=True):
        """! set the joints to be in a 'mount' position for tooling. 
        This is for CRX10's and will most likely be removed from this API
        """
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

        FANUCethernetipDriver.writeJointPositionRegister(self.robot_IP, self.PRNumber, self.CurJointPosList)
        self.start_robot(blocking=blocking)

    # This function reads register 1(sync bit for position register)
    def read_robot_start_register(self) -> int:
        """! returns value of start register
        @return             value of start register
        """
        start_register = FANUCethernetipDriver.readR_Register(self.robot_IP, self.start_register)
        return start_register

    # Toggle gripper open and close
    def schunk_gripper(self, command:str):
        """! controls schunk gripper.
        @param command      string 'open' or 'close'
        """
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
            raise Warning(f"Gripper only supports 'open' or 'closed' strings")


    # Open onRobot gripper
    def onRobot_gripper_open(self, width_in_mm:int, force_in_newtons:int):
        """! FUNCTION WILL BE MOVED TO ITS OWN MODULE: opens the onRobot gripper
        @param width_in_mm          value in mm to set gripper jaw distance
        @param force_in_newtons     value 0-120 in newtons
        """
        if width_in_mm > 160 | width_in_mm < 0:
            raise Warning(f"Width should be in the range of [0, 160], got {width_in_mm}")
        if force_in_newtons > 120 | force_in_newtons < 0:
            raise Warning(f"Force should be in the range of [0, 120], got {force_in_newtons}")
        
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 35, 1) # Instance typically 1
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 36, width_in_mm) # Set open width in mm
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 37, force_in_newtons) # Set open force in newtons
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 42, 8) # Set to 1 for use with R[43]
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 43, 50) # Register # you want data sent to
        #FANUCethernetipDriver.writeR_Register(self.robot_IP, 3, 1) # Set sync bit for onRobot gripper 1 = open
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 3, 3) # Set sync bit for onRobot gripper 1 = open

    # Close onRobot gripper
    def onRobot_gripper_close(self, width_in_mm:int, force_in_newtons:int):
        """! FUNCTION WILL BE MOVED TO ITS OWN MODULE: closes the onRobot gripper
        @param width_in_mm          value in mm to set gripper jaw distance
        @param force_in_newtons     value 0-120 in newtons
        """
        if width_in_mm > 160 | width_in_mm < 0:
            raise Warning(f"Width should be in the range of [0, 160], got {width_in_mm}")
        if force_in_newtons > 120 | force_in_newtons < 0:
            raise Warning(f"Force should be in the range of [0, 120], got {force_in_newtons}")

        FANUCethernetipDriver.writeR_Register(self.robot_IP, 35, 1) # Instance typically 1
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 39, width_in_mm) # Set close width in mm
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 40, force_in_newtons) # Set close force in newtons
        FANUCethernetipDriver.writeR_Register(self.robot_IP, 3, 2) # Set sync bit for onRobot gripper 2 = close

    # Read conveyor belt sensor: returns value of 1 or 0
    def conveyor_proximity_sensor(self, sensor) -> int:
        """! reads proximity sensors
        @param sensor               string 'right' or 'left' sensor
        """
        right_sensor_register = 31
        left_sensor_register = 30

        if sensor == "right":
            return FANUCethernetipDriver.readR_Register(self.robot_IP, right_sensor_register)
        elif sensor == "left":
            return FANUCethernetipDriver.readR_Register(self.robot_IP, left_sensor_register)
        else:
            raise Warning("Invalid Sensor, Try 'right' or 'left'\n")

    # Control conveyor belt
    def conveyor(self, command: str):
        """! Controls conveyor belt
        @param command          string 'forward' or 'reverse' or 'stop'
        """

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
            raise Warning(f"Conveyor only supports 'forward', 'reverse' or 'stop' strings")


    def read_robot_connection_bit(self) -> int:
        """
        Reads and returns the value at DI[1]
        """
        value = FANUCethernetipDriver.readDigitalInput(self.robot_IP,1)
        return value
    

    def write_robot_connection_bit(self,status) -> int:
        """
        Writes a value to R[1]->DO[1]
        """
        if status > 1 or status < 0:
            raise ValueError("Value must be either 1 or 0")
        DigitalOut = 1
        sync_register = 2
        sync_value = 1
        FANUCethernetipDriver.writeR_Register(self.robot_IP, DigitalOut, status)
        ## Set sync bit to update
        FANUCethernetipDriver.writeR_Register(self.robot_IP, sync_register, sync_value)