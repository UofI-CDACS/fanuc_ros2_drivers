#!/usr/bin/env python3
"""! @brief Example python program using robot_controller.py"""

##
# @mainpage robot controller example project
#
# @section description_main Description
# An example python program demonstrating how to use robot_controller class.
# 
# @section todo_robot_controller_example TODO
# - Clean up
#
# @section author_robot_controller_example Author(s)
# - Created by James Lasso on 6/13/2023

# Imports
import sys
import time
import random
from robot_controller import robot

# Global Constants
#drive_path = '129.101.98.214' # Bill
drive_path = '129.101.98.215' # DJ
#drive_path = '129.101.98.244' # Larry
sleep_time = 0.5

def main():
    """! Main program entry"""

    # Create new robot object
    crx10 = robot(drive_path)

    # Set robot speed
    crx10.set_speed(400)

    # Make sure conveyor is off
    crx10.conveyor("stop")

    # Move robot to home position and open gripper
    crx10.set_joints_to_home_position()
    # Sync bit and move robot
    crx10.start_robot()
    # Open gripper
    crx10.gripper("open")

    # Move to FIRST position (PREPARE TO PICK UP DICE)
    pose1 = [14.000, 20.000, -45.000, -0.737, -46.000, 16.00]
    crx10.set_pose(pose1)
    # Sync bit and move robot
    crx10.start_robot()

    #time.sleep(sleep_time)


    # Move to SECOND position (MOVE DOWN TO GRAB DICE)
    pose2 = [14.000, 24.000, -52.690, -0.867, -38.678, 14.582]
    crx10.set_pose(pose2)
    # Sync bit and move robot
    crx10.start_robot()

    #time.sleep(sleep_time)

    # Close gripper
    crx10.gripper("close")
    # Pause briefly so gripper can close
    time.sleep(0.5)

    # Move back to FIRST position (HOLDING DICE, CLEAR TABLE)
    crx10.set_pose(pose1)
    # Sync bit and move robot
    crx10.start_robot()

    # Move to THIRD position (PREPARE TO PLACE DICE)
    pose3 = [56.128,23.078,-14.487,-1.349,-76.180,-26.270]
    crx10.set_pose(pose3)
    # Sync bit and move robot
    crx10.start_robot()

    # Move to FOURTH position(MOVE DOWN AND LET GO OF DICE)
    pose4 = [56.234,26.204,-22.890,-1.416,-67.776,-26.162]
    crx10.set_pose(pose4)
    # Sync bit and move robot
    crx10.start_robot()
    # Open gripper
    crx10.gripper("open")
    # Pause briefly so gripper can open
    time.sleep(0.5)

    # Move back to THIRD position (NOT HOLDING DICE, CLEAR TABLE)
    crx10.set_pose(pose3)
    # Sync bit and move robot
    crx10.start_robot()

    # Move to HOME position
    # Move robot to home position and open gripper
    crx10.set_joints_to_home_position()

    #start conveyor
    crx10.conveyor("forward")
    conveyor_toggle = True

    # Sync bit and move robot
    crx10.start_robot()
    #time.sleep(sleep_time)

    loops = 1
    while(loops <= 3):
        # if conveyor_toggle == True:
        #     conveyor_toggle = False
        # else:
        #     crx10.conveyor("forward")
        print(f"Loops: {loops}/3")
        # conveyor_toggle = False

        try:
            conveyor_on = True
            while(conveyor_on):
                # Check Sensors
                right = crx10.conveyor_proximity_sensor("right")
                left = crx10.conveyor_proximity_sensor("left")

                # Sensor check
                if right and not left:
                    crx10.conveyor("stop")
                    conveyor_on = False
                    time.sleep(0.5)
                elif not right and left:
                    crx10.conveyor("stop")
                    conveyor_on = False
                    time.sleep(0.5)

                # Brief sleep to check sensors
                time.sleep(0.1)
        finally:
            print("Stopping conveyor belt...")
            crx10.conveyor("stop")

        #time.sleep(sleep_time)

        # Wait for proximity sensor to trigger

        # Move to FIFTH position (PREPARE TO PICK UP DICE)
        pose5 = [105.8661117553711,6.044949531555176,-22.301790237426758,-0.9746052622795105,-67.01868438720703,-74.2422103881836]
        crx10.set_pose(pose5)
        # Sync bit and move robot
        crx10.start_robot()
        #time.sleep(sleep_time)

        # Move to SIXTH position (MOVE DOWN AND PICK UP DICE)
        pose6 = [106.07597351074219,9.101266860961914,-31.484973907470703,-1.0569950342178345,-57.83364486694336,-74.26985168457031]
        crx10.set_pose(pose6)
        # Sync bit and move robot
        crx10.start_robot()
        #time.sleep(sleep_time)

        # Close gripper
        crx10.gripper("close")
        # Pause briefly so gripper can close
        time.sleep(0.5)

        # Move back to FIFTH position (HOLDING DICE, CLEAR TABLE)
        crx10.set_pose(pose5)
        # Sync bit and move robot
        crx10.start_robot()
        #time.sleep(sleep_time)

        # Move to THIRD position (PREPARE TO PLACE DICE)
        crx10.set_pose(pose3)
        # Sync bit and move robot
        crx10.start_robot()

        # Move to FOURTH position (MOVE DOWN AND LET GO OF DICE)
        crx10.set_pose(pose4)
        # Sync bit and move robot
        crx10.start_robot()
        # Open gripper
        crx10.gripper("open")
        # Pause briefly so gripper can open
        time.sleep(0.5)

        #start conveyor
        crx10.conveyor("forward")

        # Move back to THIRD position (NOT HOLDING DICE, CLEAR TABLE)
        crx10.set_pose(pose3)
        # Sync bit and move robot
        crx10.start_robot()

        #  MOVE INTO WAIT POSITION IN MIDDLE
        pose7 = [73.16226196289062,10.072640419006348,-8.073392868041992,-1.1352527141571045,-81.83869171142578,-41.75325012207031]
        crx10.set_pose(pose7)
        # Sync bit and move robot
        crx10.start_robot()

        time.sleep(0.1)

        loops += 1

    # Final stop
    try:
        conveyor_on = True
        while(conveyor_on):
            # Check Sensors
            right = crx10.conveyor_proximity_sensor("right")
            left = crx10.conveyor_proximity_sensor("left")

            # Sensor check
            if right and not left:
                crx10.conveyor("stop")
                conveyor_on = False
                time.sleep(0.5)
            elif not right and left:
                crx10.conveyor("stop")
                conveyor_on = False
                time.sleep(0.5)

            # Brief sleep to check sensors
            time.sleep(0.1)
    finally:
        print("Stopping conveyor belt...")
        crx10.conveyor("stop")

    # Move to FIFTH position (PREPARE TO PICK UP DICE)
    pose5 = [105.8661117553711,6.044949531555176,-22.301790237426758,-0.9746052622795105,-67.01868438720703,-74.2422103881836]
    crx10.set_pose(pose5)
    # Sync bit and move robot
    crx10.start_robot()
    #time.sleep(sleep_time)

    # Move to SIXTH position (MOVE DOWN AND PICK UP DICE)
    pose6 = [106.07597351074219,9.101266860961914,-31.484973907470703,-1.0569950342178345,-57.83364486694336,-74.26985168457031]
    crx10.set_pose(pose6)
    # Sync bit and move robot
    crx10.start_robot()
    #time.sleep(sleep_time)

    # Close gripper
    crx10.gripper("close")
    # Pause briefly so gripper can close
    time.sleep(0.5)

    # Move back to FIFTH position (HOLDING DICE, CLEAR TABLE)
    crx10.set_pose(pose5)
    # Sync bit and move robot
    crx10.start_robot()
    #time.sleep(sleep_time)

    #  MOVE INTO WAIT POSITION IN MIDDLE
    pose7 = [73.16226196289062,10.072640419006348,-8.073392868041992,-1.1352527141571045,-81.83869171142578,-41.75325012207031]
    crx10.set_pose(pose7)
    # Sync bit and move robot
    crx10.start_robot()

    # Move to FIRST position (PREPARE TO PICK UP DICE)
    pose1 = [14.000, 20.000, -45.000, -0.737, -46.000, 16.00]
    crx10.set_pose(pose1)
    # Sync bit and move robot
    crx10.start_robot()

    #time.sleep(sleep_time)

    # Move to SECOND position (MOVE DOWN TO GRAB DICE)
    pose2 = [14.000, 24.000, -52.690, -0.867, -38.678, 14.582]
    crx10.set_pose(pose2)
    # Sync bit and move robot
    crx10.start_robot()
    
    # Open gripper
    crx10.gripper("open")
    # Pause briefly so gripper can open
    time.sleep(0.5)

    # Move to FIRST position (PREPARE TO PICK UP DICE)
    pose1 = [14.000, 20.000, -45.000, -0.737, -46.000, 16.00]
    crx10.set_pose(pose1)
    # Sync bit and move robot
    crx10.start_robot()

    # Move robot to home position and open gripper
    crx10.set_joints_to_home_position()
    # Sync bit and move robo	
    crx10.start_robot()

    # End program
    print("==============================")
    print("END OF PROGRAM")
    print("==============================")

if __name__=="__main__":
    main()
