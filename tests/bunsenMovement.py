import asyncio
from camera import Camera
import numpy as np
import rclpy
from rclpy.node import Node
import ros_robot
from ros_robot import FanucRosNode
import cv2
from time import sleep

#constants
rest_joint = [18.446,-7.714,-12.393,.285,-76.969,70.420] #from bunsun.py

cube_grab_cart = [462.424,-12.972,-121.357,179.305,.026,89.362] 
raise_cube_cart = [665.133, -778.504, 241.712, 173.946, -3.653, 87.306] #raising up to move above big table

camera_big_show_joint = [-40.156, 2.528, -18.348, 101.338, -54.386, -97.460] #joint pos to show camera at the top

#table_joint_center = [-47.122, -28.944, -39.548, -183.951, -86.523, -3.926] #moving cube to big table
#table_cart_center = [568.346,-610.383, -45.227, 176.163, -5.950, 87.268] #moving cube to big table cart
table_cart_center = [623.929,-734.668,-65.002,179.306,.048,88.928]
table_cart_center_hover = [623.929, -734.668, 40.002, 179.306, .048, 88.928]

#table_joint_flip_left = [-53.798, 55.955, -23.957, -35.711, -103.702, 141.583] #moving arm to left from my view, making side be on top
#table_cart_flip_left = [566.497, -670.675, -110.617, 174.188, -51.101, 87.080] #caart version of the pose
table_cart_flip_left = [623.929,-760.253,-93.468,-179.812,-39.617,88.609]
table_cart_flip_left_lift = [623.929, -806.253, 0.0, -179.812, -39.617, 88.609]

#table_joint_flip_right = [-21.657, 46.048, -90.375, 104.566, -75.944, 186.307] #moving arm right, making other side be on top
#table_cart_flip_right = [579.245, -517.187, -134.401, 155.228, -74.381, -71.581] #cart version of the post
#table_cart_rotate_left = [-47.122, -28.944, -39.548, 177.090, -5.796, 89.251] #rotate whole cube to left, face at back is now facing left
table_cart_flip_right = [623.929,-656.108,-91.817,177.856,51.512,88.013]
table_cart_flip_right_hover = [623.929, -656.108, 0.0, 177.856, 51.512, 88.013]
#camera_cart_rotate_front

conveyor_front_cart = [140.793,-711.148,69.238,179.306,.048,88.928] #placing on front conveyer
conveyer_front_cart_hover = [140.793, -711.148, 170.238, 179.306, .048, 88.928] #hovering above front conveyer to avoid collisions
conveyor_rear_cart_push = [-193.022, -903.207, 79.701, -179.413, 8.511, 91.512] #getting behind dice on back conveyer to push dice forward
conveyor_rear_cart_push_end = [-211.267,-723.401, 70.495, 179.305,.045,88.935] #pushing to end of conveyer for pickup
conveyor_rear_cart_grab_center = [-211.267,-673.673, 70.495, 179.305,.045,88.935]#at end of conveyer, picking up dice from center
conveyer_rear_cart_grab_hover = [-211.267, -673.673, 234.102, 179.305, 0.045, 88.935] #hovering at end of conveyer to avoid collisions

conveyer_rear_joint_grab = [-90.433, 30.029, -27.055, 8.143, -64.334, 84.001] #change this
conveyer_rear_joint_hover = [-90.083, 6.553, -7.223, 8.141, -79.393, 84.000]

async def main():

    #create robot node
    rclpy.init()
    global bunsen

    bunsen = FanucRosNode('bunsun')
    # beaker = FanucRosNode('beaker')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bunsen)
    asyncio.create_task(ros_robot.spin_robot(executor))

    await bunsen.move_conveyor('stop')
    print("stop conveyer")
    #go rest
    await bunsen.move_joints(rest_joint)

    
    #close for push
    await bunsen.open_gripper_onrobot(True)

    for i in range(2, 8, 2):
        while not await bunsen.request_dice_ready():
            sleep(1)
        #grab cube, go back to rest
        await bunsen.move_joints(conveyer_rear_joint_hover)
        await bunsen.move_joints(conveyer_rear_joint_grab)
        await bunsen.open_gripper_onrobot(False)
        await bunsen.move_joints(conveyer_rear_joint_hover)
        await bunsen.move_joints(rest_joint)
    
        #show camera top
        top = await top_peek()

        #show camera side
        side = await side_peek()

        #receive info from beaker that the cube has been placed and that it has arrived, beaker send info of side
        await bunsen.identify_dice_location(top=top,side=side,clockwise=True)
        #await bunsen.identify_dice_location(top=1, side=3, clockwise=True)
    
        await bunsen.move_joints(rest_joint) #move to rest pos

        #setup for finding face
        await bunsen.move_cartesian(table_cart_center)
        await bunsen.open_gripper_onrobot(True)    
        #find face with pip 2
        find_pip_i = await bunsen.find_face_with_pip(pip=i, prefer_clockwise=True) #get instructions to find face
        #rotate to correct side
        await follow_move_instructions(find_pip_i) #follow the instructions
    
        #show camera 
        topFinal = top_peek()
        if (topFinal != i):
            print("PIPS DO NOT MATCH INTENDED")
     
        await bunsen.move_cartesian(table_cart_center)
        await bunsen.open_gripper_onrobot(False)
        
        if (i == 6): #if 6, go to set down dice and then rest position
            await bunsen.move_joints(rest_joint)
            await bunsen.move_cartesian(cube_grab_cart)
            await bunsen.open_gripper_onrobot(True)
            await bunsen.move_joints(rest_joint)
            break

        else: #conveyer code
            await bunsen.move_cartesian(table_cart_center_hover)
            await bunsen.move_cartesian(conveyer_front_cart_hover)
            await bunsen.move_cartesian(conveyor_front_cart)
            await bunsen.open_gripper_onrobot(True)
            await bunsen.move_conveyor('reverse')
            
            while True:
                sensor = await bunsen.read_conveyor_sensor('left')
                if sensor:
                    await bunsen.move_conveyor('stop')
                    break
                await asyncio.sleep(0.05)

            bunsen.set_dice_ready(True)

    pass

def scoot(a, scoot, direction=2):
    copy = a[:]
    copy[direction] = copy[direction] + scoot
    return copy

async def follow_move_instructions(move_sequence):
    for move in move_sequence:
        if move == 0:
            await rotate_cube_z(clockwise=True)
        elif move == 1:
            await rotate_cube_z(clockwise=False)
        elif move == 2 or move == 3:
            await rotate_cube_x()
            
async def side_peek():
    #bunsen must already be holding dice in center pos
    await bunsen.open_gripper_onrobot(True) #drop from center
    await bunsen.move_cartesian(table_cart_flip_right) #rotate to show side where clockwise = true
    await bunsen.open_gripper_onrobot(False) #grab cube
    await bunsen.move_cartesian(table_cart_flip_right_hover) #move up right to flip
    await bunsen.move_cartesian(table_cart_flip_left_lift) #rotate left
    await bunsen.move_cartesian(table_cart_flip_left) #drop to table left
    await bunsen.open_gripper_onrobot(True) #drop
    await bunsen.move_joints(rest_joint) #move out of the way
    side = await bunsen.get_dice_pip_count()
    await bunsen.move_cartesian(table_cart_center) #mvoe back to center
    await bunsen.move_cartesian(table_cart_flip_left_lift)
    await bunsen.move_cartesian(table_cart_flip_left)
    await bunsen.open_gripper_onrobot(False) #grab dice
    await bunsen.move_cartesian(table_cart_flip_left_lift)
    await bunsen.move_cartesian(table_cart_flip_right_hover) #flip back to og top
    await bunsen.move_cartesian(table_cart_flip_right) #move back to right
    await bunsen.open_gripper_onrobot(True)
    await bunsen.move_cartesian(table_cart_center) #move back to center
    return side

async def top_peek():
    #bunsen must have dice from conveyer
    await bunsen.move_cartesian(table_cart_center)
    await bunsen.open_gripper_onrobot(True)
    await bunsen.move_joints(rest_joint)
    top = await bunsen.get_dice_pip_count()
    return top

async def rotate_cube_x():
    await bunsen.move_cartesian(scoot(table_cart_center, 50))
    await bunsen.move_cartesian(table_cart_flip_left)
    await bunsen.open_gripper_onrobot(False)
    await bunsen.move_cartesian(table_cart_flip_left_lift)
    
    await bunsen.move_cartesian(scoot(table_cart_flip_right, 200))
    await bunsen.move_cartesian(table_cart_flip_right)
    await bunsen.open_gripper_onrobot(True)
    await bunsen.move_cartesian(scoot(table_cart_center, 50))

async def rotate_cube_z(clockwise=False):
    # await bunsen.move_cartesian(table_cart_center)
    # await bunsen.open_gripper_onrobot(False)
    # await bunsen.move_cartesian(table_cart_rotate_left)

    await bunsen.move_cartesian(scoot(table_cart_center, 50))
    await bunsen.move_cartesian(table_cart_center)
    await bunsen.open_gripper_onrobot(False)
    pos = table_cart_center[:]
    pos[5] = pos[5] + (-90 if clockwise else 90)
    pos[5] = (pos[5] + 180) % 360 - 180
    await bunsen.move_cartesian(pos)
    await bunsen.open_gripper_onrobot(True)
    await bunsen.move_cartesian(scoot(pos, 50))
    await bunsen.move_cartesian(scoot(table_cart_center, 50))

if __name__ == "__main__":
    asyncio.run(main())
