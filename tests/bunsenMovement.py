import asyncio
from camera import Camera
import numpy as np
import rclpy
from rclpy.node import Node
import ros_robot
from ros_robot import FanucRosNode
import cv2

#constants
rest_cart = [343.585, -74.706, 269.215, -176.394, -6.516, 28.633]
conveyer_back_place_cart = [-212.432, -686.804, 67.388, -175.662, -1.039, -1.370]
conveyer_front_grab_cart = [143.404, -783.266, 69.988, -179.693, 3.072, 217]
place_and_turn_cart = [463.121, 4.996, -118.965, -179.906, 1.838, 177.382]
camera_cube_place = [453.939, -985.030, -56.540, -173.722, 0.563, -38.100]


async def main():
    #create robot node
    rclpy.init()
    global bunsen, beaker

    bunsen = FanucRosNode('bunsen')
    beaker = FanucRosNode('beaker')

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(bunsen)
    asyncio.create_task(ros_robot.spin_robot(executor))

    #go rest
    await bunsen.move_cartesian(rest_cart)

    #await gripper
    await bunsen.open_gripper_onrobot(True)

    #receive info from beaker that the cube has been placed and that it has arrived, beaker send info of side
    
    #grab cube
    await bunsen.move_cartesian(conveyer_front_grab_cart)
    await bunsen.open_gripper_onrobot(False)
    await bunsen.move_cartesian(rest_cart)

    #check current dice, beaker sends info of side
    await bunsen.identify_dice_location(top=1, side=side, clockwise=False)
    #find face with pip 2
    find_pip_2 = await bunsen.find_face_with_pip(pip=2, prefer_clockwise=False)
    #rotate to correct side
    await follow_move_instructions(find_pip_2)
    
    #move to rest pos
    await bunsen.move_cartesian(rest_cart)

    #put dice back on front conveyer, stop when sensor triggered
    #publish dice ready

    await bunsen.move_cartesian(conveyer_back_place_cart)
    await bunsen.open_gripper_onrobot(True)
    await bunsen.read_conveyor_sensor('left')
    await bunsen.conveyor_ac() #stop

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

async def rotate_cube_x():
    await bunsen.move_cartesian(scoot(camera_cube_place, 50))
    await bunsen.move_cartesian(camera_cube_place)
    await bunsen.open_gripper_onrobot(True)
    await bunsen.move_cartesian(scoot(camera_cube_place, 100))
    await bunsen.move_cartesian(rest_cart)
    await bunsen.open_gripper_onrobot(True)
    await bunsen.move_cartesian(scoot(rest_cart, 200))
    await bunsen.move_cartesian(scoot(camera_cube_place, 50))

async def rotate_cube_z(clockwise=True):
    await bunsen.move_cartesian(scoot(camera_cube_place, 50))
    await bunsen.move_cartesian(camera_cube_place)
    await bunsen.open_gripper_onrobot(False)
    pos = camera_cube_place[:]
    pos[5] = pos[5] + (-90 if clockwise else 90)
    pos[5] = (pos[5] + 180) % 360 - 180
    await bunsen.move_cartesian(pos)
    await bunsen.open_gripper_onrobot(True)
    await bunsen.move_cartesian(scoot(pos, 50))
    await bunsen.move_cartesian(scoot(camera_cube_place, 50))


if __name__ == "__main__":
    asyncio.run(main())
