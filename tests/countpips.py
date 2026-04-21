import asyncio
from camera import Camera
import numpy as np
import rclpy
from rclpy.node import Node
import ros_robot
from ros_robot import FanucRosNode
import cv2

# Constants
rest_joint = [18.446,-7.714,-12.393,.285,-76.969,99.095]
camera_cart = [474.523,1010.305,514.833,87.937,-62.385,-178.903]
cube_grab_cart = [462.427,11.330,-178.959,179.417,.375,117.602]

camera_cart_side = [542.683,577.921,-120.016,83.716,-62.295,-175.130]
camera_cart = [542.683, 577.921, -120.016, 179.417, .375, 117.602]
conveyor_back_cart = [-204.389, 628.609, 16.305, 179.417, .375, 117.602]
conveyor_front_cart = [0]

async def main():
    #Create robot node
    rclpy.init()
    global robot, robot1
    robot = FanucRosNode('beaker')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot)
    asyncio.create_task(ros_robot.spin_robot(executor))

    #go to rest
    await robot.move_joints(rest_joint)
    #open gripper
    await robot.open_gripper_schunk('open')
    #grab cube
    await robot.move_cartesian(cube_grab_cart)
    await robot.open_gripper_schunk('close')
    #put cube in camera spot
    await robot.move_cartesian(camera_cart)
    await robot.open_gripper_schunk('open')
    await robot.move_joints(rest_joint)

    #take image
    side = await robot.get_dice_pip_count()

    #rotate cube
    await rotate_cube()
    await robot.move_joints(rest_joint)
    #take image
    top = await robot.get_dice_pip_count()
    
    #identify dice location
    await robot.identify_dice_location(top=top, side=side, clockwise=True)
    #find face with pip 1
    #rotate to spot

    #put dice on rear conveyor
    #stop conveyor when right sensor is triggered

    #publish dice ready

    #grab dice from front conveyor
    #put dice in camera spot
    #find face with pip 2
    #rotate to spot
    pass


#Direction 0 = x, 1 = y, 2 = z
def scoot(a, scoot, direction = 2):
        copy = a[:]
        copy[direction] = copy[direction] + scoot
        return copy

async def test():
    rclpy.init()
    global robot, robot1
    robot = FanucRosNode('beaker')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot)
    asyncio.create_task(ros_robot.spin_robot(executor))
    
    # robot.set_dice_ready(True)
    # on = await robot1.request_dice_ready()
    # await robot.identify_dice_location(top=3, side=1, clockwise=True)
    # move = await robot.find_face_with_pip(pip=5, prefer_clockwise=True)
    # print("Move sequence to get 5 pip face on top:", move)

    # pips = await robot.get_dice_pip_count()
    # print("Pip count:", pips)

    #sensor = await robot.read_conveyor_sensor()
    #print("Sensor reading:", sensor)

    # await robot.move_conveyor('forward')
    # await asyncio.sleep(5)
    # await robot.move_conveyor('stop')

    # frame = await robot.get_overhead_camera_frame()
    

async def rotate_cube():
    await robot.move_cartesian(scoot(camera_cart,50))
    await robot.move_cartesian(camera_cart)
    await robot.open_gripper_schunk('close')
    await robot.move_cartesian(scoot(camera_cart,100))
    await robot.move_cartesian(camera_cart_side)
    await robot.open_gripper_schunk('open')
    #await robot.move_joints(rest_joint)


if __name__ == "__main__":
    asyncio.run(main())