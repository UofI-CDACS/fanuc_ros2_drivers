#12:45 - 2:45 friday May 15th
import asyncio
import rclpy
import ros_robot
from ros_robot import FanucRosNode

# Constants
rest_joint = [18.446,-7.714,-12.393,.285,-76.969,70.420]

cube_grab_cart = [462.424,-12.972,-121.357,179.305,.026,89.362] 

camera_cart_center = [623.929,-734.668,-65.002,179.306,.048,88.928]
camera_cart_side_grab = [623.929,-806.253,-93.468,-179.812,-39.617,88.609]
camera_cart_side_place = [623.929,-656.108,-91.817,177.856,51.512,88.013]
camera_cart = [623.929,-737.356,-65.002,179.306,.048,88.928]

conveyor_front_cart = [140.793,-711.148,69.238,179.306,.048,88.928]

conveyor_rear_cart_push = [-211.267,-890.056,83.503,179.305,.045,88.935]
conveyor_rear_cart_push_end = [-211.267,-723.401,83.503,179.305,.045,88.935]
conveyor_rear_cart_grab_center = [-211.267,-673.673,68.719,179.305,.045,88.935]

async def main():
    #Create robot node
    rclpy.init()
    global robot
    robot = FanucRosNode('bunsun')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot)
    asyncio.create_task(ros_robot.spin_robot(executor))

    #open gripper
    await robot.open_gripper_onrobot(True)
    #go to rest
    await robot.move_joints(rest_joint)

    #Loop through all even faces on die
    for i in range(2,7,2):
        #Wait for other robot's signal that it's ready
        while not await robot.request_dice_ready():
            await asyncio.sleep(5)

        #grab dice from rear conveyor
        await grab_cube_conveyor()

        #Find dice
        await identify_find_dice_routine(pip=i)
        
        #grab dice from camera spot
        await robot.move_cartesian(camera_cart)
        await robot.open_gripper_onrobot(False)
        await robot.move_cartesian(scoot(camera_cart,200))
        #put dice on front conveyor
        await robot.move_cartesian(scoot(conveyor_front_cart,100))
        await robot.move_cartesian(conveyor_front_cart)
        await robot.open_gripper_onrobot(True)
        await robot.move_cartesian(scoot(conveyor_front_cart,100))
        if i != 6: #don't need to send on conveyor on last loop
            await wait_for_conveyor_sensor("left")

            #publish dice ready
            robot.set_dice_ready(True)

    #End with cube set down and everything back in starting position
    #grab dice from camera spot
    await robot.move_cartesian(camera_cart)
    await robot.open_gripper_onrobot(False)
    await robot.move_cartesian(scoot(camera_cart,200))
    #go to rest and set down cube
    await robot.move_joints(rest_joint)
    await robot.move_cartesian(cube_grab_cart)
    await robot.open_gripper_onrobot(True)
    await robot.move_joints(rest_joint)

#Direction 0 = x, 1 = y, 2 = z
def scoot(a, scoot, direction = 2):
        copy = a[:]
        copy[direction] = copy[direction] + scoot
        return copy

async def test():
    #Create robot node
    rclpy.init()
    global robot
    robot = FanucRosNode('bunsun')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot)
    asyncio.create_task(ros_robot.spin_robot(executor))

    await robot.move_conveyor("stop")
    # #Rest
    # await robot.move_joints(rest_joint)
    # await robot.open_gripper_onrobot(True)

    # while(True):
    #     await asyncio.sleep(1)


async def grab_cube_conveyor():
    #Grab from conveyor sequence
    #Go to back push position
    await robot.move_cartesian(scoot(conveyor_rear_cart_push,200))
    await robot.move_cartesian(conveyor_rear_cart_push)
    await robot.open_gripper_onrobot(False)
    #Go to end of push position
    await robot.move_cartesian(conveyor_rear_cart_push_end)
    await robot.move_cartesian(scoot(conveyor_rear_cart_push_end,100))
    await robot.open_gripper_onrobot(True)
    #Grab cube from center
    await robot.move_cartesian(scoot(conveyor_rear_cart_grab_center,100))
    await robot.move_cartesian(conveyor_rear_cart_grab_center)
    await robot.open_gripper_onrobot(False)
    await robot.move_cartesian(scoot(conveyor_rear_cart_grab_center,100))
    #Set down cube in camera location
    await robot.move_cartesian(scoot(camera_cart_center,300))
    await robot.move_cartesian(camera_cart_center)
    await robot.open_gripper_onrobot(True)
    await robot.move_cartesian(scoot(camera_cart_center,100))

async def wait_for_conveyor_sensor(side="right"):
    dir = 'forward' if side == "right" else 'reverse'
    #stop conveyor when right sensor is triggered
    await robot.move_conveyor(dir)
    while True:
        sensor = await robot.read_conveyor_sensor(side)
        if sensor:
            await robot.move_conveyor('stop')
            break
        await asyncio.sleep(0.05)

async def identify_find_dice_routine(pip=1):
    #move out of the way of the camera
    await robot.move_joints(rest_joint)
    #take image
    side = await robot.get_dice_pip_count()
    #rotate cube
    await rotate_cube_x()
    await robot.move_joints(rest_joint)
    #take image
    top = await robot.get_dice_pip_count()
    
    #identify dice location
    await robot.identify_dice_location(top=top, side=side, clockwise=True)
    #find face with pip 1
    move_sequence = await robot.find_face_with_pip(pip=pip, prefer_clockwise=True)
    #rotate to correct side
    await follow_move_instructions(move_sequence)

async def follow_move_instructions(move_sequence):
    for move in move_sequence:
        if move == 0:
            await rotate_cube_z(clockwise=True)
        elif move == 1:
            await rotate_cube_z(clockwise=False)
        elif move == 2 or move == 3:
            await rotate_cube_x()

async def rotate_cube_x():
    await robot.move_cartesian(scoot(camera_cart,50))
    await robot.move_cartesian(camera_cart_side_grab)
    await robot.open_gripper_onrobot(False)
    await robot.move_cartesian(scoot(camera_cart_side_grab,200))
    await robot.move_cartesian(scoot(camera_cart_side_place,200))
    await robot.move_cartesian(camera_cart_side_place)
    await robot.open_gripper_onrobot(True)
    await robot.move_cartesian(scoot(camera_cart,50))

async def rotate_cube_z(clockwise=True):
    await robot.move_cartesian(scoot(camera_cart_center,50))
    await robot.move_cartesian(camera_cart_center)
    await robot.open_gripper_onrobot(False)
    pos = camera_cart_center[:]
    pos[5] = pos[5] + (-90 if clockwise else 90)
    pos[5] = (pos[5] + 180) % 360 - 180 #wrap to [-180,180]
    await robot.move_cartesian(pos)
    await robot.open_gripper_onrobot(True)
    await robot.move_cartesian(scoot(pos,50))
    await robot.move_cartesian(scoot(camera_cart_center,50))
    #await robot.move_joints(rest_joint)

if __name__ == "__main__":
    asyncio.run(main())