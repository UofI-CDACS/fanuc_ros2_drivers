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
cube_grab_cart = [474.523,-38.782,-178.863,87.940,-62.387,178.921]
camera_cart = [474.523,1010.305,514.833,87.937,-62.385,-178.903]
cube_done_cart = []

mask_low = np.array([11, 185, 118])
mask_high = np.array([180, 255, 255])

NUM_CYCLES = 3

async def main():
    #initialize camera and robot
    camera = Camera()

    rclpy.init()
    global robot
    robot = FanucRosNode('beaker')
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(robot)
    asyncio.create_task(ros_robot.spin_robot(executor))
    pip_counts = []

    #go to rest
    await robot.move_joints(rest_joint)
    #open gripper
    await robot.open_gripper_schunk('open')
    #Count pips on cubes
    for cycle in range(NUM_CYCLES):
        #go to cube grab up by 5cm
        await robot.move_cartesian(scoot(cube_grab_cart, 50))
        #go down to cube
        await robot.move_cartesian(cube_grab_cart)
        #close gripper
        await robot.open_gripper_schunk('close')
        #go to camera position
        await robot.move_cartesian(camera_cart)

        #Take picture
        frame = await camera.getFrameAsync()
        #process picture to get pip count
        pip_count = count_pips(frame)
        pip_counts.append(pip_count)

        #go above cube done by 5cm
        await robot.move_cartesian(scoot(cube_grab_cart, 50))
        #go to cube done
        await robot.move_cartesian(cube_grab_cart)
        #open gripper
        await robot.open_gripper_schunk('open')
        #go to cube grab up by 5cm
        await robot.move_cartesian(scoot(cube_grab_cart, 50))
        await asyncio.sleep(3)
    
    #go to rest
    await robot.move_joints(rest_joint)

    #print pip counts
    print("Pip counts:", pip_counts)
    #print pip sum
    print("Total pips:", np.sum(pip_counts))
    camera.disable()

def count_pips(frame):
    mask = makeMask(frame)
    cv2.imwrite("mask.png", mask)  # Save the mask for debugging

    # Find contours with hierarchy (to detect holes inside the white die)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if contours is None or hierarchy is None:
        return -1

    # Find the largest contour - this is the die body
    largest_idx = max(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]))
    die_area = cv2.contourArea(contours[largest_idx])

    # Child contours of the die are the pips (holes inside white region)
    # hierarchy[0][i] = [next, prev, first_child, parent]
    pip_count = 0
    for i, h in enumerate(hierarchy[0]):
        parent = h[3]
        if parent == largest_idx:
            area = cv2.contourArea(contours[i])
            # Filter by area relative to die to ignore noise
            if area > die_area * 0.005:
                pip_count += 1

    return pip_count


def makeMask(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, mask_low, mask_high)
    kernel = np.ones((5, 5), np.uint8)
    mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask_clean

#Direction 0 = x, 1 = y, 2 = z
def scoot(a, scoot, direction = 2):
        copy = a[:]
        copy[direction] = copy[direction] + scoot
        return copy

if __name__ == "__main__":
    asyncio.run(main())