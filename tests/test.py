import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
from cubeMap import CubeMap


def main():
    cubeMap = CubeMap()
    cubeMap.identify_dice_location(top=4, side=1, clockwise=False)
    cubeMap.print_cube()
    moves = cubeMap.find_face_with_pip(pip=1, prefer_clockwise=False)
    print(moves)
    cubeMap.print_cube()


if __name__ == '__main__':
    main()
