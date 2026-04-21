import asyncio

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
from cubeMap import CubeMap

#Claude ros2 node
class DiceNode(Node):
    def __init__(self):
        super().__init__('dice_node')

        # Parameters for identify_dice_location
        self.declare_parameter('top', 0)
        self.declare_parameter('side', 0)
        self.declare_parameter('clockwise', False)

        # Parameters for find_face_with_pip
        self.declare_parameter('pip', 0)
        self.declare_parameter('prefer_clockwise', False)

        self._cube = CubeMap()

        self.create_service(Trigger, 'dice/identify_dice_location', self._identify_dice_location)
        self.create_service(Trigger, 'dice/find_face_with_pip', self._find_face_with_pip)

        self.get_logger().info('DiceNode ready')

    def _identify_dice_location(self, request, response):
        top       = self.get_parameter('top').get_parameter_value().integer_value
        side      = self.get_parameter('side').get_parameter_value().integer_value
        clockwise = self.get_parameter('clockwise').get_parameter_value().bool_value
        self.get_logger().info(f"Identifying dice location with top={top}, side={side}, clockwise={clockwise}")
        try:
            self._cube.identify_dice_location(top=top, side=side, clockwise=clockwise)
            response.success = True
            response.message = ''
        except Exception as e:
            response.success = False
            response.message = str(e)
        self._cube.print_cube()
        return response

    def _find_face_with_pip(self, request, response):
        pip             = self.get_parameter('pip').get_parameter_value().integer_value
        prefer_clockwise = self.get_parameter('prefer_clockwise').get_parameter_value().bool_value
        try:
            moves = self._cube.find_face_with_pip(pip=pip, prefer_clockwise=prefer_clockwise)
            response.success = True
            # moves is a list of ints: 0=z-cw, 1=z-ccw, 2=x-cw, 3=x-ccw
            response.message = ','.join(str(m) for m in moves) if moves else ''
        except Exception as e:
            response.success = False
            response.message = str(e)
        self._cube.print_cube()
        return response


def main(args=None):
    rclpy.init()
    node = DiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
