#!/usr/bin/env python3
import sys
import os
import rclpy

import dependencies.FANUCethernetipDriver as FANUCethernetipDriver

from dependencies.robot_controller import robot
from fanuc_interfaces.srv import SetSpeed
from rclpy.node import Node

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class set_speed(Node):
    def __init__(self):
        super().__init__('speed_srv')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.bot = robot(self.get_parameter('robot_ip').value)
        self.srv = self.create_service(SetSpeed, f"{self.get_parameter('robot_name').value}/set_speed", self.service_callback)

    def service_callback(self, request, response):
        if request.speed >= 0 and request.speed <= 300:
            self.bot.set_speed(request.speed)
            self.get_logger().info('Changing speed to: '+str(request.speed))
            response.success = True 
        else:
            response.success = False
        if FANUCethernetipDriver.DEBUG:
            self.get_logger().info('Incoming request result: ',request.speed, ' ', response.success)

        return response


def main(args=None):
    rclpy.init(args=args)

    publisher = set_speed()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
