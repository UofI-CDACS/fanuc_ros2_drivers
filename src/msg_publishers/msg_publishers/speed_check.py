#!/usr/bin/env python3
import sys
import os
import rclpy

import dependencies.FANUCethernetipDriver as FANUCethernetipDriver

from dependencies.robot_controller import robot
from fanuc_interfaces.msg import CurSpeed
from rclpy.node import Node

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')


class check_speed(Node):
    def __init__(self):
        super().__init__('speed_pub')

        self.declare_parameters(
            namespace='',
            parameters=[('robot_ip','172.29.208.0'),
                        ('robot_name','noNAME')] # custom, default
        )

        self.bot = robot(self.get_parameter('robot_ip').value)
        self.publisher_ = self.create_publisher(CurSpeed, f"{self.get_parameter('robot_name').value}/speed", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = CurSpeed()                                          
        msg.speed = self.bot.get_speed()                                    
        self.publisher_.publish(msg)
        if FANUCethernetipDriver.DEBUG:
        	self.get_logger().info('Publishing: ' % msg.speed)


def main(args=None):
    rclpy.init(args=args)

    publisher = check_speed()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
