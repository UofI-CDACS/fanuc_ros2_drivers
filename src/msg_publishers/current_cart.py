#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("src/dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_interfaces.msg import CurCartesian
from rclpy.node import Node

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

# Robot IP is passed as command line argument 1
robot_ip = sys.argv[1]

# Quick and dirty
if robot_ip == '172.29.208.124':
	name = "beaker"
elif robot_ip == '172.29.208.123':
     name = "bunsen"
else:
	name = "rogue"

class current_cartesian(Node):
    def __init__(self):
        super().__init__('cur_cart')
        self.bot = robot(robot_ip)
        self.publisher_ = self.create_publisher(CurCartesian, f'{name}/cur_cartesian', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = CurCartesian()                                          
        msg.pose = self.bot.read_current_cartesian_pose()[2:8]  # The first 2 bits don't hold positional data                                  
        self.publisher_.publish(msg)
        if FANUCethernetipDriver.DEBUG:
        	self.get_logger().info('Publishing: ' % msg.pose)


def main(args=None):
    rclpy.init(args=args)

    publisher = current_cartesian()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
