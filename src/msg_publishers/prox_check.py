#!/usr/bin/env python3
import sys
import os
import rclpy

sys.path.append("src/dependencies/")
import FANUCethernetipDriver

from robot_controller import robot
from fanuc_interfaces.msg import ProxReadings
from rclpy.node import Node

FANUCethernetipDriver.DEBUG = False

sys.path.append('./pycomm3/pycomm3')

# Robot IP is passed as command line argument 1
# robot_ip = sys.argv[1]

# # Quick and dirty
# if robot_ip == '172.29.208.124':
# 	name = "beaker"
# elif robot_ip == '172.29.208.123':
#      name = "bunsen"
# else:
# 	name = "rogue"

class check_prox(Node):
    def __init__(self):
        super().__init__('prox_pub')

        self.declare_parameter('robot_ip', rclpy.Parameter.Type.STRING) 
        robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value

        self.bot = robot(robot_ip)
        self.publisher_ = self.create_publisher(ProxReadings, f'/prox_readings', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = ProxReadings()                                          
        msg.left = bool(self.bot.conveyor_proximity_sensor("left")) 
        msg.right = bool(self.bot.conveyor_proximity_sensor("right"))        
        self.publisher_.publish(msg)
        if FANUCethernetipDriver.DEBUG:
        	self.get_logger().info('Publishing: ' % msg.left,' ',msg.right)


def main(args=None):
    rclpy.init(args=args)

    publisher = check_prox()

    rclpy.spin(publisher)

    publisher.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
