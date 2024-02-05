"""
This is a test of all the services
"""
# ROS packages
import rclpy
from rclpy.node import Node

# Fanuc packages
import fanuc_interfaces
from fanuc_interfaces.srv import Home, Mount, SetSpeed

from time import sleep

import sys
sys.path.append("../src/dependencies/")
from pynput.keyboard import KeyCode
from key_commander import KeyCommander

namespace = 'beaker'

class FanucServices(Node):
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Services
        self.mount_sc = self.create_client(Mount, f'{namespace}/go_mount')
        self.speed_sc = self.create_client(SetSpeed, f'{namespace}/set_speed')
		
    def run_test(self):
        # Mount position
        request = Mount.Request()
        while not self.mount_sc.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        future = self.mount_sc.call_async(request)
        while not future.done():
            pass # Wait to be done
        print("Mount result:",future.result().success)


        # Set Speed
        request = SetSpeed.Request()
        request.speed = 250
        while not self.speed_sc.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        future = self.speed_sc.call_async(request)
        while not future.done():
            pass # Wait to be done
        print("Speed result:",future.result().success)

        
raise DeprecationWarning

if __name__ == '__main__':
    rclpy.init()
	
    fanuc = FanucServices(namespace)

    keycom = KeyCommander([
		(KeyCode(char='s'), fanuc.run_test),
		])
    print("S")
    rclpy.spin(fanuc)
    rclpy.shutdown()
