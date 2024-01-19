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

namespace = 'bunsen'

class FanucServices(Node):
    def __init__(self, namespace):
        super().__init__("robot")
		
		# Services
        self.home_sc = self.create_client(Home, f'{namespace}/go_home')
        self.mount_sc = self.create_client(Mount, f'{namespace}/go_mount')
        self.speed_sc = self.create_client(SetSpeed, f'{namespace}/set_speed')
		
    def run_test(self):
        # Home position
        request = Home.Request()
        while not self.home_sc.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        future = self.home_sc.call_async(request)
        #while not future.done():
            #pass # Wait to be done
        sleep(5)
        print("Home result:",future.result().success)


        # Mount position
        request = Mount.Request()
        while not self.mount_sc.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        future = self.mount_sc.call_async(request)
        while not future.done():
            pass # Wait to be done
        print("Home result:",future.result().success)


        # Set Speed
        request = SetSpeed.Request()
        request.speed = 250
        while not self.speed_sc.wait_for_service(timeout_sec=1.0):
            pass # Wait for service to be ready
        future = self.speed_sc.call_async(request)
        while not future.done():
            pass # Wait to be done
        print("Home result:",future.result().success)

        


if __name__ == '__main__':
    rclpy.init()
	
    fanuc = FanucServices(namespace)
	
    rclpy.spin_until_future_complete(fanuc, fanuc.run_test())
    rclpy.shutdown()
