#!/usr/bin/env python3
"""
Quick one-shot frame grab — saves /tmp/grab.bmp.

Calls /mv_camera/capture service then reads one frame from /mv_camera/image_raw,
so it works while mv_camera_node is already running.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import time

class GrabNode(Node):
    def __init__(self):
        super().__init__('grab_node')
        self.bridge = CvBridge()
        self.image  = None
        self.create_subscription(Image, '/mv_camera/image_raw', self._cb, 1)
        self.cli = self.create_client(Trigger, '/mv_camera/capture')

    def _cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

def main():
    rclpy.init()
    node = GrabNode()

    print('Waiting for /mv_camera/capture service...')
    if not node.cli.wait_for_service(timeout_sec=5.0):
        print('ERROR: /mv_camera/capture not available — is mv_camera_node running?')
        node.destroy_node()
        rclpy.shutdown()
        return

    future = node.cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future)
    if not future.result().success:
        print('ERROR: capture service returned failure.')
        node.destroy_node()
        rclpy.shutdown()
        return

    deadline = time.time() + 3.0
    while node.image is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    if node.image is None:
        print('ERROR: timed out waiting for image frame.')
    else:
        cv2.imwrite('/tmp/grab.bmp', node.image)
        print('Saved /tmp/grab.bmp')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
