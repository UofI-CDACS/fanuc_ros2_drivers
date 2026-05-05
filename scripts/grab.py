#!/usr/bin/env python3
"""
grab.py
-------
Calls /mv_camera/capture then reads one frame from /mv_camera/image_raw
and saves it to /tmp/grab.bmp.

Requires the camera node (mv_camera_node) to already be running.
Run with: just grab
"""

import sys
import subprocess
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2

SAVE_PATH = '/tmp/grab.bmp'


class GrabNode(Node):
    def __init__(self):
        super().__init__('grab_node')
        self.bridge = CvBridge()
        self.image  = None
        self.sub    = self.create_subscription(Image, '/mv_camera/image_raw', self._cb, 10)
        self.client = self.create_client(Trigger, '/mv_camera/capture')

    def _cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def grab(self):
        if not self.client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                'Camera capture service not available — is mv_camera_node running?'
            )
            return False

        # Trigger a fresh frame
        future = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error('Camera capture service returned failure.')
            return False

        # Wait for the frame to arrive on the topic
        deadline_ns = self.get_clock().now().nanoseconds + int(3.0 * 1e9)
        while self.image is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > deadline_ns:
                self.get_logger().error('Timed out waiting for image.')
                return False

        cv2.imwrite(SAVE_PATH, self.image)
        self.get_logger().info(f'Image saved to {SAVE_PATH}')
        subprocess.Popen(['xdg-open', SAVE_PATH])
        return True


def main():
    rclpy.init()
    node = GrabNode()
    ok = node.grab()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
