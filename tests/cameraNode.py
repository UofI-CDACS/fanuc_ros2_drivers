import asyncio

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge

from camera import Camera
import cv2
import numpy as np

mask_low = np.array([11, 185, 118])
mask_high = np.array([180, 255, 255])

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self._bridge = CvBridge()
        self._camera = Camera()
        # self._pub = self.create_publisher(Image, 'camera/image_raw', 10)
        # self.create_service(Trigger, 'camera/capture', self._capture)
        self.create_service(Trigger, 'camera/count_pips', self._count_pips_service)
        self.get_logger().info('CameraNode ready — call /camera/capture to grab a frame')

#Publisher functions Claude
    # def _capture(self, request, response):
    #     try:
    #         frame = asyncio.run(self._camera.getFrameAsync())
    #         msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
    #         msg.header.stamp = self.get_clock().now().to_msg()
    #         msg.header.frame_id = 'camera_frame'
    #         self._pub.publish(msg)
    #         self.get_logger().info('Captured and published camera frame')
    #         response.success = True
    #         response.message = ''
    #     except Exception as e:
    #         response.success = False
    #         response.message = str(e)
    #     return response

    def _count_pips_service(self, request, response):
        try:
            frame = asyncio.run(self._camera.getFrameAsync())
            count = str(self.count_pips(frame))
            self.get_logger().info(f'Pip count: {count}')
            response.success = True
            response.message = count
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response


#Internal functions
    def count_pips(self, frame):
        mask = self.makeMask(frame)
        cv2.imwrite("mask.png", mask)  # Save the mask for debugging

        # Find contours with hierarchy (to detect holes inside the white die)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if contours is None or hierarchy is None:
            return -1

        # Find the largest contour - this is the die body
        largest_idx = max(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]))
        die_area = cv2.contourArea(contours[largest_idx])

        # Child contours of the die are the pips (holes inside white region)
        # hierarchy[0][i] = [next, prev, first_child, parent]
        pip_count = 0
        for i, h in enumerate(hierarchy[0]):
            parent = h[3]
            if parent == largest_idx:
                area = cv2.contourArea(contours[i])
                # Filter by area relative to die to ignore noise
                if area > die_area * 0.005:
                    pip_count += 1

        return pip_count

    def makeMask(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, mask_low, mask_high)
        kernel = np.ones((5, 5), np.uint8)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask_clean = cv2.dilate(mask_clean, kernel, iterations=1)
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return mask_clean



    def destroy_node(self):
        self._camera.disable()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
