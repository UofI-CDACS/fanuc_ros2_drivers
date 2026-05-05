#!/usr/bin/env python3
"""
Camera server node — runs on Beaker's machine.
Exposes /camera/capture_and_count service (one shared instance).
"""
import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from fanuc_interfaces.srv import CaptureAndCount
from .camera import Camera

# HSV pip detection tuning — adjust to match your lighting/dice colour
YELLOW_LO   = (18, 180, 150)
YELLOW_HI   = (24, 255, 255)
BLACK_V_MAX = 60
PIP_AREA_MIN = 30
PIP_AREA_MAX = 500


class CameraServerNode(Node):

    def __init__(self):
        super().__init__('camera_server')

        self.declare_parameter('camera_ip', '')
        camera_ip = self.get_parameter('camera_ip').value or None

        self.camera = Camera(camera_ip=camera_ip)
        self._busy = False

        self.create_service(CaptureAndCount, '/camera/capture_and_count', self._handle)
        self.get_logger().info('Camera server ready on /camera/capture_and_count')

    def _handle(self, request, response):
        if self._busy:
            self.get_logger().warn('Camera busy — rejecting concurrent request')
            response.pip_count = -1
            response.success = False
            return response

        self._busy = True
        try:
            frame = self.camera.getFrame()
            response.pip_count = self._count_pips(frame)
            response.success = True
            self.get_logger().info(f'Capture done: {response.pip_count} pip(s)')
        except Exception as exc:
            self.get_logger().error(f'Capture error: {exc}')
            response.pip_count = -1
            response.success = False
        finally:
            self._busy = False

        return response

    def _count_pips(self, image: np.ndarray) -> int:
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        yellow_mask = cv2.inRange(hsv, np.array(YELLOW_LO), np.array(YELLOW_HI))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

        y_cnts, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not y_cnts:
            self.get_logger().warn('No yellow region found — check YELLOW_LO/HI tuning')
            return 0

        filled = np.zeros_like(yellow_mask)
        cv2.drawContours(filled, [max(y_cnts, key=cv2.contourArea)], -1, 255, cv2.FILLED)

        black_mask = cv2.inRange(hsv, np.array([0, 0, 0]),
                                      np.array([180, 255, BLACK_V_MAX]))
        pip_mask = cv2.bitwise_and(black_mask, black_mask, mask=filled)
        pip_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        pip_mask = cv2.morphologyEx(pip_mask, cv2.MORPH_OPEN, pip_kernel)

        contours, _ = cv2.findContours(pip_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        pips = [c for c in contours if PIP_AREA_MIN < cv2.contourArea(c) < PIP_AREA_MAX]
        return min(len(pips), 6)

    def destroy_node(self):
        self.camera.disable()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraServerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
