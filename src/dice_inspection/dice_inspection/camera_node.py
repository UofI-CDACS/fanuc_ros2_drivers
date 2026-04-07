#!/usr/bin/env python3
"""
Camera node for FANUC dice inspection assignment.
Provides a ROS2 service that captures an image from the MindVision
overhead camera and counts the pips on the visible die face.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image

import cv2
import numpy as np

from dice_inspection import mvsdk


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.srv = self.create_service(
            Trigger,
            'capture_and_count',
            self.capture_and_count_callback
        )
        self.image_pub = self.create_publisher(Image, 'camera/image', 10)
        self.get_logger().info('Camera node ready — waiting for capture requests.')

    # ------------------------------------------------------------------
    # Camera capture (MindVision SDK)
    # ------------------------------------------------------------------
    def capture_image(self) -> np.ndarray:
        DevList = mvsdk.CameraEnumerateDevice()
        if len(DevList) < 1:
            raise RuntimeError('No MindVision camera found!')

        DevInfo = DevList[0]
        self.get_logger().info(f'Using camera: {DevInfo.GetFriendlyName()}')

        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        cap = mvsdk.CameraGetCapability(hCamera)
        mono = (cap.sIspCapacity.bMonoSensor != 0)

        if mono:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        mvsdk.CameraSetTriggerMode(hCamera, 0)   # free-run
        mvsdk.CameraSetAeState(hCamera, 0)        # manual exposure
        mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)  # 30 ms
        mvsdk.CameraPlay(hCamera)

        w_max = cap.sResolutionRange.iWidthMax
        h_max = cap.sResolutionRange.iHeightMax
        channels = 1 if mono else 3
        pFrameBuffer = mvsdk.CameraAlignMalloc(w_max * h_max * channels, 16)

        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, channels))
        finally:
            mvsdk.CameraUnInit(hCamera)
            mvsdk.CameraAlignFree(pFrameBuffer)

        return frame

    # ------------------------------------------------------------------
    # Pip counting via OpenCV
    # ------------------------------------------------------------------
    def count_pips(self, frame: np.ndarray) -> int:
        """
        Count the black circular pips on the visible yellow die face.

        Steps:
        1. HSV threshold to isolate the yellow die face.
        2. Crop to the bounding box of the largest yellow region.
        3. Within that ROI, threshold for dark (pip) pixels.
        4. Find contours and filter by area + circularity.
        """
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Yellow die face
        lower_yellow = np.array([15, 60, 60])
        upper_yellow = np.array([45, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        kernel = np.ones((5, 5), np.uint8)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel, iterations=1)

        contours, _ = cv2.findContours(yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self.get_logger().warn('No yellow region detected in image.')
            return 0

        # Largest yellow blob = die face
        largest = max(contours, key=cv2.contourArea)
        bx, by, bw, bh = cv2.boundingRect(largest)
        margin = 10
        x1 = max(0, bx - margin)
        y1 = max(0, by - margin)
        x2 = min(frame.shape[1], bx + bw + margin)
        y2 = min(frame.shape[0], by + bh + margin)

        die_roi = frame[y1:y2, x1:x2]
        gray = cv2.cvtColor(die_roi, cv2.COLOR_BGR2GRAY)

        # Pips are dark circles; threshold to find them
        _, dark_mask = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY_INV)
        dark_mask = cv2.morphologyEx(
            dark_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )

        pip_contours, _ = cv2.findContours(
            dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        pip_count = 0
        for c in pip_contours:
            area = cv2.contourArea(c)
            if area < 50 or area > 3000:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4 * np.pi * area / (perimeter ** 2)
            if circularity > 0.5:
                pip_count += 1

        # Save annotated image for debugging
        debug = die_roi.copy()
        for c in pip_contours:
            area = cv2.contourArea(c)
            perimeter = cv2.arcLength(c, True)
            if area < 50 or area > 3000 or perimeter == 0:
                continue
            if 4 * np.pi * area / (perimeter ** 2) > 0.5:
                cv2.drawContours(debug, [c], -1, (0, 255, 0), 2)
        cv2.imwrite('dice_pip_debug.jpg', debug)

        return pip_count

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------
    def capture_and_count_callback(self, request, response):
        self.get_logger().info('Capture request received.')
        try:
            frame = self.capture_image()

            # Save raw capture
            cv2.imwrite('dice_capture.jpg', frame)
            self.get_logger().info('Image saved as dice_capture.jpg')

            # Publish to ROS2 image topic
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.height, msg.width = frame.shape[:2]
            msg.encoding = 'bgr8'
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            self.image_pub.publish(msg)

            count = self.count_pips(frame)
            self.get_logger().info(f'Pip count: {count}')

            response.success = True
            response.message = str(count)

        except Exception as e:
            self.get_logger().error(f'Camera/counting error: {e}')
            response.success = False
            response.message = '-1'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
