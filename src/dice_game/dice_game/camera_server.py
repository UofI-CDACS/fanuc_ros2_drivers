#!/usr/bin/env python3
"""
camera_server.py

ROS2 service node that captures an image from the MindVision overhead camera,
counts pips on the yellow die face, and returns the count as an int32.

Service: /capture_and_count  (fanuc_interfaces/srv/CapturePip)

Based on pip_counter.py from asn1 — uses dual-threshold (Otsu + adaptive)
pip detection and retries CameraInit up to 5 times.
"""

import time
import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from fanuc_interfaces.srv import CapturePip
from dice_game import mvsdk


class CameraServer(Node):
    def __init__(self):
        super().__init__('camera_server')
        self.srv = self.create_service(
            CapturePip,
            '/capture_and_count',
            self._capture_callback,
        )
        self.get_logger().info('Camera server ready — /capture_and_count')

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------
    def _capture_image(self) -> np.ndarray:
        devs = mvsdk.CameraEnumerateDevice()
        if len(devs) < 1:
            raise RuntimeError('No MindVision camera found. Check ethernet cable.')

        hCamera = None
        last_err = None
        for attempt in range(5):
            try:
                hCamera = mvsdk.CameraInit(devs[0], -1, -1)
                break
            except Exception as e:
                last_err = e
                self.get_logger().warn(
                    f'CameraInit attempt {attempt + 1} failed: {e} — retrying in 2s')
                time.sleep(2.0)
        if hCamera is None:
            raise RuntimeError(f'CameraInit failed after 5 attempts: {last_err}')

        cap = mvsdk.CameraGetCapability(hCamera)
        mono = (cap.sIspCapacity.bMonoSensor != 0)
        fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if mono else mvsdk.CAMERA_MEDIA_TYPE_BGR8
        mvsdk.CameraSetIspOutFormat(hCamera, fmt)
        mvsdk.CameraSetTriggerMode(hCamera, 0)
        mvsdk.CameraSetAeState(hCamera, 0)
        mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
        mvsdk.CameraPlay(hCamera)

        channels = 1 if mono else 3
        buf_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * channels
        pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 2000)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (FrameHead.iHeight, FrameHead.iWidth, channels)
            )
            return frame.copy()
        finally:
            mvsdk.CameraUnInit(hCamera)
            mvsdk.CameraAlignFree(pFrameBuffer)

    # ------------------------------------------------------------------
    # Die detection
    # ------------------------------------------------------------------
    def _find_die_bbox(self, image: np.ndarray):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([15, 60, 40]), np.array([45, 255, 255]))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        margin = 8
        x = max(0, x - margin)
        y = max(0, y - margin)
        w = min(image.shape[1] - x, w + 2 * margin)
        h = min(image.shape[0] - y, h + 2 * margin)
        return (x, y, w, h)

    # ------------------------------------------------------------------
    # Pip counting — dual threshold (Otsu + adaptive), border exclusion
    # ------------------------------------------------------------------
    def _count_pips(self, image: np.ndarray, bbox: tuple) -> int:
        x, y, w, h = bbox
        roi = image[y:y+h, x:x+w].copy()

        roi_bright = cv2.convertScaleAbs(roi, alpha=3.5, beta=50)
        gray = cv2.cvtColor(roi_bright, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (3, 3), 0)

        _, dark_otsu = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        dark_adapt = cv2.adaptiveThreshold(
            blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 21, 8)
        dark = cv2.bitwise_or(dark_otsu, dark_adapt)
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))

        contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        border_x = int(w * 0.12)
        border_y = int(h * 0.12)

        pip_count = 0
        for c in contours:
            area = cv2.contourArea(c)
            if area < 20 or area > 6000:
                continue
            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            if (4 * np.pi * area / (perimeter ** 2)) <= 0.55:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            if cx < border_x or cx > w - border_x or cy < border_y or cy > h - border_y:
                continue
            pip_count += 1

        return pip_count

    # ------------------------------------------------------------------
    # Service callback
    # ------------------------------------------------------------------
    def _capture_callback(self, _request, response):
        self.get_logger().info('Capture request received.')
        try:
            frame = self._capture_image()
            cv2.imwrite('dice_capture.jpg', frame)

            bbox = self._find_die_bbox(frame)
            if bbox is None:
                self.get_logger().warn('No yellow die detected.')
                response.success = False
                response.pip_count = -1
                response.message = 'No yellow die detected in image'
                return response

            count = self._count_pips(frame, bbox)
            self.get_logger().info(f'Pip count: {count}')

            response.success = True
            response.pip_count = count
            response.message = ''

        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')
            response.success = False
            response.pip_count = -1
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = CameraServer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
