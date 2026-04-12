#!/usr/bin/env python3
"""
Camera node for the dice pip counting pipeline.

Uses the MindVision industrial camera SDK (mvsdk) for image capture.
Publishes frames on /camera/image_raw and provides the /camera/count_pips
service that captures a frame, detects pips via contour analysis, and
returns the count.

Requires MVSDK_PATH set in a .env file at the workspace root.
See .env.example for the format.

Tune the constants below (MIN_PIP_AREA, MAX_PIP_AREA, MIN_CIRCULARITY)
to match your specific camera setup and die size.
"""

import os
import sys
from datetime import datetime


def _load_env():
    """Walk up from this file to find and load a .env file."""
    here = os.path.abspath(os.path.dirname(__file__))
    for _ in range(10):
        candidate = os.path.join(here, '.env')
        if os.path.exists(candidate):
            env_dir = here
            with open(candidate) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#') or '=' not in line:
                        continue
                    key, _, val = line.partition('=')
                    key, val = key.strip(), val.strip()
                    # Resolve relative paths against the .env file's directory
                    if key.endswith('_PATH') and not os.path.isabs(val):
                        val = os.path.normpath(os.path.join(env_dir, val))
                    os.environ.setdefault(key, val)
            return
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent


_load_env()

mvsdk_path = os.environ.get('MVSDK_PATH')
if not mvsdk_path:
    raise RuntimeError(
        'MVSDK_PATH is not set. '
        'Copy .env.example to .env and fill in your SDK path.'
    )
sys.path.append(mvsdk_path)
import mvsdk  # noqa: E402

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from fanuc_interfaces.srv import CountPips
from rclpy.node import Node
from sensor_msgs.msg import Image

# ── Pip detection tuning parameters ──────────────────────────────────────────
# Area in pixels² of a single pip blob.  Adjust after testing with your camera.
MIN_PIP_AREA = 50
MAX_PIP_AREA = 2000
# Circularity threshold: 1.0 = perfect circle.  Pips are round; noise is not.
MIN_CIRCULARITY = 0.60
# Adaptive threshold C value -- lower = more sensitive (catches shadowed pips).
ADAPTIVE_C = 1
# ─────────────────────────────────────────────────────────────────────────────


def detect_pips(image: np.ndarray) -> tuple:
    """Detect dice pips in a BGR image.

    Returns (pip_count, annotated_image) where annotated_image has green
    circles drawn around each detected pip.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)

    thresh = cv2.adaptiveThreshold(
        blurred, 255,
        cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
        cv2.THRESH_BINARY_INV,
        blockSize=11,
        C=ADAPTIVE_C,
    )

    contours, _ = cv2.findContours(
        thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )

    annotated = image.copy()
    pip_count = 0

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if not (MIN_PIP_AREA < area < MAX_PIP_AREA):
            continue

        perimeter = cv2.arcLength(cnt, True)
        if perimeter == 0:
            continue

        circularity = 4 * np.pi * area / (perimeter ** 2)
        if circularity < MIN_CIRCULARITY:
            continue

        pip_count += 1
        (cx, cy), radius = cv2.minEnclosingCircle(cnt)
        cv2.circle(annotated, (int(cx), int(cy)), int(radius) + 2, (0, 255, 0), 2)

    cv2.putText(
        annotated, f'Pips: {pip_count}',
        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
    )
    return pip_count, annotated


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('image_save_dir', '/tmp/dice_images')
        # ROI crop applied before pip detection to focus on the die and exclude
        # the gripper/background.  Set all to 0 to use the full image.
        self.declare_parameter('roi_x_min', 0)
        self.declare_parameter('roi_y_min', 0)
        self.declare_parameter('roi_x_max', 0)
        self.declare_parameter('roi_y_max', 0)

        cam_idx = self.get_parameter('camera_index').value
        hz = self.get_parameter('publish_hz').value
        self.save_dir = self.get_parameter('image_save_dir').value
        self.roi_x_min = self.get_parameter('roi_x_min').value
        self.roi_y_min = self.get_parameter('roi_y_min').value
        self.roi_x_max = self.get_parameter('roi_x_max').value
        self.roi_y_max = self.get_parameter('roi_y_max').value

        os.makedirs(self.save_dir, exist_ok=True)

        # ── MindVision camera init ────────────────────────────────────────────
        dev_list = mvsdk.CameraEnumerateDevice()
        if len(dev_list) == 0:
            self.get_logger().error('No MindVision camera found. Is it plugged in?')
            raise RuntimeError('No MindVision camera found.')

        if cam_idx >= len(dev_list):
            self.get_logger().warn(
                f'camera_index {cam_idx} out of range '
                f'({len(dev_list)} camera(s) found). Using index 0.'
            )
            cam_idx = 0

        dev_info = dev_list[cam_idx]
        self.get_logger().info(
            f'Opening camera: {dev_info.GetFriendlyName()} '
            f'({dev_info.GetPortType()})'
        )

        self.hCamera = mvsdk.CameraInit(dev_info, -1, -1)
        cap_info = mvsdk.CameraGetCapability(self.hCamera)

        self.mono = (cap_info.sIspCapacity.bMonoSensor != 0)
        if self.mono:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        mvsdk.CameraSetTriggerMode(self.hCamera, 0)
        mvsdk.CameraSetAeState(self.hCamera, 0)
        mvsdk.CameraSetExposureTime(self.hCamera, 30 * 1000)

        mvsdk.CameraPlay(self.hCamera)

        buf_size = (
            cap_info.sResolutionRange.iWidthMax
            * cap_info.sResolutionRange.iHeightMax
            * (1 if self.mono else 3)
        )
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)
        # ─────────────────────────────────────────────────────────────────────

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        self.create_timer(1.0 / hz, self._publish_frame)

        self.srv = self.create_service(
            CountPips, '/camera/count_pips', self._count_pips_callback
        )
        self.get_logger().info('Camera node ready.')

    def _grab_frame(self):
        """Grab one frame from the MindVision camera. Returns np.ndarray or None."""
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
            mvsdk.CameraImageProcess(
                self.hCamera, pRawData, self.pFrameBuffer, FrameHead
            )
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            channels = 1 if self.mono else 3
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(
                self.pFrameBuffer
            )
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (FrameHead.iHeight, FrameHead.iWidth, channels)
            )
            return frame.copy()
        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                self.get_logger().warn(
                    f'CameraGetImageBuffer failed ({e.error_code}): {e.message}'
                )
            return None

    def _publish_frame(self):
        frame = self._grab_frame()
        if frame is None:
            return
        encoding = 'mono8' if self.mono else 'bgr8'
        msg = self.bridge.cv2_to_imgmsg(frame, encoding=encoding)
        self.publisher_.publish(msg)

    def _count_pips_callback(self, request, response):
        frame = self._grab_frame()
        if frame is None:
            response.success = False
            response.pip_count = 0
            response.message = 'Failed to capture image from camera.'
            self.get_logger().error(response.message)
            return response

        bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if self.mono else frame
        h, w = bgr.shape[:2]
        x0 = self.roi_x_min if self.roi_x_min > 0 else 0
        y0 = self.roi_y_min if self.roi_y_min > 0 else 0
        x1 = self.roi_x_max if self.roi_x_max > 0 else w
        y1 = self.roi_y_max if self.roi_y_max > 0 else h
        detect_img = bgr[y0:y1, x0:x1]
        pip_count, annotated = detect_pips(detect_img)

        ts = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        cv2.imwrite(os.path.join(self.save_dir, f'raw_{ts}.jpg'), bgr)
        cv2.imwrite(os.path.join(self.save_dir, f'annotated_{ts}.jpg'), annotated)

        response.pip_count = pip_count
        response.success = True
        response.message = (
            f'Detected {pip_count} pip(s). Images saved to {self.save_dir}.'
        )
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        mvsdk.CameraUnInit(self.hCamera)
        mvsdk.CameraAlignFree(self.pFrameBuffer)
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
