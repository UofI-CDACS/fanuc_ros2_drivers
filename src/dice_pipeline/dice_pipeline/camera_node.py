#!/usr/bin/env python3
"""
Camera node for the dice pip counting pipeline.

Uses the MindVision industrial camera SDK (mvsdk) for image capture.
Publishes frames on /camera/image_raw and provides the /camera/count_pips
service that captures a frame, detects pips via HSV colour masking, and
returns the count.

Requires MVSDK_PATH set in a .env file at the workspace root.
See .env.example for the format.
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
from std_msgs.msg import Int32

# ── Detection parameters ──────────────────────────────────────────────────────
DIE_SIZE_LOWER = 5000          # min contour area (px²) to be considered a die
PIP_SIZE_LOWER = 340           # min pip blob area (px²)
PIP_SIZE_UPPER = 4039          # max pip blob area (px²)
HSV_LOWER = [7, 141, 53]       # lower HSV bound for yellow die
HSV_UPPER = [18, 255, 135]     # upper HSV bound for yellow die
# ─────────────────────────────────────────────────────────────────────────────


def detect_pips(image: np.ndarray) -> tuple:
    """Detect pips on a yellow die in a BGR image using HSV colour masking.

    Finds the die by looking for a large yellow contour, then counts dark
    pip blobs within that die's bounding box.

    Returns (pip_count, annotated_image).
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(HSV_LOWER), np.array(HSV_UPPER))

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    annotated = image.copy()
    total_pips = 0
    die_num = 0

    for contour in contours:
        if cv2.contourArea(contour) <= DIE_SIZE_LOWER:
            continue

        die_num += 1
        x, y, w, h = cv2.boundingRect(contour)

        # Count pips inside this die's crop
        die_face = mask[y:y + h, x:x + w].copy()
        pip_contours, _ = cv2.findContours(
            die_face, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        num_pips = 0
        for pip in pip_contours:
            area = cv2.contourArea(pip)
            if PIP_SIZE_LOWER < area < PIP_SIZE_UPPER:
                num_pips += 1
                px, py, pw, ph = cv2.boundingRect(pip)
                cv2.rectangle(
                    annotated,
                    (x + px, y + py), (x + px + pw, y + py + ph),
                    (0, 0, 255), 2,
                )

        total_pips += num_pips

        # Green box + label around each die
        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            annotated, f'die{die_num}: {num_pips} pips',
            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )

    cv2.putText(
        annotated, f'Pips: {total_pips}',
        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
    )
    return total_pips, annotated


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('image_save_dir', '/tmp/dice_images')
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
        mvsdk.CameraSetExposureTime(self.hCamera, 50 * 1000)

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
        self.pip_pub_ = self.create_publisher(Int32, '/camera/pip_count', 10)
        self.create_timer(1.0 / hz, self._publish_frame)
        self.create_timer(1.0, self._publish_pip_count)

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

    def _publish_pip_count(self):
        frame = self._grab_frame()
        if frame is None:
            return
        bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR) if self.mono else frame
        h, w = bgr.shape[:2]
        x0 = self.roi_x_min if self.roi_x_min > 0 else 0
        y0 = self.roi_y_min if self.roi_y_min > 0 else 0
        x1 = self.roi_x_max if self.roi_x_max > 0 else w
        y1 = self.roi_y_max if self.roi_y_max > 0 else h
        pip_count, _ = detect_pips(bgr[y0:y1, x0:x1])
        msg = Int32()
        msg.data = pip_count
        self.pip_pub_.publish(msg)

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
        response.message = f'Detected {pip_count} pip(s). Images saved to {self.save_dir}.'
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
