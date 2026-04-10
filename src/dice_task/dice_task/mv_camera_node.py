#!/usr/bin/env python3
"""
mv_camera_node.py
-----------------
ROS2 node that captures frames from a MindVision camera using the MindVision
SDK (mvsdk) and publishes them as sensor_msgs/Image messages.

This node must be running before dice_roller.py is started.

Publishes:
  /mv_camera/image_raw  (sensor_msgs/msg/Image, encoding: bgr8)

Requirements:
  - MindVision MVSDK installed and Python bindings on PYTHONPATH
  - pip install opencv-python cv_bridge

Run:
  python3 mv_camera_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_srvs.srv import Trigger

import numpy as np
import cv2

try:
    import mvsdk
except ImportError:
    raise ImportError(
        'MindVision SDK Python bindings (mvsdk) not found.\n'
        'Install the MVSDK package and ensure the Python wrapper is on your PYTHONPATH.\n'
        'Download from: https://www.mindvision.com.cn/category/software/'
    )


# ══════════════════════════════════════════════════════════════════════════════
#  Configuration
# ══════════════════════════════════════════════════════════════════════════════

PUBLISH_TOPIC   = '/mv_camera/image_raw'   # must match CAMERA_TOPIC in dice_roller.py
PUBLISH_RATE_HZ = 30.0                     # capture / publish rate in frames per second


# ══════════════════════════════════════════════════════════════════════════════
#  Node
# ══════════════════════════════════════════════════════════════════════════════

class MVCameraNode(Node):

    def __init__(self):
        super().__init__('mv_camera_node')

        self.bridge = CvBridge()

        # Publisher
        self.pub = self.create_publisher(Image, PUBLISH_TOPIC, 10)

        # Open the camera (non-fatal if not connected)
        try:
            self._init_camera()
            self.camera_ready = True
        except RuntimeError as e:
            self.get_logger().warn(f'Camera not available: {e}. Node will stay alive but publish nothing.')
            self.camera_ready = False

        # Service: call /mv_camera/capture to grab one fresh frame and publish it
        self.create_service(Trigger, '/mv_camera/capture', self._capture_cb)

        # Timer fires at PUBLISH_RATE_HZ to grab and publish each frame
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_frame)

        self.get_logger().info(
            f'MVCameraNode started — publishing on {PUBLISH_TOPIC} at {PUBLISH_RATE_HZ} Hz'
        )

    # ── Camera initialisation ────────────────────────────────────────────────────────────────────────────
    def _init_camera(self):
        """Enumerate MindVision cameras and initialise the first one found."""
        dev_list = mvsdk.CameraEnumerateDevice()
        if len(dev_list) == 0:
            raise RuntimeError('No MindVision cameras detected. Check USB/GigE connection.')

        dev_info = dev_list[0]
        self.get_logger().info(f'Opening camera: {dev_info.GetFriendlyName()}')

        self.hCamera = mvsdk.CameraInit(dev_info, -1, -1)

        cap = mvsdk.CameraGetCapability(self.hCamera)

        # Choose output format: mono sensors → MONO8, colour sensors → BGR8
        if cap.sIspCapacity.bMonoSensor:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
            self.mono = True
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
            self.mono = False

        # Continuous capture, manual exposure 30 ms
        mvsdk.CameraSetTriggerMode(self.hCamera, 0)
        mvsdk.CameraSetAeState(self.hCamera, 0)
        mvsdk.CameraSetExposureTime(self.hCamera, 30 * 1000)

        mvsdk.CameraPlay(self.hCamera)

        # Pre-allocate frame buffer (worst-case = max resolution × 3 channels)
        buf_size = (
            cap.sResolutionRange.iWidthMax
            * cap.sResolutionRange.iHeightMax
            * 3
        )
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

        self.get_logger().info('Camera initialised and streaming.')

    # ── Capture service ──────────────────────────────────────────────────────────────────────────────
    def _capture_cb(self, request, response):
        """Grab one fresh frame and publish it immediately."""
        if not self.camera_ready:
            response.success = False
            response.message = 'Camera not ready'
            return response
        ok = self._publish_frame(timeout_ms=2000)
        response.success = ok
        response.message = '' if ok else 'Frame grab timed out'
        return response

    # ── Per-frame callback ─────────────────────────────────────────────────────────────────────────────
    def _publish_frame(self, timeout_ms=200):
        """Grab one frame, convert it, and publish it as a ROS2 Image message.
        Returns True if a frame was successfully published, False otherwise."""
        if not self.camera_ready:
            return False
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, timeout_ms)

            # Run ISP pipeline (demosaic, white balance, etc.) → pFrameBuffer
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)

            # Release the raw buffer back to the SDK
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            # Wrap the processed buffer as a NumPy array
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)

            if self.mono:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 3))

            # Build and publish ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mv_camera'
            self.pub.publish(msg)
            return True

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                self.get_logger().warn(f'Camera SDK error: {e}')
            return False

    # ── Clean shutdown ──────────────────────────────────────────────────────────────────────────────
    def destroy_node(self):
        """Stop the camera and free SDK resources before shutting down."""
        self.get_logger().info('Shutting down camera...')
        mvsdk.CameraStop(self.hCamera)
        mvsdk.CameraAlignFree(self.pFrameBuffer)
        mvsdk.CameraUnInit(self.hCamera)
        super().destroy_node()


# ══════════════════════════════════════════════════════════════════════════════
#  Entry point
# ══════════════════════════════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = MVCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
