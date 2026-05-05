#!/usr/bin/env python3
"""
mv_camera_node.py
-----------------
ROS2 node that captures frames from a MindVision GigE camera using the
MindVision SDK (mvsdk) and publishes them as sensor_msgs/Image messages.

Publishes:
  /mv_camera/image_raw  (sensor_msgs/msg/Image, encoding: bgr8)

Services:
  /mv_camera/capture    (std_srvs/srv/Trigger) — grab one fresh frame on demand
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
        'Ensure mvsdk.py is on your PYTHONPATH (via the .venv overlay).'
    )

PUBLISH_TOPIC   = '/mv_camera/image_raw'
PUBLISH_RATE_HZ = 30.0


class MVCameraNode(Node):

    def __init__(self):
        super().__init__('mv_camera_node')

        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, PUBLISH_TOPIC, 10)

        try:
            self._init_camera()
            self.camera_ready = True
        except RuntimeError as e:
            self.get_logger().warn(f'Camera not available: {e}. Node will stay alive but publish nothing.')
            self.camera_ready = False

        self.create_service(Trigger, '/mv_camera/capture', self._capture_cb)
        self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish_frame)

        self.get_logger().info(f'MVCameraNode started — publishing on {PUBLISH_TOPIC} at {PUBLISH_RATE_HZ} Hz')

    def _init_camera(self):
        dev_list = mvsdk.CameraEnumerateDevice()
        if len(dev_list) == 0:
            raise RuntimeError('No MindVision cameras detected. Check GigE connection and adapter IP.')

        # When the host has multiple NICs, the SDK enumerates the camera once per route
        # ('<camera_ip>-<host_ip>'). Prefer the entry whose host_ip shares a /16 with the
        # camera_ip (the route that actually carries data). Fall back to [0].
        dev_info = dev_list[0]
        for d in dev_list:
            port = d.GetPortType()
            if '-' not in port:
                continue
            cam_ip, host_ip = port.split('-', 1)
            cam_oct, host_oct = cam_ip.split('.'), host_ip.split('.')
            if len(cam_oct) >= 2 and len(host_oct) >= 2 and cam_oct[:2] == host_oct[:2]:
                dev_info = d
                break
        self.get_logger().info(f'Opening camera: {dev_info.GetFriendlyName()} via {dev_info.GetPortType()}')

        self.hCamera = mvsdk.CameraInit(dev_info, -1, -1)
        cap = mvsdk.CameraGetCapability(self.hCamera)

        if cap.sIspCapacity.bMonoSensor:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
            self.mono = True
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
            self.mono = False

        mvsdk.CameraSetTriggerMode(self.hCamera, 0)   # continuous
        mvsdk.CameraSetAeState(self.hCamera, 0)        # manual exposure
        mvsdk.CameraSetExposureTime(self.hCamera, 30 * 1000)  # 30 ms

        mvsdk.CameraPlay(self.hCamera)

        buf_size = (cap.sResolutionRange.iWidthMax
                    * cap.sResolutionRange.iHeightMax * 3)
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

        self.get_logger().info('Camera initialised and streaming.')

    def _capture_cb(self, request, response):
        if not self.camera_ready:
            response.success = False
            response.message = 'Camera not ready'
            return response
        ok = self._publish_frame(timeout_ms=2000)
        response.success = ok
        response.message = '' if ok else 'Frame grab timed out'
        return response

    def _publish_frame(self, timeout_ms=200):
        if not self.camera_ready:
            return False
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, timeout_ms)
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)

            if self.mono:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth))
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            else:
                frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 3))

            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'mv_camera'
            self.pub.publish(msg)
            return True

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                self.get_logger().warn(f'Camera SDK error: {e}')
            return False

    def destroy_node(self):
        self.get_logger().info('Shutting down camera...')
        if self.camera_ready:
            mvsdk.CameraStop(self.hCamera)
            mvsdk.CameraAlignFree(self.pFrameBuffer)
            mvsdk.CameraUnInit(self.hCamera)
        super().destroy_node()


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
