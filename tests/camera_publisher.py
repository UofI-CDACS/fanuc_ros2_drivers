"""
Publishes MindVision GigE camera frames as sensor_msgs/Image on /{robot_name}/camera_feed.

Run this in a separate terminal BEFORE running send_pose.py:
    python3 tests/camera_publisher.py --robot-name <NAME>

Requires libMVSDK.so to be installed and the MindVision GigE camera to be reachable.
Set ROBOT_NAME in tests/.env or pass --robot-name on the command line.
"""

import argparse
import os
import platform
import sys

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

# mvsdk.py must be in the same directory as this script
sys.path.insert(0, __file__.rsplit("/", 1)[0])
import mvsdk

# Load .env from the tests/ directory if present
_env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
if os.path.isfile(_env_path):
    with open(_env_path) as _f:
        for _line in _f:
            _line = _line.strip()
            if _line and not _line.startswith("#") and "=" in _line:
                _k, _v = _line.split("=", 1)
                os.environ.setdefault(_k.strip(), _v.strip())

ROBOT_NAME  = os.getenv("ROBOT_NAME", "")
PUBLISH_HZ  = 10       # frames per second
EXPOSURE_US = 60_000   # exposure time in microseconds


class CameraPublisher(Node):
    def __init__(self, robot_name):
        super().__init__("camera_publisher")
        self.br = CvBridge()

        topic = f"/{robot_name}/camera_feed"
        self.pub = self.create_publisher(Image, topic, 10)

        self._open_camera()

        self.get_logger().info(f"Publishing to: {topic}  ({PUBLISH_HZ} Hz)")
        self.create_timer(1.0 / PUBLISH_HZ, self._publish_frame)

    def _open_camera(self):
        DevList = mvsdk.CameraEnumerateDevice()
        if len(DevList) < 1:
            self.get_logger().error(
                "No MindVision camera found. "
                "Check that the camera is powered and reachable on the network."
            )
            sys.exit(1)

        DevInfo = DevList[0]
        self.get_logger().info(f"Opening camera: {DevInfo.GetFriendlyName()}")

        self.hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        cap = mvsdk.CameraGetCapability(self.hCamera)

        self.monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
        fmt = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if self.monoCamera else mvsdk.CAMERA_MEDIA_TYPE_BGR8
        mvsdk.CameraSetIspOutFormat(self.hCamera, fmt)

        mvsdk.CameraSetTriggerMode(self.hCamera, 0)    # continuous mode
        mvsdk.CameraSetAeState(self.hCamera, 0)        # manual exposure
        mvsdk.CameraSetExposureTime(self.hCamera, EXPOSURE_US)
        mvsdk.CameraPlay(self.hCamera)

        buf_size = (cap.sResolutionRange.iWidthMax *
                    cap.sResolutionRange.iHeightMax *
                    (1 if self.monoCamera else 3))
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(buf_size, 16)

    def _publish_frame(self):
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(self.pFrameBuffer, FrameHead, 1)

            channels = 1 if self.monoCamera else 3
            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8).reshape(
                (FrameHead.iHeight, FrameHead.iWidth, channels)
            ).copy()

            encoding = "mono8" if self.monoCamera else "bgr8"
            msg = self.br.cv2_to_imgmsg(frame, encoding=encoding)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.pub.publish(msg)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                self.get_logger().warning(f"Camera grab failed ({e.error_code}): {e.message}")

    def destroy_node(self):
        mvsdk.CameraUnInit(self.hCamera)
        mvsdk.CameraAlignFree(self.pFrameBuffer)
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(description="Publish MindVision GigE camera to ROS2")
    parser.add_argument("--robot-name", default=ROBOT_NAME,
                        help=f"Robot namespace (default: {ROBOT_NAME})")
    args = parser.parse_args()

    rclpy.init()
    node = CameraPublisher(args.robot_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
