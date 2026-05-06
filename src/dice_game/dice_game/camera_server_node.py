"""
camera_server_node.py  —  runs on Robot 1 (Beaker)'s machine.

Offers the /dice_game/capture_image service (fanuc_interfaces/CaptureImage).
Only ONE instance of this node runs for the whole game; both robot controllers
call it as clients.  A threading.Lock() serialises concurrent requests so only
one capture happens at a time.

Camera discovery:
    Uses mvsdk.CameraEnumerateDevice() to find all MindVision cameras on
    USB and GigE without needing a fixed IP.  The found camera's IP (if GigE)
    is logged at startup.  If the camera IP changes, just restart this node —
    no config edit needed.
"""

import sys
import threading
import numpy as np
import rclpy
from rclpy.node import Node

# mvsdk and camera.py live in the existing fanuc_ros2_drivers folder.
# Adjust this path if your lab machine has them elsewhere.
MVSDK_PATH = '/home/colin/Desktop/fanuc_ros2_drivers'
if MVSDK_PATH not in sys.path:
    sys.path.insert(0, MVSDK_PATH)

import mvsdk  # noqa: E402  (must come after sys.path insert)
from camera import Camera  # noqa: E402

from fanuc_interfaces.srv import CaptureImage

# Known camera IP — checked first before falling back to auto-discovery
CAMERA_IP = '10.8.4.210'


class CameraServerNode(Node):

    def __init__(self):
        super().__init__('dice_camera_server')
        self._lock = threading.Lock()

        # ── Connect to camera ─────────────────────────────────────────────────
        self.get_logger().info(f'Looking for camera at {CAMERA_IP}...')
        self._camera = Camera(camera_ip=CAMERA_IP)

        if self._camera.hCamera is None:
            self.get_logger().warn(
                f'Camera not found at {CAMERA_IP} — falling back to auto-discovery'
            )
            self._camera = Camera(camera_ip=None)

        if self._camera.hCamera is not None:
            self.get_logger().info('Camera ready.')
        else:
            self.get_logger().error('No camera found — capture requests will fail.')

        # ── Service server ────────────────────────────────────────────────────
        self._srv = self.create_service(
            CaptureImage,
            '/dice_game/capture_image',
            self._handle_capture,
        )
        self.get_logger().info('Camera server ready on /dice_game/capture_image')

    def _handle_capture(self, request, response):
        with self._lock:
            try:
                frame = self._camera.getFrame()          # numpy BGR array
                h, w = frame.shape[:2]
                channels = 1 if frame.ndim == 2 else frame.shape[2]
                response.image_data = frame.flatten().tolist()
                response.width      = w
                response.height     = h
                response.channels   = channels
                response.success    = True
                response.message    = 'ok'
            except Exception as exc:
                response.success = False
                response.message = str(exc)
                self.get_logger().error(f'Capture failed: {exc}')
        return response

    def destroy_node(self):
        self._camera.disable()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
