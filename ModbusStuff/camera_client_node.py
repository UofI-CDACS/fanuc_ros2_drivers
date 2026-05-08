"""
Camera client node.

Connects to the shared camera_node (hosted by whoever has the camera).
- Subscribes to /camera/image_raw  — shows live 3 fps window
- Calls       /camera/count_pips  — on demand (SPACE) or auto every N seconds
- Publishes   /camera/pip_count   — Int32, so both robots can read the result

Both robots run this node. Only one robot runs camera_node.

Usage:
    # Source ROS2 first
    source /opt/ros/jazzy/setup.bash && source install/setup.bash

    python3 ModbusStuff/camera_client_node.py

    # If camera_node is on a different machine, make sure ROS_DOMAIN_ID matches
    # and both machines are on the same network segment.

Controls (in the OpenCV window):
  SPACE  — call count_pips now and publish result
  q/Esc  — quit
"""

import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from fanuc_interfaces.srv import CountPips
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

WINDOW = 'Camera Feed  (SPACE=count pips  q=quit)'
TARGET_FPS = 3


class CameraClientNode(Node):

    def __init__(self):
        super().__init__('camera_client')

        # Subscribe to camera image stream
        self._bridge       = CvBridge()
        self._latest_frame = None
        self._frame_lock   = threading.Lock()
        self.create_subscription(Image, '/camera/image_raw', self._image_cb, 10)

        # Service client for pip counting
        self._pip_client = self.create_client(CountPips, '/camera/count_pips')

        # Publish pip count so both robots can read it
        self._pip_pub = self.create_publisher(Int32, '/camera/pip_count', 10)

        self._pip_label  = 'SPACE to count pips'
        self._pip_colour = (180, 180, 180)

        self.get_logger().info(
            'Camera client ready.\n'
            '  Waiting for /camera/image_raw ...\n'
            '  Make sure camera_node is running on one of the machines.'
        )

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self._frame_lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().warn(f'Image decode error: {e}')

    def _get_frame(self):
        with self._frame_lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    # ── Pip count request ─────────────────────────────────────────────────────

    def request_pip_count(self):
        if not self._pip_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('count_pips service not available')
            self._pip_label  = 'camera_node not reachable'
            self._pip_colour = (0, 0, 220)
            return

        self.get_logger().info('Calling /camera/count_pips ...')
        fut = self._pip_client.call_async(CountPips.Request())

        # Wait without blocking the executor
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)

        if not fut.done():
            self.get_logger().warn('count_pips timed out')
            self._pip_label  = 'Timeout'
            self._pip_colour = (0, 0, 220)
            return

        res = fut.result()
        if res and res.success:
            count = res.pip_count
            self._pip_label  = f'Pips: {count}'
            self._pip_colour = (0, 220, 0)
            self.get_logger().info(f'Pip count: {count}  ({res.message})')

            # Publish so partner can read it
            msg = Int32()
            msg.data = count
            self._pip_pub.publish(msg)
        else:
            msg_str = res.message if res else 'no response'
            self._pip_label  = f'Failed: {msg_str}'
            self._pip_colour = (0, 0, 220)
            self.get_logger().warn(f'count_pips failed: {msg_str}')

    # ── Display loop (runs on main thread) ────────────────────────────────────

    def run_display(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 900, 700)

        interval  = 1.0 / TARGET_FPS
        last_draw = 0.0

        print(f'\nWindow open — SPACE to count pips, q/Esc to quit\n')

        while True:
            now = time.monotonic()

            # Throttle display to TARGET_FPS
            if now - last_draw >= interval:
                frame = self._get_frame()
                if frame is not None:
                    display = frame.copy()
                else:
                    display = _waiting_frame()

                # Pip label
                cv2.putText(display, self._pip_label,
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX,
                            1.6, self._pip_colour, 3, cv2.LINE_AA)
                # FPS tag
                cv2.putText(display, f'{TARGET_FPS} fps',
                            (display.shape[1] - 90, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (160, 160, 160), 1)

                cv2.imshow(WINDOW, display)
                last_draw = now

            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                # Run in background thread so display doesn't freeze
                threading.Thread(target=self.request_pip_count, daemon=True).start()

            elif key in (ord('q'), 27):
                break

        cv2.destroyAllWindows()


def _waiting_frame():
    """Black frame shown before the first image arrives."""
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, 'Waiting for /camera/image_raw ...',
                (30, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (160, 160, 160), 1)
    return img


def main():
    import numpy as np   # imported here so the error message above still prints
    global np

    rclpy.init()
    node = CameraClientNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        node.run_display()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import numpy as np
    main()
