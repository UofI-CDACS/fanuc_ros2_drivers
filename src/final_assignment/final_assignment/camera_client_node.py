"""
Camera client node — final assignment.

Connects to the shared camera_node (hosted by whoever has the camera).
  - Subscribes to /camera/image_raw      live 3 fps OpenCV window
  - Calls       /camera/count_pips       on SPACE keypress
  - Publishes   /camera/pip_count        Int32, readable by both robots

Both robots run this node. Only one robot runs camera_node.

Controls (OpenCV window):
  SPACE  — request pip count and publish result
  q/Esc  — quit
"""

import os
import threading
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from fanuc_interfaces.srv import CountPips
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

WINDOW     = 'Camera Feed  (SPACE=count pips  q=quit)'
TARGET_FPS = 3

# Only attempt GUI if a display is available (avoids Qt SIGABRT when headless)
HAS_DISPLAY = bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


class CameraClientNode(Node):

    def __init__(self):
        super().__init__('camera_client')

        self._bridge       = CvBridge()
        self._latest_frame = None
        self._frame_lock   = threading.Lock()

        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10
        )

        self._pip_client = self.create_client(CountPips, '/camera/count_pips')
        self._pip_pub    = self.create_publisher(Int32, '/camera/pip_count', 10)

        self._pip_label  = 'Scanning...'
        self._pip_colour = (180, 180, 180)
        self._scan_busy  = False

        # Auto-scan every 0.5 s so /camera/pip_count is always current
        self.create_timer(0.5, self._scan_timer_cb)

        if HAS_DISPLAY:
            self.get_logger().info(
                'Camera client started (with display) — '
                'waiting for /camera/image_raw'
            )
        else:
            self.get_logger().info(
                'Camera client started (headless — no DISPLAY found). '
                'Subscribing and publishing /camera/pip_count. '
                'Run standalone for the live window.'
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

    # ── Auto-scan timer ───────────────────────────────────────────────────────

    def _scan_timer_cb(self):
        if self._scan_busy:
            return
        self._scan_busy = True
        threading.Thread(target=self._scan_and_release, daemon=True).start()

    def _scan_and_release(self):
        try:
            self.request_pip_count()
        finally:
            self._scan_busy = False

    # ── Pip count ─────────────────────────────────────────────────────────────

    def request_pip_count(self):
        if not self._pip_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/camera/count_pips not available')
            self._pip_label  = 'camera_node not reachable'
            self._pip_colour = (0, 0, 220)
            return

        self.get_logger().info('Calling /camera/count_pips ...')
        fut      = self._pip_client.call_async(CountPips.Request())
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)

        if not fut.done():
            self._pip_label  = 'Timeout'
            self._pip_colour = (0, 0, 220)
            self.get_logger().warn('count_pips timed out')
            return

        res = fut.result()
        if res and res.success:
            count            = res.pip_count
            self._pip_label  = f'Pips: {count}'
            self._pip_colour = (0, 220, 0)
            self.get_logger().info(f'Pip count: {count}')
            msg      = Int32()
            msg.data = count
            self._pip_pub.publish(msg)
        else:
            detail           = res.message if res else 'no response'
            self._pip_label  = f'Failed: {detail}'
            self._pip_colour = (0, 0, 220)
            self.get_logger().warn(f'count_pips failed: {detail}')

    # ── Display loop (main thread, only when display available) ──────────────

    def run_display(self):
        if not HAS_DISPLAY:
            # Headless — just keep the node alive; spin thread handles callbacks
            self.get_logger().info('Running headless. Spin to keep alive.')
            try:
                while rclpy.ok():
                    time.sleep(1.0)
            except KeyboardInterrupt:
                pass
            return

        import cv2  # only import when display is available
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 900, 700)

        interval  = 1.0 / TARGET_FPS
        last_draw = 0.0

        while True:
            now = time.monotonic()
            if now - last_draw >= interval:
                frame   = self._get_frame()
                display = frame if frame is not None else _waiting_frame()

                cv2.putText(display, self._pip_label,
                            (10, 55), cv2.FONT_HERSHEY_SIMPLEX,
                            1.6, self._pip_colour, 3, cv2.LINE_AA)
                cv2.putText(display, f'{TARGET_FPS} fps',
                            (display.shape[1] - 90, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (160, 160, 160), 1)
                cv2.imshow(WINDOW, display)
                last_draw = now

            key = cv2.waitKey(1) & 0xFF
            if key == ord(' '):
                threading.Thread(
                    target=self.request_pip_count, daemon=True
                ).start()
            elif key in (ord('q'), 27):
                break

        cv2.destroyAllWindows()


def _waiting_frame():
    import cv2
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, 'Waiting for /camera/image_raw ...',
                (30, 240), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (160, 160, 160), 1)
    return img


def main(args=None):
    rclpy.init(args=args)
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
    main()
