"""
HSV tuner node — final assignment.

Subscribes to /camera/image_raw, runs pip detection locally with HSV bounds and
size thresholds adjustable via OpenCV trackbars, and publishes the resulting
count on /camera/pip_count once per second.

Use this to dial in HSV_LOWER / HSV_UPPER and the size thresholds, then copy
the values back into dice_pipeline/camera_node.py.

Controls (OpenCV window):
  s      — print current slider values to console
  q/Esc  — quit
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

WINDOW_TUNER = 'HSV Tuner  (s=print values  q=quit)'
WINDOW_MASK  = 'Mask'

DEFAULT_HSV_LOWER = (7, 141, 53)
DEFAULT_HSV_UPPER = (18, 255, 135)
DEFAULT_DIE_AREA  = 5000
DEFAULT_PIP_LO    = 200
DEFAULT_PIP_HI    = 1000


class HsvTunerNode(Node):

    def __init__(self):
        super().__init__('hsv_tuner')

        self._bridge       = CvBridge()
        self._latest_frame = None
        self._frame_lock   = threading.Lock()
        self._last_count   = 0

        self.create_subscription(
            Image, '/camera/image_raw', self._image_cb, 10
        )
        self._pip_pub = self.create_publisher(Int32, '/camera/pip_count', 10)

        self.create_timer(1.0, self._publish_count)

        self.get_logger().info(
            'HSV tuner started — waiting for /camera/image_raw'
        )

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

    def _publish_count(self):
        msg = Int32()
        msg.data = self._last_count
        self._pip_pub.publish(msg)

    def set_count(self, count: int):
        self._last_count = count


def _make_trackbars():
    cv2.namedWindow(WINDOW_TUNER, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_TUNER, 1000, 700)

    def nop(_):
        pass

    cv2.createTrackbar('H min', WINDOW_TUNER, DEFAULT_HSV_LOWER[0], 179, nop)
    cv2.createTrackbar('S min', WINDOW_TUNER, DEFAULT_HSV_LOWER[1], 255, nop)
    cv2.createTrackbar('V min', WINDOW_TUNER, DEFAULT_HSV_LOWER[2], 255, nop)
    cv2.createTrackbar('H max', WINDOW_TUNER, DEFAULT_HSV_UPPER[0], 179, nop)
    cv2.createTrackbar('S max', WINDOW_TUNER, DEFAULT_HSV_UPPER[1], 255, nop)
    cv2.createTrackbar('V max', WINDOW_TUNER, DEFAULT_HSV_UPPER[2], 255, nop)
    cv2.createTrackbar('Die area min', WINDOW_TUNER, DEFAULT_DIE_AREA, 50000, nop)
    cv2.createTrackbar('Pip area min', WINDOW_TUNER, DEFAULT_PIP_LO, 5000, nop)
    cv2.createTrackbar('Pip area max', WINDOW_TUNER, DEFAULT_PIP_HI, 10000, nop)


def _read_trackbars():
    lower = np.array([
        cv2.getTrackbarPos('H min', WINDOW_TUNER),
        cv2.getTrackbarPos('S min', WINDOW_TUNER),
        cv2.getTrackbarPos('V min', WINDOW_TUNER),
    ])
    upper = np.array([
        cv2.getTrackbarPos('H max', WINDOW_TUNER),
        cv2.getTrackbarPos('S max', WINDOW_TUNER),
        cv2.getTrackbarPos('V max', WINDOW_TUNER),
    ])
    die_lo = cv2.getTrackbarPos('Die area min', WINDOW_TUNER)
    pip_lo = cv2.getTrackbarPos('Pip area min', WINDOW_TUNER)
    pip_hi = cv2.getTrackbarPos('Pip area max', WINDOW_TUNER)
    return lower, upper, die_lo, pip_lo, pip_hi


def _detect(image, hsv_lo, hsv_hi, die_lo, pip_lo, pip_hi):
    hsv  = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, hsv_lo, hsv_hi)

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    annotated  = image.copy()
    total_pips = 0
    die_num    = 0

    for contour in contours:
        if cv2.contourArea(contour) <= die_lo:
            continue

        die_num += 1
        x, y, w, h = cv2.boundingRect(contour)

        die_face = mask[y:y + h, x:x + w].copy()
        pip_contours, _ = cv2.findContours(
            die_face, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        num_pips = 0
        for pip in pip_contours:
            area = cv2.contourArea(pip)
            if pip_lo < area < pip_hi:
                num_pips += 1
                px, py, pw, ph = cv2.boundingRect(pip)
                cv2.rectangle(
                    annotated,
                    (x + px, y + py), (x + px + pw, y + py + ph),
                    (0, 0, 255), 2,
                )

        total_pips += num_pips

        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.putText(
            annotated, f'die{die_num}: {num_pips}',
            (x, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )

    cv2.putText(
        annotated, f'Pips: {total_pips}',
        (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2,
    )
    return total_pips, annotated, mask


def _waiting_frame():
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, 'Waiting for /camera/image_raw ...',
                (30, 240), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (160, 160, 160), 1)
    return img


def run_display(node: HsvTunerNode):
    _make_trackbars()
    cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_MASK, 600, 480)

    while rclpy.ok():
        frame = node._get_frame()
        hsv_lo, hsv_hi, die_lo, pip_lo, pip_hi = _read_trackbars()

        if frame is None:
            cv2.imshow(WINDOW_TUNER, _waiting_frame())
            cv2.imshow(WINDOW_MASK, np.zeros((480, 640), dtype=np.uint8))
        else:
            count, annotated, mask = _detect(
                frame, hsv_lo, hsv_hi, die_lo, pip_lo, pip_hi
            )
            node.set_count(count)
            cv2.imshow(WINDOW_TUNER, annotated)
            cv2.imshow(WINDOW_MASK, mask)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord('q'), 27):
            break
        if key == ord('s'):
            print(
                f'HSV_LOWER = {list(hsv_lo)}\n'
                f'HSV_UPPER = {list(hsv_hi)}\n'
                f'DIE_SIZE_LOWER = {die_lo}\n'
                f'PIP_SIZE_LOWER = {pip_lo}\n'
                f'PIP_SIZE_UPPER = {pip_hi}'
            )

    cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = HsvTunerNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        run_display(node)
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
