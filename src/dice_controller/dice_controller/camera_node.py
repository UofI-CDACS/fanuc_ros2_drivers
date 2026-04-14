#!/usr/bin/env python3
"""
Camera node for dice pip detection.

Responsibilities
----------------
- Continuously reads frames from the overhead USB camera and publishes them as
  sensor_msgs/Image on /{robot_name}/camera/image_raw.
- Exposes a std_srvs/Trigger service on /{robot_name}/camera/capture.
  When called it saves the current frame to disk, runs pip counting, and returns
  the result packed into the Trigger.response.message field as:
      "<pip_count>|<absolute_image_path>"

Pip counting
------------
Uses OpenCV's HoughCircles on a grayscale-blurred image.  The parameters below
(MIN_PIP_RADIUS, MAX_PIP_RADIUS, etc.) MUST be tuned once you know the camera's
field-of-view and the physical size of the dice at the camera position.
"""

import datetime
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

# ──────────────────────────────────────────────────────────────────────────────
# Pip-detection tuning parameters — adjust to your camera / dice setup
# ──────────────────────────────────────────────────────────────────────────────
HOUGH_DP         = 1.2   # inverse ratio of accumulator resolution to image resolution
HOUGH_MIN_DIST   = 15    # min pixels between pip centres
HOUGH_PARAM1     = 50    # upper Canny threshold
HOUGH_PARAM2     = 20    # accumulator threshold (lower = more false positives)
MIN_PIP_RADIUS   = 5     # pixels
MAX_PIP_RADIUS   = 25    # pixels
BLUR_KERNEL      = (7, 7)
# ──────────────────────────────────────────────────────────────────────────────


def count_pips(image: np.ndarray) -> int:
    """
    Count the pips (dots) on a dice face.

    Parameters
    ----------
    image : BGR image (numpy array)

    Returns
    -------
    int  Number of detected pips (0 if none found)
    """
    gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=HOUGH_DP,
        minDist=HOUGH_MIN_DIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=MIN_PIP_RADIUS,
        maxRadius=MAX_PIP_RADIUS,
    )
    if circles is None:
        return 0
    return len(circles[0])


def annotate_image(image: np.ndarray, pip_count: int) -> np.ndarray:
    """Draw detected circles and pip count onto a copy of the image."""
    annotated = image.copy()
    gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, BLUR_KERNEL, 0)
    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT,
        dp=HOUGH_DP,
        minDist=HOUGH_MIN_DIST,
        param1=HOUGH_PARAM1,
        param2=HOUGH_PARAM2,
        minRadius=MIN_PIP_RADIUS,
        maxRadius=MAX_PIP_RADIUS,
    )
    if circles is not None:
        for (x, y, r) in np.round(circles[0]).astype(int):
            cv2.circle(annotated, (x, y), r, (0, 255, 0), 2)
            cv2.circle(annotated, (x, y), 2, (0, 0, 255), 3)

    cv2.putText(
        annotated,
        f'Pips: {pip_count}',
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
    )
    return annotated


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_name',   'noNAME'),
                ('camera_index', 0),          # /dev/video<N> index or 0 for default
                ('save_dir',     '/tmp/dice_images'),
                ('publish_hz',   10.0),
            ],
        )

        robot_name  = self.get_parameter('robot_name').value
        cam_idx     = self.get_parameter('camera_index').value
        self.save_dir   = self.get_parameter('save_dir').value
        publish_hz  = self.get_parameter('publish_hz').value

        os.makedirs(self.save_dir, exist_ok=True)
        self.bridge = CvBridge()
        self._latest_frame: np.ndarray | None = None

        # Open camera
        self.cap = cv2.VideoCapture(cam_idx)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open camera at index {cam_idx}. '
                'Check camera_index parameter and cable connection.'
            )

        # Publisher — raw frames for visualisation / debugging
        self.image_pub = self.create_publisher(
            Image, f'/{robot_name}/camera/image_raw', 10)

        # Service — called by the master node when the robot is in position
        self.capture_srv = self.create_service(
            Trigger,
            f'/{robot_name}/camera/capture',
            self._capture_callback,
        )

        # Timer — continuously reads and publishes camera frames
        self.create_timer(1.0 / publish_hz, self._publish_frame)

        self.get_logger().info(
            f'Camera node started — publishing on /{robot_name}/camera/image_raw, '
            f'capture service at /{robot_name}/camera/capture'
        )

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _publish_frame(self):
        """Timer callback: grab the latest camera frame and publish it."""
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Camera read failed', throttle_duration_sec=5.0)
            return
        self._latest_frame = frame
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

    def _capture_callback(self, _request, response: Trigger.Response):
        """
        Service handler: save current frame, count pips, return result.

        Response message format: "<pip_count>|<absolute_path_to_annotated_image>"
        """
        if self._latest_frame is None:
            response.success = False
            response.message = '0|'
            self.get_logger().error('Capture requested but no frame is available yet')
            return response

        pip_count = count_pips(self._latest_frame)
        annotated = annotate_image(self._latest_frame, pip_count)

        timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        raw_path  = os.path.join(self.save_dir, f'dice_{timestamp}_raw.png')
        ann_path  = os.path.join(self.save_dir, f'dice_{timestamp}_annotated.png')
        cv2.imwrite(raw_path, self._latest_frame)
        cv2.imwrite(ann_path, annotated)

        response.success = True
        response.message = f'{pip_count}|{ann_path}'
        self.get_logger().info(f'Captured — pips: {pip_count}  saved: {ann_path}')
        return response

    # ── teardown ──────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
