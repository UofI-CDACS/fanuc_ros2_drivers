import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2


class CameraTest(Node):
    def __init__(self):
        super().__init__('camera_test')
        self._bridge = CvBridge()
        self._frame = None

        self._sub = self.create_subscription(Image, 'camera/image_raw', self._on_frame, 10)
        self._client = self.create_client(Trigger, 'camera/capture')

    def request_frame(self):
        self.get_logger().info('Waiting for /camera/capture service...')
        self._client.wait_for_service()

        future = self._client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if not result.success:
            self.get_logger().error(f'Capture failed: {result.message}')
            return None

        # Spin briefly to receive the published frame
        deadline = self.get_clock().now().nanoseconds + int(2e9)  # 2 second timeout
        while self._frame is None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > deadline:
                self.get_logger().error('Timed out waiting for frame')
                return None

        return self._frame

    def _on_frame(self, msg: Image):
        self._frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def main(args=None):
    rclpy.init(args=args)
    node = CameraTest()

    frame = node.request_frame()
    if frame is not None:
        cv2.imshow('Camera Frame', frame)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
