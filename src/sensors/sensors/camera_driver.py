import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

DEVICE = "/dev/video0"


class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")

        self._available = False
        self.cap = cv2.VideoCapture(DEVICE)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().warn(
                f"Cannot open {DEVICE} — camera_driver running in degraded mode (no frames published)"
            )
        else:
            self._available = True

        self.pub = self.create_publisher(Image, "/image_raw", 10)
        self.bridge = CvBridge()
        self.create_timer(1 / 30, self._cb)  # 30 Hz

    def _cb(self):
        if not self._available:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        self.pub.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
