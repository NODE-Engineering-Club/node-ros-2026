import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")

        self.declare_parameter("device", "/dev/video0")
        self.declare_parameter("topic", "/image_raw")

        device = self.get_parameter("device").get_parameter_value().string_value
        topic  = self.get_parameter("topic").get_parameter_value().string_value

        self._available = False
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().warn(f"Cannot open {device} — running in degraded mode")
        else:
            self._available = True
            self.get_logger().info(f"Camera opened: {device} → {topic}")

        self.pub    = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.create_timer(1 / 30, self._cb)

    def _cb(self):
        if not self._available:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp    = self.get_clock().now().to_msg()
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
