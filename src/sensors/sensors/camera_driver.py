import cv2
import rclpy
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class CameraDriver(Node):
    def __init__(self):
        super().__init__("camera_driver")

        self.declare_parameter("device",          "/dev/video0")
        self.declare_parameter("topic",           "/image_raw")
        self.declare_parameter("frame_id",        "front_camera")
        self.declare_parameter("camera_info_url", "")

        device          = self.get_parameter("device").get_parameter_value().string_value
        topic           = self.get_parameter("topic").get_parameter_value().string_value
        self._frame_id  = self.get_parameter("frame_id").get_parameter_value().string_value
        camera_info_url = self.get_parameter("camera_info_url").get_parameter_value().string_value

        self._available = False
        self.cap = cv2.VideoCapture(device)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            self.get_logger().warn(f"Cannot open {device} — running in degraded mode")
        else:
            self._available = True
            self.get_logger().info(f"Camera opened: {device} → {topic}")

        self._cim = CameraInfoManager(self, cname=self._frame_id, url=camera_info_url)
        if camera_info_url:
            self._cim.loadCameraInfo()

        self.pub      = self.create_publisher(Image,      topic,                  10)
        self.info_pub = self.create_publisher(CameraInfo, topic + "/camera_info", 10)
        self.bridge   = CvBridge()
        self.create_timer(1 / 30, self._cb)

    def _cb(self):
        if not self._available:
            return
        ret, frame = self.cap.read()
        if not ret:
            return
        stamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp    = stamp
        msg.header.frame_id = self._frame_id
        self.pub.publish(msg)

        try:
            info = self._cim.getCameraInfo()
        except Exception:
            from sensor_msgs.msg import CameraInfo
            info = CameraInfo()
            h, w = frame.shape[:2]
            info.height = h
            info.width = w
        self.info_pub.publish(info)

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
