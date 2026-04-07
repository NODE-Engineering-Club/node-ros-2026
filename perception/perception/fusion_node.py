import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from vision_msgs.msg import Detection2DArray

IMAGE_WIDTH = 640
CAMERA_HFOV = math.radians(60)
# Conservative distance when depth is unknown
DEFAULT_OBSTACLE_DISTANCE = 5.0


class FusionNode(Node):
    def __init__(self):
        super().__init__("fusion_node")

        self._lidar_pts: list[tuple[float, float, float]] = []
        self._vision_pts: list[tuple[float, float, float]] = []

        self.pub = self.create_publisher(PointCloud2, "/obstacles/fused", 10)
        self.create_subscription(PointCloud2, "/obstacles/lidar", self._lidar_cb, 10)
        self.create_subscription(Detection2DArray, "/yolo/detections", self._det_cb, 10)
        self.create_timer(0.1, self._publish)  # 10 Hz

    def _lidar_cb(self, msg):
        pts = []
        for i in range(msg.width):
            x, y, z = struct.unpack_from("fff", msg.data, i * msg.point_step)
            pts.append((x, y, z))
        self._lidar_pts = pts

    def _det_cb(self, msg):
        pts = []
        for det in msg.detections:
            # Estimate bearing from bbox center, project at conservative range
            bearing = (det.bbox.center.x / IMAGE_WIDTH - 0.5) * CAMERA_HFOV
            x = DEFAULT_OBSTACLE_DISTANCE * math.cos(bearing)
            y = DEFAULT_OBSTACLE_DISTANCE * math.sin(bearing)
            pts.append((x, y, 0.0))
        self._vision_pts = pts

    def _publish(self):
        fused = self._lidar_pts + self._vision_pts
        if not fused:
            return

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.height = 1
        msg.width = len(fused)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(fused)
        msg.data = b"".join(struct.pack("fff", *p) for p in fused)
        msg.is_dense = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
