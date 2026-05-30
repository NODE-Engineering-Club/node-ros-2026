import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField

OBSTACLE_RANGE = 10.0


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_node")
        self.pub = self.create_publisher(PointCloud2, "/obstacles/lidar", 10)
        self.create_subscription(LaserScan, "/lidar_driver/scan_raw", self._cb, 10)

    def _cb(self, scan):
        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < min(scan.range_max, OBSTACLE_RANGE):
                points.append((r * math.cos(angle), r * math.sin(angle), 0.0))
            angle += scan.angle_increment

        msg = PointCloud2()
        msg.header = scan.header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(points)
        msg.data = b"".join(struct.pack("fff", *p) for p in points)
        msg.is_dense = True
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
