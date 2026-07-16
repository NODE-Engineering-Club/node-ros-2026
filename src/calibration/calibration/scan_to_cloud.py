import rclpy
from laser_geometry import LaserProjection
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2


class ScanToCloud(Node):
    def __init__(self):
        super().__init__("scan_to_cloud")
        self._projector = LaserProjection()
        self.pub = self.create_publisher(PointCloud2, "/lidar_driver/cloud", 10)
        self.create_subscription(LaserScan, "/lidar_driver/scan_raw", self._cb, 10)

    def _cb(self, msg: LaserScan):
        cloud = self._projector.projectLaser(msg)
        self.pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = ScanToCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
