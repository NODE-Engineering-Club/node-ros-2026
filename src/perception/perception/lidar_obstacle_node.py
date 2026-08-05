import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


DEFAULT_MAX_OBSTACLE_RANGE_M = 10.0
DEFAULT_MIN_OBSTACLE_RANGE_M = 0.5


class LidarObstacleNode(Node):
    def __init__(self):
        super().__init__("lidar_obstacle_node")

        self.declare_parameter(
            "minimum_obstacle_range_m",
            DEFAULT_MIN_OBSTACLE_RANGE_M,
        )
        self.declare_parameter(
            "maximum_obstacle_range_m",
            DEFAULT_MAX_OBSTACLE_RANGE_M,
        )

        self._minimum_obstacle_range_m = float(
            self.get_parameter(
                "minimum_obstacle_range_m"
            ).value
        )
        self._maximum_obstacle_range_m = float(
            self.get_parameter(
                "maximum_obstacle_range_m"
            ).value
        )

        self.pub = self.create_publisher(
            PointCloud2,
            "/obstacles/lidar",
            10,
        )

        self.create_subscription(
            LaserScan,
            "/lidar_driver/scan_raw",
            self._cb,
            10,
        )

        self.get_logger().info(
            "lidar_obstacle_node ready: "
            f"minimum_range={self._minimum_obstacle_range_m:.2f} m, "
            f"maximum_range={self._maximum_obstacle_range_m:.2f} m"
        )

    def _cb(self, scan):
        points = []
        angle = scan.angle_min

        minimum_range = max(
            float(scan.range_min),
            self._minimum_obstacle_range_m,
        )
        maximum_range = min(
            float(scan.range_max),
            self._maximum_obstacle_range_m,
        )

        for distance in scan.ranges:
            if (
                math.isfinite(distance)
                and minimum_range < distance < maximum_range
            ):
                points.append(
                    (
                        distance * math.cos(angle),
                        distance * math.sin(angle),
                        0.0,
                    )
                )

            angle += scan.angle_increment

        message = PointCloud2()
        message.header = scan.header
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(
                name="x",
                offset=0,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="y",
                offset=4,
                datatype=PointField.FLOAT32,
                count=1,
            ),
            PointField(
                name="z",
                offset=8,
                datatype=PointField.FLOAT32,
                count=1,
            ),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = 12 * len(points)
        message.data = b"".join(
            struct.pack("fff", *point)
            for point in points
        )
        message.is_dense = True

        self.pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()