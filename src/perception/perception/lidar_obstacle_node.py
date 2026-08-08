import math
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField


DEFAULT_MAX_OBSTACLE_RANGE_M = 20.0
DEFAULT_MIN_OBSTACLE_RANGE_M = 0.5
# The RPLIDAR S2 (DenseBoost mode) emits far more points per revolution than
# the A-series it replaced. Downstream consumers of /obstacles/lidar include
# an O(n^2) Euclidean clustering step (fusion/geo_fusion_node.py), so bound
# the published cloud to one nearest-point per angular bin regardless of the
# driver's native sample density.
DEFAULT_ANGULAR_DECIMATION_DEG = 1.0


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
        self.declare_parameter(
            "angular_decimation_deg",
            DEFAULT_ANGULAR_DECIMATION_DEG,
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
        self._angular_decimation_rad = math.radians(float(
            self.get_parameter(
                "angular_decimation_deg"
            ).value
        ))

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
            f"maximum_range={self._maximum_obstacle_range_m:.2f} m, "
            f"angular_decimation={math.degrees(self._angular_decimation_rad):.2f} deg"
        )

    def _cb(self, scan):
        angle = scan.angle_min

        minimum_range = max(
            float(scan.range_min),
            self._minimum_obstacle_range_m,
        )
        maximum_range = min(
            float(scan.range_max),
            self._maximum_obstacle_range_m,
        )

        # Nearest-point-per-angular-bin decimation: keeps the published
        # cloud's point count independent of the driver's native scan
        # resolution.
        nearest_by_bin = {}
        for distance in scan.ranges:
            if (
                math.isfinite(distance)
                and minimum_range < distance < maximum_range
            ):
                bin_idx = int((angle - scan.angle_min) / self._angular_decimation_rad)
                if distance < nearest_by_bin.get(bin_idx, (math.inf, 0.0))[0]:
                    nearest_by_bin[bin_idx] = (distance, angle)

            angle += scan.angle_increment

        points = [
            (dist * math.cos(a), dist * math.sin(a), 0.0)
            for dist, a in nearest_by_bin.values()
        ]

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