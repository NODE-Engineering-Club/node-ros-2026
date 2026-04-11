import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

SERIAL_PORT = "/dev/ttyUSB0"
SCAN_HZ = 10
RANGE_MIN = 0.15
RANGE_MAX = 12.0
NUM_READINGS = 360


class LidarDriver(Node):
    def __init__(self):
        super().__init__("lidar_driver")
        self.pub = self.create_publisher(LaserScan, "/scan", 10)
        # TODO: open serial port, init hardware
        self.create_timer(1 / SCAN_HZ, self._cb)

    def _cb(self):
        ranges = self._read_scan()

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar"
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / NUM_READINGS
        msg.time_increment = (1 / SCAN_HZ) / NUM_READINGS
        msg.scan_time = 1 / SCAN_HZ
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        self.pub.publish(msg)

    def _read_scan(self) -> list[float]:
        # Hardware-specific: replace with actual serial protocol parsing
        return [float("inf")] * NUM_READINGS


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
