import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rplidar import RPLidar, RPLidarException

BAUD_RATE = 115200
MOTOR_PWM = 660  # ~10 Hz rotation
SCAN_HZ = 10
RANGE_MIN = 0.2
RANGE_MAX = 12.0
NUM_READINGS = 360
RECONNECT_S = 3.0


class LidarDriver(Node):
    def __init__(self):
        super().__init__("lidar_driver")

        self.declare_parameter("device", "/dev/ttyUSB0")
        self._serial_port = self.get_parameter("device").get_parameter_value().string_value

        self.pub = self.create_publisher(LaserScan, "/lidar_driver/scan_raw", 10)

        self._lock = threading.Lock()
        self._latest_ranges = [float("inf")] * NUM_READINGS
        self._lidar = None
        self._scan_thread = None

        self.create_timer(1.0 / SCAN_HZ, self._publish_cb)
        self.create_timer(RECONNECT_S, self._connect)
        self._connect()

    def _connect(self):
        if self._lidar is not None:
            return
        try:
            lidar = RPLidar(self._serial_port)
            info = lidar.get_info()
            health = lidar.get_health()
            self.get_logger().info(f"RPLIDAR connected — info: {info}, health: {health}")
            self._lidar = lidar
            self._scan_thread = threading.Thread(target=self._scan_loop, daemon=True)
            self._scan_thread.start()
        except Exception:
            self.get_logger().warn(
                f"RPLIDAR not available on {self._serial_port}, retrying in {RECONNECT_S:.0f}s",
                throttle_duration_sec=30.0,
            )

    def _scan_loop(self):
        try:
            for scan in self._lidar.iter_scans():
                ranges = [float("inf")] * NUM_READINGS
                for _, angle, distance in scan:
                    if distance == 0:
                        continue
                    idx = int(angle) % NUM_READINGS
                    metres = distance / 1000.0
                    if metres < ranges[idx]:
                        ranges[idx] = metres
                with self._lock:
                    self._latest_ranges = ranges
        except RPLidarException as e:
            self.get_logger().error(f"RPLIDAR scan error: {e} — will reconnect")
            self._lidar = None

    def _publish_cb(self):
        with self._lock:
            ranges = list(self._latest_ranges)

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar"
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = 2 * math.pi / NUM_READINGS
        msg.time_increment = (1.0 / SCAN_HZ) / NUM_READINGS
        msg.scan_time = 1.0 / SCAN_HZ
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        self.pub.publish(msg)

    def destroy_node(self):
        if self._lidar is not None:
            try:
                self._lidar.stop()
                self._lidar.stop_motor()
                self._lidar.disconnect()
            except RPLidarException:
                pass
        super().destroy_node()


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