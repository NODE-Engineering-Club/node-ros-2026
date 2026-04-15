import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from pyrplidar import PyRPlidar

SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 115200
MOTOR_PWM = 660  # ~10 Hz rotation
SCAN_HZ = 10
RANGE_MIN = 0.2
RANGE_MAX = 12.0
NUM_READINGS = 360  # 1° resolution bins


class LidarDriver(Node):
    def __init__(self):
        super().__init__("lidar_driver")
        self.pub = self.create_publisher(LaserScan, "/scan", 10)

        self._lidar = PyRPlidar()
        self._lidar.connect(port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=3)
        self._lidar.set_motor_pwm(MOTOR_PWM)

        self._thread = threading.Thread(target=self._scan_loop, daemon=True)
        self._thread.start()

    def _scan_loop(self):
        scan_generator = self._lidar.start_scan()
        pending = []

        for measurement in scan_generator():
            if measurement.new_scan and pending:
                self._publish(pending)
                pending = []

            if measurement.distance > 0:
                pending.append(
                    (
                        math.radians(measurement.angle),
                        measurement.distance / 1000.0,  # mm → m
                    )
                )

    def _publish(self, measurements: list[tuple[float, float]]):
        ranges = [float("inf")] * NUM_READINGS
        angle_increment = 2 * math.pi / NUM_READINGS

        for angle_rad, dist_m in measurements:
            if RANGE_MIN <= dist_m <= RANGE_MAX:
                idx = int(angle_rad / angle_increment) % NUM_READINGS
                # keep closest reading per bin
                if dist_m < ranges[idx]:
                    ranges[idx] = dist_m

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "lidar"
        msg.angle_min = 0.0
        msg.angle_max = 2 * math.pi
        msg.angle_increment = angle_increment
        msg.time_increment = (1.0 / SCAN_HZ) / NUM_READINGS
        msg.scan_time = 1.0 / SCAN_HZ
        msg.range_min = RANGE_MIN
        msg.range_max = RANGE_MAX
        msg.ranges = ranges
        self.pub.publish(msg)

    def destroy_node(self):
        self._lidar.stop()
        self._lidar.set_motor_pwm(0)
        self._lidar.disconnect()
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
