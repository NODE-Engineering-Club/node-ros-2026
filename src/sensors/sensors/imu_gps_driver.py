"""Relays MAVROS IMU/GPS topics to standard names, decoupling the stack from MAVROS."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix


class ImuGpsDriver(Node):
    def __init__(self):
        super().__init__("imu_gps_driver")

        self.imu_pub = self.create_publisher(Imu, "/imu_driver/imu_raw", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps_driver/gps_raw", 10)

        # mavros publishes these BEST_EFFORT; a default (RELIABLE) subscription
        # is QoS-incompatible with that and silently never receives anything.
        self.create_subscription(Imu, "/mavros/imu/data", self._imu_cb, qos_profile_sensor_data)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/raw/fix", self._gps_cb, qos_profile_sensor_data
        )

    def _imu_cb(self, msg):
        self.imu_pub.publish(msg)

    def _gps_cb(self, msg):
        self.gps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuGpsDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
