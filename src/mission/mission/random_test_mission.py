"""Ad-hoc test mission: reads the boat's current GPS fix, computes a goal
point OFFSET_M metres away in a random bearing, and sends that single
waypoint through the existing /mission/start service. Nav2 (via
mission_manager's NavigateToPose call) handles GPS->map conversion
(/fromLL), path planning, and control — this script only decides *where*
to go. Same structure as north_test_mission.py, generalized to any bearing.
"""
import math
import random

import rclpy
from geographic_msgs.msg import GeoPoint
from njord_msgs.srv import StartMission
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

from mission.geo_utils import offset_latlon

OFFSET_M = 5.0


class RandomTestMission(Node):
    def __init__(self):
        super().__init__("random_test_mission")
        self._client = self.create_client(StartMission, "/mission/start")
        self._fix = None
        self.create_subscription(NavSatFix, "/gps_driver/gps_raw", self._fix_cb, 10)

    def _fix_cb(self, msg):
        self._fix = msg

    def wait_for_fix(self, timeout_sec=30.0):
        deadline = self.get_clock().now().nanoseconds + int(timeout_sec * 1e9)
        while self._fix is None and self.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self._fix is not None

    def send(self):
        bearing_deg = random.uniform(0.0, 360.0)
        bearing_rad = math.radians(bearing_deg)
        north_m = OFFSET_M * math.cos(bearing_rad)
        east_m  = OFFSET_M * math.sin(bearing_rad)
        lat_b, lon_b = offset_latlon(self._fix.latitude, self._fix.longitude, north_m=north_m, east_m=east_m)

        self.get_logger().info(
            f"Current fix: ({self._fix.latitude:.7f}, {self._fix.longitude:.7f}) "
            f"-> goal {OFFSET_M} m @ bearing {bearing_deg:.1f}°: ({lat_b:.7f}, {lon_b:.7f})"
        )

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /mission/start service...")

        req = StartMission.Request()
        req.waypoints = [GeoPoint(latitude=lat_b, longitude=lon_b, altitude=0.0)]

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res.success:
            self.get_logger().info(f"Mission accepted: {res.message}")
        else:
            self.get_logger().error(f"Mission rejected: {res.message}")


def main(args=None):
    rclpy.init(args=args)
    node = RandomTestMission()
    try:
        if not node.wait_for_fix():
            node.get_logger().error("No GPS fix received within timeout — aborting")
        else:
            node.send()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
