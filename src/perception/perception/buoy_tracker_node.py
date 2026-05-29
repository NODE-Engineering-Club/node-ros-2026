"""Buoy tracker — converts /buoys/detected robot-frame positions to GPS lat/lon.

Subscribes to:
  /buoys/detected      (std_msgs/String, JSON from fusion_node)
  /gps/fix             (sensor_msgs/NavSatFix)
  /odometry/filtered   (nav_msgs/Odometry)

Publishes:
  /buoys/nav           (std_msgs/String, JSON)
  Format: [{"color","range","bearing_deg","lat","lon"}, ...]
"""

import json
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

EARTH_R = 6_371_000.0  # metres


class BuoyTrackerNode(Node):
    def __init__(self):
        super().__init__("buoy_tracker_node")

        self._boat_lat:     float | None = None
        self._boat_lon:     float | None = None
        self._boat_yaw_deg: float        = 0.0
        self._buoys_body:   list         = []

        self.create_subscription(String,    "/buoys/detected",    self._cb_buoys, 10)
        self.create_subscription(NavSatFix, "/gps/fix",           self._cb_gps,   10)
        self.create_subscription(Odometry,  "/odometry/filtered", self._cb_odom,  10)

        self._pub = self.create_publisher(String, "/buoys/nav", 10)
        self.create_timer(0.2, self._publish)  # 5 Hz
        self.get_logger().info("BuoyTracker ready")

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _cb_buoys(self, msg: String):
        try:
            self._buoys_body = json.loads(msg.data)
        except (json.JSONDecodeError, ValueError):
            self._buoys_body = []

    def _cb_gps(self, msg: NavSatFix):
        if msg.status.status >= 0:  # STATUS_NO_FIX = -1
            self._boat_lat = msg.latitude
            self._boat_lon = msg.longitude

    def _cb_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._boat_yaw_deg = math.degrees(math.atan2(siny, cosy))

    # ── GPS projection ────────────────────────────────────────────────────────

    def _buoy_latlon(self, bx: float, by: float):
        """Project base_link (x=fwd, y=left) buoy position to GPS lat/lon.

        Uses REP-103 convention: yaw=0 → East, positive counterclockwise.
        """
        if self._boat_lat is None:
            return None, None
        r = math.sqrt(bx ** 2 + by ** 2)
        if r < 0.05:
            return round(self._boat_lat, 8), round(self._boat_lon, 8)

        # World angle of buoy measured from East, counterclockwise
        world_angle = math.radians(self._boat_yaw_deg) + math.atan2(by, bx)
        # Geographic bearing from North, clockwise: 90° - world_angle
        geo_bearing = math.pi / 2.0 - world_angle

        lat1 = math.radians(self._boat_lat)
        lon1 = math.radians(self._boat_lon)
        d    = r / EARTH_R
        lat2 = math.asin(
            math.sin(lat1) * math.cos(d)
            + math.cos(lat1) * math.sin(d) * math.cos(geo_bearing)
        )
        lon2 = lon1 + math.atan2(
            math.sin(geo_bearing) * math.sin(d) * math.cos(lat1),
            math.cos(d) - math.sin(lat1) * math.sin(lat2),
        )
        return round(math.degrees(lat2), 8), round(math.degrees(lon2), 8)

    # ── Publish ───────────────────────────────────────────────────────────────

    def _publish(self):
        out = []
        for b in self._buoys_body:
            bx, by = b.get("x", 0.0), b.get("y", 0.0)
            lat, lon = self._buoy_latlon(bx, by)
            out.append({
                "color":       b.get("color", "unknown"),
                "range":       b.get("range", 0.0),
                "bearing_deg": b.get("bearing_deg", 0.0),
                "lat":         lat,
                "lon":         lon,
            })
        msg      = String()
        msg.data = json.dumps(out)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BuoyTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
