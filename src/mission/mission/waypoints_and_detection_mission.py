"""Quick ad-hoc test mission "Waypoints and Detection": five fixed GPS
waypoints sent through the existing /mission/start service, ending at a
red/green buoy gate the boat should cross through the middle of.

Same structure as north_test_mission.py/random_test_mission.py, except the
waypoints are fixed absolute coordinates instead of an offset from the
current fix. Nav2 (via mission_manager's NavigateToPose call) handles
GPS->map conversion (/fromLL), path planning, and control -- this script
only decides *where* to go.

The buoy gate itself has no waypoint of its own: waypoint 5 is assumed to
sit on the far side of the gate, so mission_manager's straight-line transit
from waypoint 4 through waypoint 5 already threads between the two buoys.
Actually staying clear of each buoy on the way through is handled by
boat_bt's GlobalSafety reflex (buoy standoff + bearing-only avoidance),
which runs unconditionally for any task/mission -- no docking/docking_
parallel suppression applies here, see simple_boat.xml's GlobalSafety
subtree. Nothing else in this script is needed for the gate crossing.
"""
import rclpy
from geographic_msgs.msg import GeoPoint
from njord_msgs.srv import StartMission
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

WAYPOINTS = [
    (63.4414064888, 10.4236967835),
    (63.4413457176, 10.4233534093),
    (63.4412177779, 10.4235894791),
    (63.4411378153, 10.4232174903),
    (63.4409714923, 10.4232174903),
]


class WaypointsAndDetectionMission(Node):
    def __init__(self):
        super().__init__("waypoints_and_detection_mission")
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
        self.get_logger().info(
            f"Sending {len(WAYPOINTS)} waypoints for mission 'Waypoints and Detection'"
        )

        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for /mission/start service...")

        req = StartMission.Request()
        req.waypoints = [
            GeoPoint(latitude=lat, longitude=lon, altitude=0.0)
            for lat, lon in WAYPOINTS
        ]

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res.success:
            self.get_logger().info(f"Mission accepted: {res.message}")
        else:
            self.get_logger().error(f"Mission rejected: {res.message}")


def main(args=None):
    rclpy.init(args=args)
    node = WaypointsAndDetectionMission()
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
