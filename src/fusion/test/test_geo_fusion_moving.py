"""In-process integration test for geo_fusion_node's velocity_bearing_deg
field (added for Task 9.2's marker-vessel COLREG give-way logic).

Runs MovingScenePublisher + GeoFusionNode + a collector in one rclpy process
and asserts the fused, unclassified, moving obstacle's published
velocity_bearing_deg converges to atan2(VY, VX) in degrees. No Gazebo, no
YOLO, no rosbag required — deterministic and fast, mirrors
test_geo_fusion.py's structure exactly (single scenario, same in-process
executor pattern).

Run:  python3 src/fusion/test/test_geo_fusion_moving.py
Exit code 0 = pass.
"""

import math
import os
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(__file__))
from moving_scene_publisher import VX, VY, MovingScenePublisher  # noqa: E402

from fusion.geo_fusion_node import GeoFusionNode  # noqa: E402
from njord_msgs.msg import ObstacleArray  # noqa: E402

EXPECTED_VELOCITY_BEARING_DEG = math.degrees(math.atan2(VY, VX))
TOLERANCE_DEG = 20.0  # generous — the KF's velocity estimate is noisy early on


class Collector(Node):
    def __init__(self):
        super().__init__("collector")
        self.last = None
        self.create_subscription(ObstacleArray, "/obstacles/global", self._cb, 10)

    def _cb(self, msg):
        self.last = msg


def main():
    rclpy.init()
    scene = MovingScenePublisher()
    fusion = GeoFusionNode()
    collector = Collector()

    ex = SingleThreadedExecutor()
    for n in (scene, fusion, collector):
        ex.add_node(n)

    # Run well past track_confirm_hits (default 3) so the constant-velocity
    # Kalman filter's (vx, vy) estimate has time to settle.
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        ex.spin_once(timeout_sec=0.1)

    msg = collector.last
    failures = []

    def check(cond, label):
        print(f"  [{'PASS' if cond else 'FAIL'}] {label}")
        if not cond:
            failures.append(label)

    print("geo_fusion_node moving-obstacle (velocity_bearing_deg) test:")
    check(msg is not None, "received an ObstacleArray on /obstacles/global")
    if msg is not None:
        check(len(msg.obstacles) == 1, f"exactly one obstacle (got {len(msg.obstacles)})")
        if msg.obstacles:
            o = msg.obstacles[0]
            check(o.class_id == "unknown", f"class_id is 'unknown' ('{o.class_id}')")
            check(o.lidar_confirmed, "obstacle is lidar_confirmed")
            check(o.speed_mps > 0.1, f"obstacle has nonzero tracked speed ({o.speed_mps:.3f} m/s)")
            error_deg = abs(o.velocity_bearing_deg - EXPECTED_VELOCITY_BEARING_DEG)
            check(
                error_deg < TOLERANCE_DEG,
                f"velocity_bearing_deg ~= {EXPECTED_VELOCITY_BEARING_DEG:.1f} deg "
                f"(got {o.velocity_bearing_deg:.1f} deg, error {error_deg:.1f} deg)",
            )

    for n in (scene, fusion, collector):
        n.destroy_node()
    rclpy.shutdown()

    print()
    if failures:
        print(f"RESULT: FAIL ({len(failures)} check(s) failed)")
        return 1
    print("RESULT: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
