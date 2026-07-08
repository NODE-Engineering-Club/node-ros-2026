"""In-process integration test for geo_fusion_node.

Runs ScenePublisher + GeoFusionNode + a collector in one rclpy process and
asserts that the fused output is a single, correctly geo-referenced, labelled
obstacle. No Gazebo, no YOLO, no rosbag required — deterministic and fast.

Run:  python3 src/fusion/test/test_geo_fusion.py
Exit code 0 = pass.
"""

import os
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

sys.path.insert(0, os.path.dirname(__file__))
from scene_publisher import LAT0, LON0, ScenePublisher  # noqa: E402

from fusion.geo_fusion_node import GeoFusionNode  # noqa: E402
from njord_msgs.msg import ObstacleArray  # noqa: E402


class Collector(Node):
    def __init__(self):
        super().__init__("collector")
        self.last = None
        self.create_subscription(ObstacleArray, "/obstacles/global", self._cb, 10)

    def _cb(self, msg):
        self.last = msg


def main():
    rclpy.init()
    scene = ScenePublisher()
    fusion = GeoFusionNode()
    collector = Collector()

    ex = SingleThreadedExecutor()
    for n in (scene, fusion, collector):
        ex.add_node(n)

    deadline = time.monotonic() + 6.0
    while time.monotonic() < deadline:
        ex.spin_once(timeout_sec=0.1)
        if collector.last is not None and collector.last.obstacles:
            # let the tracker confirm (min_hits) and settle a few frames
            if collector.last.obstacles[0].id >= 1:
                time.sleep(0.3)
                ex.spin_once(timeout_sec=0.1)
                break

    msg = collector.last
    failures = []

    def check(cond, label):
        print(f"  [{'PASS' if cond else 'FAIL'}] {label}")
        if not cond:
            failures.append(label)

    print("geo_fusion_node integration test:")
    check(msg is not None, "received an ObstacleArray on /obstacles/global")
    if msg is not None:
        check(msg.header.frame_id == "map", "output frame_id is 'map'")
        check(abs(msg.boat_position.latitude - LAT0) < 1e-6, "boat latitude == datum")
        check(abs(msg.boat_position.longitude - LON0) < 1e-6, "boat longitude == datum")
        check(len(msg.obstacles) == 1, f"exactly one obstacle (got {len(msg.obstacles)})")
        if msg.obstacles:
            o = msg.obstacles[0]
            check(o.class_id == "2", f"class_id carried through ('{o.class_id}')")
            check(o.lidar_confirmed, "obstacle is lidar_confirmed")
            check(o.id >= 1, f"obstacle has a tracking id ({o.id})")
            check(abs(o.position_map.x - 5.0) < 0.5, f"position_map.x ~= 5 m ({o.position_map.x:.2f})")
            check(abs(o.position_map.y - 0.0) < 0.5, f"position_map.y ~= 0 m ({o.position_map.y:.2f})")
            # 5 m east -> longitude increases, latitude ~unchanged
            check(o.position.latitude - LAT0 < 1e-4 and abs(o.position.latitude - LAT0) < 1e-4,
                  "obstacle latitude ~= datum (object is due east)")
            check(o.position.longitude > LON0, "obstacle longitude > datum (object is east)")
            east_m = (o.position.longitude - LON0) * 111320.0 * 0.446  # cos(63.43 deg)
            check(abs(east_m - 5.0) < 1.0, f"obstacle ~5 m east of datum ({east_m:.2f} m)")

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
