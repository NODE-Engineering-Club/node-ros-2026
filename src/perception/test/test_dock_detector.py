"""Integration test for dock_detector_node against a synthetic two-berth
dock with one berth occupied by a decoy boat (see dock_scene_publisher.py
for how the synthetic /obstacles/lidar scenes are built).

No Gazebo needed -- publishes one synthetic PointCloud2 per test case
directly, waits for the node to process and respond, and checks the
result. This is the fast/deterministic half of the multi-berth occupancy
test plan; a required real-Gazebo run against dockingWorldOccupied.sdf is
the other half -- this script alone does not prove sim/hardware fidelity
(occlusion, sensor noise, etc.), only that the clustering/RANSAC/
U-matching/occupancy algorithm is correct against known-good points.

Run against the CURRENT node as-is first (pre-refactor baseline), then
again after the multi-berth/occupancy refactor lands (DockTarget.occupied
and DockTargetArray are detected automatically via getattr/import checks,
so this same script works unmodified for both runs).

Run:  python3 src/perception/test/test_dock_detector.py
Exit code 0 = pass (no hard-invariant failures).
"""

import os
import sys
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

sys.path.insert(0, os.path.dirname(__file__))
from dock_scene_publisher import (  # noqa: E402
    BERTH_A_CENTER_Y,
    BERTH_B_CENTER_Y,
    OPENING_X,
    build_scene_cloud,
    to_boat_frame,
    vantage_pose,
)

from njord_msgs.msg import DockTarget  # noqa: E402
from perception.dock_detector_node import DockDetectorNode  # noqa: E402

try:
    from njord_msgs.msg import DockTargetArray
    HAS_ARRAY_MSG = True
except ImportError:
    HAS_ARRAY_MSG = False

DISTANCES = [3.0, 5.0, 8.0]
ANGLES = [-30.0, 0.0, 30.0]
OCCUPIED_CASES = ["A", "B", None]  # None = both-free control


class ScenePub(Node):
    def __init__(self):
        super().__init__("dock_scene_pub")
        self.pub = self.create_publisher(PointCloud2, "/obstacles/lidar", 10)


class Collector(Node):
    def __init__(self):
        super().__init__("dock_test_collector")
        self.last_single = None
        self.last_array = None
        self.create_subscription(DockTarget, "/perception/dock_target", self._cb_single, 10)
        if HAS_ARRAY_MSG:
            self.create_subscription(DockTargetArray, "/perception/dock_targets", self._cb_array, 10)

    def _cb_single(self, msg):
        self.last_single = msg

    def _cb_array(self, msg):
        self.last_array = msg


def expected_opening(distance, angle_deg, berth):
    cy = BERTH_A_CENTER_Y if berth == "A" else BERTH_B_CENTER_Y
    bx, by, bh = vantage_pose(distance, angle_deg)
    return to_boat_frame([(OPENING_X, cy)], bx, by, bh)[0]


def run_case(ex, pub_node, collector, distance, angle_deg, occupied_berth):
    collector.last_single = None
    collector.last_array = None
    cloud = build_scene_cloud(distance, angle_deg, occupied_berth)
    cloud.header.stamp = pub_node.get_clock().now().to_msg()
    pub_node.pub.publish(cloud)

    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        ex.spin_once(timeout_sec=0.1)
        if collector.last_single is not None:
            break
    return collector.last_single, collector.last_array


def main():
    rclpy.init()
    pub_node = ScenePub()
    detector = DockDetectorNode()
    collector = Collector()

    ex = SingleThreadedExecutor()
    for n in (pub_node, detector, collector):
        ex.add_node(n)

    failures = []
    soft_failures = []

    def check(cond, label):
        print(f"  [{'PASS' if cond else 'FAIL'}] {label}")
        if not cond:
            failures.append(label)

    def soft_check(cond, label):
        print(f"  [{'INFO-PASS' if cond else 'INFO-FAIL'}] {label}")
        if not cond:
            soft_failures.append(label)

    print("dock_detector_node multi-berth occupancy test:")
    print(f"  DockTarget.occupied field present: {hasattr(DockTarget(), 'occupied')}")
    print(f"  DockTargetArray message present:   {HAS_ARRAY_MSG}")
    print()

    for distance in DISTANCES:
        for angle in ANGLES:
            for occupied in OCCUPIED_CASES:
                label = f"d={distance}m a={angle}deg occupied={occupied}"
                single, array = run_case(ex, pub_node, collector, distance, angle, occupied)

                check(single is not None, f"[{label}] received a /perception/dock_target message")
                if single is None:
                    continue

                if occupied is None:
                    # Both-free control case: no hard invariant, just informational.
                    soft_check(single.detected, f"[{label}] a berth is detected (control case)")
                    continue

                occ_berth = occupied
                exp_occ = expected_opening(distance, angle, occ_berth)
                exp_free = expected_opening(distance, angle, "B" if occ_berth == "A" else "A")

                if single.detected:
                    dist_to_occ = ((single.opening_center.x - exp_occ[0]) ** 2 +
                                   (single.opening_center.y - exp_occ[1]) ** 2) ** 0.5
                    dist_to_free = ((single.opening_center.x - exp_free[0]) ** 2 +
                                     (single.opening_center.y - exp_free[1]) ** 2) ** 0.5
                    is_occ_berth = dist_to_occ < dist_to_free and dist_to_occ < 0.6
                    is_free_berth = dist_to_free <= dist_to_occ and dist_to_free < 0.6

                    if hasattr(single, "occupied"):
                        # Post-refactor: occupied berth may be reported, but must be flagged.
                        check(not (is_occ_berth and not single.occupied),
                              f"[{label}] occupied berth (if reported) is flagged occupied=true")
                    else:
                        # Pre-refactor baseline: no occupied concept -- must simply
                        # never surface the occupied berth as *the* detected target.
                        check(not is_occ_berth,
                              f"[{label}] occupied berth not reported as the detected target (baseline, no occupied field)")

                    soft_check(is_free_berth or not single.detected,
                               f"[{label}] free berth ({('B' if occ_berth == 'A' else 'A')}) is the one detected")
                else:
                    soft_check(False, f"[{label}] free berth detected at all (nothing detected)")

    print()
    print(f"Hard-invariant failures: {len(failures)}")
    print(f"Soft/informational misses: {len(soft_failures)}")
    if failures:
        print("RESULT: FAIL")
    else:
        print("RESULT: PASS")

    for n in (pub_node, detector, collector):
        n.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
