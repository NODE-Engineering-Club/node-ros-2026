"""Dock detector node — recognizes the U-shaped docking berth (Task 3.1,
"normal docking") from the LiDAR obstacle cloud.

Pipeline, per scan:
  1. DBSCAN separates the raw /obstacles/lidar cloud into candidate objects
     (so an unrelated buoy/obstacle elsewhere in the scan doesn't get mixed
     into the dock structure's points).
  2. For each candidate object, iterative RANSAC extracts straight wall
     segments from its (possibly noisy) points — robust to the outlier
     returns a real LiDAR produces off a flat painted panel.
  3. The extracted segments are matched against a U template: two roughly
     parallel arm segments, each roughly perpendicular to a connecting back
     wall segment, spaced apart by the expected berth width, with the
     opening facing the boat (LiDAR origin).

Output: /perception/dock_target (njord_msgs/DockTarget), boat-relative
(base_link frame) — no GPS/TF lifting, since docking is a short-range
final-approach maneuver where LiDAR-relative geometry is what matters.

Not yet done: temporal filtering/tracking across scans (mirrors the natural
next step taken by fusion/geo_fusion_node.py's Tracker), and consuming this
from the mission/behavior-tree layer (boat_bt/bt_xml/simple_boat.xml
deliberately leaves docking out for now).
"""

import itertools
import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from njord_msgs.msg import DockTarget
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN

ORIGIN = np.array([0.0, 0.0])


class DockDetectorNode(Node):
    def __init__(self):
        super().__init__("dock_detector_node")

        # Mount calibration — /obstacles/lidar carries raw lidar-local x,y
        # (lidar_obstacle_node applies no TF), and the physical mount has a
        # known, verified yaw offset from base_link forward (see
        # asket.urdf.xacro's lidar_mount_joint comment: dead-ahead reads as
        # local x~=0, y~=-range, i.e. the raw cloud is rotated -90deg from
        # true base_link). Same knob/convention as fusion/geo_fusion_node.py's
        # lidar_yaw_offset_deg. Defaults to the URDF's documented +90deg so
        # this node's "base_link" frame_id claim on DockTarget is actually
        # true; override if the mount is recalibrated.
        self.declare_parameter("lidar_yaw_offset_deg", 90.0)

        # Object-level clustering (DBSCAN)
        self.declare_parameter("cluster_eps", 0.4)
        self.declare_parameter("cluster_min_samples", 3)

        # Wall extraction (RANSAC)
        self.declare_parameter("ransac_dist_threshold_m", 0.05)
        self.declare_parameter("ransac_iterations", 200)
        self.declare_parameter("ransac_min_inliers", 6)
        self.declare_parameter("max_lines_per_cluster", 3)

        # U-shape template matching
        self.declare_parameter("berth_width_m", 2.0)
        self.declare_parameter("width_tolerance_m", 0.4)
        self.declare_parameter("arm_length_min_m", 0.6)
        self.declare_parameter("arm_length_max_m", 3.0)
        self.declare_parameter("parallel_angle_tol_deg", 12.0)
        self.declare_parameter("perp_angle_tol_deg", 12.0)
        self.declare_parameter("corner_gap_tol_m", 0.35)

        p = self.get_parameter
        theta = math.radians(p("lidar_yaw_offset_deg").value)
        self._cos_t, self._sin_t = math.cos(theta), math.sin(theta)

        self._cluster_eps = p("cluster_eps").value
        self._cluster_min_samples = p("cluster_min_samples").value

        self._ransac_dist_threshold = p("ransac_dist_threshold_m").value
        self._ransac_iterations = p("ransac_iterations").value
        self._ransac_min_inliers = p("ransac_min_inliers").value
        self._max_lines_per_cluster = p("max_lines_per_cluster").value

        self._berth_width = p("berth_width_m").value
        self._width_tol = p("width_tolerance_m").value
        self._arm_len_min = p("arm_length_min_m").value
        self._arm_len_max = p("arm_length_max_m").value
        self._parallel_tol = math.radians(p("parallel_angle_tol_deg").value)
        self._perp_tol = math.radians(p("perp_angle_tol_deg").value)
        self._corner_gap_tol = p("corner_gap_tol_m").value

        self._rng = np.random.default_rng()

        self.pub = self.create_publisher(DockTarget, "/perception/dock_target", 10)
        self.create_subscription(PointCloud2, "/obstacles/lidar", self._cb, 10)

        self.get_logger().info("dock_detector_node ready — publishing /perception/dock_target")

    # ── Callback ─────────────────────────────────────────────────────────────

    def _cb(self, msg):
        points = _read_xy(msg)
        if len(points) > 0 and (self._cos_t != 1.0 or self._sin_t != 0.0):
            x, y = points[:, 0].copy(), points[:, 1].copy()
            points[:, 0] = self._cos_t * x - self._sin_t * y
            points[:, 1] = self._sin_t * x + self._cos_t * y

        header = msg.header
        header.frame_id = "base_link"

        best = None
        if len(points) >= self._cluster_min_samples:
            labels = DBSCAN(
                eps=self._cluster_eps,
                min_samples=self._cluster_min_samples,
            ).fit_predict(points)

            for label in set(labels):
                if label == -1:
                    continue
                cluster_pts = points[labels == label]
                if len(cluster_pts) < self._ransac_min_inliers:
                    continue

                segments = _ransac_lines(
                    cluster_pts,
                    dist_threshold=self._ransac_dist_threshold,
                    iterations=self._ransac_iterations,
                    min_inliers=self._ransac_min_inliers,
                    max_lines=self._max_lines_per_cluster,
                    rng=self._rng,
                )
                if len(segments) < 3:
                    continue

                match = _find_u_shape(
                    segments,
                    berth_width=self._berth_width,
                    width_tol=self._width_tol,
                    arm_len_min=self._arm_len_min,
                    arm_len_max=self._arm_len_max,
                    parallel_tol=self._parallel_tol,
                    perp_tol=self._perp_tol,
                    corner_gap_tol=self._corner_gap_tol,
                )
                if match is not None and (best is None or match["confidence"] > best["confidence"]):
                    best = match

        self.pub.publish(_to_msg(header, best))


# ── PointCloud2 decoding ─────────────────────────────────────────────────────

def _read_xy(cloud):
    """Extract an Nx2 numpy array of (x, y) from a PointCloud2 with float32 xyz fields."""
    n = cloud.width * cloud.height
    step = cloud.point_step
    pts = np.empty((n, 2), dtype=np.float32)
    for i in range(n):
        x, y, _z = struct.unpack_from("fff", cloud.data, i * step)
        pts[i] = (x, y)
    return pts


# ── RANSAC wall-segment extraction ──────────────────────────────────────────

def _ransac_lines(points, dist_threshold, iterations, min_inliers, max_lines, rng):
    """Iteratively fit straight lines to `points` via RANSAC (2D).

    Each iteration: sample 2 points, count inliers within dist_threshold of
    the line through them, keep the best model over `iterations` trials, and
    if it clears min_inliers, extract a segment (endpoints = the inliers'
    extreme projections along the line) and remove those points from the
    pool. Repeats until max_lines segments are found or too few points
    remain. Returns a list of dicts: {p1, p2, inliers, total} (numpy points).
    """
    remaining = points.copy()
    segments = []

    for _ in range(max_lines):
        n = remaining.shape[0]
        if n < min_inliers:
            break

        best_mask = None
        best_count = 0
        best_p1 = best_dir = None

        for _ in range(iterations):
            idx = rng.choice(n, size=2, replace=False)
            p1, p2 = remaining[idx[0]], remaining[idx[1]]
            d = p2 - p1
            norm = math.hypot(float(d[0]), float(d[1]))
            if norm < 1e-6:
                continue
            dx, dy = d[0] / norm, d[1] / norm
            nx, ny = -dy, dx  # unit normal

            diffs = remaining - p1
            dist = np.abs(diffs[:, 0] * nx + diffs[:, 1] * ny)
            mask = dist <= dist_threshold
            count = int(np.count_nonzero(mask))

            if count > best_count:
                best_count = count
                best_mask = mask
                best_p1 = p1
                best_dir = (dx, dy)

        if best_mask is None or best_count < min_inliers:
            break

        inliers = remaining[best_mask]
        dx, dy = best_dir
        proj = (inliers[:, 0] - best_p1[0]) * dx + (inliers[:, 1] - best_p1[1]) * dy
        i_min, i_max = int(np.argmin(proj)), int(np.argmax(proj))
        direction = np.array([dx, dy])
        seg_p1 = best_p1 + proj[i_min] * direction
        seg_p2 = best_p1 + proj[i_max] * direction

        segments.append({"p1": seg_p1, "p2": seg_p2, "inliers": best_count, "total": n})
        remaining = remaining[~best_mask]

    return segments


# ── U-shape template matching ───────────────────────────────────────────────

def _seg_vec(seg):
    return seg["p2"] - seg["p1"]


def _seg_angle(seg):
    v = _seg_vec(seg)
    return math.atan2(float(v[1]), float(v[0]))


def _seg_length(seg):
    return float(np.hypot(*_seg_vec(seg)))


def _angle_diff_mod_pi(a, b):
    """Undirected angle difference, folded into [0, pi/2]. 0 = parallel, pi/2 = perpendicular."""
    d = (a - b) % math.pi
    return min(d, math.pi - d)


def _near_far(seg, origin):
    """Return (near, far) endpoints of seg relative to origin."""
    d1 = float(np.hypot(*(seg["p1"] - origin)))
    d2 = float(np.hypot(*(seg["p2"] - origin)))
    return (seg["p1"], seg["p2"]) if d1 <= d2 else (seg["p2"], seg["p1"])


def _find_u_shape(segments, berth_width, width_tol, arm_len_min, arm_len_max,
                   parallel_tol, perp_tol, corner_gap_tol):
    """Search `segments` for the best U-shape match. Returns a result dict or None."""
    best = None

    for back_idx, seg_back in enumerate(segments):
        others = [i for i in range(len(segments)) if i != back_idx]
        for arm_i_idx, arm_k_idx in itertools.combinations(others, 2):
            seg_i, seg_k = segments[arm_i_idx], segments[arm_k_idx]

            len_i, len_k = _seg_length(seg_i), _seg_length(seg_k)
            if not (arm_len_min <= len_i <= arm_len_max):
                continue
            if not (arm_len_min <= len_k <= arm_len_max):
                continue

            angle_i, angle_k, angle_back = _seg_angle(seg_i), _seg_angle(seg_k), _seg_angle(seg_back)

            parallel_err = _angle_diff_mod_pi(angle_i, angle_k)
            if parallel_err > parallel_tol:
                continue

            perp_err_i = abs(_angle_diff_mod_pi(angle_i, angle_back) - math.pi / 2)
            perp_err_k = abs(_angle_diff_mod_pi(angle_k, angle_back) - math.pi / 2)
            if perp_err_i > perp_tol or perp_err_k > perp_tol:
                continue

            near_i, far_i = _near_far(seg_i, ORIGIN)
            near_k, far_k = _near_far(seg_k, ORIGIN)

            back_p1, back_p2 = seg_back["p1"], seg_back["p2"]
            d_i1 = float(np.hypot(*(far_i - back_p1)))
            d_i2 = float(np.hypot(*(far_i - back_p2)))
            d_k1 = float(np.hypot(*(far_k - back_p1)))
            d_k2 = float(np.hypot(*(far_k - back_p2)))

            if d_i1 + d_k2 <= d_i2 + d_k1:
                connect_err = (d_i1 + d_k2) / 2.0
            else:
                connect_err = (d_i2 + d_k1) / 2.0
            if connect_err > corner_gap_tol:
                continue

            width = float(np.hypot(*(near_i - near_k)))
            width_err = abs(width - berth_width)
            if width_err > width_tol:
                continue

            opening_center = (near_i + near_k) / 2.0
            back_mid = (back_p1 + back_p2) / 2.0

            bd = back_p2 - back_p1
            bn = float(np.hypot(*bd))
            if bn < 1e-6:
                continue
            nx, ny = -bd[1] / bn, bd[0] / bn
            to_back = back_mid - ORIGIN
            if nx * to_back[0] + ny * to_back[1] < 0:
                nx, ny = -nx, -ny
            heading = math.atan2(ny, nx)

            depth = (len_i + len_k) / 2.0

            inlier_ratio = sum(
                seg["inliers"] / seg["total"] for seg in (seg_i, seg_k, seg_back)
            ) / 3.0

            error_score = (
                parallel_err / parallel_tol
                + (perp_err_i + perp_err_k) / 2.0 / perp_tol
                + width_err / width_tol
                + connect_err / corner_gap_tol
            ) / 4.0
            confidence = max(0.0, min(1.0, 1.0 - error_score)) * inlier_ratio

            if best is None or confidence > best["confidence"]:
                best = {
                    "opening_center": opening_center,
                    "heading": heading,
                    "width": width,
                    "depth": depth,
                    "confidence": confidence,
                }

    return best


# ── Publishing ───────────────────────────────────────────────────────────────

def _to_msg(header, match):
    msg = DockTarget()
    msg.header = header

    if match is None:
        msg.detected = False
        return msg

    msg.detected = True
    msg.opening_center = Point(
        x=float(match["opening_center"][0]),
        y=float(match["opening_center"][1]),
        z=0.0,
    )
    msg.heading = float(match["heading"])
    msg.width = float(match["width"])
    msg.depth = float(match["depth"])
    msg.confidence = float(match["confidence"])
    return msg


def main(args=None):
    rclpy.init(args=args)
    node = DockDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
