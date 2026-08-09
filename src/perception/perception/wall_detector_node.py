"""LiDAR-based detector for a straight pier/berth wall (Task 3.2 "parallel
docking").

Input:
    /obstacles/lidar
        sensor_msgs/msg/PointCloud2

Outputs:
    /perception/wall_targets
        njord_msgs/msg/WallTargetArray

    /perception/wall_target
        njord_msgs/msg/WallTarget

Pipeline (same shape as dock_detector_node's, deliberately duplicated
rather than imported from it — dock_detector_node is the tested Task 3.1
path and this module should not be able to regress it):
    1. Decode the PointCloud2 scan.
    2. Rotate LiDAR-local coordinates into base_link coordinates.
    3. Separate point groups with DBSCAN.
    4. Extract straight wall segments from every DBSCAN cluster using RANSAC.
    5. Combine the wall segments from all clusters into one global list.
    6. Keep segments whose length matches the expected berth wall length.
    7. Classify each candidate as occupied or free.
    8. Publish all matches and the highest-confidence free match.

There is no "pier"/"wall" class in the YOLO segmentation model
(vision/node.py's class_id 0-5 are buoy/cardinal marks only), so — same as
Task 3.1's own dock_detector_node — this is LIDAR-only. There is nothing
for a vision fusion step to add here.

UNVERIFIED: this pipeline has never been run against a real berth wall
(on the bench or in the water) — geometry parameters below (expected wall
length, standoff distance) are best-effort defaults, not measured values.
See DockingParallelTask in boat_bt/bt_xml/simple_boat.xml for the
consuming controller's own caveats.
"""

import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from njord_msgs.msg import WallTarget, WallTargetArray
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN


def _read_xy(cloud):
    """Extract an Nx2 float32 array from PointCloud2 XYZ fields."""
    point_count = cloud.width * cloud.height

    points = np.empty((point_count, 2), dtype=np.float32)

    for index in range(point_count):
        offset = index * cloud.point_step

        x_coordinate, y_coordinate, _ = struct.unpack_from(
            "fff", cloud.data, offset
        )

        points[index] = (x_coordinate, y_coordinate)

    finite_mask = np.isfinite(points).all(axis=1)

    return points[finite_mask]


def _ransac_lines(points, dist_threshold, iterations, min_inliers, max_lines, rng):
    """Extract straight 2D wall segments using iterative RANSAC.

    Identical algorithm to dock_detector_node's _ransac_lines — see that
    module for the derivation. Duplicated here rather than imported so
    this file has no import-time dependency on dock_detector_node.
    """
    remaining_indices = np.arange(points.shape[0])

    segments = []

    for _ in range(max_lines):
        remaining_points = points[remaining_indices]
        point_count = len(remaining_points)

        if point_count < min_inliers:
            break

        best_mask = None
        best_count = 0
        best_origin = None
        best_direction = None

        for _ in range(iterations):
            selected = rng.choice(point_count, size=2, replace=False)

            first_point = remaining_points[selected[0]]
            second_point = remaining_points[selected[1]]

            direction = second_point - first_point
            norm = math.hypot(float(direction[0]), float(direction[1]))

            if norm < 1e-6:
                continue

            direction_x = direction[0] / norm
            direction_y = direction[1] / norm

            normal_x = -direction_y
            normal_y = direction_x

            relative = remaining_points - first_point

            distances = np.abs(
                relative[:, 0] * normal_x + relative[:, 1] * normal_y
            )

            mask = distances <= dist_threshold
            inlier_count = int(np.count_nonzero(mask))

            if inlier_count > best_count:
                best_count = inlier_count
                best_mask = mask
                best_origin = first_point
                best_direction = np.array(
                    [direction_x, direction_y], dtype=np.float32
                )

        if best_mask is None or best_count < min_inliers:
            break

        inlier_points = remaining_points[best_mask]

        projections = (inlier_points - best_origin) @ best_direction

        minimum_projection = float(np.min(projections))
        maximum_projection = float(np.max(projections))

        segment_start = best_origin + minimum_projection * best_direction
        segment_end = best_origin + maximum_projection * best_direction

        segments.append(
            {
                "p1": segment_start,
                "p2": segment_end,
                "inliers": best_count,
                "total": point_count,
                "inlier_idx": remaining_indices[best_mask],
            }
        )

        remaining_indices = remaining_indices[~best_mask]

    return segments


def _segment_vector(segment):
    return segment["p2"] - segment["p1"]


def _segment_length(segment):
    return float(np.hypot(*_segment_vector(segment)))


def _segment_angle(segment):
    vector = _segment_vector(segment)
    return math.atan2(float(vector[1]), float(vector[0]))


def _canonicalize_heading(angle):
    """Fold a line's direction into the branch within +/-90 deg of +x.

    A RANSAC-fit line direction is arbitrary up to a 180 deg flip. The
    controller wants "heading correction needed to lie parallel to the
    wall", so the branch closer to the boat's current heading (base_link
    +x) is the meaningful one.

    KNOWN LIMITATION, confirmed via test_wall_detector.py: when the boat is
    heading exactly dead-on at the wall (wall direction exactly +/-90 deg
    from +x), the fold boundary itself is ambiguous -- which side RANSAC's
    arbitrary two-point line pick lands on becomes a coin flip, scan to
    scan, with no temporal smoothing to break the tie (this node has none,
    same as dock_detector_node -- see TODOS.md's "Temporal filtering/
    tracking" item, not fixed here either). In practice a real approach is
    very unlikely to sit exactly on that boundary rather than drifting
    slightly to one side, so this hasn't been treated as blocking.
    """
    folded = math.atan2(math.sin(angle), math.cos(angle))

    if folded > math.pi / 2.0:
        folded -= math.pi
    elif folded < -math.pi / 2.0:
        folded += math.pi

    return folded


def _classify_occupied(match, all_points, standoff, margin, min_points):
    """Determine whether unexplained scan points occupy the standoff band.

    Same idea as dock_detector_node's _classify_occupied: points that fall
    inside the zone between the wall and the boat's intended standoff
    line, within the wall's length span, and are not part of the wall's
    own RANSAC inliers, likely mean something is already moored there.
    """
    if len(all_points) == 0:
        return False

    heading = match["heading"]

    longitudinal_axis = np.array([math.cos(heading), math.sin(heading)])
    lateral_axis = np.array([-math.sin(heading), math.cos(heading)])

    relative_points = all_points - match["midpoint"]

    longitudinal_distances = relative_points @ longitudinal_axis
    lateral_distances = relative_points @ lateral_axis

    half_length = match["length"] / 2.0
    usable_half_length = max(0.0, half_length - margin)

    # match["lateral_sign"] > 0 means the boat is on the +lateral_axis
    # side of the wall; the standoff band sits between the wall (0) and
    # the boat's approach line (standoff), on that same side.
    band_near = margin
    band_far = max(band_near, standoff - margin)

    signed_lateral = match["lateral_sign"] * lateral_distances

    inside = (
        (np.abs(longitudinal_distances) <= usable_half_length)
        & (signed_lateral >= band_near)
        & (signed_lateral <= band_far)
    )

    inside_indices = set(np.nonzero(inside)[0].tolist())
    explained_indices = set(match["wall_inlier_idx"].tolist())

    unexplained_indices = inside_indices - explained_indices

    return len(unexplained_indices) >= min_points


def _to_message(header, match):
    """Convert an internal match dictionary into WallTarget."""
    message = WallTarget()
    message.header = header

    if match is None:
        message.detected = False
        message.occupied = False
        return message

    message.detected = True
    message.aim_point = Point(
        x=float(match["aim_point"][0]),
        y=float(match["aim_point"][1]),
        z=0.0,
    )
    message.heading = float(match["heading"])
    message.length = float(match["length"])
    message.confidence = float(match["confidence"])
    message.occupied = bool(match.get("occupied", False))

    return message


ORIGIN = np.array([0.0, 0.0], dtype=np.float32)


class WallDetectorNode(Node):
    """Detect a straight pier/berth wall from a 2D LiDAR point cloud."""

    def __init__(self):
        super().__init__("wall_detector_node")

        # LiDAR mounting calibration — same default as dock_detector_node
        # (same physical sensor).
        self.declare_parameter("lidar_yaw_offset_deg", 90.0)

        # DBSCAN point clustering.
        self.declare_parameter("cluster_eps", 0.6)
        self.declare_parameter("cluster_min_samples", 3)

        # RANSAC line extraction.
        self.declare_parameter("ransac_dist_threshold_m", 0.03)
        self.declare_parameter("ransac_iterations", 200)
        self.declare_parameter("ransac_min_inliers", 10)
        self.declare_parameter("max_lines_per_cluster", 8)

        # Expected berth wall geometry. Per the Njord 2026 spec's ~4m
        # berth width — NOT independently measured on-site, see this
        # node's module docstring.
        self.declare_parameter("wall_length_min_m", 2.5)
        self.declare_parameter("wall_length_max_m", 5.5)

        # How far off the wall the boat should aim to stop before the
        # close-range controller's own final-approach step closes the
        # rest of the gap. Placeholder — not measured against the boat's
        # actual beam; tune once available.
        self.declare_parameter("standoff_distance_m", 0.5)

        # Occupancy classification.
        self.declare_parameter("occupancy_margin_m", 0.15)
        self.declare_parameter("occupancy_min_points", 3)

        # Diagnostics.
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_period_sec", 1.0)

        get_parameter = self.get_parameter

        lidar_yaw = math.radians(float(get_parameter("lidar_yaw_offset_deg").value))
        self._cos_yaw = math.cos(lidar_yaw)
        self._sin_yaw = math.sin(lidar_yaw)

        self._cluster_eps = float(get_parameter("cluster_eps").value)
        self._cluster_min_samples = int(get_parameter("cluster_min_samples").value)

        self._ransac_dist_threshold = float(
            get_parameter("ransac_dist_threshold_m").value
        )
        self._ransac_iterations = int(get_parameter("ransac_iterations").value)
        self._ransac_min_inliers = int(get_parameter("ransac_min_inliers").value)
        self._max_lines_per_cluster = int(
            get_parameter("max_lines_per_cluster").value
        )

        self._wall_length_min = float(get_parameter("wall_length_min_m").value)
        self._wall_length_max = float(get_parameter("wall_length_max_m").value)

        self._standoff_distance = float(get_parameter("standoff_distance_m").value)

        self._occupancy_margin = float(get_parameter("occupancy_margin_m").value)
        self._occupancy_min_points = int(get_parameter("occupancy_min_points").value)

        self._debug = bool(get_parameter("debug").value)
        self._debug_period_ns = int(
            float(get_parameter("debug_period_sec").value) * 1e9
        )
        self._last_debug_ns = 0

        self._rng = np.random.default_rng()

        self._target_publisher = self.create_publisher(
            WallTarget, "/perception/wall_target", 10
        )
        self._targets_publisher = self.create_publisher(
            WallTargetArray, "/perception/wall_targets", 10
        )

        self._lidar_subscription = self.create_subscription(
            PointCloud2, "/obstacles/lidar", self._point_cloud_callback, 10
        )

        self.get_logger().info(
            "wall_detector_node ready — UNVERIFIED against a real berth "
            "wall, see module docstring"
        )

    def _debug_due(self):
        if not self._debug:
            return False

        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self._last_debug_ns < self._debug_period_ns:
            return False

        self._last_debug_ns = now_ns
        return True

    def _rotate_into_base_link(self, points):
        if len(points) == 0:
            return points

        if self._cos_yaw == 1.0 and self._sin_yaw == 0.0:
            return points

        rotated = points.copy()

        x_coordinates = points[:, 0]
        y_coordinates = points[:, 1]

        rotated[:, 0] = self._cos_yaw * x_coordinates - self._sin_yaw * y_coordinates
        rotated[:, 1] = self._sin_yaw * x_coordinates + self._cos_yaw * y_coordinates

        return rotated

    def _point_cloud_callback(self, message):
        points = _read_xy(message)
        debug_this_scan = self._debug_due()

        points = self._rotate_into_base_link(points)

        header = message.header
        header.frame_id = "base_link"

        global_segments = []
        cluster_count = 0
        noise_count = 0

        if len(points) >= self._cluster_min_samples:
            labels = DBSCAN(
                eps=self._cluster_eps, min_samples=self._cluster_min_samples
            ).fit_predict(points)

            valid_labels = sorted(
                int(label) for label in set(labels) if label != -1
            )

            cluster_count = len(valid_labels)
            noise_count = int(np.count_nonzero(labels == -1))

            for label in valid_labels:
                cluster_global_indices = np.nonzero(labels == label)[0]
                cluster_points = points[cluster_global_indices]

                if len(cluster_points) < self._ransac_min_inliers:
                    continue

                segments = _ransac_lines(
                    cluster_points,
                    dist_threshold=self._ransac_dist_threshold,
                    iterations=self._ransac_iterations,
                    min_inliers=self._ransac_min_inliers,
                    max_lines=self._max_lines_per_cluster,
                    rng=self._rng,
                )

                for segment in segments:
                    segment["inlier_idx"] = cluster_global_indices[
                        segment["inlier_idx"]
                    ]
                    global_segments.append(segment)

        all_matches = self._find_walls(global_segments)

        for match in all_matches:
            match["occupied"] = _classify_occupied(
                match=match,
                all_points=points,
                standoff=self._standoff_distance,
                margin=self._occupancy_margin,
                min_points=self._occupancy_min_points,
            )

        array_message = WallTargetArray()
        array_message.header = header
        array_message.targets = [_to_message(header, match) for match in all_matches]
        self._targets_publisher.publish(array_message)

        free_matches = [match for match in all_matches if not match["occupied"]]

        best_free_match = max(
            free_matches, key=lambda match: match["confidence"], default=None
        )

        self._target_publisher.publish(_to_message(header, best_free_match))

        if debug_this_scan:
            self.get_logger().info(
                "Wall debug summary: "
                f"points={len(points)}, clusters={cluster_count}, "
                f"noise={noise_count}, segments={len(global_segments)}, "
                f"matches={len(all_matches)}, "
                f"selected={'none' if best_free_match is None else 'yes'}"
            )

    def _find_walls(self, segments):
        """Keep RANSAC segments whose length matches the expected berth wall."""
        candidates = []

        for segment in segments:
            length = _segment_length(segment)

            if not (self._wall_length_min <= length <= self._wall_length_max):
                continue

            angle = _segment_angle(segment)
            heading = _canonicalize_heading(angle)

            midpoint = (segment["p1"] + segment["p2"]) / 2.0

            # Perpendicular from the wall line toward the boat (origin) —
            # this picks which side of the wall the boat is currently on,
            # which is also the side the standoff aim point should sit on.
            lateral_axis = np.array(
                [-math.sin(heading), math.cos(heading)], dtype=np.float32
            )

            vector_to_boat = ORIGIN - midpoint
            lateral_sign = 1.0 if float(np.dot(vector_to_boat, lateral_axis)) >= 0.0 else -1.0

            aim_point = midpoint + lateral_sign * self._standoff_distance * lateral_axis

            expected_length = (self._wall_length_min + self._wall_length_max) / 2.0
            length_error = abs(length - expected_length)
            length_span = (self._wall_length_max - self._wall_length_min) / 2.0

            inlier_ratio = segment["inliers"] / max(segment["total"], 1)

            confidence = max(
                0.0, min(1.0, 1.0 - length_error / max(length_span, 1e-6))
            ) * inlier_ratio

            candidates.append(
                {
                    "aim_point": aim_point,
                    "midpoint": midpoint,
                    "heading": heading,
                    "length": length,
                    "confidence": confidence,
                    "lateral_sign": lateral_sign,
                    "wall_inlier_idx": segment["inlier_idx"],
                }
            )

        candidates.sort(key=lambda match: match["confidence"], reverse=True)

        return candidates


def main(args=None):
    rclpy.init(args=args)

    node = WallDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
