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
    6. Match the global segment list against a U template: one long "back
       wall" (the wall the boat lies against, ~4 m) flanked by two shorter
       perpendicular "arms" (~2 m) — same U-matching algorithm as
       dock_detector_node's _find_u_shapes, reparametrized for Task 3.2's
       ~4m-opening/~2m-arm berth instead of Task 3.1's ~2m-opening/~2m-arm
       one. Matching against the full U (not just a bare length-filtered
       segment) both raises detection confidence (three corroborating
       segments, not one) and means a lone ~4m wall with no arms attached
       is correctly NOT mistaken for this berth.
    7. From each accepted match, compute the standoff aim point and
       wall-parallel heading from the back wall's own two endpoints (NOT
       the U's opening/arm-tip geometry — Task 3.2 needs the boat to end
       up alongside the back wall itself, unlike Task 3.1 which enters
       through the opening toward it).
    8. Classify each candidate as occupied or free.
    9. Publish all matches and the highest-confidence free match.

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

import itertools
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


def _angle_difference_mod_pi(first_angle, second_angle):
    """Return undirected line angle difference in [0, pi / 2].

    Identical to dock_detector_node's helper of the same name.
    """
    difference = (first_angle - second_angle) % math.pi
    return min(difference, math.pi - difference)


def _near_far(segment, origin):
    """Return segment endpoints ordered by distance from the origin."""
    first_distance = float(np.hypot(*(segment["p1"] - origin)))
    second_distance = float(np.hypot(*(segment["p2"] - origin)))

    if first_distance <= second_distance:
        return (segment["p1"], segment["p2"])

    return (segment["p2"], segment["p1"])


def _point_to_segment_distance(point, segment_start, segment_end):
    """Return clamped Euclidean distance from point to line segment."""
    segment_vector = segment_end - segment_start
    segment_length_squared = float(np.dot(segment_vector, segment_vector))

    if segment_length_squared < 1e-9:
        return float(np.hypot(*(point - segment_start)))

    projection_ratio = float(
        np.clip(
            np.dot(point - segment_start, segment_vector) / segment_length_squared,
            0.0,
            1.0,
        )
    )

    projected_point = segment_start + projection_ratio * segment_vector

    return float(np.hypot(*(point - projected_point)))


def _find_u_walls(
    segments,
    berth_width,
    width_tolerance,
    arm_length_min,
    arm_length_max,
    back_wall_length_min,
    back_wall_length_max,
    parallel_tolerance,
    perpendicular_tolerance,
    corner_gap_tolerance,
):
    """Find U-shaped berth candidates: a back wall flanked by two
    perpendicular arms, opening toward the boat.

    Same matching algorithm as dock_detector_node's _find_u_shapes
    (duplicated, not imported — see module docstring), reparametrized for
    Task 3.2's back-wall-is-the-target geometry: unlike Task 3.1, this
    returns the back wall's own endpoints (back_p1/back_p2), not the U's
    opening_center, since the boat needs to end up alongside the back wall
    itself rather than entering through the opening toward it.
    """
    candidates = []

    if len(segments) < 3:
        return candidates

    ORIGIN_LOCAL = np.array([0.0, 0.0], dtype=np.float32)

    for back_index, back_segment in enumerate(segments):
        back_length = _segment_length(back_segment)

        if not (back_wall_length_min <= back_length <= back_wall_length_max):
            continue

        other_indices = [i for i in range(len(segments)) if i != back_index]

        for first_arm_index, second_arm_index in itertools.combinations(other_indices, 2):
            first_arm = segments[first_arm_index]
            second_arm = segments[second_arm_index]

            first_arm_length = _segment_length(first_arm)
            second_arm_length = _segment_length(second_arm)

            if not (arm_length_min <= first_arm_length <= arm_length_max):
                continue
            if not (arm_length_min <= second_arm_length <= arm_length_max):
                continue

            first_arm_angle = _segment_angle(first_arm)
            second_arm_angle = _segment_angle(second_arm)
            back_angle = _segment_angle(back_segment)

            parallel_error = _angle_difference_mod_pi(first_arm_angle, second_arm_angle)
            if parallel_error > parallel_tolerance:
                continue

            first_perpendicular_error = abs(
                _angle_difference_mod_pi(first_arm_angle, back_angle) - math.pi / 2.0
            )
            second_perpendicular_error = abs(
                _angle_difference_mod_pi(second_arm_angle, back_angle) - math.pi / 2.0
            )

            if (
                first_perpendicular_error > perpendicular_tolerance
                or second_perpendicular_error > perpendicular_tolerance
            ):
                continue

            first_near, first_far = _near_far(first_arm, ORIGIN_LOCAL)
            second_near, second_far = _near_far(second_arm, ORIGIN_LOCAL)

            back_start = back_segment["p1"]
            back_end = back_segment["p2"]

            first_corner_gap = _point_to_segment_distance(first_far, back_start, back_end)
            second_corner_gap = _point_to_segment_distance(second_far, back_start, back_end)
            average_corner_gap = (first_corner_gap + second_corner_gap) / 2.0

            if average_corner_gap > corner_gap_tolerance:
                continue

            measured_width = float(np.hypot(*(first_near - second_near)))
            width_error = abs(measured_width - berth_width)

            if width_error > width_tolerance:
                continue

            inlier_ratio = sum(
                segment["inliers"] / max(segment["total"], 1)
                for segment in (first_arm, second_arm, back_segment)
            ) / 3.0

            error_score = (
                parallel_error / parallel_tolerance
                + (first_perpendicular_error + second_perpendicular_error)
                / 2.0
                / perpendicular_tolerance
                + width_error / width_tolerance
                + average_corner_gap / corner_gap_tolerance
            ) / 4.0

            confidence = max(0.0, min(1.0, 1.0 - error_score)) * inlier_ratio

            segment_indices = frozenset((back_index, first_arm_index, second_arm_index))

            candidates.append(
                {
                    "back_p1": back_start,
                    "back_p2": back_end,
                    "back_length": back_length,
                    "confidence": confidence,
                    "segment_idxs": segment_indices,
                    "wall_inlier_idx": np.unique(
                        np.concatenate(
                            (
                                first_arm["inlier_idx"],
                                second_arm["inlier_idx"],
                                back_segment["inlier_idx"],
                            )
                        )
                    ),
                }
            )

    candidates.sort(key=lambda match: match["confidence"], reverse=True)

    kept_candidates = []
    for candidate in candidates:
        duplicate = any(
            len(candidate["segment_idxs"] & existing["segment_idxs"]) >= 2
            for existing in kept_candidates
        )
        if duplicate:
            continue
        kept_candidates.append(candidate)

    return kept_candidates


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

        # Expected U-shaped berth geometry: a ~4m back wall (the boat lies
        # against this) flanked by two ~2m perpendicular arms, opening
        # ~4m wide (arms attach at the two ends of the back wall). Per the
        # Njord 2026 spec — NOT independently measured on-site, see this
        # node's module docstring. Same parameter names/roles as
        # dock_detector_node's U-matcher, different defaults.
        self.declare_parameter("back_wall_length_min_m", 3.0)
        self.declare_parameter("back_wall_length_max_m", 5.0)
        self.declare_parameter("berth_width_m", 4.0)
        self.declare_parameter("width_tolerance_m", 0.6)
        self.declare_parameter("arm_length_min_m", 1.5)
        self.declare_parameter("arm_length_max_m", 2.5)
        self.declare_parameter("parallel_angle_tol_deg", 12.0)
        self.declare_parameter("perp_angle_tol_deg", 12.0)
        self.declare_parameter("corner_gap_tol_m", 0.35)

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

        self._back_wall_length_min = float(get_parameter("back_wall_length_min_m").value)
        self._back_wall_length_max = float(get_parameter("back_wall_length_max_m").value)
        self._berth_width = float(get_parameter("berth_width_m").value)
        self._width_tolerance = float(get_parameter("width_tolerance_m").value)
        self._arm_length_min = float(get_parameter("arm_length_min_m").value)
        self._arm_length_max = float(get_parameter("arm_length_max_m").value)
        self._parallel_tolerance = math.radians(
            float(get_parameter("parallel_angle_tol_deg").value)
        )
        self._perpendicular_tolerance = math.radians(
            float(get_parameter("perp_angle_tol_deg").value)
        )
        self._corner_gap_tolerance = float(get_parameter("corner_gap_tol_m").value)

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
        """Match the global segment list against the U-shaped berth
        template, then compute the standoff aim point / heading for each
        accepted back wall."""
        u_matches = _find_u_walls(
            segments,
            berth_width=self._berth_width,
            width_tolerance=self._width_tolerance,
            arm_length_min=self._arm_length_min,
            arm_length_max=self._arm_length_max,
            back_wall_length_min=self._back_wall_length_min,
            back_wall_length_max=self._back_wall_length_max,
            parallel_tolerance=self._parallel_tolerance,
            perpendicular_tolerance=self._perpendicular_tolerance,
            corner_gap_tolerance=self._corner_gap_tolerance,
        )

        candidates = []

        for u_match in u_matches:
            back_p1 = u_match["back_p1"]
            back_p2 = u_match["back_p2"]
            length = u_match["back_length"]

            angle = math.atan2(
                float(back_p2[1] - back_p1[1]), float(back_p2[0] - back_p1[0])
            )
            heading = _canonicalize_heading(angle)

            midpoint = (back_p1 + back_p2) / 2.0

            # Perpendicular from the wall line toward the boat (origin) —
            # this picks which side of the wall the boat is currently on,
            # which is also the side the standoff aim point should sit on.
            lateral_axis = np.array(
                [-math.sin(heading), math.cos(heading)], dtype=np.float32
            )

            vector_to_boat = ORIGIN - midpoint
            lateral_sign = 1.0 if float(np.dot(vector_to_boat, lateral_axis)) >= 0.0 else -1.0

            aim_point = midpoint + lateral_sign * self._standoff_distance * lateral_axis

            candidates.append(
                {
                    "aim_point": aim_point,
                    "midpoint": midpoint,
                    "heading": heading,
                    "length": length,
                    "confidence": u_match["confidence"],
                    "lateral_sign": lateral_sign,
                    "wall_inlier_idx": u_match["wall_inlier_idx"],
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
