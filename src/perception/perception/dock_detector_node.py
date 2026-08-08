"""LiDAR-based detector for U-shaped docking berths.

Input:
    /obstacles/lidar
        sensor_msgs/msg/PointCloud2

Outputs:
    /perception/dock_targets
        njord_msgs/msg/DockTargetArray

    /perception/dock_target
        njord_msgs/msg/DockTarget

Pipeline:
    1. Decode the PointCloud2 scan.
    2. Rotate LiDAR-local coordinates into base_link coordinates.
    3. Separate point groups with DBSCAN.
    4. Extract straight wall segments from every DBSCAN cluster using RANSAC.
    5. Combine the wall segments from all clusters into one global list.
    6. Search the global segment list for U-shaped berths.
    7. Classify each detected berth as occupied or free.
    8. Publish all matches and the highest-confidence free match.

The global segment matching is important because one physical berth can be
split into several DBSCAN clusters when point spacing at the wall corners is
larger than cluster_eps.
"""

import itertools
import math
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from njord_msgs.msg import DockTarget, DockTargetArray
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN


ORIGIN = np.array([0.0, 0.0], dtype=np.float32)


class DockDetectorNode(Node):
    """Detect U-shaped docking berths from a 2D LiDAR point cloud."""

    def __init__(self):
        super().__init__("dock_detector_node")

        # LiDAR mounting calibration.
        self.declare_parameter("lidar_yaw_offset_deg", 90.0)

        # DBSCAN point clustering.
        self.declare_parameter("cluster_eps", 0.6)
        self.declare_parameter("cluster_min_samples", 3)

        # RANSAC line extraction.
        self.declare_parameter("ransac_dist_threshold_m", 0.03)
        self.declare_parameter("ransac_iterations", 200)
        self.declare_parameter("ransac_min_inliers", 10)
        self.declare_parameter("max_lines_per_cluster", 8)

        # U-shape geometry.
        self.declare_parameter("berth_width_m", 2.0)
        self.declare_parameter("width_tolerance_m", 0.4)
        self.declare_parameter("arm_length_min_m", 0.6)
        self.declare_parameter("arm_length_max_m", 3.0)

        # A back wall can span several adjoining berths.
        self.declare_parameter("back_wall_length_min_m", 0.8)
        self.declare_parameter("back_wall_length_max_m", 8.0)

        self.declare_parameter("parallel_angle_tol_deg", 12.0)
        self.declare_parameter("perp_angle_tol_deg", 12.0)
        self.declare_parameter("corner_gap_tol_m", 0.35)

        # Occupancy classification.
        self.declare_parameter("occupancy_margin_m", 0.15)
        self.declare_parameter("occupancy_min_points", 3)

        # Diagnostics.
        self.declare_parameter("debug", True)
        self.declare_parameter("debug_period_sec", 1.0)

        get_parameter = self.get_parameter

        lidar_yaw = math.radians(
            float(get_parameter("lidar_yaw_offset_deg").value)
        )
        self._cos_yaw = math.cos(lidar_yaw)
        self._sin_yaw = math.sin(lidar_yaw)

        self._cluster_eps = float(
            get_parameter("cluster_eps").value
        )
        self._cluster_min_samples = int(
            get_parameter("cluster_min_samples").value
        )

        self._ransac_dist_threshold = float(
            get_parameter("ransac_dist_threshold_m").value
        )
        self._ransac_iterations = int(
            get_parameter("ransac_iterations").value
        )
        self._ransac_min_inliers = int(
            get_parameter("ransac_min_inliers").value
        )
        self._max_lines_per_cluster = int(
            get_parameter("max_lines_per_cluster").value
        )

        self._berth_width = float(
            get_parameter("berth_width_m").value
        )
        self._width_tolerance = float(
            get_parameter("width_tolerance_m").value
        )

        self._arm_length_min = float(
            get_parameter("arm_length_min_m").value
        )
        self._arm_length_max = float(
            get_parameter("arm_length_max_m").value
        )

        self._back_wall_length_min = float(
            get_parameter("back_wall_length_min_m").value
        )
        self._back_wall_length_max = float(
            get_parameter("back_wall_length_max_m").value
        )

        self._parallel_tolerance = math.radians(
            float(
                get_parameter(
                    "parallel_angle_tol_deg"
                ).value
            )
        )
        self._perpendicular_tolerance = math.radians(
            float(
                get_parameter(
                    "perp_angle_tol_deg"
                ).value
            )
        )
        self._corner_gap_tolerance = float(
            get_parameter("corner_gap_tol_m").value
        )

        self._occupancy_margin = float(
            get_parameter("occupancy_margin_m").value
        )
        self._occupancy_min_points = int(
            get_parameter("occupancy_min_points").value
        )

        self._debug = bool(
            get_parameter("debug").value
        )
        self._debug_period_ns = int(
            float(
                get_parameter(
                    "debug_period_sec"
                ).value
            )
            * 1e9
        )
        self._last_debug_ns = 0

        self._rng = np.random.default_rng()

        self._target_publisher = self.create_publisher(
            DockTarget,
            "/perception/dock_target",
            10,
        )
        self._targets_publisher = self.create_publisher(
            DockTargetArray,
            "/perception/dock_targets",
            10,
        )

        self._lidar_subscription = self.create_subscription(
            PointCloud2,
            "/obstacles/lidar",
            self._point_cloud_callback,
            10,
        )

        self.get_logger().info(
            "dock_detector_node ready — global cross-cluster "
            "segment matching enabled"
        )

    def _debug_due(self):
        """Return True at most once per configured debug interval."""
        if not self._debug:
            return False

        now_ns = self.get_clock().now().nanoseconds

        if (
            now_ns - self._last_debug_ns
            < self._debug_period_ns
        ):
            return False

        self._last_debug_ns = now_ns
        return True

    def _point_cloud_callback(self, message):
        """Process one PointCloud2 scan."""
        points = _read_xy(message)
        debug_this_scan = self._debug_due()

        points = self._rotate_into_base_link(points)

        header = message.header
        header.frame_id = "base_link"

        cluster_debug = []
        cluster_count = 0
        noise_count = 0

        # Important:
        # Segments from every DBSCAN cluster are added to this shared list.
        global_segments = []

        if len(points) >= self._cluster_min_samples:
            labels = DBSCAN(
                eps=self._cluster_eps,
                min_samples=self._cluster_min_samples,
            ).fit_predict(points)

            valid_labels = sorted(
                int(label)
                for label in set(labels)
                if label != -1
            )

            cluster_count = len(valid_labels)
            noise_count = int(
                np.count_nonzero(labels == -1)
            )

            for label in valid_labels:
                cluster_global_indices = np.nonzero(
                    labels == label
                )[0]

                cluster_points = points[
                    cluster_global_indices
                ]

                cluster_info = {
                    "label": label,
                    "points": int(len(cluster_points)),
                    "segments": 0,
                    "segment_details": [],
                    "stopped": "",
                }

                if (
                    len(cluster_points)
                    < self._ransac_min_inliers
                ):
                    cluster_info["stopped"] = (
                        "fewer than "
                        f"{self._ransac_min_inliers} "
                        "RANSAC points"
                    )
                    cluster_debug.append(cluster_info)
                    continue

                segments = _ransac_lines(
                    cluster_points,
                    dist_threshold=(
                        self._ransac_dist_threshold
                    ),
                    iterations=(
                        self._ransac_iterations
                    ),
                    min_inliers=(
                        self._ransac_min_inliers
                    ),
                    max_lines=(
                        self._max_lines_per_cluster
                    ),
                    rng=self._rng,
                )

                cluster_info["segments"] = int(
                    len(segments)
                )

                for segment in segments:
                    # Convert cluster-local point indices back to indices
                    # in the complete scan.
                    segment["inlier_idx"] = (
                        cluster_global_indices[
                            segment["inlier_idx"]
                        ]
                    )

                    segment["cluster_label"] = label

                    global_segments.append(segment)

                    cluster_info[
                        "segment_details"
                    ].append(
                        {
                            "length": _segment_length(
                                segment
                            ),
                            "inliers": int(
                                segment["inliers"]
                            ),
                            "p1": segment["p1"],
                            "p2": segment["p2"],
                        }
                    )

                if not segments:
                    cluster_info["stopped"] = (
                        "no RANSAC wall segments"
                    )

                cluster_debug.append(cluster_info)

        # U-shape matching now happens once, after all DBSCAN clusters
        # have contributed their wall segments.
        all_matches = _find_u_shapes(
            segments=global_segments,
            berth_width=self._berth_width,
            width_tolerance=self._width_tolerance,
            arm_length_min=self._arm_length_min,
            arm_length_max=self._arm_length_max,
            back_wall_length_min=(
                self._back_wall_length_min
            ),
            back_wall_length_max=(
                self._back_wall_length_max
            ),
            parallel_tolerance=(
                self._parallel_tolerance
            ),
            perpendicular_tolerance=(
                self._perpendicular_tolerance
            ),
            corner_gap_tolerance=(
                self._corner_gap_tolerance
            ),
            debug=debug_this_scan,
            logger=self.get_logger(),
        )

        for match in all_matches:
            match["occupied"] = _classify_occupied(
                match=match,
                all_points=points,
                margin=self._occupancy_margin,
                min_points=self._occupancy_min_points,
            )

        array_message = DockTargetArray()
        array_message.header = header
        array_message.targets = [
            _to_message(header, match)
            for match in all_matches
        ]
        self._targets_publisher.publish(array_message)

        free_matches = [
            match
            for match in all_matches
            if not match["occupied"]
        ]

        best_free_match = max(
            free_matches,
            key=lambda match: match["confidence"],
            default=None,
        )

        self._target_publisher.publish(
            _to_message(
                header,
                best_free_match,
            )
        )

        if debug_this_scan:
            self._log_pipeline_debug(
                points=points,
                cluster_count=cluster_count,
                noise_count=noise_count,
                cluster_debug=cluster_debug,
                global_segments=global_segments,
                all_matches=all_matches,
                best_free_match=best_free_match,
            )

    def _rotate_into_base_link(self, points):
        """Rotate raw LiDAR-local XY coordinates into base_link."""
        if len(points) == 0:
            return points

        if (
            self._cos_yaw == 1.0
            and self._sin_yaw == 0.0
        ):
            return points

        rotated = points.copy()

        x_coordinates = points[:, 0]
        y_coordinates = points[:, 1]

        rotated[:, 0] = (
            self._cos_yaw * x_coordinates
            - self._sin_yaw * y_coordinates
        )
        rotated[:, 1] = (
            self._sin_yaw * x_coordinates
            + self._cos_yaw * y_coordinates
        )

        return rotated

    def _log_pipeline_debug(
        self,
        points,
        cluster_count,
        noise_count,
        cluster_debug,
        global_segments,
        all_matches,
        best_free_match,
    ):
        """Publish compact diagnostics through the ROS logger."""
        occupied_count = sum(
            bool(match.get("occupied", False))
            for match in all_matches
        )

        free_count = (
            len(all_matches)
            - occupied_count
        )

        self.get_logger().info(
            "Dock debug summary: "
            f"points={len(points)}, "
            f"clusters={cluster_count}, "
            f"noise={noise_count}, "
            f"global_segments={len(global_segments)}, "
            f"matches={len(all_matches)}, "
            f"free={free_count}, "
            f"occupied={occupied_count}"
        )

        for info in cluster_debug:
            detail = (
                f"cluster={info['label']}, "
                f"points={info['points']}, "
                f"segments={info['segments']}"
            )

            if info["stopped"]:
                detail += (
                    f", stopped={info['stopped']}"
                )

            self.get_logger().info(
                f"Dock debug detail: {detail}"
            )

            for index, segment in enumerate(
                info["segment_details"]
            ):
                self.get_logger().info(
                    "Dock debug segment: "
                    f"cluster={info['label']}, "
                    f"index={index}, "
                    f"length={segment['length']:.2f}, "
                    f"inliers={segment['inliers']}, "
                    f"p1=("
                    f"{segment['p1'][0]:.2f}, "
                    f"{segment['p1'][1]:.2f}), "
                    f"p2=("
                    f"{segment['p2'][0]:.2f}, "
                    f"{segment['p2'][1]:.2f})"
                )

        if best_free_match is None:
            self.get_logger().info(
                "Dock debug result: "
                "no free berth selected"
            )
            return

        opening_center = best_free_match[
            "opening_center"
        ]

        self.get_logger().info(
            "Dock debug result: selected free berth "
            f"center=("
            f"{opening_center[0]:.2f}, "
            f"{opening_center[1]:.2f}), "
            f"width={best_free_match['width']:.2f}, "
            f"depth={best_free_match['depth']:.2f}, "
            f"confidence="
            f"{best_free_match['confidence']:.3f}"
        )


def _read_xy(cloud):
    """Extract an Nx2 float32 array from PointCloud2 XYZ fields."""
    point_count = cloud.width * cloud.height

    points = np.empty(
        (point_count, 2),
        dtype=np.float32,
    )

    for index in range(point_count):
        offset = index * cloud.point_step

        x_coordinate, y_coordinate, _ = (
            struct.unpack_from(
                "fff",
                cloud.data,
                offset,
            )
        )

        points[index] = (
            x_coordinate,
            y_coordinate,
        )

    finite_mask = np.isfinite(points).all(
        axis=1
    )

    return points[finite_mask]


def _ransac_lines(
    points,
    dist_threshold,
    iterations,
    min_inliers,
    max_lines,
    rng,
):
    """Extract straight 2D wall segments using iterative RANSAC."""
    remaining_indices = np.arange(
        points.shape[0]
    )

    segments = []

    for _ in range(max_lines):
        remaining_points = points[
            remaining_indices
        ]

        point_count = len(
            remaining_points
        )

        if point_count < min_inliers:
            break

        best_mask = None
        best_count = 0
        best_origin = None
        best_direction = None

        for _ in range(iterations):
            selected = rng.choice(
                point_count,
                size=2,
                replace=False,
            )

            first_point = remaining_points[
                selected[0]
            ]
            second_point = remaining_points[
                selected[1]
            ]

            direction = (
                second_point
                - first_point
            )

            norm = math.hypot(
                float(direction[0]),
                float(direction[1]),
            )

            if norm < 1e-6:
                continue

            direction_x = (
                direction[0] / norm
            )
            direction_y = (
                direction[1] / norm
            )

            normal_x = -direction_y
            normal_y = direction_x

            relative = (
                remaining_points
                - first_point
            )

            distances = np.abs(
                relative[:, 0] * normal_x
                + relative[:, 1] * normal_y
            )

            mask = (
                distances
                <= dist_threshold
            )

            inlier_count = int(
                np.count_nonzero(mask)
            )

            if inlier_count > best_count:
                best_count = inlier_count
                best_mask = mask
                best_origin = first_point
                best_direction = np.array(
                    [
                        direction_x,
                        direction_y,
                    ],
                    dtype=np.float32,
                )

        if (
            best_mask is None
            or best_count < min_inliers
        ):
            break

        inlier_points = remaining_points[
            best_mask
        ]

        projections = (
            inlier_points - best_origin
        ) @ best_direction

        minimum_projection = float(
            np.min(projections)
        )
        maximum_projection = float(
            np.max(projections)
        )

        segment_start = (
            best_origin
            + minimum_projection
            * best_direction
        )
        segment_end = (
            best_origin
            + maximum_projection
            * best_direction
        )

        segments.append(
            {
                "p1": segment_start,
                "p2": segment_end,
                "inliers": best_count,
                "total": point_count,
                "inlier_idx": (
                    remaining_indices[
                        best_mask
                    ]
                ),
            }
        )

        remaining_indices = (
            remaining_indices[
                ~best_mask
            ]
        )

    return segments


def _segment_vector(segment):
    return (
        segment["p2"]
        - segment["p1"]
    )


def _segment_length(segment):
    return float(
        np.hypot(
            *_segment_vector(segment)
        )
    )


def _segment_angle(segment):
    vector = _segment_vector(segment)

    return math.atan2(
        float(vector[1]),
        float(vector[0]),
    )


def _angle_difference_mod_pi(
    first_angle,
    second_angle,
):
    """Return undirected line angle difference in [0, pi / 2]."""
    difference = (
        first_angle - second_angle
    ) % math.pi

    return min(
        difference,
        math.pi - difference,
    )


def _near_far(segment, origin):
    """Return segment endpoints ordered by distance from the origin."""
    first_distance = float(
        np.hypot(
            *(
                segment["p1"]
                - origin
            )
        )
    )
    second_distance = float(
        np.hypot(
            *(
                segment["p2"]
                - origin
            )
        )
    )

    if first_distance <= second_distance:
        return (
            segment["p1"],
            segment["p2"],
        )

    return (
        segment["p2"],
        segment["p1"],
    )


def _point_to_segment_distance(
    point,
    segment_start,
    segment_end,
):
    """Return clamped Euclidean distance from point to line segment."""
    segment_vector = (
        segment_end
        - segment_start
    )

    segment_length_squared = float(
        np.dot(
            segment_vector,
            segment_vector,
        )
    )

    if segment_length_squared < 1e-9:
        return float(
            np.hypot(
                *(
                    point
                    - segment_start
                )
            )
        )

    projection_ratio = float(
        np.clip(
            np.dot(
                point - segment_start,
                segment_vector,
            )
            / segment_length_squared,
            0.0,
            1.0,
        )
    )

    projected_point = (
        segment_start
        + projection_ratio
        * segment_vector
    )

    return float(
        np.hypot(
            *(
                point
                - projected_point
            )
        )
    )


def _find_u_shapes(
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
    debug=False,
    logger=None,
):
    """Find all valid U-shaped berth candidates in a global segment list."""
    candidates = []

    if len(segments) < 3:
        return candidates

    for back_index, back_segment in enumerate(
        segments
    ):
        back_length = _segment_length(
            back_segment
        )

        if not (
            back_wall_length_min
            <= back_length
            <= back_wall_length_max
        ):
            continue

        other_indices = [
            index
            for index in range(len(segments))
            if index != back_index
        ]

        for (
            first_arm_index,
            second_arm_index,
        ) in itertools.combinations(
            other_indices,
            2,
        ):
            first_arm = segments[
                first_arm_index
            ]
            second_arm = segments[
                second_arm_index
            ]

            first_arm_length = (
                _segment_length(first_arm)
            )
            second_arm_length = (
                _segment_length(second_arm)
            )

            if not (
                arm_length_min
                <= first_arm_length
                <= arm_length_max
            ):
                continue

            if not (
                arm_length_min
                <= second_arm_length
                <= arm_length_max
            ):
                continue

            first_arm_angle = (
                _segment_angle(first_arm)
            )
            second_arm_angle = (
                _segment_angle(second_arm)
            )
            back_angle = (
                _segment_angle(back_segment)
            )

            parallel_error = (
                _angle_difference_mod_pi(
                    first_arm_angle,
                    second_arm_angle,
                )
            )

            if (
                parallel_error
                > parallel_tolerance
            ):
                continue

            first_perpendicular_error = abs(
                _angle_difference_mod_pi(
                    first_arm_angle,
                    back_angle,
                )
                - math.pi / 2.0
            )
            second_perpendicular_error = abs(
                _angle_difference_mod_pi(
                    second_arm_angle,
                    back_angle,
                )
                - math.pi / 2.0
            )

            if (
                first_perpendicular_error
                > perpendicular_tolerance
                or second_perpendicular_error
                > perpendicular_tolerance
            ):
                continue

            first_near, first_far = _near_far(
                first_arm,
                ORIGIN,
            )
            second_near, second_far = _near_far(
                second_arm,
                ORIGIN,
            )

            back_start = back_segment["p1"]
            back_end = back_segment["p2"]

            first_corner_gap = (
                _point_to_segment_distance(
                    first_far,
                    back_start,
                    back_end,
                )
            )
            second_corner_gap = (
                _point_to_segment_distance(
                    second_far,
                    back_start,
                    back_end,
                )
            )

            average_corner_gap = (
                first_corner_gap
                + second_corner_gap
            ) / 2.0

            if (
                average_corner_gap
                > corner_gap_tolerance
            ):
                continue

            measured_width = float(
                np.hypot(
                    *(
                        first_near
                        - second_near
                    )
                )
            )

            width_error = abs(
                measured_width
                - berth_width
            )

            if (
                width_error
                > width_tolerance
            ):
                continue

            opening_center = (
                first_near
                + second_near
            ) / 2.0

            back_midpoint = (
                back_start
                + back_end
            ) / 2.0

            back_direction = (
                back_end
                - back_start
            )

            back_direction_length = float(
                np.hypot(
                    *back_direction
                )
            )

            if (
                back_direction_length
                < 1e-6
            ):
                continue

            normal_x = (
                -back_direction[1]
                / back_direction_length
            )
            normal_y = (
                back_direction[0]
                / back_direction_length
            )

            vector_to_back = (
                back_midpoint
                - opening_center
            )

            if (
                normal_x * vector_to_back[0]
                + normal_y * vector_to_back[1]
                < 0.0
            ):
                normal_x = -normal_x
                normal_y = -normal_y

            heading = math.atan2(
                normal_y,
                normal_x,
            )

            depth = (
                first_arm_length
                + second_arm_length
            ) / 2.0

            inlier_ratio = sum(
                segment["inliers"]
                / max(
                    segment["total"],
                    1,
                )
                for segment in (
                    first_arm,
                    second_arm,
                    back_segment,
                )
            ) / 3.0

            error_score = (
                parallel_error
                / parallel_tolerance
                + (
                    first_perpendicular_error
                    + second_perpendicular_error
                )
                / 2.0
                / perpendicular_tolerance
                + width_error
                / width_tolerance
                + average_corner_gap
                / corner_gap_tolerance
            ) / 4.0

            confidence = (
                max(
                    0.0,
                    min(
                        1.0,
                        1.0 - error_score,
                    ),
                )
                * inlier_ratio
            )

            segment_indices = frozenset(
                (
                    back_index,
                    first_arm_index,
                    second_arm_index,
                )
            )

            candidate = {
                "opening_center": opening_center,
                "heading": heading,
                "width": measured_width,
                "depth": depth,
                "confidence": confidence,
                "segment_idxs": segment_indices,
                "wall_inlier_idx": np.unique(
                    np.concatenate(
                        (
                            first_arm[
                                "inlier_idx"
                            ],
                            second_arm[
                                "inlier_idx"
                            ],
                            back_segment[
                                "inlier_idx"
                            ],
                        )
                    )
                ),
            }

            candidates.append(candidate)

            if (
                debug
                and logger is not None
            ):
                logger.info(
                    "Dock match accepted: "
                    f"back={back_index}, "
                    f"arms=("
                    f"{first_arm_index},"
                    f"{second_arm_index}), "
                    f"clusters=("
                    f"{back_segment.get('cluster_label', -1)},"
                    f"{first_arm.get('cluster_label', -1)},"
                    f"{second_arm.get('cluster_label', -1)}), "
                    f"width={measured_width:.3f}, "
                    f"depth={depth:.3f}, "
                    f"confidence={confidence:.3f}"
                )

    candidates.sort(
        key=lambda match: (
            match["confidence"]
        ),
        reverse=True,
    )

    kept_candidates = []

    for candidate in candidates:
        duplicate = any(
            len(
                candidate["segment_idxs"]
                & existing["segment_idxs"]
            )
            >= 2
            for existing in kept_candidates
        )

        if duplicate:
            continue

        kept_candidates.append(candidate)

    return kept_candidates


def _classify_occupied(
    match,
    all_points,
    margin,
    min_points,
):
    """Determine whether unexplained scan points occupy the berth interior."""
    if len(all_points) == 0:
        return False

    heading = match["heading"]

    longitudinal_axis = np.array(
        [
            math.cos(heading),
            math.sin(heading),
        ]
    )
    lateral_axis = np.array(
        [
            -math.sin(heading),
            math.cos(heading),
        ]
    )

    relative_points = (
        all_points
        - match["opening_center"]
    )

    longitudinal_distances = (
        relative_points
        @ longitudinal_axis
    )
    lateral_distances = (
        relative_points
        @ lateral_axis
    )

    half_width = (
        match["width"] / 2.0
    )

    usable_half_width = max(
        0.0,
        half_width - margin,
    )
    usable_depth = max(
        0.0,
        match["depth"] - margin,
    )

    inside = (
        (longitudinal_distances >= margin)
        & (
            longitudinal_distances
            <= usable_depth
        )
        & (
            np.abs(lateral_distances)
            <= usable_half_width
        )
    )

    inside_indices = set(
        np.nonzero(inside)[0].tolist()
    )
    explained_indices = set(
        match[
            "wall_inlier_idx"
        ].tolist()
    )

    unexplained_indices = (
        inside_indices
        - explained_indices
    )

    return (
        len(unexplained_indices)
        >= min_points
    )


def _to_message(header, match):
    """Convert an internal match dictionary into DockTarget."""
    message = DockTarget()
    message.header = header

    if match is None:
        message.detected = False
        message.occupied = False
        return message

    message.detected = True
    message.opening_center = Point(
        x=float(
            match["opening_center"][0]
        ),
        y=float(
            match["opening_center"][1]
        ),
        z=0.0,
    )
    message.heading = float(
        match["heading"]
    )
    message.width = float(
        match["width"]
    )
    message.depth = float(
        match["depth"]
    )
    message.confidence = float(
        match["confidence"]
    )
    message.occupied = bool(
        match.get(
            "occupied",
            False,
        )
    )

    return message


def main(args=None):
    """Run the ROS 2 node."""
    rclpy.init(args=args)

    node = DockDetectorNode()

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