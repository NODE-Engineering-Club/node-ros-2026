"""Synthetic dock-scene point generation for exercising dock_detector_node
without Gazebo.

Models the two-berth dock structure and decoy boat defined in
description/worlds/dockingWorldOccupied.sdf directly in a plain
boat-forward-x local frame (matching that world's pre-yaw local-frame
authoring). Only the INNER (berth-facing) surface of each wall is sampled
-- the surface a LiDAR grazing into the berth opening would actually
return -- not full 3D ray-casting/occlusion, which is intentionally out of
scope here: the required real-Gazebo run (see the docking plan) is the
fidelity check for physical effects like occlusion; this harness tests the
clustering/RANSAC/U-matching/occupancy ALGORITHM against known-correct
points.

Points are generated directly in a "base_link-equivalent" frame (dead ahead
= +x) and then rotated into the "raw lidar" frame the node expects on
/obstacles/lidar, via the exact inverse of the node's own
lidar_yaw_offset_deg correction -- so this test exercises the node with its
stock default parameters end to end, no parameter overrides needed.
"""

import struct

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField

# Local-frame wall geometry (matches dockingWorldOccupied.sdf's pre-yaw
# local-frame authoring exactly).
OPENING_X = 5.0
BACK_WALL_INNER_X = 7.0
ARM_Y = {"left": 2.10, "middle": 0.0, "right": -2.10}
ARM_HALF_THICK = 0.05

BERTH_A_CENTER_Y = (ARM_Y["left"] - ARM_HALF_THICK + ARM_Y["middle"] + ARM_HALF_THICK) / 2.0  # ~1.05
BERTH_B_CENTER_Y = (ARM_Y["middle"] - ARM_HALF_THICK + ARM_Y["right"] + ARM_HALF_THICK) / 2.0  # ~-1.05
BERTH_CENTER_X = (OPENING_X + BACK_WALL_INNER_X) / 2.0  # 6.0

DECOY_LENGTH = 1.3
DECOY_BEAM = 0.5


def _line(p1, p2, spacing):
    p1, p2 = np.array(p1, dtype=float), np.array(p2, dtype=float)
    length = float(np.hypot(*(p2 - p1)))
    n = max(2, int(length / spacing) + 1)
    return [tuple(p1 + (p2 - p1) * t) for t in np.linspace(0.0, 1.0, n)]


def dock_wall_points(spacing=0.08):
    """Points along the inner (berth-facing) surface of all 4 walls."""
    pts = []
    pts += _line((OPENING_X, ARM_Y["left"] - ARM_HALF_THICK),
                 (BACK_WALL_INNER_X, ARM_Y["left"] - ARM_HALF_THICK), spacing)
    pts += _line((OPENING_X, ARM_Y["middle"] + ARM_HALF_THICK),
                 (BACK_WALL_INNER_X, ARM_Y["middle"] + ARM_HALF_THICK), spacing)  # middle, berth A side
    pts += _line((OPENING_X, ARM_Y["middle"] - ARM_HALF_THICK),
                 (BACK_WALL_INNER_X, ARM_Y["middle"] - ARM_HALF_THICK), spacing)  # middle, berth B side
    pts += _line((OPENING_X, ARM_Y["right"] + ARM_HALF_THICK),
                 (BACK_WALL_INNER_X, ARM_Y["right"] + ARM_HALF_THICK), spacing)
    pts += _line((BACK_WALL_INNER_X, ARM_Y["left"] - ARM_HALF_THICK),
                 (BACK_WALL_INNER_X, ARM_Y["right"] + ARM_HALF_THICK), spacing)
    return pts


def decoy_hull_points(occupied_berth, spacing=0.08):
    """Rectangular outline approximating the decoy hull footprint, centered
    in the given berth ("A" or "B")."""
    cy = BERTH_A_CENTER_Y if occupied_berth == "A" else BERTH_B_CENTER_Y
    cx = BERTH_CENTER_X
    hl, hb = DECOY_LENGTH / 2.0, DECOY_BEAM / 2.0
    corners = [
        (cx - hl, cy - hb), (cx + hl, cy - hb),
        (cx + hl, cy + hb), (cx - hl, cy + hb),
    ]
    pts = []
    for i in range(4):
        pts += _line(corners[i], corners[(i + 1) % 4], spacing)
    return pts


def vantage_pose(distance, angle_deg):
    """Boat pose (x, y, heading_rad) `distance` m from the dock opening
    center, at bearing `angle_deg` off dead-ahead, heading pointed at the
    opening center (mirrors approaching a fixed GPS waypoint from an
    off-axis start, as in the real Task 3.1 point-7 -> point-8 approach)."""
    angle_rad = np.radians(angle_deg)
    target = np.array([OPENING_X, 0.0])
    boat = target + distance * np.array([-np.cos(angle_rad), np.sin(angle_rad)])
    heading = float(np.arctan2(target[1] - boat[1], target[0] - boat[0]))
    return float(boat[0]), float(boat[1]), heading


def to_boat_frame(points, boat_x, boat_y, boat_heading):
    """World-frame (x, y) points -> boat-relative, dead-ahead = +x."""
    c, s = np.cos(-boat_heading), np.sin(-boat_heading)
    out = []
    for x, y in points:
        dx, dy = x - boat_x, y - boat_y
        out.append((c * dx - s * dy, s * dx + c * dy))
    return out


def to_raw_lidar_frame(points, yaw_offset_deg=90.0):
    """Inverse of dock_detector_node's lidar_yaw_offset_deg correction --
    so points authored in the boat-relative frame land back there after the
    node applies its own (default) correction."""
    theta = np.radians(yaw_offset_deg)
    c, s = np.cos(theta), np.sin(theta)
    return [(c * x + s * y, -s * x + c * y) for x, y in points]


def make_pointcloud2(points, frame_id="lidar", stamp=None):
    msg = PointCloud2()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = b"".join(struct.pack("fff", x, y, 0.0) for x, y in points)
    msg.is_dense = True
    return msg


def build_scene_cloud(distance, angle_deg, occupied_berth=None, spacing=0.08,
                       frame_id="lidar", stamp=None, yaw_offset_deg=90.0):
    """occupied_berth: "A", "B", or None (both free -- control case).
    Returns a PointCloud2 ready to publish on /obstacles/lidar."""
    world_pts = dock_wall_points(spacing)
    if occupied_berth is not None:
        world_pts = world_pts + decoy_hull_points(occupied_berth, spacing)
    boat_x, boat_y, boat_heading = vantage_pose(distance, angle_deg)
    boat_pts = to_boat_frame(world_pts, boat_x, boat_y, boat_heading)
    raw_pts = to_raw_lidar_frame(boat_pts, yaw_offset_deg)
    return make_pointcloud2(raw_pts, frame_id=frame_id, stamp=stamp)
