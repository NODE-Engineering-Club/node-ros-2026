"""Synthetic pier-wall scene generation for exercising wall_detector_node
without Gazebo or a real berth wall.

Same approach as dock_scene_publisher.py (see that module's docstring for
the general reasoning: this tests the clustering/RANSAC/canonicalization/
occupancy ALGORITHM against known-correct points, not sim/hardware
fidelity like occlusion or sensor noise — there is no Gazebo world for
Task 3.2's berth yet to cross-check against, unlike Task 3.1's
dockingWorldOccupied.sdf).

The wall is authored as a single straight segment in a local/world frame,
independent of the boat's vantage pose, then rotated into that vantage's
boat-relative frame and finally into the raw-lidar frame the node expects
on /obstacles/lidar — reusing dock_scene_publisher's generic (not
dock-specific) frame-transform helpers rather than re-deriving them.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from dock_scene_publisher import (  # noqa: E402
    _line,
    make_pointcloud2,
    to_boat_frame,
    to_raw_lidar_frame,
)

WALL_X = 6.0
WALL_LENGTH = 4.0
WALL_Y_MIN = -WALL_LENGTH / 2.0
WALL_Y_MAX = WALL_LENGTH / 2.0
WALL_MIDPOINT = (WALL_X, 0.0)

DECOY_LENGTH = 1.0
DECOY_BEAM = 0.4


def wall_points(spacing=0.08):
    """Points along the single straight berth wall."""
    return _line((WALL_X, WALL_Y_MIN), (WALL_X, WALL_Y_MAX), spacing)


def decoy_hull_points(standoff, lateral_offset=0.0, spacing=0.08):
    """Rectangular outline approximating a boat already moored in the
    standoff band between the wall and the approach line, centered on the
    given lateral offset along the wall."""
    cx = WALL_X - standoff / 2.0
    cy = lateral_offset
    hl, hb = DECOY_BEAM / 2.0, DECOY_LENGTH / 2.0
    corners = [
        (cx - hl, cy - hb), (cx + hl, cy - hb),
        (cx + hl, cy + hb), (cx - hl, cy + hb),
    ]
    pts = []
    for i in range(4):
        pts += _line(corners[i], corners[(i + 1) % 4], spacing)
    return pts


def vantage_pose(distance, angle_deg, lateral_offset=0.0):
    """Boat pose (x, y, heading_rad) `distance` m from the point on the
    wall at `lateral_offset` from its midpoint, at bearing `angle_deg` off
    dead-ahead, heading pointed at that point -- same convention as
    dock_scene_publisher's vantage_pose."""
    angle_rad = np.radians(angle_deg)
    target = np.array([WALL_X, lateral_offset])
    boat = target + distance * np.array([-np.cos(angle_rad), np.sin(angle_rad)])
    heading = float(np.arctan2(target[1] - boat[1], target[0] - boat[0]))
    return float(boat[0]), float(boat[1]), heading


def expected_aim_point(distance, angle_deg, lateral_offset, standoff):
    """Independently-computed expected WallTarget.aim_point (boat frame).

    aim_point targets the wall's CENTER (WALL_MIDPOINT), not wherever the
    boat currently sits laterally -- that's the whole point of centering
    the boat in the berth (see WallTarget.msg). lateral_offset only moves
    the boat's vantage point, not the target.
    """
    bx, by, bh = vantage_pose(distance, angle_deg, lateral_offset)
    aim_world = (WALL_X - standoff, 0.0)
    return to_boat_frame([aim_world], bx, by, bh)[0]


def expected_heading(distance, angle_deg, lateral_offset):
    """Independently-computed expected WallTarget.heading: the wall's
    direction (world +y, i.e. vertical) expressed in the boat's frame and
    folded to the branch within +/-90 deg of the boat's own +x."""
    _, _, bh = vantage_pose(distance, angle_deg, lateral_offset)

    # World direction vector (0, 1) rotated by -bh into the boat frame.
    dir_x = np.sin(bh)
    dir_y = np.cos(bh)
    angle = float(np.arctan2(dir_y, dir_x))

    folded = np.arctan2(np.sin(angle), np.cos(angle))
    if folded > np.pi / 2.0:
        folded -= np.pi
    elif folded < -np.pi / 2.0:
        folded += np.pi
    return float(folded)


def build_scene_cloud(distance, angle_deg, lateral_offset=0.0, occupied=False,
                       standoff=0.5, spacing=0.08, frame_id="lidar", stamp=None,
                       yaw_offset_deg=90.0):
    """Returns a PointCloud2 ready to publish on /obstacles/lidar."""
    world_pts = wall_points(spacing)
    if occupied:
        world_pts = world_pts + decoy_hull_points(standoff, lateral_offset, spacing)
    boat_x, boat_y, boat_heading = vantage_pose(distance, angle_deg, lateral_offset)
    boat_pts = to_boat_frame(world_pts, boat_x, boat_y, boat_heading)
    raw_pts = to_raw_lidar_frame(boat_pts, yaw_offset_deg)
    return make_pointcloud2(raw_pts, frame_id=frame_id, stamp=stamp)
