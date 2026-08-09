"""Synthetic U-shaped berth scene generation for exercising
wall_detector_node without Gazebo or a real berth.

Same approach as dock_scene_publisher.py (see that module's docstring for
the general reasoning: this tests the clustering/RANSAC/U-matching/
canonicalization/occupancy ALGORITHM against known-correct points, not
sim/hardware fidelity like occlusion or sensor noise — there is no Gazebo
world for Task 3.2's berth yet to cross-check against, unlike Task 3.1's
dockingWorldOccupied.sdf).

The berth is a U: a ~4m back wall (the wall the boat ends up lying
against) flanked by two ~2m perpendicular arms, opening ~4m wide toward
the boat — same overall shape as dock_scene_publisher's dock, just
different proportions (Task 3.1's is a ~2m opening with ~2m arms). The
geometry is authored in a local/world frame, then rotated into the boat's
vantage-relative frame and finally into the raw-lidar frame the node
expects on /obstacles/lidar — reusing dock_scene_publisher's generic (not
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

ARM_LENGTH = 2.0
HALF_WIDTH = 2.0  # berth_width_m / 2

BACK_WALL_X = 6.0
OPENING_X = BACK_WALL_X - ARM_LENGTH

ARM_Y = {"top": HALF_WIDTH, "bottom": -HALF_WIDTH}

DECOY_LENGTH = 1.0
DECOY_BEAM = 0.4


def berth_points(spacing=0.08):
    """Points along the back wall and both perpendicular arms."""
    pts = []
    pts += _line((BACK_WALL_X, -HALF_WIDTH), (BACK_WALL_X, HALF_WIDTH), spacing)
    pts += _line((OPENING_X, ARM_Y["top"]), (BACK_WALL_X, ARM_Y["top"]), spacing)
    pts += _line((OPENING_X, ARM_Y["bottom"]), (BACK_WALL_X, ARM_Y["bottom"]), spacing)
    return pts


def decoy_hull_points(standoff, lateral_offset=0.0, spacing=0.08):
    """Rectangular outline approximating a boat already moored in the
    standoff band between the back wall and the approach line, centered on
    the given lateral offset along the wall."""
    cx = BACK_WALL_X - standoff / 2.0
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
    back wall at `lateral_offset` from its midpoint, at bearing `angle_deg`
    off dead-ahead, heading pointed at that point -- same convention as
    dock_scene_publisher's vantage_pose."""
    angle_rad = np.radians(angle_deg)
    target = np.array([BACK_WALL_X, lateral_offset])
    boat = target + distance * np.array([-np.cos(angle_rad), np.sin(angle_rad)])
    heading = float(np.arctan2(target[1] - boat[1], target[0] - boat[0]))
    return float(boat[0]), float(boat[1]), heading


def expected_aim_point(distance, angle_deg, lateral_offset, standoff):
    """Independently-computed expected WallTarget.aim_point (boat frame).

    aim_point targets the back wall's CENTER, not wherever the boat
    currently sits laterally -- that's the whole point of centering the
    boat in the berth (see WallTarget.msg). lateral_offset only moves the
    boat's vantage point, not the target.
    """
    bx, by, bh = vantage_pose(distance, angle_deg, lateral_offset)
    aim_world = (BACK_WALL_X - standoff, 0.0)
    return to_boat_frame([aim_world], bx, by, bh)[0]


def expected_heading(distance, angle_deg, lateral_offset):
    """Independently-computed expected WallTarget.heading: the back wall's
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
    world_pts = berth_points(spacing)
    if occupied:
        world_pts = world_pts + decoy_hull_points(standoff, lateral_offset, spacing)
    boat_x, boat_y, boat_heading = vantage_pose(distance, angle_deg, lateral_offset)
    boat_pts = to_boat_frame(world_pts, boat_x, boat_y, boat_heading)
    raw_pts = to_raw_lidar_frame(boat_pts, yaw_offset_deg)
    return make_pointcloud2(raw_pts, frame_id=frame_id, stamp=stamp)


def build_bare_wall_cloud(distance, angle_deg, spacing=0.08, frame_id="lidar",
                           stamp=None, yaw_offset_deg=90.0):
    """A lone ~4m wall with NO arms -- negative-control scene for
    confirming the U-matcher correctly rejects a wall that isn't actually
    part of a U-shaped berth (unlike the old bare-length-filter approach,
    which would have accepted this)."""
    world_pts = _line((BACK_WALL_X, -HALF_WIDTH), (BACK_WALL_X, HALF_WIDTH), spacing)
    boat_x, boat_y, boat_heading = vantage_pose(distance, angle_deg, 0.0)
    boat_pts = to_boat_frame(world_pts, boat_x, boat_y, boat_heading)
    raw_pts = to_raw_lidar_frame(boat_pts, yaw_offset_deg)
    return make_pointcloud2(raw_pts, frame_id=frame_id, stamp=stamp)
