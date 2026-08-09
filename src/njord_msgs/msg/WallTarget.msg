# A straight pier/berth wall segment detected in the LiDAR scan (Task 3.2
# "parallel docking"), boat-relative — same reasoning as DockTarget.msg:
# this is a short-range final-approach maneuver where LiDAR-relative
# geometry is what actually matters, not GPS/TF-lifted geometry.
#
# Produced by perception/wall_detector_node: DBSCAN separates the raw scan
# into candidate objects, RANSAC extracts straight wall segments from each,
# and the segments are matched against a U template (a ~4m back wall
# flanked by two ~2m perpendicular arms, opening toward the boat) — the
# same U-matching algorithm dock_detector_node uses, reparametrized for
# Task 3.2's berth proportions. The fields below describe the matched back
# wall specifically (NOT the U's opening/arm-tip geometry) — Task 3.2
# needs the boat to end up alongside the back wall itself, unlike Task 3.1
# which enters through the opening toward it. There is no "pier"/"wall"
# class in the YOLO segmentation model (vision/node.py's class_id 0-5 are
# buoy/cardinal marks only), so unlike a first guess might assume, this is
# LIDAR-only — same as Task 3.1's own dock_detector_node, which is also
# LIDAR-only despite the vision pipeline existing elsewhere in the stack.

std_msgs/Header header                # frame_id = "base_link"
bool detected                         # true if a wall segment was recognized

geometry_msgs/Point aim_point         # target point for the approach
                                       # controller: the wall segment's
                                       # midpoint (centers the boat in the
                                       # berth along the wall), offset away
                                       # from the wall by
                                       # wall_detector_node's configured
                                       # standoff_distance_m so the boat
                                       # aims short of actually touching
                                       # the wall. base_link frame:
                                       # x = forward, y = left.
float32 heading                       # rad, wall's line direction,
                                       # canonicalized to the branch within
                                       # +/-90 deg of the boat's current
                                       # +x — this is the heading
                                       # correction needed to lie parallel
                                       # to the wall, same convention as
                                       # DockTarget.heading (0 = boat's
                                       # current heading already matches)
float32 length                        # m, wall segment length
float32 confidence                    # 0..1, how well the 3-segment U match
                                       # fit (parallelism, perpendicularity,
                                       # opening width, corner alignment,
                                       # RANSAC inlier ratio)
bool occupied                         # true if points near the wall,
                                       # inside the standoff band, aren't
                                       # explained by the wall segment
                                       # itself (likely something already
                                       # moored there)
