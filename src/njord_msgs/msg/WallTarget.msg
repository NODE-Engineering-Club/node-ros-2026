# A straight pier/berth wall segment detected in the LiDAR scan (Task 3.2
# "parallel docking"), boat-relative — same reasoning as DockTarget.msg:
# this is a short-range final-approach maneuver where LiDAR-relative
# geometry is what actually matters, not GPS/TF-lifted geometry.
#
# Produced by perception/wall_detector_node: DBSCAN separates the raw scan
# into candidate objects, RANSAC extracts straight wall segments from each
# (the same line-extraction step dock_detector_node uses for its U-shape's
# back wall), and segments matching the expected berth wall length are
# published as candidates. There is no "pier"/"wall" class in the YOLO
# segmentation model (vision/node.py's class_id 0-5 are buoy/cardinal
# marks only), so unlike a first guess might assume, this is LIDAR-only —
# same as Task 3.1's own dock_detector_node, which is also LIDAR-only
# despite the vision pipeline existing elsewhere in the stack.

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
float32 confidence                    # 0..1, how well the segment matched
                                       # the expected berth wall length
bool occupied                         # true if points near the wall,
                                       # inside the standoff band, aren't
                                       # explained by the wall segment
                                       # itself (likely something already
                                       # moored there)
