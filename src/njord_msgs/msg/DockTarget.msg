# U-shaped docking berth detected in the LiDAR scan (Task 3.1 "normal
# docking"), boat-relative — no GPS/TF lifting, since docking is a
# short-range final-approach maneuver where LiDAR-relative geometry is what
# actually matters.
#
# Produced by perception/dock_detector_node: DBSCAN separates the raw scan
# into candidate objects, RANSAC extracts straight wall segments from each,
# and the segments are matched against a U template (two parallel arms +
# a perpendicular back wall, opening toward the boat).

std_msgs/Header header               # frame_id = "base_link"
bool detected                        # true if a U-shape was recognized this frame

geometry_msgs/Point opening_center   # midpoint of the U's opening, base_link frame
float32 heading                      # rad, inward-facing normal of the back wall
                                      # (boat's target approach heading), 0 = boat +x
float32 width                        # m, spacing between the two arm tips (inner)
float32 depth                        # m, arm length (berth depth)
float32 confidence                   # 0..1, how well the cluster matched the U template
bool occupied                        # true if points inside the berth interior aren't
                                      # explained by the 3 matched walls (likely obstructed)
