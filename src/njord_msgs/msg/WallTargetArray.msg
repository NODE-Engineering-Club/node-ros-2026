# All wall-segment candidates recognized this scan (both free and
# occupied), boat-relative (base_link frame). Empty array = no wall
# recognized this scan. Produced by perception/wall_detector_node
# alongside the singular WallTarget on /perception/wall_target
# (highest-confidence FREE wall, if any) — same pattern as
# DockTarget/DockTargetArray.

std_msgs/Header header                # frame_id = "base_link"
WallTarget[] targets                  # detected == true for every entry
