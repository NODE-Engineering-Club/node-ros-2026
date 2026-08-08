# All U-shaped berth candidates recognized this scan (both free and
# occupied), boat-relative (base_link frame). Empty array = no berth
# recognized this scan. Produced by perception/dock_detector_node
# alongside the backward-compatible singular DockTarget on
# /perception/dock_target (highest-confidence FREE berth, if any).

std_msgs/Header header                # frame_id = "base_link"
DockTarget[] targets                  # detected == true for every entry
