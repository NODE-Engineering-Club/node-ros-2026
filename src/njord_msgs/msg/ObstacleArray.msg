# A time-matched snapshot of every obstacle plus the boat's own pose, all
# expressed in / anchored to the global GPS frame. This is the single message
# the decision / path-planning node consumes.

std_msgs/Header header                  # stamp of the fused snapshot; frame_id = "map"

geographic_msgs/GeoPoint boat_position  # boat ABSOLUTE position (WGS84 lat/lon/alt)
float64 boat_heading                    # boat yaw in the map frame (rad, ENU); 0 if unknown

Obstacle[] obstacles
