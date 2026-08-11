# A single obstacle, geo-referenced to the global GPS frame.
#
# Produced by the fusion node by associating a vision detection (class/colour)
# with a lidar cluster (metric range), then lifting the result into the global
# frame so the position is stable as the boat moves.

uint32 id                          # tracking id (0 = not tracked / frame-local)
string class_id                    # class / colour / type from vision; "unknown" if lidar-only
float32 confidence                 # detection confidence [0..1]; 0 for lidar-only

geographic_msgs/GeoPoint position  # ABSOLUTE position (WGS84 lat/lon/alt)
geometry_msgs/Point position_map   # same point in the map frame (x=East, y=North, metres)

float32 radius                     # rough size estimate (m); 0 if unknown
bool lidar_confirmed               # true if a lidar return supports this obstacle

# Boat-relative kinematics from the fusion node's Kalman tracker.
float32 range_m                    # metric distance boat->obstacle (m); 0 if lidar-unconfirmed
float32 bearing_deg                # bearing from boat forward axis (deg, + = port/left)
float32 speed_mps                  # speed estimate from the tracker (m/s); 0 until tracked
float32 velocity_bearing_deg       # direction of the obstacle's estimated relative-velocity
                                    # vector, same convention as bearing_deg (0=boat forward,
                                    # +=port); 0.0 if speed_mps == 0 (direction undefined). Not
                                    # ego-motion compensated, same caveat as speed_mps.
