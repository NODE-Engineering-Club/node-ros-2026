# Health of the Cerulean Omniscan 3D link and of the data it is producing.
# Published at 1 Hz by omniscan_bridge.
std_msgs/Header header

bool    connected                # socket open and packets arriving within timeout
float32 actual_ping_rate_hz      # measured from END_PING_INFO / point set arrivals
float32 commanded_ping_rate_hz   # what we last asked for; divergence is a symptom
uint32  points_per_ping          # from the most recent OS3D_POINT_SET
float32 speed_of_sound           # m/s, as reported by the sonar (not assumed)
float32 range_setting_m
uint8   gain_setting
float32 packet_loss_ratio        # 0..1, estimated from ping_number gaps
int64   clock_offset_ms          # sonar utc_msec minus Jetson clock. Drift here
                                 # silently ruins the whole dataset — see docs.
float32 pitch_deg                # sonar-internal attitude (ATTITUDE_REPORT)
float32 roll_deg
uint64  packets_parsed
uint64  checksum_errors
uint64  bytes_discarded          # resync losses; non-zero means a framing problem
