# Heading is the dominant error source in the survey: 1 deg ~= 90 cm at 50 m.
# It gets its own message so its provenance travels with its value.
std_msgs/Header header

float32 heading_deg
string  source               # "gnss_compass" | "magnetometer" | "cog" | "none"
bool    valid
float32 accuracy_deg         # reported by the source where available, else NaN
float32 cog_deg
float32 sog_ms
float32 divergence_deg       # heading - COG, wrapped to [-180, 180]
bool    divergence_meaningful # false below a speed threshold, where COG is noise
