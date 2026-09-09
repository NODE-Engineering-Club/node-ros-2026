# Shore-link quality as measured by gui_backend.
std_msgs/Header header

string  active_link          # "wifi" | "4g" | "ltem" | "ethernet" | "none"
float32 quality              # 0..1, normalised RSSI / success rate
float32 rtt_ms
float32 rate_bytes_per_s     # currently being pushed to clients
string  profile              # "full" | "reduced" | "minimal"
bool    profile_manual       # true when an operator has overridden auto-select
uint32  connected_clients
