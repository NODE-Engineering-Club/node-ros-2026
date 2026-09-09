# State of mission_recorder.
std_msgs/Header header

uint8 IDLE=0
uint8 RECORDING=1
uint8 STOPPING=2
uint8 ERROR=3

uint8   state
string  mission_name          # empty when IDLE
string  mission_dir           # absolute path on the Jetson
float64 elapsed_s
uint64  bytes_written
uint64  disk_free_bytes
float64 estimated_remaining_s # recording time left at the current write rate
string  error_message         # populated when state == ERROR
