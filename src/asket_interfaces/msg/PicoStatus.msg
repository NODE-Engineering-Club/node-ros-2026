# Confirmed vessel state as reported by the Pico.
#
# NOTE: this is the message asket_sim publishes. The REAL pico_bridge message is
# not known here and this repository must not modify pico_bridge — see
# docs/open_questions.md Q7. gui_backend reaches this through an adapter
# configured in gui_backend/config/topics.yaml, so swapping in the real type is
# a config change plus one adapter function.
std_msgs/Header header

uint8 MODE_ESTOP=0
uint8 MODE_MANUAL=1
uint8 MODE_AUTONOMOUS=2

uint8   mode                 # CONFIRMED mode, never a requested one
bool    armed
bool    estop_latched        # soft latch; the hardware killswitch is separate
bool[]  relay_states
uint8[] esc_status           # per-thruster status byte, hull-specific
bool    rc_link_ok
uint8   rc_channel8_raw_pct  # readback of the sovereign kill channel, 0..100
float32 battery_voltage
float32 battery_current
