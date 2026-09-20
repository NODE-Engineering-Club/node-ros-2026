# Confirmed vessel state as reported by the Pico.
#
# This is the message asket_sim publishes. The real pico_bridge publishes a
# std_msgs/String carrying the firmware's raw "STATE key=value" line, which
# gui_backend parses in core/pico_state.py; both shapes reach the GUI through
# one adapter, configured in gui_backend/config/topics.yaml.
#
# NOTE ON MODE NUMBERING. The constants below count from 0. The *firmware*
# counts from 1 (enum OperationMode { MODE_ESTOP = 1, MODE_MANUAL = 2,
# MODE_AUTONOMOUS = 3 }). The two scales are bridged by name, never by adding
# one, because an off-by-one here turns MANUAL into AUTONOMOUS on screen.
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

# Raw SBUS counts, 172..1811 with 991 at centre. NOT percentages: the firmware
# thresholds (Ch8 < 700 = ESTOP, Ch7 > 1000 = armed) are on this scale, and an
# earlier field named "_raw_pct" held a raw count while the GUI alarmed below
# 25 — so the alarm never fired, including at the bottom of the travel.
uint16  rc_channel7_raw      # arm switch, 2-position
uint16  rc_channel8_raw      # mode switch, 3-position

uint8   firmware_version     # FW_VERSION the Pico announced; 0 = not reported

float32 battery_voltage
float32 battery_current
