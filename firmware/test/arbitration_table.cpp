// Dump the firmware's full mode/arming decision table, one row per line.
//
// This binary asserts nothing on its own. It exists so that
// src/asket_common/test/test_firmware_arbitration_matches.py can compare the
// *real* sketch, compiled, against the Python mirror in
// asket_common.mode_arbitration -- every combination, cell for cell.
//
// Why drive the whole sketch rather than lift the function out of it:
// update_state() is not pure. It reads sbus_channels[], sbus_failsafe_flag,
// serial_wants_auto, estop_latched and the heartbeat clock, and it writes its
// answer into current_mode/armed via apply_transition(). Extracting a pure
// arbitrate_mode() would be a firmware edit, and firmware edits belong on the
// bench with the boat out of the water. Compiling the sketch against the stub
// and setting its globals directly tests the code that actually ships.
//
//   make -C firmware/test table
//
// Output columns, space separated:
//   ch8 ch7 wantauto link failsafe latch_in  mode armed latch_out
//
// mode is the firmware's OperationMode enum: 1 ESTOP, 2 MANUAL, 3 AUTONOMOUS.

#include "ArduinoStub.h"
unsigned long g_millis = 0;
bool g_auto_ms = true;
std::vector<int> g_pin_state(32, 0);
SerialStub Serial;
SerialStub Serial2;
TinyUSBDeviceStub TinyUSBDevice;
#define Servo_h
#include "sketch.inc"

// A fixed clock. update_state() consults millis() only through
// serial_link_live(), so freezing it makes "heartbeat fresh" an input we set
// rather than a race we hope for.
static const unsigned long NOW = 100000;

int main() {
  g_auto_ms = false;
  g_millis = NOW;

  // Boundaries as well as zone interiors: an off-by-one in a threshold is
  // exactly the kind of drift this table is here to catch.
  const uint16_t ch8_values[] = {172, 699, 700, 991, 1399, 1400, 1811};
  const uint16_t ch7_values[] = {172, 1000, 1001, 1811};

  for (unsigned a = 0; a < sizeof(ch8_values) / sizeof(*ch8_values); a++) {
    for (unsigned b = 0; b < sizeof(ch7_values) / sizeof(*ch7_values); b++) {
      for (int wants = 0; wants < 2; wants++) {
        for (int link = 0; link < 2; link++) {
          for (int fs = 0; fs < 2; fs++) {
            for (int latch = 0; latch < 2; latch++) {
              // Start from a known state. ESTOP/disarmed is the firmware's own
              // power-on state, and it means that when the decision *is*
              // ESTOP/disarmed no transition fires and the values already read
              // correctly -- so this works whether or not update_state()
              // decides anything changed.
              current_mode = MODE_ESTOP;
              armed = false;
              relay_on = false;

              sbus_channels[CH_MODE] = ch8_values[a];
              sbus_channels[CH_ARM] = ch7_values[b];
              serial_wants_auto = (wants == 1);
              sbus_failsafe_flag = (fs == 1);
              estop_latched = (latch == 1);
              last_heartbeat_ms = link ? NOW : NOW - (HEARTBEAT_TIMEOUT_MS + 1);

              update_state();

              printf("%u %u %d %d %d %d %d %d %d\n",
                     (unsigned)ch8_values[a], (unsigned)ch7_values[b],
                     wants, link, fs, latch,
                     (int)current_mode, armed ? 1 : 0, estop_latched ? 1 : 0);

              Serial.out.clear();   // transitions log; we only want the table
            }
          }
        }
      }
    }
  }
  return 0;
}
