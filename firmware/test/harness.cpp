#include "ArduinoStub.h"
unsigned long g_millis = 0;
bool g_auto_ms = true;
std::vector<int> g_pin_state(32, 0);
SerialStub Serial;
SerialStub Serial2;
TinyUSBDeviceStub TinyUSBDevice;
#define Servo_h
#include "sketch.inc"

// ---- test helpers -------------------------------------------------------
static void tick(unsigned long ms) { for (unsigned long i = 0; i < ms; i++) { g_millis++; loop(); } }
// Clear first, then settle: a phase shorter than the 250 ms status throttle
// would otherwise assert against the PREVIOUS phase's line.
static std::string lastStateLine() {
  std::string best;
  size_t p = 0;
  while ((p = Serial.out.find("STATE ", p)) != std::string::npos) {
    size_t e = Serial.out.find('\n', p);
    best = Serial.out.substr(p, e - p);
    p = e == std::string::npos ? Serial.out.size() : e;
  }
  return best;
}
static void feedSbus(uint16_t ch7, uint16_t ch8, uint16_t thr = 991, uint16_t yaw = 991) {
  // Build a 25-byte SBUS frame with all 16 channels, then push it at Serial2.
  uint16_t ch[16]; for (int i = 0; i < 16; i++) ch[i] = 991;
  ch[CH_THROTTLE] = thr; ch[CH_YAW] = yaw; ch[CH_ARM] = ch7; ch[CH_MODE] = ch8;
  uint8_t f[25] = {0}; f[0] = 0x0F;
  int bit = 0;
  for (int c = 0; c < 16; c++)
    for (int b = 0; b < 11; b++, bit++)
      if (ch[c] & (1 << b)) f[1 + bit / 8] |= (1 << (bit % 8));
  f[23] = 0x00; f[24] = 0x04;   // FrSky-style end byte: v3 would reject this
  for (int i = 0; i < 25; i++) Serial2.in.push_back((char)f[i]);
}

static int failures = 0;
static void check(bool ok, const char* what) {
  printf("%s  %s\n", ok ? "  PASS" : "  FAIL", what);
  if (!ok) failures++;
}
static bool has(const std::string& hay, const char* needle) {
  return hay.find(needle) != std::string::npos;
}

int main() {
  setup();
  g_auto_ms = false;
  printf("[boot banner]\n%s\n", Serial.out.c_str());
  check(has(Serial.out, "[VER] pico-node 4"), "startup announces [VER] pico-node 4");
  check(!has(Serial.out, "BENCH"), "no bench-override banner");

  // --- Ch8 high + Ch7 high: autonomy permitted, armed, but MANUAL by default
  Serial.out.clear();
  for (int i = 0; i < 40; i++) { feedSbus(1811, 1811); tick(10); }
  std::string st = lastStateLine();
  printf("[state] %s\n", st.c_str());
  check(st.rfind("STATE ver=4 ", 0) == 0, "ver= is the first STATE field");
  check(has(st, " mode=2 "), "Ch8 high alone stays MANUAL (autonomy permitted, not granted)");
  check(has(st, " armed=1"), "Ch7 high arms");
  check(has(st, "sbusok="), "SBUS counters present");
  check(!has(st, "sbusok=0 "), "frames with a non-zero end byte are accepted (v3 rejected these)");

  // --- MODE AUTO is honoured in the high zone
  Serial.feed("MODE AUTO\n");
  for (int i = 0; i < 60; i++) { feedSbus(1811, 1811); tick(10); }
  st = lastStateLine();
  check(has(st, " mode=3 "), "MODE AUTO grants autonomy in the high zone");
  check(has(st, " wantauto=1"), "wantauto reported");
  check(has(st, " link=1"), "heartbeat fresh -> link=1");

  // --- Ch8 mid forces MANUAL even though the Jetson still wants auto
  for (int i = 0; i < 60; i++) { feedSbus(1811, 1000); tick(10); }
  st = lastStateLine();
  check(has(st, " mode=2 "), "Ch8 mid forces MANUAL over a standing MODE AUTO");
  check(has(st, " wantauto=1"), "...and the request is still visible, not silently dropped");

  // --- Ch8 low is ESTOP and disarms
  for (int i = 0; i < 60; i++) { feedSbus(1811, 200); tick(10); }
  st = lastStateLine();
  check(has(st, " mode=1 "), "Ch8 low forces ESTOP");
  check(has(st, " armed=0"), "ESTOP disarms");
  check(g_pin_state[ESTOP_RELAY_PIN] == LOW, "ESTOP opens the relay");
  check(g_pin_state[RED_LIGHT_PIN] == HIGH && g_pin_state[GREEN_LIGHT_PIN] == LOW, "ESTOP shows red only");

  // --- the reader never blocks on a partial line
  Serial.out.clear();
  Serial.feed("0.5,-0.5");          // no newline: half a command
  unsigned long t0 = g_millis;
  for (int i = 0; i < 5; i++) { feedSbus(1811, 1811); tick(10); }
  check(g_millis - t0 == 50, "a partial line does not stall the loop");
  check(!has(Serial.out, "Invalid cmd"), "a partial line is held, not rejected");
  Serial.feed("\n");
  tick(5);
  check(!has(Serial.out, "Invalid cmd"), "the completed line parses");

  // --- an over-long line is dropped whole, never split into two commands
  Serial.out.clear();
  Serial.feed(std::string(200, 'x') + "\n");
  tick(5);
  check(has(Serial.out, "line too long"), "an over-long line is reported and dropped");

  // --- light convention
  for (int i = 0; i < 30; i++) { feedSbus(1811, 1000); tick(10); }   // manual, armed
  check(g_pin_state[YELLOW_LIGHT_PIN] == HIGH, "manual armed = yellow");
  Serial.feed("MODE AUTO\n");
  for (int i = 0; i < 30; i++) { feedSbus(1811, 1811); tick(10); }
  check(g_pin_state[GREEN_LIGHT_PIN] == HIGH, "autonomous armed = green");
  for (int i = 0; i < 30; i++) { feedSbus(200, 1811); tick(10); }    // disarm
  check(g_pin_state[RED_LIGHT_PIN] == HIGH, "disarmed = red");

  printf("\n%s (%d failure(s))\n", failures ? "FAILED" : "ALL PASS", failures);
  return failures ? 1 : 0;
}
