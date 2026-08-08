// ============================================================================
//  Asket EC  -  Pico 2 (RP2350) Low-Level Controller   [REBUILT + FOXGLOVE]
//  Core: Earle Philhower arduino-pico  |  USB stack: Adafruit TinyUSB
//
//  Modes (Ch8, 3-pos):  ESTOP / "MANUAL or AUTONOMY-PERMITTED"
//  Arm   (Ch7, 2-pos):  high = armed
//
//  MODE AUTHORITY (final spec):
//    Ch8 LOW    -> ESTOP            (hardware priority, nothing overrides)
//    Ch8 MID    -> MANUAL forced    (RC drives; Foxglove cannot go auto here)
//    Ch8 HIGH   -> autonomy PERMITTED:
//                    default = MANUAL, until Foxglove/Pi sends "MODE AUTO".
//                    Foxglove/Pi sends "MODE MANUAL" to come back to manual.
//                    No stick-override: moving sticks does NOT change mode.
//    Serial heartbeat lost while AUTONOMOUS -> hold neutral, relay stays ON,
//                    wait for Pi to return (RC can pull Ch8->MID to take over).
//
//  Serial protocol (Pi -> Pico), matches pico_bridge.py / the team doc exactly:
//    "<L> <R>" or "L,R"  bare values = motor command (microseconds or -1..1 norm)
//    "MODE AUTO"    request autonomous (only effective when Ch8 HIGH)
//    "MODE MANUAL"  request manual (within Ch8 HIGH zone)
//    "PING"         heartbeat (Pi should send ~every 100 ms)
//
//  Non-blocking throughout: no delay() anywhere, millis() only.
//
//  FIX (this version): parse_sbus_frame() used to reject any frame whose
//  footer byte (byte 24) wasn't exactly 0x00. That footer carries CH17/CH18
//  + frame-lost/failsafe flag bits, not a fixed "all good" marker -- most
//  receivers set one of those bits at least some of the time, which meant
//  every frame was silently dropped and sbus_channels[] never moved even
//  with a perfectly healthy SBUS link. Fixed to only require the sync byte,
//  and to read the flags byte instead of using it as a reject condition.
//  Also added SBUS health counters to the STATE line so link health is
//  visible on the serial monitor without recompiling.
// ============================================================================

#include <Servo.h>
#include <Adafruit_TinyUSB.h>

// ---------------------------------------------------------------------------
//  PIN MAP  (unchanged from your board)
// ---------------------------------------------------------------------------
#define THRUSTER_LEFT_PIN   15
#define THRUSTER_RIGHT_PIN  16

#define RED_LIGHT_PIN       12
#define GREEN_LIGHT_PIN     10
#define YELLOW_LIGHT_PIN    11
#define BUZZER_PIN          13

#define ESTOP_RELAY_PIN     21   // OUT: HIGH = ESC power on, LOW = ESC power cut
#define ESTOP_READ_PIN      20   // IN : HIGH = rail present, LOW = no power (divider)

// ---------------------------------------------------------------------------
//  PWM (ESC) - bidirectional
// ---------------------------------------------------------------------------
const int PWM_MIN_US     = 1000;  // full reverse
const int PWM_NEUTRAL_US = 1500;  // stop
const int PWM_MAX_US     = 2000;  // full forward

// ---------------------------------------------------------------------------
//  SBUS CALIBRATION  (tune here)
//  Stick endpoints ~172 / ~991 (center) / ~1811
// ---------------------------------------------------------------------------
const uint16_t SBUS_MIN      = 172;
const uint16_t SBUS_MID      = 991;
const uint16_t SBUS_MAX      = 1811;
const uint16_t SBUS_DEADBAND = 30;   // counts around center treated as 0

// ---------------------------------------------------------------------------
//  MIXING TUNABLES
// ---------------------------------------------------------------------------
const float YAW_GAIN   = 0.7f;   // how aggressive turning is (0..1+)
const int   YAW_INVERT = +1;     // flip to -1 if it steers the wrong way
const int   THR_INVERT = +1;     // flip to -1 if forward/reverse are swapped

const float MANUAL_AUTHORITY = 1.0f;  // 1.0 = full 1000-2000; 0.8 = 1100-1900 (old code)
#define     PIVOT_AT_MAX_YAW   1       // 1 = spin in place when yaw is hard-over
const float PIVOT_THRESHOLD = 0.9f;    // fraction of yaw travel that triggers pivot

// ---------------------------------------------------------------------------
//  CHANNEL ASSIGNMENT  (0-indexed: Ch3 -> index 2, etc.)
// ---------------------------------------------------------------------------
const int CH_THROTTLE = 2;   // Ch3
const int CH_YAW      = 3;   // Ch4
const int CH_ARM      = 6;   // Ch7
const int CH_MODE     = 7;   // Ch8

const int ARM_THRESHOLD  = 1000;  // Ch7 above this = armed
const int MODE_LOW_MAX   = 700;   // Ch8 < this = ESTOP
const int MODE_MID_MAX   = 1400;  // Ch8 < this = MANUAL forced, else AUTONOMY PERMITTED

// ---------------------------------------------------------------------------
//  ESC ARMING / RELAY
// ---------------------------------------------------------------------------
const unsigned long ESC_ARM_DELAY_MS = 2000;  // hold neutral after relay closes

// ---------------------------------------------------------------------------
//  E-STOP FEEDBACK (GPIO20 voltage divider) - disabled for bench
// ---------------------------------------------------------------------------
#define ESTOP_FEEDBACK_ENABLED 0
const unsigned long ESTOP_GRACE_MS    = 500;
const unsigned long ESTOP_DEBOUNCE_MS = 100;

// ---------------------------------------------------------------------------
//  BENCH-ONLY OVERRIDE
//  Set to 1 to force mode=AUTONOMY-PERMITTED + armed with NO RC receiver at
//  all, so you can drive purely from a serial terminal. This REMOVES the
//  hardware E-stop's authority -- only ever use it on the bench with props
//  out of the water and the boat secured. Leave at 0 for real operation.
// ---------------------------------------------------------------------------
#define BENCH_NO_RC_OVERRIDE 0

// ---------------------------------------------------------------------------
//  SBUS FRAME (manual parse, hardware inversion)
// ---------------------------------------------------------------------------
const int  NUM_CHANNELS    = 16;
const int  SBUS_FRAME_SIZE = 25;
const byte SBUS_SYNC_BYTE   = 0x0F;

byte     sbus_frame[SBUS_FRAME_SIZE] = {0};
uint16_t sbus_channels[NUM_CHANNELS] = {0};
bool     sbus_frame_ready = false;
int      sbus_frame_index = 0;
unsigned long last_sbus_read_ms   = 0;
const unsigned long SBUS_FAILSAFE_TIMEOUT_MS = 500;

// SBUS health counters, surfaced in the STATE line (see loop()).
unsigned long sbus_frames_ok  = 0;   // sync byte matched, channels decoded
unsigned long sbus_frames_bad = 0;   // sync byte mismatch (lost byte alignment)
bool          sbus_failsafe_flag = false;  // receiver-reported failsafe (footer bit 3)
bool          sbus_frame_lost_flag = false; // receiver-reported frame lost (footer bit 2)

// ---------------------------------------------------------------------------
//  AUTONOMOUS SERIAL FAILSAFE + HEARTBEAT
// ---------------------------------------------------------------------------
unsigned long last_serial_command_ms = 0;          // any motor command
const unsigned long SERIAL_FAILSAFE_TIMEOUT_MS = 500;

unsigned long last_heartbeat_ms = 0;               // PING or any valid serial line
const unsigned long HEARTBEAT_TIMEOUT_MS = 600;    // auto considered "live" within this

// ---------------------------------------------------------------------------
//  FOXGLOVE/Pi-REQUESTED SUB-MODE (only consulted when Ch8 HIGH)
//  Default MANUAL: Ch8 HIGH alone stays manual until the Pi asks for AUTO.
// ---------------------------------------------------------------------------
bool serial_wants_auto = false;     // set by "MODE AUTO" / cleared by "MODE MANUAL"

// ---------------------------------------------------------------------------
//  MODES + STATE
// ---------------------------------------------------------------------------
enum OperationMode { MODE_ESTOP = 1, MODE_MANUAL = 2, MODE_AUTONOMOUS = 3 };

OperationMode current_mode = MODE_ESTOP;
bool armed = false;

bool          relay_on      = false;
unsigned long relay_on_ms   = 0;

bool          estop_latched = false;
bool          power_low_pending = false;
unsigned long power_low_since_ms = 0;

Servo thruster_left;
Servo thruster_right;

// ---------------------------------------------------------------------------
//  NON-BLOCKING BEEPER  (finite pulse train)
// ---------------------------------------------------------------------------
struct Beeper {
  bool          active   = false;
  int           remaining = 0;
  unsigned long onMs     = 0;
  unsigned long offMs    = 0;
  bool          isOn     = false;
  unsigned long phaseStart = 0;
} beeper;

void startBeeps(int count, unsigned long onMs, unsigned long offMs) {
  if (count <= 0) { beeper.active = false; digitalWrite(BUZZER_PIN, LOW); return; }
  beeper.active     = true;
  beeper.remaining  = count;
  beeper.onMs       = onMs;
  beeper.offMs      = offMs;
  beeper.isOn       = true;
  beeper.phaseStart = millis();
  digitalWrite(BUZZER_PIN, HIGH);
}

void updateBeeper() {
  if (!beeper.active) return;
  unsigned long now = millis();
  if (beeper.isOn) {
    if (now - beeper.phaseStart >= beeper.onMs) {
      digitalWrite(BUZZER_PIN, LOW);
      beeper.isOn = false;
      beeper.phaseStart = now;
      beeper.remaining--;
      if (beeper.remaining <= 0) beeper.active = false;
    }
  } else {
    if (now - beeper.phaseStart >= beeper.offMs) {
      digitalWrite(BUZZER_PIN, HIGH);
      beeper.isOn = true;
      beeper.phaseStart = now;
    }
  }
}

// ---------------------------------------------------------------------------
//  PROTOTYPES
// ---------------------------------------------------------------------------
void  init_gpio();
void  startup_light_sequence();
void  set_light(OperationMode mode, bool live);
void  handle_sbus();
void  parse_sbus_frame();
void  update_state();
void  apply_transition(OperationMode new_mode, bool want_armed);
void  set_relay(bool on);
void  update_motors();
void  handle_serial_input();
bool  process_motor_command(const String &cmd, int &left, int &right);
void  check_power_feedback();
void  trigger_estop(const char *reason);
float sbus_to_norm(uint16_t v);
void  write_thrusters(int left, int right);
void  set_both(int pulse);
bool  serial_link_live();

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  unsigned long t0 = millis();
  while (!TinyUSBDevice.mounted() && (millis() - t0 < 3000)) { /* spin */ }

  init_gpio();

  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.setInvertRX(true);
  Serial2.begin(100000, SERIAL_8E2);

  thruster_left.attach(THRUSTER_LEFT_PIN,  PWM_MIN_US, PWM_MAX_US);
  thruster_right.attach(THRUSTER_RIGHT_PIN, PWM_MIN_US, PWM_MAX_US);
  set_both(PWM_NEUTRAL_US);

  last_serial_command_ms = millis();
  last_sbus_read_ms      = millis();
  last_heartbeat_ms      = millis();

  startup_light_sequence();
  set_light(MODE_ESTOP, false);

  Serial.println(F("=== Asket EC Pico Controller (rebuilt + foxglove) ==="));
  Serial.println(F("Ch8 LOW=ESTOP  MID=MANUAL  HIGH=autonomy-permitted"));
  Serial.println(F("Serial: 'L R' or 'L,R' motors | 'MODE AUTO' | 'MODE MANUAL' | 'PING'"));
  Serial.print  (F("E-STOP feedback: "));
  Serial.println(ESTOP_FEEDBACK_ENABLED ? F("ENABLED") : F("DISABLED (bench)"));
#if BENCH_NO_RC_OVERRIDE
  Serial.println(F("*** BENCH_NO_RC_OVERRIDE=1: RC gate bypassed, terminal-only control. ***"));
  Serial.println(F("*** DO NOT use this build on the water. ***"));
#endif
  Serial.println(F("===================================================="));
}

// ============================================================================
//  MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  handle_sbus();
  handle_serial_input();      // always parsed, so MODE/PING work in any zone
  update_state();
  check_power_feedback();

  updateBeeper();
  update_motors();

  // --- Failsafes ---
  if (armed) {
    if (current_mode != MODE_AUTONOMOUS &&
        (now - last_sbus_read_ms > SBUS_FAILSAFE_TIMEOUT_MS)) {
      trigger_estop("SBUS timeout");
    }
    // Serial/motor lost in autonomous: hold neutral (don't kill power)
    if (current_mode == MODE_AUTONOMOUS &&
        (now - last_serial_command_ms > SERIAL_FAILSAFE_TIMEOUT_MS)) {
      set_both(PWM_NEUTRAL_US);
    }
  }

  // --- Periodic status (parsable for Foxglove) ---
  static unsigned long last_status = 0;
  if (now - last_status > 250) {
    // Machine-readable line. Fields are key=value, space separated.
    Serial.print(F("STATE mode="));   Serial.print((int)current_mode);
    Serial.print(F(" armed="));       Serial.print(armed ? 1 : 0);
    Serial.print(F(" relay="));       Serial.print(relay_on ? 1 : 0);
    Serial.print(F(" wantauto="));    Serial.print(serial_wants_auto ? 1 : 0);
    Serial.print(F(" link="));        Serial.print(serial_link_live() ? 1 : 0);
    Serial.print(F(" estoplatch="));  Serial.print(estop_latched ? 1 : 0);
    Serial.print(F(" thr="));         Serial.print(sbus_channels[CH_THROTTLE]);
    Serial.print(F(" yaw="));         Serial.print(sbus_channels[CH_YAW]);
    Serial.print(F(" ch7="));         Serial.print(sbus_channels[CH_ARM]);
    Serial.print(F(" ch8="));         Serial.print(sbus_channels[CH_MODE]);
    Serial.print(F(" sbusok="));      Serial.print(sbus_frames_ok);
    Serial.print(F(" sbusbad="));     Serial.print(sbus_frames_bad);
    Serial.print(F(" sbusfs="));      Serial.print(sbus_failsafe_flag ? 1 : 0);
    Serial.print(F(" sbuslost="));    Serial.println(sbus_frame_lost_flag ? 1 : 0);
    last_status = now;
  }
}

// ============================================================================
//  GPIO
// ============================================================================
void init_gpio() {
  pinMode(RED_LIGHT_PIN,    OUTPUT);
  pinMode(GREEN_LIGHT_PIN,  OUTPUT);
  pinMode(YELLOW_LIGHT_PIN, OUTPUT);
  pinMode(BUZZER_PIN,       OUTPUT);
  pinMode(ESTOP_RELAY_PIN,  OUTPUT);
  pinMode(ESTOP_READ_PIN,   INPUT);

  digitalWrite(ESTOP_RELAY_PIN, LOW);
  relay_on = false;
  digitalWrite(RED_LIGHT_PIN,    LOW);
  digitalWrite(GREEN_LIGHT_PIN,  LOW);
  digitalWrite(YELLOW_LIGHT_PIN, LOW);
  digitalWrite(BUZZER_PIN,       LOW);
}

void startup_light_sequence() {
  const int   pins[3] = { RED_LIGHT_PIN, YELLOW_LIGHT_PIN, GREEN_LIGHT_PIN };
  const int   steps   = 6;
  const unsigned long step_ms = 150;
  unsigned long start = millis();
  int last_step = -1;
  while (true) {
    unsigned long e = millis() - start;
    int step = e / step_ms;
    if (step >= steps) break;
    if (step != last_step) {
      digitalWrite(RED_LIGHT_PIN,    LOW);
      digitalWrite(YELLOW_LIGHT_PIN, LOW);
      digitalWrite(GREEN_LIGHT_PIN,  LOW);
      digitalWrite(pins[step % 3], HIGH);
      last_step = step;
    }
  }
  digitalWrite(RED_LIGHT_PIN,    LOW);
  digitalWrite(YELLOW_LIGHT_PIN, LOW);
  digitalWrite(GREEN_LIGHT_PIN,  LOW);
}

void set_light(OperationMode mode, bool live) {
  digitalWrite(RED_LIGHT_PIN,    LOW);
  digitalWrite(YELLOW_LIGHT_PIN, LOW);
  digitalWrite(GREEN_LIGHT_PIN,  LOW);
  if (!live || mode == MODE_ESTOP) { digitalWrite(RED_LIGHT_PIN, HIGH); return; }
  if (mode == MODE_MANUAL)         { digitalWrite(YELLOW_LIGHT_PIN, HIGH); return; }
  if (mode == MODE_AUTONOMOUS)     { digitalWrite(GREEN_LIGHT_PIN,  HIGH); return; }
}

// ============================================================================
//  SBUS
// ============================================================================
void handle_sbus() {
  unsigned long now = millis();
  while (Serial2.available()) {
    byte b = Serial2.read();
    if (sbus_frame_index == 0 && b == SBUS_SYNC_BYTE) {
      sbus_frame[0] = b;
      sbus_frame_index = 1;
    } else if (sbus_frame_index > 0) {
      sbus_frame[sbus_frame_index++] = b;
      if (sbus_frame_index >= SBUS_FRAME_SIZE) {
        parse_sbus_frame();
        if (sbus_frame_ready) { last_sbus_read_ms = now; sbus_frame_ready = false; }
        sbus_frame_index = 0;
      }
    }
  }
}

void parse_sbus_frame() {
  // FIX: only the sync byte identifies a valid frame. The old footer check
  // (`sbus_frame[24] != 0x00`) rejected every frame whose flags byte had any
  // bit set -- but that byte legitimately carries CH17/CH18 digital-channel
  // states plus frame-lost/failsafe flags, so on most receivers it is *never*
  // reliably 0x00. That silently dropped 100% of frames on a perfectly
  // healthy SBUS link.
  if (sbus_frame[0] != SBUS_SYNC_BYTE) { sbus_frames_bad++; return; }

  sbus_channels[0]  = ((sbus_frame[1]      | sbus_frame[2]  << 8) & 0x7FF);
  sbus_channels[1]  = ((sbus_frame[2]  >> 3| sbus_frame[3]  << 5) & 0x7FF);
  sbus_channels[2]  = ((sbus_frame[3]  >> 6| sbus_frame[4]  << 2 | sbus_frame[5] << 10) & 0x7FF);
  sbus_channels[3]  = ((sbus_frame[5]  >> 1| sbus_frame[6]  << 7) & 0x7FF);
  sbus_channels[4]  = ((sbus_frame[6]  >> 4| sbus_frame[7]  << 4) & 0x7FF);
  sbus_channels[5]  = ((sbus_frame[7]  >> 7| sbus_frame[8]  << 1 | sbus_frame[9] << 9) & 0x7FF);
  sbus_channels[6]  = ((sbus_frame[9]  >> 2| sbus_frame[10] << 6) & 0x7FF);
  sbus_channels[7]  = ((sbus_frame[10] >> 5| sbus_frame[11] << 3) & 0x7FF);
  sbus_channels[8]  = ((sbus_frame[12]     | sbus_frame[13] << 8) & 0x7FF);
  sbus_channels[9]  = ((sbus_frame[13] >> 3| sbus_frame[14] << 5) & 0x7FF);
  sbus_channels[10] = ((sbus_frame[14] >> 6| sbus_frame[15] << 2 | sbus_frame[16] << 10) & 0x7FF);
  sbus_channels[11] = ((sbus_frame[16] >> 1| sbus_frame[17] << 7) & 0x7FF);
  sbus_channels[12] = ((sbus_frame[17] >> 4| sbus_frame[18] << 4) & 0x7FF);
  sbus_channels[13] = ((sbus_frame[18] >> 7| sbus_frame[19] << 1 | sbus_frame[20] << 9) & 0x7FF);
  sbus_channels[14] = ((sbus_frame[20] >> 2| sbus_frame[21] << 6) & 0x7FF);
  sbus_channels[15] = ((sbus_frame[21] >> 5| sbus_frame[22] << 3) & 0x7FF);

  // Flags byte (index 23): bit2 = frame lost, bit3 = failsafe. Surfaced for
  // diagnostics; a transient frame-lost bit doesn't invalidate the frame.
  byte flags = sbus_frame[23];
  sbus_frame_lost_flag = flags & 0x04;
  sbus_failsafe_flag   = flags & 0x08;

  sbus_frames_ok++;
  sbus_frame_ready = true;
}

// ============================================================================
//  STATE MACHINE  (final authority spec)
// ============================================================================
bool serial_link_live() {
  return (millis() - last_heartbeat_ms) <= HEARTBEAT_TIMEOUT_MS;
}

void update_state() {
  uint16_t arm_ch  = sbus_channels[CH_ARM];
  uint16_t mode_ch = sbus_channels[CH_MODE];

  // --- Resolve Ch8 zone ---
  OperationMode new_mode;
#if BENCH_NO_RC_OVERRIDE
  new_mode = serial_wants_auto ? MODE_AUTONOMOUS : MODE_MANUAL;
  bool arm_high = true;
#else
  if (mode_ch < MODE_LOW_MAX) {
    // ZONE 1: hardware ESTOP, absolute priority. Nothing below can override.
    new_mode = MODE_ESTOP;
  } else if (mode_ch < MODE_MID_MAX) {
    // ZONE 2: MANUAL forced. The Pi cannot take autonomy here.
    new_mode = MODE_MANUAL;
  } else {
    // ZONE 3: autonomy PERMITTED. Default MANUAL until the Pi asks for AUTO,
    // and only while the serial link is alive (heartbeat fresh).
    if (serial_wants_auto && serial_link_live()) new_mode = MODE_AUTONOMOUS;
    else                                         new_mode = MODE_MANUAL;
  }
  // Receiver-declared failsafe overrides everything, same as a stale link.
  if (sbus_failsafe_flag) new_mode = MODE_ESTOP;

  bool arm_high = (arm_ch > ARM_THRESHOLD);
#endif

  bool want_armed = arm_high && (new_mode != MODE_ESTOP);

  // Clear the power-loss latch when operator disarms or selects ESTOP
  if (!arm_high || new_mode == MODE_ESTOP) estop_latched = false;
  if (estop_latched) want_armed = false;

  if (new_mode != current_mode || want_armed != armed) {
    apply_transition(new_mode, want_armed);
  }
}

void apply_transition(OperationMode new_mode, bool want_armed) {
  set_both(PWM_NEUTRAL_US);

  current_mode = new_mode;
  armed        = want_armed;

  if (current_mode == MODE_ESTOP) {
    set_relay(false);
    set_light(MODE_ESTOP, false);
    startBeeps(1, 1000, 0);
    Serial.println(F(">>> MODE: E-STOP"));
    return;
  }

  if (armed) {
    set_relay(true);
    set_light(current_mode, true);
    if (current_mode == MODE_MANUAL)      startBeeps(2, 200, 200);
    else                                  startBeeps(3, 150, 150);
    Serial.print(F(">>> ARMED: "));
    Serial.println(current_mode == MODE_MANUAL ? F("MANUAL") : F("AUTONOMOUS"));
  } else {
    set_relay(false);
    set_light(current_mode, false);
    Serial.print(F(">>> DISARMED ("));
    Serial.print(current_mode == MODE_MANUAL ? F("manual") : F("auto"));
    Serial.println(F(")"));
  }
}

void set_relay(bool on) {
  if (on) {
    if (!relay_on) {
      digitalWrite(ESTOP_RELAY_PIN, HIGH);
      relay_on    = true;
      relay_on_ms = millis();
      power_low_pending = false;
    }
  } else {
    digitalWrite(ESTOP_RELAY_PIN, LOW);
    relay_on = false;
    power_low_pending = false;
  }
}

// ============================================================================
//  MOTOR OUTPUT
// ============================================================================
void update_motors() {
  if (!armed) { set_both(PWM_NEUTRAL_US); return; }

  if (millis() - relay_on_ms < ESC_ARM_DELAY_MS) { set_both(PWM_NEUTRAL_US); return; }

  if (current_mode == MODE_MANUAL) {
    float thr     = sbus_to_norm(sbus_channels[CH_THROTTLE]) * THR_INVERT;
    float yaw_raw = sbus_to_norm(sbus_channels[CH_YAW]) * YAW_INVERT;
    float yaw     = yaw_raw * YAW_GAIN;

    float left, right;

#if PIVOT_AT_MAX_YAW
    if (fabs(yaw_raw) >= PIVOT_THRESHOLD) {
      left  = (yaw_raw > 0) ? +1.0f : -1.0f;
      right = -left;
    } else
#endif
    {
      left  = thr + yaw;
      right = thr - yaw;
      float m = max(fabs(left), fabs(right));
      if (m > 1.0f) { left /= m; right /= m; }
    }

    int span = (int)(500.0f * MANUAL_AUTHORITY);
    write_thrusters(PWM_NEUTRAL_US + (int)(left  * span),
                    PWM_NEUTRAL_US + (int)(right * span));
  }
  // AUTONOMOUS thrust is written directly in handle_serial_input()
}

float sbus_to_norm(uint16_t v) {
  int c = (int)v - (int)SBUS_MID;
  if (abs(c) < (int)SBUS_DEADBAND) return 0.0f;
  float n;
  if (c > 0) n = (float)(c - SBUS_DEADBAND) / (float)((int)SBUS_MAX - (int)SBUS_MID - (int)SBUS_DEADBAND);
  else       n = (float)(c + SBUS_DEADBAND) / (float)((int)SBUS_MID - (int)SBUS_MIN - (int)SBUS_DEADBAND);
  return constrain(n, -1.0f, 1.0f);
}

void set_both(int pulse) {
  thruster_left.writeMicroseconds(pulse);
  thruster_right.writeMicroseconds(pulse);
}

void write_thrusters(int left, int right) {
  left  = constrain(left,  PWM_MIN_US, PWM_MAX_US);
  right = constrain(right, PWM_MIN_US, PWM_MAX_US);
  thruster_left.writeMicroseconds(left);
  thruster_right.writeMicroseconds(right);
}

// ============================================================================
//  SERIAL INPUT  (Pi -> Pico): motor values, MODE verbs, PING heartbeat
// ============================================================================
void handle_serial_input() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // Any valid line from the Pi counts as a heartbeat.
  last_heartbeat_ms = millis();

  // --- Mode verbs (effective only where update_state() allows) ---
  String up = line;
  up.toUpperCase();

  if (up == F("PING")) {
    return;  // heartbeat only
  }
  if (up == F("MODE AUTO")) {
    serial_wants_auto = true;
    Serial.println(F("[ACK] MODE AUTO requested"));
    return;
  }
  if (up == F("MODE MANUAL")) {
    serial_wants_auto = false;
    Serial.println(F("[ACK] MODE MANUAL requested"));
    return;
  }

  // --- Otherwise: motor command (bare values, unchanged behavior) ---
  // Only act on motors when actually armed & autonomous & past arm window.
  int l, r;
  if (process_motor_command(line, l, r)) {
    last_serial_command_ms = millis();
    if (armed && current_mode == MODE_AUTONOMOUS &&
        (millis() - relay_on_ms >= ESC_ARM_DELAY_MS)) {
      write_thrusters(l, r);
    }
    // If not in autonomous, command is accepted as heartbeat but NOT applied.
  } else {
    Serial.print(F("Invalid cmd: "));
    Serial.println(line);
  }
}

bool process_motor_command(const String &command, int &left, int &right) {
  String cmd = command;
  cmd.replace('[', ' '); cmd.replace(']', ' ');
  cmd.replace(',', ' '); cmd.replace(';', ' ');
  cmd.trim();

  float v1 = 0, v2 = 0;
  int n = sscanf(cmd.c_str(), "%f %f", &v1, &v2);
  if (n >= 1) {
    bool norm = (fabs(v1) <= 1.5f) && (n == 1 || fabs(v2) <= 1.5f);
    if (norm) {
      left  = PWM_NEUTRAL_US + (int)(constrain(v1, -1.0f, 1.0f) * 500.0f);
      right = (n == 2) ? PWM_NEUTRAL_US + (int)(constrain(v2, -1.0f, 1.0f) * 500.0f) : left;
    } else {
      left  = constrain((int)v1, PWM_MIN_US, PWM_MAX_US);
      right = (n == 2) ? constrain((int)v2, PWM_MIN_US, PWM_MAX_US) : left;
    }
    return true;
  }
  return false;
}

// ============================================================================
//  E-STOP POWER FEEDBACK  (disabled until hardware ready)
// ============================================================================
void check_power_feedback() {
#if ESTOP_FEEDBACK_ENABLED
  if (!armed || !relay_on) { power_low_pending = false; return; }
  if (millis() - relay_on_ms < ESTOP_GRACE_MS) { power_low_pending = false; return; }

  bool powered = (digitalRead(ESTOP_READ_PIN) == HIGH);
  if (powered) {
    power_low_pending = false;
  } else {
    unsigned long now = millis();
    if (!power_low_pending) { power_low_pending = true; power_low_since_ms = now; }
    else if (now - power_low_since_ms >= ESTOP_DEBOUNCE_MS) {
      trigger_estop("power feedback LOW");
    }
  }
#else
  (void)power_low_pending;
#endif
}

void trigger_estop(const char *reason) {
  armed = false;
  estop_latched = true;
  set_relay(false);
  set_both(PWM_NEUTRAL_US);
  set_light(MODE_ESTOP, false);
  startBeeps(1, 1000, 0);
  Serial.print(F("[E-STOP] "));
  Serial.println(reason);
}
