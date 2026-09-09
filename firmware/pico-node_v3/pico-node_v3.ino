// ============================================================================
//  Asket EC  -  Pico 2 (RP2350) Low-Level Controller   [REBUILT]
//  Core: Earle Philhower arduino-pico  |  USB stack: Adafruit TinyUSB
//
//  Modes (Ch8, 3-pos):  ESTOP / MANUAL / AUTONOMOUS
//  Arm   (Ch7, 2-pos):  high = armed
//  Manual: skid-steer mix of throttle (Ch3) + yaw (Ch4)
//  Autonomous: motor pulses over USB serial
//
//  Serial commands (added): "CMD MODE <ESTOP|MANUAL|AUTONOMOUS>" and "CMD ESTOP".
//  The RC transmitter stays sovereign - see the arbitration table below.
//
//  Non-blocking throughout: no delay() anywhere, millis() only.
// ============================================================================

#include <Servo.h>
#include <Adafruit_TinyUSB.h>

// snprintf / strncmp / strncpy, used by the command parser and the status line.
// Arduino.h normally pulls these in; naming them is cheap insurance.
#include <stdio.h>
#include <string.h>

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
const int MODE_MID_MAX   = 1400;  // Ch8 < this = MANUAL, else AUTONOMOUS

// ---------------------------------------------------------------------------
//  ESC ARMING / RELAY
//  Relay cuts ESC power, so each power-up needs a neutral window.
// ---------------------------------------------------------------------------
const unsigned long ESC_ARM_DELAY_MS = 2000;  // hold neutral after relay closes

// ---------------------------------------------------------------------------
//  E-STOP FEEDBACK (GPIO20 voltage divider)
//  DISABLED for bench testing (trace cut). Set to 1 once hardware is ready.
//  WARNING: divider rail->10k->pin->3.3k->gnd is only pin-safe up to ~13.3 V.
// ---------------------------------------------------------------------------
#define ESTOP_FEEDBACK_ENABLED 0
const unsigned long ESTOP_GRACE_MS    = 500;  // ignore feedback this long after relay close
const unsigned long ESTOP_DEBOUNCE_MS = 100;  // continuous LOW required to confirm loss

// ---------------------------------------------------------------------------
//  SBUS FRAME (manual parse, hardware inversion) - kept, it works for you
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

// Autonomous serial failsafe
unsigned long last_serial_command_ms = 0;
const unsigned long SERIAL_FAILSAFE_TIMEOUT_MS = 500;

// ---------------------------------------------------------------------------
//  SOFTWARE MODE REQUESTS
//
//  A software request can only ever take authority away. It never persists
//  beyond SOFTWARE_REQUEST_TIMEOUT_MS without being refreshed, so a GUI that
//  crashes while holding one cannot lock the vessel out until reboot.
//
//  A LATCHED e-stop is a different thing and deliberately does not expire:
//  trigger_estop() sets estop_latched, cleared only by the operator cycling the
//  arm switch or selecting ESTOP on Ch8. That behaviour is unchanged.
// ---------------------------------------------------------------------------

//  May software ever request AUTONOMOUS?
//    0 = downward-only. The GUI may request MANUAL or ESTOP and nothing else.
//        To return to AUTONOMOUS it stops refreshing and lets the request
//        expire, which hands authority back to Ch8.
//    1 = AUTONOMOUS is requestable too, still clamped by Ch8.
//  Mirrored by SOFTWARE_UPWARD_REQUESTS_ALLOWED in
//  src/asket_common/asket_common/mode_arbitration.py - change both or neither.
#define SOFTWARE_UPWARD_REQUESTS_ALLOWED 0

const unsigned long SOFTWARE_REQUEST_TIMEOUT_MS = 5000;

//  0 = no active request. The OperationMode enum starts at 1, so 0 is free.
const int REQUEST_NONE = 0;

int           software_request    = REQUEST_NONE;
unsigned long software_request_ms = 0;

// ---------------------------------------------------------------------------
//  MODES + STATE
// ---------------------------------------------------------------------------
enum OperationMode { MODE_ESTOP = 1, MODE_MANUAL = 2, MODE_AUTONOMOUS = 3 };

// ===========================================================================
//  MODE ARBITRATION TABLE
//
//  The RC transmitter is sovereign. Software may only ever request a state at
//  least as restrictive as what Ch8 currently allows.
//
//      Ordering, least to most permissive:  ESTOP < MANUAL < AUTONOMOUS
//      Effective mode = the MORE RESTRICTIVE of (Ch8, active software request)
//
//  The enum values above are already in that order, so the whole rule is a
//  numeric comparison and the table below is what it produces.
//
//  Rows are the RC switch. Columns are the software request. "-" = no active
//  request (never sent, or expired). Each cell is the effective mode, and
//  (rej:reason) marks a request that is REJECTED rather than silently clamped -
//  the GUI is told why instead of showing a button press that did nothing.
//
//  SOFTWARE_UPWARD_REQUESTS_ALLOWED = 0  (default build)
//
//    Ch8 \ req |    -     |  ESTOP  |  MANUAL          |  AUTONOMOUS
//    ----------+----------+---------+------------------+---------------------
//    ESTOP     | ESTOP    | ESTOP   | ESTOP (rej:rc)   | ESTOP  (rej:upward)
//    MANUAL    | MANUAL   | ESTOP   | MANUAL           | MANUAL (rej:upward)
//    AUTONOMOUS| AUTONOM. | ESTOP   | MANUAL           | AUTONOM.(rej:upward)
//
//  SOFTWARE_UPWARD_REQUESTS_ALLOWED = 1
//
//    Ch8 \ req |    -     |  ESTOP  |  MANUAL          |  AUTONOMOUS
//    ----------+----------+---------+------------------+---------------------
//    ESTOP     | ESTOP    | ESTOP   | ESTOP (rej:rc)   | ESTOP  (rej:rc)
//    MANUAL    | MANUAL   | ESTOP   | MANUAL           | MANUAL (rej:rc)
//    AUTONOMOUS| AUTONOM. | ESTOP   | MANUAL           | AUTONOMOUS
//
//  Read the two invariants off the table rather than trusting the prose:
//    1. No cell is ever more permissive than its row. Software cannot add
//       authority, only remove it.
//    2. The ESTOP column is ESTOP everywhere. "Cut propulsion" works from any
//       state, in either build.
//
//  This table is mirrored, cell for cell, by arbitrate() in
//  src/asket_common/asket_common/mode_arbitration.py, whose test enumerates all
//  24 combinations. That test is what stands in for hardware here.
// ===========================================================================

struct ModeDecision {
  OperationMode effective;
  bool          accepted;
  const char   *reason;    // "" when accepted
};

// Pure: depends only on its arguments. No globals, no clock, no I/O - so the
// table above can be reasoned about (and mirrored in Python) without a boat.
ModeDecision arbitrate_mode(OperationMode rc_mode, int request, bool allow_upward) {
  ModeDecision d = { rc_mode, true, "" };

  if (request == REQUEST_NONE) return d;

  if (request != MODE_ESTOP && request != MODE_MANUAL && request != MODE_AUTONOMOUS) {
    d.accepted = false;
    d.reason   = "unknown_mode";
    return d;
  }

  // ESTOP is always available, from any state, in either build. It is the one
  // request that can only ever remove authority.
  if (request == MODE_ESTOP) {
    d.effective = MODE_ESTOP;
    return d;
  }

  if (request == MODE_AUTONOMOUS && !allow_upward) {
    d.accepted = false;
    d.reason   = "upward_disabled";
    return d;
  }

  if (request > (int)rc_mode) {          // less restrictive than Ch8 allows
    d.accepted = false;
    d.reason   = "rc_clamp";
    return d;
  }

  d.effective = (OperationMode)request;  // request <= rc_mode, so this is the
  return d;                              // more restrictive of the two
}

OperationMode current_mode = MODE_ESTOP;
bool armed = false;

bool          relay_on      = false;   // tracks actual relay state
unsigned long relay_on_ms   = 0;       // when relay last went off->on

bool          estop_latched = false;   // power-loss latch (blocks auto re-arm)
bool          power_low_pending = false;
unsigned long power_low_since_ms = 0;

Servo thruster_left;
Servo thruster_right;

// ---------------------------------------------------------------------------
//  NON-BLOCKING SERIAL OUTPUT
//
//  Serial.print() blocks once the USB CDC buffer fills, which would stall
//  loop() and with it the failsafes. Lines are queued here and drained only as
//  far as availableForWrite() allows.
//
//  The queue is deliberately small. If it fills, a STATUS line is dropped (the
//  next one is 250 ms away) but an ACK is not: an acknowledgement the GUI never
//  receives is reported to the operator as a command that failed, which is the
//  correct reading, but losing it silently would not be.
// ---------------------------------------------------------------------------
const int  OUT_QUEUE_LINES = 6;
const int  OUT_LINE_MAX    = 112;

char out_queue[OUT_QUEUE_LINES][OUT_LINE_MAX];
int  out_head = 0;      // next to send
int  out_count = 0;     // lines waiting
bool out_dropped = false;

void emit_line(const char *s) {
  if (out_count >= OUT_QUEUE_LINES) { out_dropped = true; return; }
  int slot = (out_head + out_count) % OUT_QUEUE_LINES;
  strncpy(out_queue[slot], s, OUT_LINE_MAX - 1);
  out_queue[slot][OUT_LINE_MAX - 1] = '\0';
  out_count++;
}

// Send whatever fits right now. Never waits.
void flush_serial_out() {
  while (out_count > 0) {
    const char *line = out_queue[out_head];
    int needed = (int)strlen(line) + 2;            // + CRLF
    if (Serial.availableForWrite() < needed) return;
    Serial.println(line);
    out_head = (out_head + 1) % OUT_QUEUE_LINES;
    out_count--;
  }
  if (out_dropped && Serial.availableForWrite() > 32) {
    out_dropped = false;
    Serial.println(F("[WARN] serial output dropped"));
  }
}

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
      if (beeper.remaining <= 0) beeper.active = false;  // done after last ON
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
OperationMode rc_mode_from_channel(uint16_t mode_ch);
void  apply_transition(OperationMode new_mode, bool want_armed);
void  set_relay(bool on);
void  update_motors();
void  handle_serial_input();
void  process_serial_line(char *line);
bool  process_command(const char *body);
void  expire_software_request();
bool  process_motor_command(const String &cmd, int &left, int &right);
void  emit_line(const char *s);
void  flush_serial_out();
void  check_power_feedback();
void  trigger_estop(const char *reason);
float sbus_to_norm(uint16_t v);
void  write_thrusters(int left, int right);
void  set_both(int pulse);

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // Wait (non-blocking) up to 3 s for USB CDC to mount
  unsigned long t0 = millis();
  while (!TinyUSBDevice.mounted() && (millis() - t0 < 3000)) { /* spin */ }

  init_gpio();

  // SBUS receiver on Serial2 (UART1) with hardware inversion
  Serial2.setRX(5);
  Serial2.setTX(4);
  Serial2.setInvertRX(true);
  Serial2.begin(100000, SERIAL_8E2);

  thruster_left.attach(THRUSTER_LEFT_PIN,  PWM_MIN_US, PWM_MAX_US);
  thruster_right.attach(THRUSTER_RIGHT_PIN, PWM_MIN_US, PWM_MAX_US);
  set_both(PWM_NEUTRAL_US);

  last_serial_command_ms = millis();
  last_sbus_read_ms      = millis();

  startup_light_sequence();      // millis-based, non-blocking
  set_light(MODE_ESTOP, false);  // start safe: red

  Serial.println(F("=== Asket EC Pico Controller (rebuilt) ==="));
  Serial.println(F("Mode: ESTOP / MANUAL / AUTONOMOUS"));
  Serial.print  (F("E-STOP feedback: "));
  Serial.println(ESTOP_FEEDBACK_ENABLED ? F("ENABLED") : F("DISABLED (bench)"));
  Serial.println(F("=========================================="));
}

// ============================================================================
//  MAIN LOOP  (no delay(), millis only)
// ============================================================================
void loop() {
  unsigned long now = millis();

  handle_sbus();              // read + parse SBUS
  handle_serial_input();      // CMD in any mode; setpoints only in AUTONOMOUS
  update_state();             // arbitrate Ch8 vs software request, then arm
  check_power_feedback();     // e-stop divider (disabled until hardware ready)

  updateBeeper();
  update_motors();

  // --- Failsafes ---
  if (armed) {
    // SBUS lost (manual or while waiting): drop to e-stop
    if (current_mode != MODE_AUTONOMOUS &&
        (now - last_sbus_read_ms > SBUS_FAILSAFE_TIMEOUT_MS)) {
      trigger_estop("SBUS timeout");
    }
    // Serial lost in autonomous: hold neutral (don't kill power)
    if (current_mode == MODE_AUTONOMOUS &&
        (now - last_serial_command_ms > SERIAL_FAILSAFE_TIMEOUT_MS)) {
      set_both(PWM_NEUTRAL_US);
    }
  }

  // --- Periodic status (throttled) ---
  //
  // Field names and order are UNCHANGED. gui_backend/core/pico_state.py parses
  // this line by exact key, and `Mode` and `Mode(Ch8)` are two different fields
  // carrying two different units - do not rename either without changing the
  // parser and its tests.
  static unsigned long last_status = 0;
  if (now - last_status > 250) {
    char line[OUT_LINE_MAX];
    snprintf(line, sizeof(line),
             "[STAT] Mode:%d Armed:%c Relay:%s "
             "Thr(Ch3):%u Yaw(Ch4):%u Arm(Ch7):%u Mode(Ch8):%u",
             (int)current_mode,
             armed ? 'Y' : 'N',
             relay_on ? "ON" : "OFF",
             (unsigned)sbus_channels[CH_THROTTLE],
             (unsigned)sbus_channels[CH_YAW],
             (unsigned)sbus_channels[CH_ARM],
             (unsigned)sbus_channels[CH_MODE]);
    emit_line(line);
    last_status = now;
  }

  flush_serial_out();         // sends only what fits; never waits
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
  pinMode(ESTOP_READ_PIN,   INPUT);   // external divider sets the level; break = LOW = safe

  digitalWrite(ESTOP_RELAY_PIN, LOW); // relay off = ESC power cut at boot
  relay_on = false;
  digitalWrite(RED_LIGHT_PIN,    LOW);
  digitalWrite(GREEN_LIGHT_PIN,  LOW);
  digitalWrite(YELLOW_LIGHT_PIN, LOW);
  digitalWrite(BUZZER_PIN,       LOW);
}

// Non-blocking boot blink (millis-based)
void startup_light_sequence() {
  const int   pins[3] = { RED_LIGHT_PIN, YELLOW_LIGHT_PIN, GREEN_LIGHT_PIN };
  const int   steps   = 6;     // 2 cycles x 3 colors
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

// Light convention: RED whenever not live (disarmed or estop).
// Yellow = manual & armed, Green = autonomous & armed.
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
  if (sbus_frame[0] != SBUS_SYNC_BYTE || sbus_frame[24] != 0x00) return;
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
  sbus_frame_ready = true;
}

// ============================================================================
//  STATE MACHINE
// ============================================================================
// Ch8 to a mode. The single place that threshold lives.
OperationMode rc_mode_from_channel(uint16_t mode_ch) {
  if (mode_ch < MODE_LOW_MAX) return MODE_ESTOP;
  if (mode_ch < MODE_MID_MAX) return MODE_MANUAL;
  return MODE_AUTONOMOUS;
}

void update_state() {
  uint16_t arm_ch  = sbus_channels[CH_ARM];
  uint16_t mode_ch = sbus_channels[CH_MODE];

  OperationMode rc_mode = rc_mode_from_channel(mode_ch);

  expire_software_request();

  // Re-arbitrated every pass, not just when a command arrives: the operator can
  // move Ch8 while a request is held, and Ch8 wins the moment they do.
  ModeDecision d = arbitrate_mode(rc_mode, software_request,
                                  SOFTWARE_UPWARD_REQUESTS_ALLOWED);
  OperationMode new_mode = d.effective;

  bool arm_high   = (arm_ch > ARM_THRESHOLD);
  bool want_armed = arm_high && (new_mode != MODE_ESTOP);

  // Clear the power-loss latch when the operator disarms or selects ESTOP.
  //
  // Keyed on the OPERATOR's switch, never on the effective mode: a software
  // ESTOP request must not be able to clear the latch it just set. Clearing it
  // stays a physical act, exactly as before this change.
  if (!arm_high || rc_mode == MODE_ESTOP) estop_latched = false;
  if (estop_latched) want_armed = false;

  if (new_mode != current_mode || want_armed != armed) {
    apply_transition(new_mode, want_armed);
  }
}

void apply_transition(OperationMode new_mode, bool want_armed) {
  set_both(PWM_NEUTRAL_US);     // always neutral on any change first

  current_mode = new_mode;
  armed        = want_armed;

  if (current_mode == MODE_ESTOP) {
    set_relay(false);
    set_light(MODE_ESTOP, false);
    startBeeps(1, 1000, 0);                    // 1 s
    emit_line(">>> MODE: E-STOP");
    return;
  }

  if (armed) {
    set_relay(true);                            // energize (resets arm window if was off)
    set_light(current_mode, true);
    if (current_mode == MODE_MANUAL)      startBeeps(2, 200, 200);   // 2 short
    else                                  startBeeps(3, 150, 150);   // 3 short
    emit_line(current_mode == MODE_MANUAL ? ">>> ARMED: MANUAL"
                                          : ">>> ARMED: AUTONOMOUS");
  } else {
    set_relay(false);
    set_light(current_mode, false);             // red while disarmed
    emit_line(current_mode == MODE_MANUAL ? ">>> DISARMED (manual)"
                                          : ">>> DISARMED (auto)");
  }
}

void set_relay(bool on) {
  if (on) {
    if (!relay_on) {                 // only stamp the timer on a real off->on edge
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

  // ESC cold-boot: hold neutral until the arming window elapses
  if (millis() - relay_on_ms < ESC_ARM_DELAY_MS) { set_both(PWM_NEUTRAL_US); return; }

  if (current_mode == MODE_MANUAL) {
    float thr     = sbus_to_norm(sbus_channels[CH_THROTTLE]) * THR_INVERT;
    float yaw_raw = sbus_to_norm(sbus_channels[CH_YAW]) * YAW_INVERT;  // pre-gain, for pivot test
    float yaw     = yaw_raw * YAW_GAIN;

    float left, right;

#if PIVOT_AT_MAX_YAW
    if (fabs(yaw_raw) >= PIVOT_THRESHOLD) {
      // Hard-over yaw: pivot in place with full opposite thrust
      left  = (yaw_raw > 0) ? +1.0f : -1.0f;
      right = -left;
    } else
#endif
    {
      left  = thr + yaw;                       // skid-steer mix
      right = thr - yaw;
      // Proportional normalize: keep the turn ratio instead of clipping one side
      float m = max(fabs(left), fabs(right));
      if (m > 1.0f) { left /= m; right /= m; }
    }

    int span = (int)(500.0f * MANUAL_AUTHORITY);
    write_thrusters(PWM_NEUTRAL_US + (int)(left  * span),
                    PWM_NEUTRAL_US + (int)(right * span));
  }
  // AUTONOMOUS thrust is written directly in handle_serial_input()
}

// Map an SBUS channel to -1..+1 with a center deadband.
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
//  SERIAL INPUT
//
//  Read on EVERY loop, in every mode. It has to be: "CMD ESTOP" and a downward
//  mode request must work while the vessel is in MANUAL, and the old code only
//  called this in AUTONOMOUS + armed, so nothing sent in any other state was
//  ever read at all.
//
//  What is accepted still depends on the mode:
//    - "CMD ..." lines      : always parsed, in every mode.
//    - motor setpoints      : ONLY in AUTONOMOUS + armed + past the arm window.
//                             Discarded silently otherwise, so a setpoint sent
//                             just before a mode change cannot sit in a buffer
//                             and be applied after it.
//
//  Serial.readStringUntil() is gone: it blocks for up to its timeout, which was
//  survivable while this ran rarely and is not now that it runs every loop.
// ============================================================================
const int SERIAL_LINE_MAX = 96;
char   serial_line[SERIAL_LINE_MAX];
int    serial_len = 0;
bool   serial_overflow = false;    // discarding until the next newline

void handle_serial_input() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\r') continue;

    if (c == '\n') {
      if (serial_overflow) {
        serial_overflow = false;         // the overlong line ends here
        serial_len = 0;
        emit_line("Invalid cmd: <line too long>");
        continue;
      }
      serial_line[serial_len] = '\0';
      process_serial_line(serial_line);
      serial_len = 0;
      continue;
    }

    if (serial_overflow) continue;       // still swallowing the overlong line

    if (serial_len < SERIAL_LINE_MAX - 1) {
      serial_line[serial_len++] = c;
    } else {
      serial_overflow = true;            // never overrun the buffer
      serial_len = 0;
    }
  }
}

void process_serial_line(char *line) {
  while (*line == ' ' || *line == '\t') line++;      // trim leading
  int n = (int)strlen(line);
  while (n > 0 && (line[n - 1] == ' ' || line[n - 1] == '\t')) line[--n] = '\0';
  if (n == 0) return;

  if (strncmp(line, "CMD ", 4) == 0) {
    process_command(line + 4);
    return;
  }

  // The bridge's heartbeat. Deliberately does NOT refresh
  // last_serial_command_ms: PING means "no fresh setpoint", so the autonomous
  // serial failsafe should still fall to neutral, which is the whole point of
  // sending it. Ignored silently - it is not an invalid command.
  //
  // (Before this, PING fell through to the motor parser and printed
  // "Invalid cmd: PING" at the heartbeat rate, 20 Hz, whenever the vessel was
  // in AUTONOMOUS and armed.)
  if (strcmp(line, "PING") == 0) return;

  // Not a command, so it can only be a motor setpoint. Motor setpoints apply
  // in exactly one state; in every other, drop it without comment.
  if (current_mode != MODE_AUTONOMOUS || !armed) return;
  if (millis() - relay_on_ms < ESC_ARM_DELAY_MS) return;   // arm window

  int l, r;
  if (process_motor_command(String(line), l, r)) {
    last_serial_command_ms = millis();
    write_thrusters(l, r);
  } else {
    char msg[OUT_LINE_MAX];
    snprintf(msg, sizeof(msg), "Invalid cmd: %s", line);
    emit_line(msg);
  }
}

// "MODE <ESTOP|MANUAL|AUTONOMOUS>" or "ESTOP".
//
// Every command is acknowledged, accepted or rejected, so the GUI can show a
// real confirmation instead of assuming one. The reason code is machine
// readable and matches mode_arbitration.py, which owns the operator wording.
bool process_command(const char *body) {
  while (*body == ' ') body++;

  char ack[OUT_LINE_MAX];

  // Immediate cut. Always accepted, from any state, and it latches.
  if (strcmp(body, "ESTOP") == 0) {
    software_request    = MODE_ESTOP;
    software_request_ms = millis();
    emit_line("[ACK] ESTOP accepted");
    trigger_estop("commanded over serial");
    return true;
  }

  if (strncmp(body, "MODE ", 5) == 0) {
    const char *word = body + 5;
    while (*word == ' ') word++;

    int requested = REQUEST_NONE;
    if      (strcmp(word, "ESTOP")      == 0) requested = MODE_ESTOP;
    else if (strcmp(word, "MANUAL")     == 0) requested = MODE_MANUAL;
    else if (strcmp(word, "AUTONOMOUS") == 0) requested = MODE_AUTONOMOUS;
    else {
      snprintf(ack, sizeof(ack), "[ACK] MODE %s rejected unknown_mode", word);
      emit_line(ack);
      return false;
    }

    OperationMode rc_mode = rc_mode_from_channel(sbus_channels[CH_MODE]);
    ModeDecision d = arbitrate_mode(rc_mode, requested,
                                    SOFTWARE_UPWARD_REQUESTS_ALLOWED);

    if (!d.accepted) {
      // A rejected request must not disturb one already held.
      snprintf(ack, sizeof(ack), "[ACK] MODE %s rejected %s", word, d.reason);
      emit_line(ack);
      return false;
    }

    software_request    = requested;
    software_request_ms = millis();
    snprintf(ack, sizeof(ack), "[ACK] MODE %s accepted", word);
    emit_line(ack);

    // An accepted ESTOP request cuts power now rather than waiting for the
    // next update_state() pass.
    if (requested == MODE_ESTOP) trigger_estop("commanded over serial");
    return true;
  }

  snprintf(ack, sizeof(ack), "[ACK] %s rejected unknown_command", body);
  emit_line(ack);
  return false;
}

// A held request that stops being refreshed lapses, handing authority back to
// Ch8. Without this a GUI that died holding an ESTOP request would lock the
// vessel out until somebody power-cycled the Pico.
void expire_software_request() {
  if (software_request == REQUEST_NONE) return;
  if (millis() - software_request_ms <= SOFTWARE_REQUEST_TIMEOUT_MS) return;
  software_request = REQUEST_NONE;
  emit_line("[ACK] REQUEST expired");
}

bool process_motor_command(const String &command, int &left, int &right) {
  String cmd = command;
  cmd.replace('[', ' '); cmd.replace(']', ' ');
  cmd.replace(',', ' '); cmd.replace(';', ' ');
  cmd.trim();

  float v1 = 0, v2 = 0;
  int n = sscanf(cmd.c_str(), "%f %f", &v1, &v2);
  if (n >= 1) {
    // If values look normalized (-1..1) treat as norm, else as raw microseconds
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
  // Let the relay close + divider settle before trusting the read
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
  (void)power_low_pending;  // feature off: never disarms on the bench
#endif
}

void trigger_estop(const char *reason) {
  armed = false;
  estop_latched = true;          // block auto re-arm until operator cycles arm/mode
  set_relay(false);
  set_both(PWM_NEUTRAL_US);
  set_light(MODE_ESTOP, false);
  startBeeps(1, 1000, 0);
  char msg[OUT_LINE_MAX];
  snprintf(msg, sizeof(msg), "[E-STOP] %s", reason);
  emit_line(msg);
}
