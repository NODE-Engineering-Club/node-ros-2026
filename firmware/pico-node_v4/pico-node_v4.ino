// ============================================================================
//  pico-node v4  -  Asket EC low-level controller  -  Pico 2 (RP2350)
//  Core: Earle Philhower arduino-pico  |  USB stack: Adafruit TinyUSB
//
//  Built from pico-node_v3 [REBUILT] and asket_ec_pico [REBUILT + FOXGLOVE].
//  See firmware/README.md for what came from where and what was dropped.
//
//  Modes (Ch8, 3-pos):  ESTOP / MANUAL / AUTONOMY-PERMITTED
//  Arm   (Ch7, 2-pos):  high = armed
//
//  MODE AUTHORITY -- the whole safety argument, in five lines:
//    Ch8 LOW    -> ESTOP            hardware priority, nothing overrides it
//    Ch8 MID    -> MANUAL forced    the Jetson cannot take autonomy here
//    Ch8 HIGH   -> autonomy PERMITTED, not granted:
//                    default MANUAL, until the Jetson sends "MODE AUTO",
//                    and only while its heartbeat is fresh.
//  Software can restrict what the transmitter allows. It can never extend it.
//
//  Manual: skid-steer mix of throttle (Ch3) + yaw (Ch4), pivot at hard-over.
//  Autonomous: "L R" / "L,R" motor pulses over USB serial.
//
//  Non-blocking throughout: no delay() anywhere, millis() only, and the serial
//  reader is an accumulator rather than readStringUntil() -- see
//  handle_serial_input().
//
//  TELLING VERSIONS APART is the one thing v3 and FOXGLOVE could not do, and
//  it cost months of a silently dead uplink. So this build announces itself:
//    * "[VER] pico-node 4" once at startup
//    * "ver=4" as the FIRST field of every STATE line
//  pico_bridge reads it, the GUI shows it, and pre-flight FAILS on a mismatch.
//  If you fork this file, bump FW_VERSION. If you change the wire format in a
//  way a parser would notice, you MUST bump it.
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

// NOTE: FOXGLOVE carried a BENCH_NO_RC_OVERRIDE compile flag here which, at 1,
// replaced the entire Ch8 zone logic with `serial_wants_auto ? AUTONOMOUS :
// MANUAL` and forced arm_high = true -- removing the hardware E-stop's
// authority outright. It is deliberately NOT carried into v4. A build flag
// that silently deletes the safety chain is one wrong upload away from a boat
// that cannot be stopped from the beach, and nothing distinguishes the two
// binaries once they are flashed. Bench-test with the receiver powered and the
// props out of the water, like everything else.

// ---------------------------------------------------------------------------
//  FIRMWARE IDENTITY
//  Bump FW_VERSION on any change a parser could notice. The Jetson refuses to
//  fly against a version it does not know how to read, which is the point.
// ---------------------------------------------------------------------------
#define FW_NAME    "pico-node"
#define FW_VERSION 4

// Longest line the Jetson ever sends is "MODE MANUAL" (11) or a motor command
// like "-0.123,-0.123" (13). 64 is generous and costs nothing.
#define RX_BUF_LEN 64

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
void  apply_transition(OperationMode new_mode, bool want_armed);
void  set_relay(bool on);
void  update_motors();
void  handle_serial_input();
void  handle_serial_line(const String &line);
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
  last_heartbeat_ms      = millis();

  startup_light_sequence();      // millis-based, non-blocking
  set_light(MODE_ESTOP, false);  // start safe: red

  // Machine-readable identity, before any human-readable banner, so a host
  // that opens the port at boot learns what it is talking to immediately
  // rather than waiting up to 250 ms for the first STATE line.
  Serial.print(F("[VER] "));
  Serial.print(F(FW_NAME));
  Serial.print(' ');
  Serial.println(FW_VERSION);

  Serial.println(F("=== Asket EC Pico Controller (pico-node v4) ==="));
  Serial.println(F("Ch8 LOW=ESTOP  MID=MANUAL  HIGH=autonomy-permitted"));
  Serial.println(F("Serial: 'L R' or 'L,R' motors | 'MODE AUTO' | 'MODE MANUAL' | 'PING'"));
  Serial.print  (F("E-STOP feedback: "));
  Serial.println(ESTOP_FEEDBACK_ENABLED ? F("ENABLED") : F("DISABLED (bench)"));
  Serial.println(F("===================================================="));
}

// ============================================================================
//  MAIN LOOP
// ============================================================================
void loop() {
  unsigned long now = millis();

  handle_sbus();              // read + parse SBUS
  // Always drained, in every mode. v3 gated this on (AUTONOMOUS && armed),
  // which meant nothing the Jetson sent was ever consumed the rest of the
  // time -- so MODE/PING could not work, and whatever the host wrote piled up
  // in the USB buffer until it blocked, was dropped, or was replayed stale on
  // the next mode change. Draining costs microseconds; the verbs are gated by
  // update_state(), not by whether we bothered to read them.
  handle_serial_input();
  update_state();             // arm / mode transitions
  check_power_feedback();     // e-stop divider (disabled until hardware ready)

  updateBeeper();
  update_motors();

  // --- Failsafes ---
  if (armed) {
    // SBUS lost (manual or while waiting): drop to e-stop. Out of autonomy the
    // transmitter is the only thing driving, so silence means nobody is.
    if (current_mode != MODE_AUTONOMOUS &&
        (now - last_sbus_read_ms > SBUS_FAILSAFE_TIMEOUT_MS)) {
      trigger_estop("SBUS timeout");
    }
    // Serial/motor lost in autonomous: hold neutral, do NOT kill power. The
    // transmitter is still there and the operator can take manual; cutting the
    // relay would cost another 2 s arm window before they could manoeuvre.
    if (current_mode == MODE_AUTONOMOUS &&
        (now - last_serial_command_ms > SERIAL_FAILSAFE_TIMEOUT_MS)) {
      set_both(PWM_NEUTRAL_US);
    }
  }

  // --- Periodic status (parsable for Foxglove) ---
  static unsigned long last_status = 0;
  if (now - last_status > 250) {
    // Machine-readable line. Fields are key=value, space separated.
    //
    // ver= is FIRST and deliberately so: a parser that does not recognise the
    // version can reject the line before it misreads a single field. Bump
    // FW_VERSION whenever anything below this comment changes shape.
    Serial.print(F("STATE ver="));    Serial.print(FW_VERSION);
    Serial.print(F(" mode="));        Serial.print((int)current_mode);
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
  pinMode(ESTOP_READ_PIN,   INPUT);   // external divider sets the level; break = LOW = safe

  digitalWrite(ESTOP_RELAY_PIN, LOW); // relay off = ESC power cut at boot
  relay_on = false;
  digitalWrite(RED_LIGHT_PIN,    LOW);
  digitalWrite(GREEN_LIGHT_PIN,  LOW);
  digitalWrite(YELLOW_LIGHT_PIN, LOW);
  digitalWrite(BUZZER_PIN,       LOW);
}

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

  bool want_armed = arm_high && (new_mode != MODE_ESTOP);

  // Clear the power-loss latch when operator disarms or selects ESTOP
  if (!arm_high || new_mode == MODE_ESTOP) estop_latched = false;
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
    startBeeps(1, 1000, 0);
    Serial.println(F(">>> MODE: E-STOP"));
    return;
  }

  if (armed) {
    set_relay(true);                            // energize (resets arm window if was off)
    set_light(current_mode, true);
    if (current_mode == MODE_MANUAL)      startBeeps(2, 200, 200);   // 2 short
    else                                  startBeeps(3, 150, 150);   // 3 short
    Serial.print(F(">>> ARMED: "));
    Serial.println(current_mode == MODE_MANUAL ? F("MANUAL") : F("AUTONOMOUS"));
  } else {
    set_relay(false);
    set_light(current_mode, false);             // red while disarmed
    Serial.print(F(">>> DISARMED ("));
    Serial.print(current_mode == MODE_MANUAL ? F("manual") : F("auto"));
    Serial.println(F(")"));
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
// Accumulate bytes until a newline, then act on one complete line.
//
// v3 and FOXGLOVE both used Serial.readStringUntil('\n'), which blocks until
// the terminator arrives or the 1000 ms stream timeout expires. A line that
// straddles two USB packets costs a millisecond; a Jetson that dies mid-line
// costs a full second. During that second loop() does not run, so SBUS is not
// read, the motors are not updated, and NEITHER FAILSAFE IS EVALUATED -- while
// the thrusters hold their last commanded pulse. A one-second stall is exactly
// what a 500 ms failsafe exists to prevent, so the reader must never block.
//
// This drains whatever is available and returns immediately. A line longer
// than the buffer is truncated and dropped rather than silently split into two
// half-commands, because half of "0.8,-0.8" is a valid command for one motor.
void handle_serial_input() {
  static char    rx[RX_BUF_LEN];
  static uint8_t rx_len      = 0;
  static bool    rx_overflow = false;

  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c != '\n' && c != '\r') {
      if (rx_len < RX_BUF_LEN - 1) rx[rx_len++] = c;
      else                         rx_overflow = true;   // poisoned; drop at EOL
      continue;
    }
    if (rx_len == 0 && !rx_overflow) continue;           // bare newline

    rx[rx_len] = '\0';
    String line(rx);
    rx_len = 0;
    bool overflowed = rx_overflow;
    rx_overflow = false;

    if (overflowed) {
      Serial.println(F("Invalid cmd: line too long"));
      continue;
    }
    line.trim();
    if (line.length() == 0) continue;
    handle_serial_line(line);
  }
}

// One complete line from the Jetson.
void handle_serial_line(const String &line) {
  // Any valid line from the Pi counts as a heartbeat.
  last_heartbeat_ms = millis();

  // --- Mode verbs (effective only where update_state() allows) ---
  String up = line;
  up.toUpperCase();

  if (up == F("PING")) {
    return;  // heartbeat only -- last_heartbeat_ms is already stamped above
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
  Serial.print(F("[E-STOP] "));
  Serial.println(reason);
}
