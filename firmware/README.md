# Pico firmware

`pico-node_v4/pico-node_v4.ino` is the low-level controller for the Asket EC:
mode arbitration, SBUS decoding, thruster PWM, failsafes, relay and light tower.

It is the **one** place where the boat can be stopped without a working
computer, so it is deliberately small enough to read in one sitting.

```
make -C firmware/test      # host-side tests, no Pico and no Arduino toolchain
```

## Where v4 came from

v4 is a merge of two earlier firmwares that had diverged, and the divergence is
worth understanding because it cost months of a silently dead telemetry uplink.

| | `pico-node_v3` | `asket_ec_pico` |
|---|---|---|
| Header | `[REBUILT]` | `[REBUILT + FOXGLOVE]` |
| Lives in | `NODE-Engineering-Club/pico-node` | this repo, branch `GPS_Fix_and_Task`, commit `920773f` |
| Status line | `[STAT] Mode:3 Armed:Y …` | `STATE mode=3 armed=1 …` |
| Was it flashed? | **Yes** | No |
| Did the Jetson expect it? | No | **Yes** |

The Jetson was written against one, the Pico was flashed with the other, and
neither could say which it was. `pico_bridge` filtered for lines beginning
`STATE`; the Pico emitted `[STAT]`; every status line was dropped silently,
with no counter and no log, and the downlink kept working so nothing looked
broken. See `docs/comprendre-le-depot.html` §7.3 for the full story.

**v4 takes `STATE key=value` as the format, full stop.** The parsers are
single-format on purpose: two accepted formats means one of them is never
exercised, which is how this happened in the first place.

### Taken from FOXGLOVE

* The `STATE key=value` line and its full field set — `wantauto`, `link`,
  `estoplatch` and the four `sbus*` counters. The counters are the only
  measurement of radio *quality* anywhere in the system.
* The three-zone `update_state()`, with `if (sbus_failsafe_flag) new_mode =
  MODE_ESTOP;` written last so no branch can bypass it.
* The serial heartbeat (`last_heartbeat_ms`, `HEARTBEAT_TIMEOUT_MS`,
  `serial_link_live()`) gating the grant of autonomy.
* `MODE AUTO` / `MODE MANUAL` and their `[ACK]` replies.
* `handle_serial_input()` called unconditionally from `loop()`.
* The SBUS end-byte fix — see below.

### Taken from v3

Its **comments**, which FOXGLOVE had stripped, and nothing else.

This contradicts the brief, which expected v4 to take the driving from v3.
`update_motors()`, `updateBeeper()` and `startBeeps()` are byte-identical
between the two files once comments are removed: pivot-at-hard-over,
the non-blocking beeper and proportional mix normalisation are **already in
FOXGLOVE**, unchanged. There was nothing to port.

### Added in v4

* **Version announcement.** `[VER] pico-node 4` at startup and `ver=4` as the
  first field of every `STATE` line. `pico_bridge` reads it, the GUI shows it
  beside the mode, and pre-flight **fails** on a mismatch. This is the fix that
  stops the whole class of problem rather than one instance of it.

  `ver=` is first so that a parser which does not recognise the version can
  reject the line before misreading a single field. **Bump `FW_VERSION`
  whenever the wire format changes in a way a parser would notice.**

* **A non-blocking serial reader.** Both earlier firmwares used
  `Serial.readStringUntil('\n')`, which blocks until the terminator arrives or
  the 1000 ms stream timeout expires. A line split across two USB packets costs
  a millisecond; a Jetson that dies mid-line costs a full second — during which
  `loop()` does not run, so SBUS is not read, the motors are not updated and
  **neither failsafe is evaluated**, while the thrusters hold their last
  commanded pulse. A one-second stall is precisely what a 500 ms failsafe
  exists to prevent. v4 accumulates bytes and returns immediately.

  An over-length line is dropped whole rather than split, because half of
  `0.8,-0.8` is a valid command for one motor.

### Deliberately dropped

* **`BENCH_NO_RC_OVERRIDE`**, a FOXGLOVE compile flag that at `1` replaced the
  entire Ch8 zone logic with `serial_wants_auto ? AUTONOMOUS : MANUAL` and
  forced `arm_high = true` — removing the hardware E-stop's authority outright.
  Default was `0`, and its banner warned against it, but nothing distinguishes
  the two binaries once they are flashed. A build flag that silently deletes
  the safety chain is one wrong upload away from a boat that cannot be stopped
  from the beach.

* **A serial ESTOP verb.** Not in this round, by instruction. `MODE_ESTOP`
  exists and works, but only the transmitter can reach it. Adding a serial path
  into the safety chain is its own bench step, with the boat out of the water.

## The SBUS end-byte fix

v3 validated frames with:

```cpp
if (sbus_frame[0] != SBUS_SYNC_BYTE || sbus_frame[24] != 0x00) return;
```

Byte 24 is the SBUS **end byte**. It is `0x00` on Futaba receivers, but FrSky
and several others emit `0x04`, `0x14`, `0x24` or `0x34` depending on mode.
Against such a receiver that line rejects **100% of frames** — silently, with
the channels frozen at their last value and no counter to say so.

Demonstrated by compiling v3 against the test stub and feeding it frames that
differ only in the end byte:

```
v3, end byte 0x00 -> ch8 decoded as 1811  (frame ACCEPTED)
v3, end byte 0x04 -> ch8 decoded as 0     (frame REJECTED)
v3, end byte 0x14 -> ch8 decoded as 0     (frame REJECTED)
```

FOXGLOVE removed that test, kept the sync-byte check, added `sbus_frames_ok` /
`sbus_frames_bad`, and reads the frame-lost and failsafe bits from byte **23**,
which is the flags byte. v4 keeps all of that.

One correction to FOXGLOVE's own comment, which is wrong and is fixed in v4:
it claims byte 24 carries the CH17/CH18 and failsafe bits. Those are in byte
23, which is what its code actually reads. The fix is right; the explanation
was not.

## What the tests do and do not cover

`firmware/test` compiles the sketch against a stub Arduino core and drives it
with synthetic SBUS frames and serial input. It checks the mode arbitration in
all three Ch8 zones, that `MODE AUTO` is honoured only where permitted, the
ESTOP path down to the relay pin and the light, the serial reader's behaviour
on partial and over-long lines, the light convention, and the shape of the
`STATE` line.

It says **nothing** about timing, electrical behaviour, the real USB stack, or
whether the pin numbers match the loom. Those need the bench.

## Known, unfixed

`startup_light_sequence()` is a 900 ms busy-wait `while (true)` loop. It is
`millis()`-based rather than `delay()`-based, so the "non-blocking throughout"
claim in the header is not quite true — but it runs once in `setup()`, before
anything can be armed, so it is harmless. Left alone rather than churned.
