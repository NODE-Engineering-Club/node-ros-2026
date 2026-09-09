# Firmware — a staging copy, not the source of truth

**The Pico firmware lives in [`NODE-Engineering-Club/pico-node`](https://github.com/NODE-Engineering-Club/pico-node).**
What is in this directory is a copy of `pico-node_v3/pico-node_v3/pico-node_v3.ino`
with one change applied, staged here because the session that wrote it could
read that repository but had no credential to push to it.

## Read this before trusting anything here

A second copy of firmware source is exactly the condition that produced the bug
this change exists to fix: the firmware printed `[STAT]`, `pico_bridge.py` looked
for `STATE`, and `/pico/status` published nothing at all for as long as nobody
checked. Two copies of a file drift the same way two ends of a wire do.

So:

- **Do not flash from this directory.** Apply the patch to `pico-node`, flash
  from there, and delete this directory once that is done.
- **Do not edit the `.ino` here** in preference to `pico-node`. If this copy and
  `pico-node` ever disagree, `pico-node` is right by definition.
- The copy is pinned to `pico-node` at commit `6efc583`
  ("pico node v3 fixed most issues"), which is what `main` pointed at when the
  patch was written.

## Applying it

```bash
git clone https://github.com/NODE-Engineering-Club/pico-node
cd pico-node
git checkout -b feat/serial-mode-commands
git apply /path/to/node-ros-2026/firmware/pico-node_v3/0001-serial-mode-commands.patch
```

The patch was checked with `git apply --check` against a clean checkout of
`6efc583` and applies without fuzz.

## What the change does

Adds a serial command path so `/pico/mode_request` has something to talk to, and
so the GUI's "Cut propulsion" button does something real. Full rationale is in
`INTEGRATION_STATUS.md` §7.

| | |
|---|---|
| `CMD MODE ESTOP\|MANUAL\|AUTONOMOUS` | a software mode request, clamped by RC |
| `CMD ESTOP` | cuts the relay and latches, accepted from any state |
| `[ACK] ... accepted` / `[ACK] ... rejected <reason>` | one acknowledgement per command |

The governing rule, and the table it produces, are written into the sketch as a
comment above `arbitrate_mode()`. In short: **the RC transmitter is sovereign,
and the effective mode is the more restrictive of (Ch8, software request).**
Software can take authority away and never add it.

Three structural changes came with it, none of them optional:

1. **Serial is now read on every loop, in every mode.** It used to be read only
   in `AUTONOMOUS && armed`, which meant a mode command sent in MANUAL was never
   read at all — so "request ESTOP while in MANUAL" was not implementable
   without this. Motor setpoints are still applied *only* in `AUTONOMOUS &&
   armed && past the arm window`, and are discarded silently in every other
   state so a setpoint sent just before a mode change cannot be applied after it.
2. **`Serial.readStringUntil()` is gone**, replaced by a non-blocking character
   accumulator. It blocks up to its timeout, which was survivable while the
   function ran rarely and is not now that it runs every loop.
3. **Output is queued**, not printed directly. `Serial.print()` blocks once the
   USB CDC buffer fills, which would stall `loop()` and with it the failsafes.
   `emit_line()` queues and `flush_serial_out()` drains only as far as
   `availableForWrite()` allows.

What deliberately did **not** change: the `[STAT]` line's field names and order,
the e-stop latch semantics, the 500 ms failsafes, the 2000 ms ESC arm window, and
the rule that the latch is cleared only by a physical act (the operator cycling
the arm switch or selecting ESTOP). The latch clear is still keyed on the RC
switch and never on the effective mode — a software ESTOP request must not be
able to clear the latch it just set.

## How it was checked, and how it was not

**Not compiled for the RP2350, and not run on hardware.** There is no Arduino
toolchain here and the sketch needs `Servo.h` and `Adafruit_TinyUSB.h`.

What *was* checked:

- `arbitrate_mode()` is pure — no globals, no clock, no I/O — so
  `src/asket_common/test/test_firmware_arbitration_matches.py` lifts it out of
  this `.ino`, compiles it with `g++ -Wall -Wextra -Werror`, and drives all 24
  RC × request × configuration combinations against the Python mirror in
  `asket_common.mode_arbitration`. Both sides must agree cell for cell, and the
  test also checks that the two `SOFTWARE_UPWARD_REQUESTS_ALLOWED` defaults and
  the two request timeouts match. That is the drift guard.
- Braces and prototypes balance; every prototype has a definition.

That is the whole of it. **Nothing here has been validated against SBUS, a
relay, an ESC, or the USB CDC buffer.** The bench list is in
`INTEGRATION_STATUS.md` §7.
