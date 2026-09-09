# GUI integration — status

The mission GUI overlay merged into the Njord workspace on `gui-integration`.

**Read this before merging anywhere.** It says what was verified, what was
assumed, and what is still open.

**Update — the Pico firmware source has now been read.** That closed the biggest
provisional item (§5.1, the status line format), found a bug that meant
`/pico/status` published nothing at all (§2a), and produced a firmware change
adding the serial mode commands the GUI's buttons needed (§7). One question is
left for a human, in §7.

---

## 1. What was verified, and what was not

| | State |
|---|---|
| Merge, both histories intact | **verified** — 122 club + 15 GUI + merges = 141 commits |
| GUI Python test suite (no ROS) | **verified** — 608 passed, 1 skipped |
| Status-line parser, against the real format | **verified** — see §5.1 |
| Mode arbitration, all 24 RC × request cases | **verified** — in Python *and* in the firmware's own C++, see §7 |
| Frontend production build | **verified** — builds into `gui_backend/static/` |
| `colcon build --symlink-install` | **NOT VERIFIED** — see §6 |
| Gazebo end-to-end | **NOT VERIFIED** — see §6 |
| **Firmware on hardware** | **NOT VERIFIED** — not compiled for RP2350, never flashed. See §7 |

The build and the Gazebo run were attempted in a sandbox with no GPU and a
Python 3.11/3.12 mismatch against ROS Jazzy. They failed on a **competition**
package before reaching any GUI package, which means they told us nothing about
this merge. They belong on the Jetson, where the environment is already right.
The commands are in §6.

---

## 2. What conflicted in the merge, and how

`git merge gui/... --allow-unrelated-histories` produced exactly two conflicts.
Both are root files; no file under `src/` collided, because the package sets are
disjoint.

| File | Resolution |
|---|---|
| `README.md` | **Club's kept verbatim** — all 572 lines — with a 113-line GUI section appended after a horizontal rule. It is the team's reference document and nothing was removed from it. |
| `.gitignore` | **Union.** Club's entries first and untouched; the GUI's additions appended under a marked comment block so it is obvious which half is which. |

Three files arrived without conflict because the club repo had no equivalent:

| File | Note |
|---|---|
| `CLAUDE.md` | The club had none. Landed from the GUI side, then its header was rewritten to describe the **merged** workspace rather than a standalone overlay, and its "Boundaries" section now states the competition-package rules. |
| `.flake8` | The club had none, so the GUI's applies. It does **not** affect competition packages: those are linted by `colcon test` through `ament_flake8`, which uses its own configuration. |
| `.github/workflows/` | No collision — the club has `build.yaml`, the GUI had none. |

### One change made beyond straight conflict resolution

`pytest.ini` and `conftest.py` came from the GUI side with `testpaths = src` and
a `sys.path` insert for every package in `src/`. In the merged workspace that
would have swept the competition packages' ROS-dependent tests into a bare
`pytest`, so a missing `rclpy` would have looked like the GUI breaking the
club's tests. Both are now scoped to the GUI packages by name.

**Competition packages: `colcon test`. GUI packages: `pytest`.**

---

## 2a. `/pico/status` was publishing nothing at all

**The firmware writes `[STAT]`. `pico_bridge` filtered for lines starting with
`STATE`.** No line has ever matched, so `/pico/status` has published nothing for
as long as both have existed in their current form, and every Pico-derived field
in the GUI has been rendering "not sent".

```python
# control/control/pico_bridge.py, before
if line.startswith("STATE"):
    self._status_pub.publish(String(data=line))
```

```cpp
// pico-node_v3.ino
Serial.print(F("[STAT] Mode:"));   Serial.print(current_mode);
```

Fixed by accepting both prefixes. The bridge now also classifies every line it
reads instead of dropping what it does not recognise: status to `/pico/status`,
acknowledgements to `/pico/command_ack`, transitions and e-stops to
`/pico/events`, and anything it cannot place is logged once with the raw text.
Silence is what let this survive; a line nobody claims is now a warning.

This is the second time the two ends of this wire have disagreed about a format
without anything noticing, which is why §7's arbitration is checked by compiling
the firmware's own function — see there.

---

## 3. Competition packages touched

**Two files now, not one.** This changed, and it changed a promise made earlier
in this document, so it is spelled out rather than folded into a diffstat.

```
src/bringup/launch/njord.launch.py   | 27 +++++++++++++++++++++++++++
src/control/control/pico_bridge.py   | rewritten in place
```

### `bringup/launch/njord.launch.py` — unchanged from before

- `enable_gui` launch argument, **default `false`**, following the existing
  per-subsystem flag pattern.
- `gui_port` launch argument, default `8090`.
- An `IncludeLaunchDescription` of `gui_backend/launch/gui.launch.py`, guarded
  by that flag.
- One import (`FindPackageShare`).

### `control/control/pico_bridge.py` — a boundary deliberately crossed

`CLAUDE.md` says, in as many words: *do not modify `pico_bridge`; if a change
there looks necessary, raise it — do not make it.* That rule was written to keep
the GUI from costing the navigation team anything, and it is a good rule.

It was crossed here **on Auxence's explicit instruction**, after the firmware
source showed the node was not working: it filtered for `STATE` while the
firmware writes `[STAT]`, so `/pico/status` published nothing at all (§2a). A
node that publishes nothing is not a boundary worth protecting.

What changed, and nothing else:

- Accepts both `[STAT]` and `STATE` prefixes. **This is the fix for §2a.**
- Classifies every line it reads instead of dropping the unrecognised: status,
  `[ACK]`, events, boot banners, and a one-time warning for anything left over.
- Publishes `/pico/command_ack` and `/pico/events` alongside `/pico/status`.
- Accepts `ESTOP` and `RELEASE` on `/pico/mode_request` as well as
  `AUTO`/`MANUAL`, and holds a request by re-sending it at 1 Hz so the
  firmware's 5 s expiry does not drop it.

What did **not** change: the 20 Hz heartbeat, the skid-steer mix, the serial
write path, the neutral-on-exit behaviour, and every existing parameter name and
default. The two pre-existing `E221` lint warnings were left alone — they are the
author's alignment style, and competition packages are linted by `ament_flake8`
with its own configuration, not by this repository's `.flake8`.

**This needs a navigation-team review before it merges**, which is exactly what
the boundary rule exists to force.

Nothing else under `src/control`, `src/perception`, `src/sensors`,
`src/mission`, `src/vision`, `src/description`, `src/boat_bt`, `src/fusion`,
`src/competition_manager`, `src/njord_msgs` or `src/calibration` is modified.
Verify with:

```bash
git diff --stat club/main..gui-integration -- src/control src/perception src/sensors \
  src/mission src/vision src/description src/boat_bt src/fusion \
  src/competition_manager src/njord_msgs src/calibration src/bringup
```

---

## 4. Topic mapping — verified against source, not against a running system

Every mapping below was read out of the club README and the node source. None of
it has been observed on a live topic. "Verified" in the table means *the message
type and field access were checked against the real message definition and unit
tested*; it does **not** mean data has flowed.

Configured in `src/gui_backend/config/topics.yaml`. Nothing is hard-coded.

| GUI stream | Topic | Type | Adapter | State |
|---|---|---|---|---|
| Position | `/gps_driver/gps_raw` | `sensor_msgs/NavSatFix` | `vessel_from_odometry` | verified |
| Position (map frame) | `/odometry/gps` | `nav_msgs/Odometry` | — | wired, unused |
| Speed, heading, attitude | `/odometry/filtered` | `nav_msgs/Odometry` | `vessel_from_odometry` | verified |
| IMU | `/imu_driver/imu_raw` | `sensor_msgs/Imu` | `vessel_from_odometry` | verified, fallback only |
| Lidar raw | `/lidar_driver/scan_raw` | `sensor_msgs/LaserScan` | `lidar_from_ros` | verified |
| Lidar filtered | `/obstacles/lidar` | `sensor_msgs/PointCloud2` | `obstacles_from_pointcloud` | verified |
| Obstacles fused | `/obstacles/fused` | `sensor_msgs/PointCloud2` | `obstacles_from_pointcloud` | verified, optional |
| Commanded effort | `/control/effort` | `geometry_msgs/Twist` | — | wired, unused |
| Vessel status | `/pico/status` | `std_msgs/String` | `pico_from_ros` | **verified against firmware source — see §5.1**. Was publishing nothing at all; see §2a |
| Command acks | `/pico/command_ack` | `std_msgs/String` | `parse_ack_line` | new, see §7 |
| Firmware events | `/pico/events` | `std_msgs/String` | `parse_event_line` | new, see §5.1 |
| Mode request | `/pico/mode_request` | `std_msgs/String` | — | verified. Now reaches the firmware — it previously went nowhere, see §7 |

### Decisions worth knowing about

**Heading, speed and attitude come from one message.** `/odometry/filtered` is
the EKF output, so the three cannot drift apart the way four independent MAVROS
topics could. Heading source is reported to the GUI as `ekf`.

**ENU yaw is converted to a compass bearing** (`90 − yaw`). ROS is
counter-clockwise from east; a bearing is clockwise from north. Getting this
backwards would put the entire survey on the wrong side of the boat, plausibly,
with nothing on screen looking wrong — so it is pinned by a test at four
cardinal points.

**Course over ground is derived independently of heading**, from the same
message's twist rotated out of the body frame. Comparing two numbers that came
from the same one says nothing, and that comparison is the row the heading panel
exists for.

**The filtered obstacle set comes from perception, not from re-filtering the raw
scan here.** Filtering twice in two places eventually produces two different
answers about where the obstacles are, and the navigation stack's is the one
that matters.

**Satellite count is absent, not zero.** Neither `NavSatFix` nor `Odometry`
carries one and this stack has no MAVROS `GPSRAW`. "0 satellites" would read as
a GNSS failure and ground a healthy vessel, so the pre-flight reports SKIPPED
instead. Position accuracy is taken from `position_covariance` when it is
non-zero, and reported as a metre figure rather than converted into an HDOP by a
made-up UERE.

---

## 5. Still PROVISIONAL

Everything below is greppable:

```bash
grep -rn PROVISIONAL src/
```

### 5.1 The Pico status line format — **closed**

Was the most important provisional item. The firmware source has now been read,
so the format is transcribed rather than guessed:

```
[STAT] Mode:2 Armed:Y Relay:ON Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):172 Mode(Ch8):172
```

| Field | Values |
|---|---|
| `Mode` | `1` ESTOP, `2` MANUAL, `3` AUTONOMOUS |
| `Armed` | `Y` / `N` |
| `Relay` | `ON` / `OFF` |
| `Thr(Ch3)` `Yaw(Ch4)` `Arm(Ch7)` `Mode(Ch8)` | raw SBUS, 172–1811, centre 991 |

- Parser: `src/gui_backend/gui_backend/core/pico_state.py` — still the only file
  in the repository that knows the wire format.
- `FORMAT_VERIFIED = True`.
- **The trap:** `Mode` appears twice, as `Mode:` and as `Mode(Ch8):`. Splitting
  on `:` conflates a mode enum (1–3) with a raw SBUS count (172–1811), and the
  result looks entirely plausible. The parser matches whole keys, and a test
  drives a line where the two deliberately disagree so a parser that mixed them
  up cannot pass.
- The behavioural tests are unchanged and still there — never raise, never
  invent, absent stays absent — because knowing the format does not make a
  serial link stop truncating lines.

**`Mode(Ch8)` turned out to matter as much as `Mode`.** One is what the firmware
settled on, the other is what the operator's switch is asking for. They differ
whenever software is clamping the vessel, and an operator who cannot see both
has no way to tell a clamp from a fault. Both now reach the GUI.

**The pre-flight still checks, every run**, that the lines actually arriving
parse. A flag set once is a promise about the past; a firmware change that alters
the format would otherwise produce a panel full of nulls that reads as a quiet
vessel rather than a broken parser. `pico.state_format` FAILs on an unreadable
line and WARNs on an unrecognised field.

Event lines are captured too, into `/pico/events` and from there the mission
event log: `>>> MODE: E-STOP`, `>>> ARMED: MANUAL|AUTONOMOUS`,
`>>> DISARMED (manual|auto)`, `[E-STOP] <reason>`, `Invalid cmd: <line>`, and
the `[ACK]` lines added in §7.

### 5.2 The rest

| What | Where | Note |
|---|---|---|
| Sonar mounting angle and lever arm (Q2) | `src/omniscan_bridge/config/mounting.yaml` | `measured: false`. Pre-flight WARNs every run until measured. |
| Survey area and sonar range (Q1) | `src/asket_bringup/config/mission_defaults.yaml` | Tuned on site; range is adjustable from the GUI at runtime. |
| Battery capacity and hotel load (Q5) | `src/gui_backend/config/topics.yaml` | 1200 Wh / 85 W assumed. |
| EKF heading accuracy | `src/asket_common/asket_common/heading.py` | 3.0° nominal. The EKF publishes a pose covariance — somebody should confirm `robot_localization` is filling it in rather than leaving the default, then use it. Until then the panel marks the figure "assumed". |
| `OS3D_POINT_SET` layout (Q8) | `src/omniscan_bridge/core/ping_protocol.py` | From Cerulean's published docs, never run against a real device. |
| ESC status codes (Q7) | `src/asket_gui/src/lib/hull.js` | Codes beyond `0` stay uninterpreted. The firmware reports no ESC code over serial at all — `esc_status` arrives only from `asket_sim`. |
| E-stop power feedback | firmware `ESTOP_FEEDBACK_ENABLED` | **0.** Nothing verifies the ESC rail actually collapsed when the relay was commanded open. Pre-flight `pico.estop_feedback` WARNs every run until it is 1. |

**Closed by reading the firmware:** the relay count. The GUI showed
`0/4 relays closed`; there is one relay, `ESTOP_RELAY_PIN` on GPIO21, cutting ESC
power. The `4` came from `num_relays` in `asket_sim`'s placeholder config and the
GUI faithfully rendered whatever length of array arrived. Both ends corrected,
and the relay is now named because its function is confirmed in firmware source
rather than guessed.

---

## 6. What to run on the Jetson

Where I stopped. The environment there is already configured; nothing below
needs the workarounds that failed in the sandbox.

### 6.1 Build

```bash
cd ~/node-ros-2026            # or wherever the workspace lives
git fetch origin
git checkout gui-integration

# The frontend must be built before the backend can serve it.
cd src/asket_gui && npm install && npm run build && cd ../..

source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y -r
colcon build --symlink-install
```

Then confirm no competition package regressed:

```bash
colcon test --packages-select control perception sensors mission fusion \
  competition_manager boat_bt njord_msgs bringup
colcon test-result --verbose
```

### 6.2 GUI tests without ROS

These run anywhere, including a laptop:

```bash
pytest                                            # GUI packages only
python3 -m flake8 --max-line-length=100 src/gui_backend src/system_test
```

### 6.3 Gazebo

```bash
source install/setup.bash
ros2 launch bringup njord.launch.py use_sim:=true enable_vision:=false enable_gui:=true
```

Then open **http://\<jetson\>:8090** from the laptop.

What to check, in order:

1. **Position** — the vessel appears on the map near the Trondheim datum, and
   the Vessel panel's position updates.
2. **Speed and heading** — the Heading panel shows a bearing with source
   `EKF (fused)`. Drive the boat and confirm heading and course over ground
   differ when it crabs, and agree in a straight line.
3. **Lidar** — the Obstacles panel shows returns, and `Returns N of 360 beams`
   rather than `0 of 0`. Toggle Filtered/Raw and confirm the two differ.
4. **Obstacles fused** — absent with `enable_vision:=false`; that is correct,
   and the panel should say "not sent" rather than showing zero.
5. **Pre-flight** — press Run pre-flight. Expect **two amber warnings**:
   `Sonar mounting geometry` (§5.2), correct until somebody measures the boat,
   and `E-stop power feedback` (§5.2), correct until `ESTOP_FEEDBACK_ENABLED`
   is 1. `Pico status format` should now **pass** — it warned on every run while
   the format was a guess, and it is no longer a guess.
6. **The Pico stream stays absent in sim** — expected. There is no Pico in
   Gazebo, so the vessel panel's mode and armed state read "not sent".

If the map is blank, that is the offline-tiles path, not a fault: see
`docs/SETUP.md` §5.

### 6.4 Port check

The GUI is on **8090**, chosen clear of `foxglove_bridge` (8765) and of the
commented-out `rosbridge_websocket` (9090) and `web_video_server` (8080) in
`njord.launch.py`. Confirm nothing else took it:

```bash
ss -lntp | grep -E '8090|8765|9090|8080'
curl -s http://<jetson>:8090/api/tiles/info     # from the laptop, not the Jetson
```

The backend binds `0.0.0.0`. If that curl times out and the Jetson is otherwise
reachable, it is a firewall, not the GUI.

---

## 7. "Cut propulsion" — now real, with one question left

### What changed

The old §7 asked what the button should do, because `pico_bridge` accepted only
`AUTO` and `MANUAL` and there was no software ESTOP anywhere in the stack. The
firmware source settled it: **`handle_serial_input()` only ever parsed motor
setpoints.** There was no serial command path at all, so `/pico/mode_request`
had never done anything, in any mode.

So the firmware was changed rather than the button relabelled. Option A from the
old §7, in substance.

```
CMD MODE ESTOP | MANUAL | AUTONOMOUS     a software mode request
CMD ESTOP                                relay open + latch, accepted from any state
[ACK] <subject> accepted                 one acknowledgement per command
[ACK] <subject> rejected <reason>        ...or the reason it was refused
```

### The governing rule

**The RC transmitter is sovereign.** The effective mode is the *more restrictive*
of (channel 8, active software request), ordering `ESTOP < MANUAL < AUTONOMOUS`.
Software can take authority away and never add it. `CMD ESTOP` is the one
request accepted from any state, in any build, because it can only ever restrict.

A held request **expires after 5 s** without a refresh, so a GUI that dies
holding an ESTOP cannot lock the vessel out until somebody power-cycles the Pico.
`pico_bridge` refreshes at 1 Hz while a request is held and stops on `RELEASE`.
The **latch** is deliberately not on that clock: `trigger_estop()` still clears
only when the operator cycles the arm switch or selects ESTOP on channel 8.

The full table is written into the sketch as a comment above `arbitrate_mode()`.

### Three structural changes that were not optional

1. **Serial is now read every loop, in every mode.** It used to be read only
   inside `if (current_mode == MODE_AUTONOMOUS && armed)`. "Request ESTOP while
   in MANUAL" was therefore *unimplementable* without moving it — nothing sent
   in any other state was ever read. Motor setpoints still apply only in
   `AUTONOMOUS && armed && past the arm window`, and are dropped silently
   otherwise so a setpoint sent just before a mode change cannot land after it.
2. **`Serial.readStringUntil()` is gone.** It blocks up to its timeout, which was
   survivable while that function ran rarely and is not now it runs every loop.
   Replaced by a non-blocking accumulator.
3. **Output is queued.** `Serial.print()` blocks once the USB CDC buffer fills,
   which would stall `loop()` and with it the failsafes. `emit_line()` queues,
   `flush_serial_out()` drains only as far as `availableForWrite()` allows.
   (There was no `availableForWrite()` discipline in v3 to inherit — that call
   appeared nowhere in the repository.)

A fourth, found on the way: **`PING` was printing `Invalid cmd: PING` at 20 Hz**
whenever the vessel was in AUTONOMOUS and armed. The bridge's heartbeat fell
through to the motor parser, which cannot read it. It is now recognised and
ignored — deliberately *without* refreshing `last_serial_command_ms`, since PING
means "no fresh setpoint" and the neutral failsafe should still fire.

### How it was tested without a boat

`arbitrate_mode()` is pure — no globals, no clock, no I/O. So
`src/asket_common/test/test_firmware_arbitration_matches.py` lifts the C++ out of
the `.ino`, compiles it with `g++ -Wall -Wextra -Werror`, and drives **all 24**
RC × request × configuration combinations against the Python mirror in
`asket_common.mode_arbitration`. Both must agree cell for cell. The same file
checks that the mirrored constants — the upward flag, the request timeout, the
SBUS thresholds, the arm delay, the e-stop feedback flag — still match the
sketch, and that the firmware still prints every key the parser expects.

That is the drift guard, and it exists because §2a was the *second* time the two
ends of this wire disagreed about a format with nothing noticing.

**It is not bench validation.** Nothing has been compiled for the RP2350 or
flashed. See the bench list below.

### The question left for you

**Should the GUI be able to request AUTONOMOUS at all, or only downward?**

Implemented as a single constant, defaulting to the stricter answer:

```c
#define SOFTWARE_UPWARD_REQUESTS_ALLOWED 0      // firmware
```
```python
SOFTWARE_UPWARD_REQUESTS_ALLOWED = False        # asket_common.mode_arbitration
```

Both must change together; a test fails if they diverge.

At `0`, the GUI may request MANUAL and ESTOP only. The consequence worth knowing
about: **the way back up is to release the clamp**, not to request AUTONOMOUS.
The Autonomous button is kept, and on a downward-only build it sends `RELEASE` —
the software request lapses and channel 8 decides. The panel says which of the
two it is doing rather than leaving a button whose meaning quietly changed.

Set it to `1` if missions should be startable from the GUI. Requests are still
clamped by channel 8 either way.

### Before this goes near the water

The firmware change is staged at `firmware/pico-node_v3/` with a patch that
applies cleanly to `NODE-Engineering-Club/pico-node` at `6efc583`. **Read
`firmware/README.md` first** — that directory is a staging copy and a second copy
of firmware source is the exact condition that caused §2a.

On the bench, boat out of the water:

1. Flash, and confirm `[STAT]` still arrives at 4 Hz and `/pico/status` now
   publishes. That alone is §2a fixed.
2. With Ch8 in MANUAL, send `CMD MODE AUTONOMOUS` — expect
   `[ACK] MODE AUTONOMOUS rejected upward_disabled`.
3. With Ch8 in AUTONOMOUS and armed, send `CMD MODE MANUAL` — expect acceptance,
   `Mode:2` in the status line while `Mode(Ch8)` still reads high, and the GUI
   showing a software clamp rather than a fault.
4. Stop sending for 6 s — expect `[ACK] REQUEST expired` and a return to
   AUTONOMOUS.
5. `CMD ESTOP` from each of the three switch positions — expect the relay to
   open every time, and re-arming to stay blocked until the arm switch is cycled.
6. Confirm `loop()` still keeps up: no missed `[STAT]` lines, and the SBUS
   failsafe still fires within 500 ms with the transmitter switched off.

Item 6 is the one to take seriously. Serial is read every loop now, and the
failsafes share that loop.

## 8. Branch conflict warning

Checked against the club repo at time of writing:

**`GPS_Fix_and_Task` is 37 commits ahead of `main` and touches both files this
integration touches.**

| Branch | Ahead of `main` | Touches |
|---|---|---|
| `GPS_Fix_and_Task` | 37 | `pico_bridge.py`, `njord.launch.py` (+363 lines), `ekf.yaml`, `navsat.yaml`, `nav2_params.yaml` |
| `gazebo_debugg` | 5 | `pico_bridge.py`, `njord.launch.py` |
| `feat/report` | 2 | `njord.launch.py` |
| `piddebugg`, `Nav2-testing` | 1 | `pico_bridge.py`, `njord.launch.py` |

If `GPS_Fix_and_Task` merges first, expect a conflict in `njord.launch.py`
(mechanical — my addition is 27 self-contained lines) and **check its
`pico_bridge` changes against §5.1**, since anything altering the STATE contract
changes what the parser has to read.

---

## 9. Also flagged

`scripts/deploy-pi.sh` targets `pi@boat.local` and looks stale — this project
moved to a Jetson. Left untouched: it needs somebody who knows the current
deployment to decide what it should be.
