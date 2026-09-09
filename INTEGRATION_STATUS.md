# GUI integration — status

The mission GUI overlay merged into the Njord workspace on `gui-integration`.

**Read this before merging anywhere.** It says what was verified, what was
assumed, and what is still open. Two things in here need a human decision and
one needs a capture from the Jetson.

---

## 1. What was verified, and what was not

| | State |
|---|---|
| Merge, both histories intact | **verified** — 127 club + 15 GUI + 1 merge = 143 commits |
| GUI Python test suite (no ROS) | **verified** — 424 passed, 1 skipped |
| Adapter and STATE-parser unit tests | **verified** — 33 tests, written for this integration |
| Frontend production build | **verified** — builds into `gui_backend/static/` |
| `colcon build --symlink-install` | **NOT VERIFIED** — see §6 |
| Gazebo end-to-end | **NOT VERIFIED** — see §6 |

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

## 3. Competition packages touched

**One file, 27 additive lines, nothing removed.**

```
src/bringup/launch/njord.launch.py | 27 +++++++++++++++++++++++++++
```

- `enable_gui` launch argument, **default `false`**, following the existing
  per-subsystem flag pattern.
- `gui_port` launch argument, default `8090`.
- An `IncludeLaunchDescription` of `gui_backend/launch/gui.launch.py`, guarded
  by that flag.
- One import (`FindPackageShare`).

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
| Vessel status | `/pico/status` | `std_msgs/String` | `pico_from_ros` | **ASSUMED — see §5** |
| Mode request | `/pico/mode_request` | `std_msgs/String` | — | verified against `pico_bridge` |

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

### 5.1 The Pico `STATE` line format — the important one

`/pico/status` is a `std_msgs/String`. `pico_bridge` reads lines off the serial
link and republishes anything starting with `STATE` **verbatim, unparsed**. So
the GUI parses text, and **the format it parses has never been checked against
real firmware output.**

- Parser: `src/gui_backend/gui_backend/core/pico_state.py` — the only file in
  the repository that knows the wire format.
- `FORMAT_VERIFIED = False`.
- Pre-flight check `pico.state_format` returns **WARN on every run** until that
  flag is set, and **FAIL** if a line arrives that the parser cannot read.
- The tests deliberately assert *behaviour*, not spelling: never raise on
  anything a serial link can produce, never invent a value, absent stays absent,
  and an unreadable line is reported as unreadable rather than quietly becoming
  a mode. Asserting a guessed format would make it look verified.

**Do not read a green Pico panel as evidence the format is right.**

To close it, on the Jetson with the Pico connected:

```bash
ros2 topic echo /pico/status --field data
```

Paste a few real lines into `src/gui_backend/test/test_pico_state.py` as a new
test, adjust `_FIELD_ALIASES` or `parse_state_line` to match, and set
`FORMAT_VERIFIED = True`.

### 5.2 The rest

| What | Where | Note |
|---|---|---|
| Sonar mounting angle and lever arm (Q2) | `src/omniscan_bridge/config/mounting.yaml` | `measured: false`. Pre-flight WARNs every run until measured. |
| Survey area and sonar range (Q1) | `src/asket_bringup/config/mission_defaults.yaml` | Tuned on site; range is adjustable from the GUI at runtime. |
| Battery capacity and hotel load (Q5) | `src/gui_backend/config/topics.yaml` | 1200 Wh / 85 W assumed. |
| EKF heading accuracy | `src/asket_common/asket_common/heading.py` | 3.0° nominal. The EKF publishes a pose covariance — somebody should confirm `robot_localization` is filling it in rather than leaving the default, then use it. Until then the panel marks the figure "assumed". |
| `OS3D_POINT_SET` layout (Q8) | `src/omniscan_bridge/core/ping_protocol.py` | From Cerulean's published docs, never run against a real device. |
| Relay and ESC meanings (Q7) | `src/asket_gui/src/lib/hull.js` | `RELAY_LABELS` is empty on purpose — no relay is given a name nobody has confirmed. |

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
   `Sonar mounting geometry` (§5.2) and `Pico status format` (§5.1). Both are
   correct until somebody measures the boat and captures a STATE line.
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

## 7. Open question: what should "Cut propulsion" do?

**This needs a decision. It is not mine to make.**

### The problem

`pico_bridge` accepts exactly two mode words — `AUTO` and `MANUAL` — and logs a
warning for anything else. There is no software ESTOP anywhere in the stack, and
no `MODE ESTOP` in the firmware bridge. The GUI's "Cut propulsion" button
therefore has nothing to send.

### What it does in the interim

It requests **`MANUAL`**.

That is the safest thing available, for a specific reason. From `pico_bridge`'s
own docstring:

> *Moteurs appliqués côté Pico SEULEMENT si armé + AUTONOMOUS + 2s après relais.*

The Pico drives the thrusters only when armed **and** in AUTONOMOUS. Requesting
MANUAL therefore removes software authority over the thrusters entirely and
hands the boat back to the RC pilot.

**It does not stop the boat.** The hardware killswitch and RC channel 8 remain
the only things that cut propulsion, exactly as the safety rules require.

### How that is made visible

The button is **not** labelled "Cut propulsion" against this stack. The backend
declares what a propulsion cut can actually do in the `hello` message, and the
frontend labels the button from that rather than assuming a capability the
vessel may not have:

```yaml
# src/gui_backend/config/topics.yaml
estop:
  mode: mode_request_manual
  label: "Drop to MANUAL"
  effect: >-
    Hands control back to the RC pilot. The Pico only drives the thrusters when
    armed and in AUTONOMOUS, so this removes software authority — it does not
    stop the boat. The hardware killswitch and RC channel 8 are the only things
    that cut propulsion.
```

That `effect` sentence is rendered under the button. A button reading "Cut
propulsion" that quietly dropped the mode instead would be the most dangerous
thing on the screen.

In `asket_sim` the button keeps its real meaning, because the simulated Pico
does model a soft latch. The two are declared separately on purpose.

### The two options

**A — Add a `MODE ESTOP` path to firmware and `pico_bridge`.** The button means
what it says. Costs a firmware change plus a `pico_bridge` change, and
`pico_bridge` is a competition package this branch does not touch. Someone has
to own the firmware side and decide what ESTOP does at the Pico: neutral PWM,
relays open, or a latch that needs a physical reset.

**B — Keep the mode drop, permanently.** No firmware change. The GUI keeps
saying what it does. The soft ESTOP concept disappears from the GUI's
vocabulary, which is arguably more honest: the hardware killswitch is the ESTOP,
and having a second thing called ESTOP that is weaker invites confusion at
exactly the wrong moment.

I have deliberately not chosen. Whichever you pick, `estop.mode`, `estop.label`
and `estop.effect` in `topics.yaml` are the only things that change on the GUI
side.

---

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
