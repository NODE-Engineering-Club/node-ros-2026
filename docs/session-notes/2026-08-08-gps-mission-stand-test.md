# 2026-08-08/09 — GPS/mission stand-test session

Stand test (boat on stand, props clear) after repowering the Pixhawk, on
`GPS_Fix_and_Task`. Real findings from this session, in case they need to be
re-derived at the competition site:

## Fixed and committed

1. **mavros crash-loop root cause** (previously misdiagnosed as a startup
   race in `mavros_plugin_collision_bug` notes). `njord.launch.py` set
   `name="mavros"` on the mavros `Node()`, which makes `launch_ros` inject a
   global `-r __node:=mavros` remap. mavros's plugin sub-nodes are supposed
   to be immune to that (`use_global_arguments(false)` in `plugin.cpp`), but
   in the mavros build bundled for Jazzy that protection doesn't hold — every
   plugin's topics collapse onto `/mavros/mavros/<leaf>` instead of
   `/mavros/<plugin>/<leaf>`, and `rc_io`'s `in`/`out` leaf names collide
   fatally with a differently-typed entity, SIGABRT. Fix: drop `name=`
   entirely — the executable's compiled-in default name is already `mavros`.
   100% reproducible on this specific build, not timing-sensitive; confirmed
   fixed against the real Pixhawk (`/dev/ttyACM1`), zero crashes across
   multiple clean-container runs.

2. **GPS/battery silent without an explicit stream request.** This
   FCU/link doesn't auto-stream `GLOBAL_POSITION_INT`/`BATTERY_STATUS` on
   connect (raw sensors — IMU, `GPS_RAW_INT` — come through fine without
   it). Added a `TimerAction` that calls `/mavros/set_stream_rate`
   (`STREAM_ALL`) ~10s after mavros starts.

3. **Nav2 rotate-to-heading was wrong for this boat.** `FRAME_TYPE=0`
   confirmed by pulling FCU params directly via `/mavros/param` — normal
   rudder+throttle steering, not skid-steer. With rotate-to-heading on, a
   reroute goal made the controller try to rotate in place — physically
   meaningless for a rudder — so it commanded steering with ~zero throttle
   and the boat never moved. Set `use_rotate_to_heading: false` and
   `allow_reversing: false` in `nav2_params.yaml`'s `FollowPath` block.

4. **`pid_controller` had no watchdog on `/control/setpoint`.** Confirmed
   live: after `/mission/abort`, steering returned to neutral but throttle
   stayed at a real non-zero value indefinitely — `nav_to_pid` and
   `twist_mux` are purely event-driven and never publish an explicit zero
   when their own input goes away, so the PID loop just kept outputting
   whatever the last real setpoint produced, forever. Added a 0.5s
   staleness check that resets both PID setpoints to zero and calls
   `.reset()` on them (setpoint alone doesn't clear accumulated integral).
   Verified non-actuating (fake `/control/setpoint` → effort ramps up, stops
   publishing → effort drops to exactly `0.0` within the timeout) and live
   through the real Pico path (abort → full stop in ~1.6s, within the 2s
   bar from `TODOS.md`'s stand-test checklist).

5. **`firmware/pico/asket_ec_pico.ino` added to the repo** — was living
   only on the Pico/off-repo before. Mode authority: Ch8 (3-pos) is
   ESTOP/MANUAL/AUTONOMY-PERMITTED, Ch7 is the arm switch; the Pi/Foxglove
   can only request AUTO when Ch8 is HIGH, and there's no stick-override.
   `pico_bridge.py` (`control` package) talks to it — `/control/effort`
   in, `L,R` normalized motor values out, continuous heartbeat so the Pico
   fails safe to neutral if the link drops.

## Process-hygiene lesson (cost significant time this session)

`pkill -f 'ros2 launch'` (or any signal to the `ros2 launch` parent) does
**not** reliably cascade to its children — orphans can keep running
indefinitely (`--init` reparents them to PID 1, so they don't even show up
as zombies). Mid-session cleanup that only targets specific node names by
`pkill -f <name>` will miss the rest of that launch tree. This produced a
real incident: two complete duplicate sets of the control chain
(`mission_manager`, `twist_mux`, `actuator_driver`, ...) ran simultaneously
for ~20 minutes, racing to write `/mavros/rc/override`, which looked like a
"stuck throttle" bug but was actually a stale orphan process. No harm
(boat was on a stand), but **verify a clean single-instance process tree
(`ps aux`, grep each node name, expect exactly 1 each) before any actuation
test, every time** — don't trust that a previous cleanup fully worked. A
full container restart is cheap and removes the ambiguity entirely; prefer
it over trying to surgically kill a partially-torn-down tree.

## Not yet resolved

- Foxglove: GPS point / LiDAR point cloud heading appears to drift/move
  even when the boat is stationary — likely `ekf.yaml` fusing yaw from
  gyro-only (no magnetometer per the README), which drifts. Needs a proper
  look, flagged but not fixed this session.
- `Containerfile`'s `python3-scikit-learn` → `python3-sklearn` edit
  (uncommitted, pre-existing before this session) is probably the wrong
  package name for this base image — the image currently has neither
  package baked in, so `dock_detector_node`/perception need
  `pip install scikit-learn` done by hand in any fresh container until this
  is fixed and the image is rebuilt.
- `mission_maneuvering_pathfinding`'s `maneuvering.yaml` still has a
  **placeholder Barcelona waypoint** (41.39, 2.15406) — do not run
  `enable_maneuvering_pathfinding_mission` for real until this is replaced
  with the actual course.
