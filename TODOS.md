# Njord 2026 — Outstanding Gaps

## HIGH PRIORITY — Blocking Water Test (Wednesday)

- [x] **Disable RPP rotate-to-heading** — done. `FRAME_TYPE` confirmed `0`
  (`FRAME_CLASS=2`, Boat) by pulling FCU params directly via
  `/mavros/param` against the real Pixhawk (2026-08-08/09 stand test) — this
  boat is normal rudder+throttle steering, not skid-steer. Set
  `use_rotate_to_heading: false` and `allow_reversing: false` in
  `bringup/config/nav2_params.yaml`'s `FollowPath` block per the decision
  above. Root cause of a real symptom hit live: with rotate-to-heading on,
  a bypass/reroute goal made the controller try to rotate the boat in
  place — physically meaningless for a rudder — so it commanded steering
  deflection with ~zero throttle and the boat never visibly moved.

- [ ] **Stand-test dry-run before water**
  1. Boat on a stand, FCU + RPi + thrusters connected.
  2. Launch the stack. Confirm: `/mavros/state.connected=true`,
     `/imu_driver/imu_raw` + `/gps_driver/gps_raw` publishing,
     `/odometry/filtered` position updates when boat is physically carried
     a few meters outside.
  3. Call `/mission/start` with a waypoint ~10 m away. Confirm thrusters
     spin in a direction that would drive toward the goal.
  4. Call `/mission/abort`. Confirm thrusters stop within 2 s.

## Navigation (Docking)

Dock **detection** now exists: `perception/dock_detector_node` clusters
`/obstacles/lidar` (DBSCAN), extracts wall segments (RANSAC), and matches
them against a U-shaped berth template — including multiple adjoining
berths sharing a wall, each independently classified occupied/free.
Publishes every recognized berth on `/perception/dock_targets`
(`njord_msgs/DockTargetArray`), plus a backward-compatible
`/perception/dock_target` (highest-confidence FREE berth only).
`description/worlds/dockingWorld.sdf` (single berth) and
`dockingWorldOccupied.sdf` (two berths, one occupied by a static decoy
boat) provide sim testing worlds. This superseded the vision/AprilTag
dock-pose idea below — LiDAR gives short-range geometry directly without
needing a fiducial marker on the dock. What's still missing is everything
downstream of detection:

- [x] **Multi-berth + occupancy detection** — done. Verified both via a
  synthetic test suite (`src/perception/test/test_dock_detector.py`, no
  Gazebo needed — 27-case distance/angle/occupied-berth matrix, 0 failures)
  and against real simulated LiDAR data in `dockingWorldOccupied.sdf`
  (confirmed: the occupied berth is flagged `occupied=true` and excluded
  from `/perception/dock_target`; the free berth reports `occupied=false`).
  The real-Gazebo pass caught 3 bugs the synthetic-only test couldn't:
  (1) a shared back wall's per-berth corner can fall mid-segment, not at
  an endpoint — `_find_u_shapes`' corner-gap check now measures distance
  to the back-wall *segment*, not just its two endpoints; (2) real
  (non-uniform) LiDAR sampling can fragment one physical wall into
  multiple DBSCAN clusters — `cluster_eps` raised 0.4→0.6; (3) a border-line
  weak RANSAC fit (exactly at the old `ransac_min_inliers=6` floor) could
  absorb a few of an occupying boat's hull points as if they were "wall,"
  silently defeating the occupancy check — raised to 10, and a real
  index-mapping bug (`wall_inlier_idx` was cluster-local but compared
  against the full-scan point array) was also fixed. `ransac_dist_threshold_m`
  was tightened 0.05→0.03 so RANSAC cleanly separates a shared wall's two
  faces (~0.1 m apart) instead of fitting one straddling "compromise" line.

- [ ] **Temporal filtering/tracking for `dock_detector_node`**
  Detection currently runs per-scan only — no smoothing or persistence of
  `detected` across frames. Reuse the `Tracker` class already implemented in
  `src/fusion/fusion/geo_fusion_node.py` (~line 368: constant-velocity Kalman
  filter per track, gating, hit-confirmation, miss-count-based death) — the
  dock node's own docstring points at this as the intended next step.

- [x] **Wire docking into the behavior tree** — done (PR #16 + follow-ups).
  `boat_bt/src/docking_nodes.cpp` implements a state machine
  (`WAITING_FOR_TARGET → ALIGNING → APPROACHING → FINAL_ENTRY → DOCKED` →
  hold → reverse → complete) consuming the singular `/perception/dock_target`
  topic, wired into `simple_boat.xml` as the `DockingTask` subtree
  (`ExecuteDocking`), selected via `competition_manager`'s
  `/competition/set_task`. Live-tested end to end (real `dock_detector_node`
  + `boat_bt_node` + `competition_manager`, synthetic-physics closed loop —
  see PR #16 review): reaches the berth, holds `docking_hold_duration_sec`
  (10 s default), reverses out at `docking_reverse_speed_mps`
  (−0.25 m/s default) for `docking_reverse_duration_sec` (4 s default), and
  reports completion via `/competition/complete`. Still uses the singular
  `/perception/dock_target` only — `/perception/dock_targets` (multi-berth
  array) has no consumer yet.

- [x] **Docking-approach path planning / maneuver** — done, as a BT-internal
  proportional bearing/heading controller in `docking_nodes.cpp`
  (`docking_bearing_gain`/`docking_heading_gain`, not a Nav2 goal sequence).

- [x] **Mission-manager / lifecycle hookup** — done via `competition_manager`
  (new package). `/competition/set_task` + `/competition/start` select and
  launch a task; waypoint-less tasks (docking, collision avoidance) skip
  `mission_manager` entirely and run the BT directly, reporting back via
  `/competition/complete`. See "Competition Behavior Tree" section below for
  what's still open (path finding/maneuvering, tests, AR-tags, Task 3.2).

- [ ] **Add reacquisition robustness for near-symmetric multi-berth scenes**
  Found while live-testing the fix above: if `dock_detector_node`'s berth
  pick flickers between two similarly-scored free berths (e.g. a perfectly
  symmetric two-berth layout — likely an edge case, not typical competition
  geometry) while `boat_bt_node` is `APPROACHING`, the boat can oscillate
  hard before losing lock. `docking_reacquire_timeout_sec` now recovers from
  a *lost* target, but doesn't smooth out a *flickering* one. Consider berth
  ID hysteresis/sticky-selection in the detector, or a jump-limiter on
  boat_bt's steering command.

- [ ] **Improve detection robustness/range against `dockingWorldOccupied.sdf`**
  Occupancy classification itself is verified (see above), but detection is
  still viewing-angle-sensitive: from the default spawn pose (dead-center,
  ~5 m out, symmetric between both berths) the two-berth structure isn't
  cleanly resolved at all (`detected=false` — a safe fallback, not a
  false positive, but not useful either); off-center vantage points closer
  to one berth resolve cleanly. Worth tuning further (segment budget,
  clustering, or a wider approach-angle sweep in the BT/mission layer) so a
  boat navigating straight in on the GPS waypoint doesn't need to be
  laterally offset to get a clean read.

- [ ] **Tune detection parameters against real hardware LiDAR noise**
  Current defaults (`cluster_eps=0.6`, `ransac_dist_threshold_m=0.03`,
  `ransac_min_inliers=10`, angle/width tolerances) were tuned against sim
  data (including the multi-berth/occupancy fixes above) and are untested
  on hardware. Also verify `lidar_yaw_offset_deg` (currently 90°,
  sim-derived) against the real mount.

- [ ] **Resolve the orphaned `opennav_docking` wiring**
  `src/bringup/launch/navigation_no_collision.launch.py` already
  instantiates Nav2's stock `opennav_docking` `DockingServer` (lifecycle
  node + component), but this launch file isn't included by
  `njord.launch.py` and isn't referenced anywhere else in the repo. Decide:
  consolidate it into the new LiDAR-geometric approach, repurpose it for a
  different dock type (e.g. a charging dock vs. the Task 3.1 competition
  berth), or delete it — leaving it as dead code next to the new,
  actually-wired `dock_detector_node` invites confusion about which is the
  real docking path.

## Competition Behavior Tree / Task Orchestration

`boat_bt` (BT.CPP 4 tree, `boat_bt_node`) + `competition_manager` (task
selection/lifecycle, new package) landed via PR #16. Reviewed against the
official Njord 2026 task specs (9.1 Maneuvering/Path Finding, 9.2 Collision
Avoidance, 9.3 Docking) and live-tested; see the PR's review comments for
full evidence. Docking is covered above. Status of the rest:

- [x] **CRITICAL FIX — BT stopped ticking forever after any waypoint-based
  task's first abort/failure** — found and fixed 2026-08-12 during a
  full-stack smoke test that cycled all six competition tasks through one
  long-running `boat_bt_node` process (the realistic pattern for an actual
  competition day). Root cause: `tick_tree()` latches `tree_finished_ =
  true` (and then permanently skips ticking the tree at all — `GlobalSafety`
  included, not just the selected task) whenever `MainTree`'s
  `ReactiveSequence` resolves to `SUCCESS`/`FAILURE`, which `MissionMonitor`
  causes for ANY waypoint-based task (maneuvering, path_finding, and now
  collision_avoidance) ending in `FAILED`/`ABORTED` — e.g. a plain
  `/mission/abort`. Only `docking_task_started`/`docking_parallel_task_started`
  ever reset it back to `false`, so a single aborted attempt at *any*
  waypoint-based task silently killed BT ticking for every task attempted
  afterward, with no symptom beyond one `"Behavior Tree failed"` ERROR log
  — `competition_manager`/`mission_manager` state kept progressing normally
  throughout since that's independent of tree ticking, which is exactly
  what made this easy to miss (confirmed live: task 2/3's tree genuinely
  never ticked after task 1's deliberate abort, in a smoke test that
  otherwise looked completely healthy at the service-response level).
  **This predates the 9.2 work above but was masked until now** — nothing
  before this session ever ran more than one waypoint-based task attempt
  in the same process during testing. Fixed by resetting `tree_finished_`
  generically on any task transitioning into `STATE_RUNNING`
  (`boat_bt_node.cpp`'s new `any_task_started`), not just docking/
  docking_parallel. Re-verified live after the fix: `SelectPathFindingTask`/
  `SelectCollisionAvoidanceTask` both confirmed ticking (100+ times each)
  throughout their active window on the very next task after an aborted
  maneuvering attempt, where before the fix they ticked zero times.

- [x] **Task 9.2 (Collision Avoidance) — gate-crossing + COLREG give-way** —
  done 2026-08-12. Brought up to the same standard as 9.1/3.1/3.2:
  `mission_collision_avoidance` sequencer (GPS point 5 → 6 via
  `mission_manager`/Nav2, same pattern as 9.1 — confirmed live: "Mission
  started: 2 waypoints" → "Waiting for Nav2..." not an instant
  `STATE_RUNNING` jump, now that `collision_avoidance.yaml` carries
  waypoints), a task-scoped 2-knot speed-setpoint override
  (`/controller_server/set_parameters`, restored after), and real
  task-specific BT logic in `collision_nodes.cpp`: `updateGateState` pairs
  green/red buoys into gate 1 then gate 2 and detects line-crossing for
  each; `updateMarkerVesselState` identifies the marker vessel ("The Otter
  of Njord", no matching YOLO class) kinematically — nearest moving,
  unclassified obstacle in the forward sector; `colregGiveWaySide` is a
  pragmatic, documented simplification of COLREG Rules 14/15, now using the
  new `Obstacle.velocity_bearing_deg` field (relative-velocity *direction*,
  not just the previously-exposed scalar speed) to only give way when the
  vessel is actually converging. `GlobalSafety`'s generic bearing-only
  reflex is unchanged and still runs unconditionally for this task as the
  range-closing backstop — this new logic layers on top, same pattern as
  Maneuvering/Path Finding's cardinal-marker handling.
  Live-verified (bench, zero-actuation, synthetic `/obstacles/global`): gate
  1/2 pairing + crossing, marker-vessel identification, and give-way target
  generation all confirmed firing correctly (`Give-way target generated:
  ... side=starboard` for a vessel converging from starboard).
  **Still not a full CPA/judging-accurate COLREG classifier**, and
  **UNVERIFIED against real gates or a real vessel**, on the bench or in
  the water — see `enable_collision_avoidance_mission`'s launch-arg
  description. Do a bench check before enabling for a real attempt.

- [ ] **Collision Avoidance — no real course configured**
  Same gap as Maneuvering/Path Finding below:
  `competition_manager/competition_tasks/collision_avoidance.yaml`'s point
  5/point 6 are still the venue's single address point duplicated, not the
  real on-site gate/vessel course.

- [ ] **Maneuvering / Path Finding — no real course configured**
  `mission_maneuvering_pathfinding` (sequencer), the resume-from-point-3
  rule, and the retry-recovery fix are all done and dry-run verified (see
  git log 2026-08-09/10). What's still missing: real GPS waypoints.
  `competition_manager/competition_tasks/maneuvering.yaml` and
  `path_finding.yaml` both currently just duplicate the venue's single
  address point (not `[]` anymore — that was fixed earlier — but still not
  a real course). Per the official spec
  (njord.gitbook.io/2026/9-task-descriptions/9.1-maneuvering-and-path-finding,
  read 2026-08-10, supersedes the imprecise "point 1 → waypoints 1.1–1.10"
  note this item used to cite): one combined course, GPS point 1 → 3 → 4,
  8–15 intermediate waypoints across the two parts — `maneuvering.yaml`
  needs the point 1→3 leg, `path_finding.yaml` needs the point 3→4 leg.
  Once real waypoints land, still needs verification that Nav2 actually
  drives the real course end to end (only ever tested against a single
  placeholder point so far). **Owner: Sara (Nav2/path-finding).**

- [ ] **No automated tests for `boat_bt` or `competition_manager`**
  ~1,500 new C++ lines across `docking_nodes.cpp`, `collision_nodes.cpp`,
  `cardinal_nodes.cpp`, `mission_monitor.cpp`, plus `competition_manager`'s
  entire task/state machine, ship with only boilerplate lint tests
  (`test_copyright.py`/`test_flake8.py`/`test_pep257.py`). No regression
  coverage for the docking state machine, the bypass-side logic, or task
  selection/rejection — unlike `perception`'s
  `test_dock_detector.py` precedent (a real synthetic integration suite).

- [ ] **No AR-tag/ArUco detection for docking**
  Spec 9.3 frames 3 AR-tags as the primary berth-identification method
  (LiDAR-shape detection as the documented fallback when tags aren't
  available); the current pipeline only implements the fallback.

- [x] **Task 3.2 (parallel docking)** — done 2026-08-09/10. `TASK_DOCKING_PARALLEL`
  `CompetitionState` value, `ExecuteDockingParallel` BT controller
  (`parallel_docking_nodes.cpp`), `wall_detector_node` (LiDAR U-shape match,
  reparametrized from `dock_detector_node` for the 4m-wall/2m-arm berth),
  `docking_parallel.yaml` task definition, and `mission_docking_parallel`
  sequencer (GPS points 10/11/12, matching spec 9.3) all exist. Hold
  duration corrected to 5s per spec 9.3 (was wrongly copying 3.1's 10s).
  Dry-run verified (zero-actuation, synthetic perception input) — **still
  UNVERIFIED against a real wall**, on the bench or in the water, and no
  Gazebo world exists for this berth yet unlike 3.1's `dockingWorld.sdf`.

- [ ] **No Surprise task definition**
  `SurpriseTask` is an explicit `<AlwaysSuccess/>` placeholder —
  intentional, pending an official task definition.

- [ ] **`/perception/dock_targets` (multi-berth array) has no consumer**
  `boat_bt_node`'s docking controller only subscribes to the singular
  `/perception/dock_target` (best free berth). Fine for a single-target
  competition task, but the multi-berth-aware output has no use yet — worth
  revisiting if a future task needs to choose among several free berths or
  reason about which one is occupied.

## Simulation Performance (no-GPU / headless sandboxes)

Found while getting a real-Gazebo docking run working in a GPU-less sandbox
(see PR #16 review and the `fix/docking-fov-tracking-loss` branch):

- [x] **Gazebo sensor rendering hangs in server-only (`-s`) mode** — root
  cause identified: `-s` mode deadlocks `gz-sim`'s `Sensors` render thread
  regardless of software-rendering setup (Xvfb, `LIBGL_ALWAYS_SOFTWARE`,
  `--headless-rendering`, explicit `--render-engine-server` flags all
  tried, all hung identically at `Sensors.cc: Waiting for init`). **GUI-
  attached mode (`headless:=false`) works** — same software (llvmpipe)
  rendering underneath, just not server-only. Real GPU rendering was never
  tested here (no GPU in this sandbox); untried but promising: enabling
  actual GPU passthrough (`--gpus=all`) in `.devcontainer/devcontainer.json`
  for machines that have one — WSL2 + Docker Desktop should support this
  natively for an NVIDIA GPU. `runArgs` currently requests none at all.

- [ ] **Real-time factor is very low under software rendering** — measured
  directly (sim `/clock` vs wall clock): RTF ≈ 0.08 (~12x slower than
  real-time) with `headless:=false` + camera/gpu_lidar sensors active.
  Since `docking_reacquire_timeout_sec` and friends are sim-time durations,
  this makes a "3 second" timeout take ~35 real seconds — painful for
  interactive testing, though the underlying control logic still behaves
  correctly in sim-time terms (bearing convergence traced cleanly:
  88°→18° over ~2.3 sim-seconds). Real hardware is entirely unaffected
  (no simulated rendering involved at all). Worth revisiting if GPU
  passthrough becomes available.

- [ ] **`gpu_lidar` → CPU-raycast `lidar` sensor type: tried, reverted**
  Attempted switching Asket's LiDAR sensor (`asket.urdf.xacro`) from
  `type="gpu_lidar"` to `type="lidar"` to sidestep Ogre2 rendering
  entirely for the one sensor docking actually consumes (cameras aren't
  used by docking). Same `<ray>` schema, should be a drop-in swap per the
  gz-sensors docs. In practice it produced zero scan data in this
  gz-sensors8 build — confirmed at both the ROS topic and native `gz
  topic` level, even 45+ seconds after spawn. Didn't dig further into
  whether this is a genuine version gap or a missing config; reverted to
  the confirmed-working `gpu_lidar`. Worth another look if someone wants
  faster headless testing and has time to debug the CPU lidar plugin
  directly (check for silent errors in `~/.gz/sim/log/*/server_console.log`
  around sensor creation, or try a minimal single-sensor test world first).

## Sensor Data Processing Tests

- [ ] **Verify `lidar_obstacle_node` output in sim**
  Launch with `use_sim:=true`, check `/obstacles/lidar` is published at ~15 Hz with `width > 0`.
  Also confirm `header.frame_id = "lidar"` and that range filtering (0.1–10 m) works correctly
  (objects at >10 m should not appear).

- [ ] **Verify `fusion_node` lidar passthrough (no YOLO)**
  With `enable_vision:=false`, `fusion_node` should echo all `/obstacles/lidar` points into
  `/obstacles/fused` with `frame_id = "base_link"`. Confirm: same point count, correct frame.
  TF lookup (`lidar → front_camera`) should succeed (check no `LookupException` in logs).

- [ ] **Verify `fusion_node` with YOLO active**
  With `enable_vision:=true`, place a visible object in Gazebo. Confirm `/yolo/detections`
  arrives, `/yolo/seg_mask` arrives, and `/obstacles/fused` combines both sources.
  YOLO-only detections (no lidar match) should appear at `DEFAULT_OBSTACLE_DISTANCE = 5.0 m`.

- [ ] **Verify EKF input rates**
  After `use_sim:=true` launch, check:
  - `ros2 topic hz /odom` → ~30 Hz (Gazebo OdometryPublisher)
  - `ros2 topic hz /imu_driver/imu_raw` → ~200 Hz (Gazebo IMU)
  - `ros2 topic hz /odometry/filtered` → ~30 Hz (EKF output)
  Low or missing rates indicate a broken bridge or plugin.

- [ ] **Verify costmap receives `/obstacles/fused`**
  After nav2 activates, echo `/local_costmap/costmap` and move a sim obstacle near the robot.
  Confirm the costmap inflates around the obstacle position reported by `/obstacles/fused`.

- [ ] **Verify sensor drivers start cleanly on hardware (no hardware attached)**
  `sllidar_node` (RPLIDAR S2, launched as `lidar_driver`) should report a clean connection failure without crashing the launch — verify actual behaviour on the S2 (untested since the A-series → S2 driver swap; the old custom driver's reconnect-loop behaviour does not carry over).
  `camera_driver` should log a degraded-mode warning without crashing.
  `imu_gps_driver` should wait for MAVROS without crashing.

- [ ] **Retune LiDAR-dependent perception params for the RPLIDAR S2**
  Swapped from the A-series (360 fixed rays, 0.2-12 m, custom driver) to the S2 (`sllidar_ros2`, DenseBoost mode, far denser point cloud, ~0.05-30 m). `lidar_obstacle_node`'s `maximum_obstacle_range_m` default was bumped 10→20 m and an `angular_decimation_deg` param was added to bound output point count (protects `geo_fusion_node`'s O(n²) Euclidean clustering from the S2's much higher native density) — needs validation on real water/buoy returns. `geo_fusion_node`'s clustering/tracking constants (`cluster_tolerance`, `min_cluster_points`, Kalman noise params) were validated against A-series density and are unchanged; they may need retuning once real S2 data is available.

## Perception / Fusion

- [ ] **Implement real late-fusion projection in `fusion_node`**
  `perception/perception/fusion_node.py` does not do actual camera projection.
  It estimates bearing from bbox centre pixel and places obstacles at a hardcoded
  5 m range. Replace with proper pipeline:
  1. Subscribe to raw `/points` (PointCloud2 from lidar) **in addition to** `/obstacles/lidar`
  2. ~~Subscribe to `/camera/camera_info` for intrinsics matrix K~~ — **Done**: `fusion_node` now subscribes to `/front_camera_driver/image_raw/camera_info` and updates fx/fy/cx/cy live.
  3. Look up `camera_optical_link → lidar_link` TF at message time
  4. Project each 3D LIDAR point onto the image plane, check if it falls inside a
     YOLO segmentation bbox (or mask when available); label matching points semantically
  5. Fall back to clustered `/obstacles/lidar` for points outside any detection

- [x] **Publish `CameraInfo` from `camera_driver`**
  Done. `camera_driver` now loads a calibration YAML via `camera_info_manager`
  and publishes `/front_camera_driver/image_raw/camera_info` on every frame.
  Run `ros2 launch bringup calibrate_camera.launch.py` to generate
  `bringup/config/front_camera.yaml`.

- [ ] **Add in-memory object persistence to `fusion_node`**
  The node is stateless — the same buoy is re-fused every frame. Add a
  lightweight object map (dict of id → position + last_seen timestamp) with
  nearest-neighbour association (threshold ~2 m) and a configurable TTL
  (e.g. 8 s). Publish map state as a separate `/obstacles/tracked` topic.

## Control

- [ ] **Close the speed loop in `pid_controller`**
  `control/control/pid_controller.py` runs the speed PID open-loop (no feedback
  sensor). Provide speed feedback — options: use `/mavros/local_position/velocity_body`
  (ArduPilot EKF output), or `/odometry/filtered` from robot_localization.
  Subscribe to whichever is available and feed the measured linear speed as the
  process variable.

- [ ] **Verify RC channel mapping in `actuator_driver`**
  Channel indices (`CHAN_STEERING=0`, `CHAN_THROTTLE=2`) and the `RC_RANGE`
  scaling are placeholders. Confirm against the ArduPilot frame/channel
  assignment for the specific boat configuration (Rover skid-steer vs rudder+throttle).

## Navigation (Nav2)

- [ ] **Tune Nav2 controller for boat dynamics**
  `config/nav2_params.yaml` uses `RegulatedPurePursuitController`. For a USV
  with inertia and no skid-steering, evaluate switching to MPPI
  (`nav2_mppi_controller`) or tuning DWB with a diff-drive model that matches
  the boat's turning radius and maximum surge speed. At minimum, set
  `desired_linear_vel`, `lookahead_dist`, and `min_lookahead_dist` based on
  real on-water measurements.

## Calibration

- [x] **Camera intrinsic calibration**
  Done via `calibrate_camera.launch.py` — `bringup/config/front_camera.yaml`
  committed. Note: the solve's own reprojection error was never recorded (only
  shown live in the calibrator GUI, not logged, and the raw calibration images
  weren't saved anywhere persistent). Everything downstream — the LiDAR-camera
  extrinsic below, `fusion_node`'s projection — inherits whatever error is
  baked into these intrinsics. Worth redoing with more checkerboard samples
  and actually noting the on-screen error next time.

- [x] **LiDAR-camera extrinsic calibration**
  Done via `calibrate_lidar_camera.launch.py` + `ros2 run calibration
  calibrate` — `bringup/config/lidar_camera_extrinsic.yaml` committed,
  2.51 px mean reprojection error (7/12 RANSAC inliers). Not yet visually
  verified end-to-end — see below.

- [ ] **Visually verify LiDAR-camera alignment in RViz2**
  Per the README's documented last step: launch RViz2, add Image
  (`/front_camera_driver/image_raw`) + PointCloud2 (`/lidar_driver/cloud`,
  fixed frame `front_camera_cal`), confirm LiDAR points actually land on
  visible surfaces in the image. The 2.51 px figure is a curve-fit quality
  metric, not proof the whole pipeline (TF wiring, frame conventions) is
  correct end-to-end — this is the real sanity check and hasn't been run yet.

- [ ] **Live-test `fusion_node`/`geo_fusion_node` with the calibrated extrinsic**
  Both were only verified against synthetic/unit data so far
  (`src/fusion/test/test_geo_fusion.py`, and a hand-built synthetic TF for
  `geo_fusion_node.lidar_to_camera`). Run `njord.launch.py` with
  `lidar_camera_extrinsic:=$(pwd)/src/bringup/config/lidar_camera_extrinsic.yaml`
  against a real object and confirm `/obstacles/fused` and `/obstacles/global`
  report sane positions.

- [x] **Fix LiDAR mount yaw sign in URDF**
  `lidar_mount_joint` was `-90°`; verified empirically (an object measured
  dead ahead of the boat read as lidar-local `y≈-range` on
  `/lidar_driver/scan_raw` — only `+90°` predicts that sign) to be wrong, and
  fixed. This joint feeds the `base_link<->lidar` TF used by anything
  consuming `/obstacles/lidar` via tf2 (e.g. Nav2 costmap layers) — with the
  wrong sign, an obstacle dead ahead of the boat would resolve to roughly
  180° from its true position in `base_link`/`map` frame.

- [ ] **Fix URDF sensor heights to match physical hardware**
  Measured heights above hull (`base_link`):
  - LiDAR scan plane: ~52.5 mm (URDF has 174.8 mm — delta −122 mm)
  - Camera lens: ~24.5 mm (URDF has 137.3 mm — delta −113 mm)

  Update `src/description/urdf/asket.urdf.xacro`:
  - `front_camera_joint` origin z: `0.137275` → `0.0245`
  - `lidar_mount_joint`  origin z: `0.137275` → `0.015`
    (so `lidar_mount` z + `lidar_joint` z = 0.015 + 0.0375 = 0.0525)

  Verify in Foxglove/RViz2 that the sensor frames appear at the correct heights on the hull mesh.
  Note: do not change sim Gazebo sensor positions (those are set by `<pose>` in the URDF Gazebo extensions, which may differ).
  Note: does not affect the LiDAR-camera extrinsic above (solved directly from
  point correspondences, independent of URDF geometry) — but does affect the
  scan-plane guide overlay's accuracy and the nominal/uncalibrated fallback
  path (`front_camera` without `lidar_camera_extrinsic` set).

## Infrastructure

- [ ] **Bind-mount config at runtime instead of baking it in the image**
  `config/` is `COPY`-ed into the image at build time. Field params (EKF
  covariances, Nav2 speeds, waypoints) change between tests. Mount
  `./config:/config:ro` in `compose.yaml` so tuning doesn't require a rebuild.
