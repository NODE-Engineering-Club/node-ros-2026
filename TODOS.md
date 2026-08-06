# Njord 2026 — Outstanding Gaps

## HIGH PRIORITY — Blocking Water Test (Wednesday)

- [ ] **Disable RPP rotate-to-heading**
  `bringup/config/nav2_params.yaml:32` enables in-place rotation
  (`rotate_to_heading_angular_vel: 0.5`). Blocked on confirming `FRAME_TYPE`
  on the Pixhawk via QGC. If `FRAME_TYPE=2` (skid-steer), leave enabled and
  also enable `allow_reversing: true` and add `spin`/`back_up` to
  `behavior_server`. If `FRAME_TYPE=0` (normal steering), set
  `use_rotate_to_heading: false` and `allow_reversing: false`.

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

- [x] **Collision avoidance — task subtree + adaptive bypass side** — done.
  `CollisionAvoidanceTask` runs the real avoidance sequence (previously an
  `<AlwaysSuccess/>` stub); `avoidance_side_` now follows the obstacle's
  bearing (port-side obstacle → bypass starboard, starboard-side → bypass
  port, ±2° centreline deadband defaults to starboard) instead of being
  hardcoded to starboard always. Live-verified at three bearings. Still not
  a full COLREG/CPA classifier — no relative-velocity-direction reasoning,
  no 2-knot task speed setpoint, no vessel-detection signaling, no
  gate-crossing start/end logic tied to the task specifically. `GlobalSafety`
  runs the same reflex unconditionally regardless of selected task (except
  during docking), so in practice this is one always-on avoidance behavior
  rather than a collision-avoidance-task-specific one.

- [ ] **Maneuvering / Path Finding — no course configured**
  `competition_manager/competition_tasks/maneuvering.yaml` and
  `path_finding.yaml` both have `mission: waypoints: []`.
  `competition_manager` now rejects `/competition/start` for either task
  with a clear error instead of silently reaching `STATE_RUNNING` and doing
  nothing — but the task itself still can't run. Needs real GPS waypoints
  per the spec (point 1 → waypoints 1.1–1.10 → point 4 for path finding; a
  similar course for maneuvering) and, once `mission_manager` has a course
  to run, verification that Nav2 actually drives it end to end. **Owner:
  Sara (Nav2/path-finding).**

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

- [ ] **No Task 3.2 (parallel docking)**
  Only normal docking (3.1, 2m×2m berth) exists. Parallel docking (3.2,
  2m×4m berth, 5 s hold instead of 10 s, separate GPS points 9/10) has no
  `CompetitionState` task value, no BT subtree, and no task YAML.

- [ ] **No Surprise task definition**
  `SurpriseTask` is an explicit `<AlwaysSuccess/>` placeholder —
  intentional, pending an official task definition.

- [ ] **`/perception/dock_targets` (multi-berth array) has no consumer**
  `boat_bt_node`'s docking controller only subscribes to the singular
  `/perception/dock_target` (best free berth). Fine for a single-target
  competition task, but the multi-berth-aware output has no use yet — worth
  revisiting if a future task needs to choose among several free berths or
  reason about which one is occupied.

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
  `lidar_driver` should log "RPLIDAR not available... retrying" without crashing.
  `camera_driver` should log a degraded-mode warning without crashing.
  `imu_gps_driver` should wait for MAVROS without crashing.

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
