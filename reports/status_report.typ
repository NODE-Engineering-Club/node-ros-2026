#set document(title: "Njord 2026 / Asket — Project Status Report")
#set page(paper: "us-letter", margin: (x: 2.6cm, y: 2.6cm), numbering: "1")
#set text(size: 11pt)
#set heading(numbering: "1.1")
#set par(justify: true)

#set table(
  stroke: none,
  inset: 6pt,
)
#set table.hline(stroke: 0.6pt)
#show table.cell.where(y: 0): strong

#let callout(title, body) = block(
  fill: rgb("#fff3cd"),
  stroke: rgb("#e0a800"),
  inset: 10pt,
  radius: 4pt,
  width: 100%,
)[
  #text(weight: "bold")[#title]
  #linebreak()
  #body
]

#let done_tag = text(fill: rgb("#1a7f37"), weight: "bold")[DONE]
#let open_tag = text(fill: rgb("#9a6700"), weight: "bold")[OPEN]
#let stale_tag = text(fill: rgb("#cf222e"), weight: "bold")[STALE TODO]

// ---------------------------------------------------------------------------
// Title block
// ---------------------------------------------------------------------------

#align(center)[
  #text(size: 20pt, weight: "bold")[Njord 2026 / Asket]
  #linebreak()
  #text(size: 15pt)[Autonomous Surface Vessel Software Stack — Project Status Report]
  #v(0.6em)
  #text(size: 11pt)[
    Team Captain #h(2em) Software Lead #h(2em) Hardware Lead
  ]
  #v(0.15em)
  #text(size: 8.5pt, style: "italic", fill: rgb("#777777"))[
    Author list open — team is larger than these three roles; see
    "Contributors" (§3.2) for the full list of code contributors by commit
    volume. Names/roles to be finalized before distribution.
  ]
  #v(0.3em)
  #text(size: 10pt, fill: rgb("#555555"))[NODE Engineering Club]
  #v(0.3em)
  August 6, 2026 (rev. 2 — updated post `feat/competition-bt` merge)
]

#v(1em)

// ---------------------------------------------------------------------------
// Abstract
// ---------------------------------------------------------------------------

#block(inset: (x: 1.2cm))[
  #text(weight: "bold")[Abstract.] Njord 2026 is a ROS 2 Jazzy autonomous surface
  vessel stack developed for the competition robot Asket. Over four months of
  development (121 commits on `main` since April 2026), the team has built a
  full sensor-to-actuation pipeline — camera and LiDAR perception fused with
  GPS/IMU localization, a Nav2-based planner/controller, a competition
  Behavior Tree and task-orchestration layer, and a MAVROS bridge to an
  ArduPilot-driven Pixhawk flight controller — and validated it end-to-end in
  Gazebo simulation. Camera and LiDAR-camera calibration have been completed
  and verified on physical hardware. As of this revision, `feat/competition-bt`
  (PR #16) has just merged into `main`: docking is now fully wired end to end
  — detection, behavior-tree state machine, and task lifecycle — and
  live-tested in a synthetic-physics closed loop, though not yet on real
  hardware. Two blocking items remain before a water test can proceed:
  confirming the ArduPilot `FRAME_TYPE` steering configuration, and completing
  a stand-test dry run. Several `TODOS.md` entries were found, on
  cross-reference against the commit history, to already be resolved in code
  but not yet marked as such at the time of the first revision of this report
  — these are called out explicitly below. The PR #16 merge also surfaced new,
  genuinely open gaps: the Maneuvering and Path Finding competition tasks have
  no GPS course configured and cannot currently run, and roughly 1,500 new
  lines of behavior-tree/task-orchestration C++ ship with no automated test
  coverage.
]

#v(0.5em)

// ---------------------------------------------------------------------------
= System Architecture
// ---------------------------------------------------------------------------

The stack is organized as a pipeline of six ROS 2 subsystems, each toggleable
independently from the single `njord.launch.py` entry point:

+ *Sensors* — `camera_driver`, `lidar_driver`, and `imu_gps_driver` normalize
  raw hardware (or, under `use_sim:=true`, Gazebo Harmonic simulated sensor
  plugins) into standard topics: `/front_camera_driver/image_raw`,
  `/lidar_driver/scan_raw`, `/imu_driver/imu_raw`, `/gps_driver/gps_raw`.
+ *Perception* — three parallel consumers of the raw sensor topics:
  - `vision_node` runs YOLO26n-seg (ONNX Runtime, CPU) for object detection
    and instance segmentation.
  - `lidar_obstacle_node` converts LiDAR scans into a filtered obstacle point
    cloud (`/obstacles/lidar`).
  - `fusion_node` projects LiDAR points into the camera frame via TF and
    correlates them with YOLO's segmentation mask, publishing a labeled,
    `base_link`-frame obstacle cloud (`/obstacles/fused`) that feeds Nav2's
    costmaps. A second, parallel node, `geo_fusion_node`, produces a
    GPS-anchored, tracked obstacle list (`/obstacles/global`) and is being
    evaluated alongside `fusion_node`.
  - `dock_detector_node` (new this cycle) recognizes U-shaped competition
    berths from LiDAR geometry alone (DBSCAN clustering + RANSAC wall
    fitting), including multi-berth scenes with occupancy classification.
+ *Localization* — `robot_localization`'s EKF fuses IMU and odometry into
  `/odometry/filtered`; `navsat_transform_node` converts the GPS fix into the
  map frame. Together these establish the `map → odom → base_link` transform
  chain that every downstream subsystem depends on.
+ *Navigation* — a standard Nav2 stack (global/local costmaps fed by
  `/obstacles/fused`, `NavfnPlanner`, `RegulatedPurePursuitController`,
  `bt_navigator`) plans paths and issues `/cmd_vel` velocity commands.
+ *Control* — `nav_to_pid` clamps Nav2's output to safe speed/yaw-rate limits;
  `pid_controller` runs closed-loop speed and yaw PID (via the `simple-pid`
  library); `actuator_driver` maps the resulting effort to MAVROS RC channel
  overrides.
+ *Mission* — `mission_manager` sequences hardcoded GPS waypoints through
  Nav2's `NavigateToPose` action, converting each via
  `robot_localization`'s `/fromLL` service.

*MAVROS / ArduPilot integration.* The ROS 2 side of the stack operates
entirely in terms of goals, paths, and velocity commands; it has no direct
knowledge of the vehicle's low-level actuation. MAVROS is the bridge: it
exposes the Pixhawk flight controller (running ArduPilot's Rover firmware) as
a set of ROS 2 topics and services, relaying IMU/GPS telemetry upstream and
accepting RC channel overrides (`/mavros/rc/override`) downstream. This
separation means the same navigation and perception code runs unmodified in
Gazebo simulation (`use_sim:=true`, `enable_mavros:=false`) and on the
physical boat.

// ---------------------------------------------------------------------------
= Development History
// ---------------------------------------------------------------------------

== Timeline

The `main` branch contains 121 commits spanning April 7 -- August 6, 2026.
Development proceeded in six broad phases:

#table(
  columns: (auto, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Period], [Focus]),
  table.hline(stroke: 0.5pt),
  [Apr 7 -- Apr 26], [Infrastructure: single-container build, Pi deployment
    scripts, telemetry server. First URDF work begins.],
  [May 8 -- May 30], [Simulation stack: URDF completed, Gazebo/`ros_gz`
    bridge wired, TF2 frame tree implemented. First full-stack sim milestone
    (May 17): Nav2 activates and successfully navigates GPS waypoints.
    Foxglove bridge added. Two same-day experiments (a web GUI, an early
    fusion node) were added and reverted.],
  [Jun 3 -- Jun 17], [Control and behavior-tree foundation: PID controller
    migrated to closed-loop (`simple-pid`) speed feedback; `boat_bt`
    behavior-tree package introduced; Nav2 parameter fixes; MAVROS/BlueOS
    connection fixes.],
  [Jul 6 -- Jul 16], [Calibration: camera intrinsic calibration tooling,
    LiDAR-camera extrinsic calibration (2.51 px mean reprojection error),
    LiDAR mount yaw-sign fix, and several fusion-node projection-math
    corrections (BGR/RGB mismatch, inverted TF direction, missing rotation
    component).],
  [Jul 18 -- Jul 29], [Behavior tree and docking: cardinal-marker handling
    and collision-avoidance bypass logic added to `boat_bt`; the new
    LiDAR-geometric dock detector added, then extended to multi-berth
    occupancy classification.],
  [Jul 30 -- Aug 6], [Competition Behavior Tree and task orchestration
    (PR #16, merged): a new `competition_manager` package for task
    selection/lifecycle; `boat_bt` refactored into a modular,
    task-oriented architecture with a full docking state machine
    (`docking_nodes.cpp`) and adaptive-side collision avoidance; docking
    wired end to end and live-tested in a synthetic-physics closed loop;
    a missing GPS sensor TF fix for simulation (most recent commit on
    `main`).],
  table.hline(stroke: 0.8pt),
)

== Contributors

Commit authorship on `main` (by git identity; some contributors committed
under more than one name/email):

#table(
  columns: (auto, auto, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Commits], [Identity], [Primary area]),
  table.hline(stroke: 0.5pt),
  [31], [tompeace / Tom Peace], [Infrastructure, deployment, container build],
  [26], [Heleri Koltsin], [`boat_bt` behavior tree, `competition_manager`,
    docking state machine, mission manager],
  [15], [PaintDumpster], [URDF, camera/LiDAR calibration, dock detection],
  [14], [salvadorc], [Calibration tooling, fusion-node projection fixes],
  [12], [Jokar-man], [CI, Gazebo/MAVROS bridging, sim package],
  [6],  [auxenceIAAC], [Camera driver robustness],
  [6],  [Salvador Cantuarias Brañes], [Calibration and competition-BT PR
    merges],
  [3],  [Priyam Gulati], [PID controller closed-loop migration],
  [3],  [Sara], [Nav2 parameter fixes, waypoint flow],
  [2],  [Chakshu Chopra], [Foxglove bridge integration],
  [1],  [Kooshaaf], [`geo_fusion_node` (geo-referenced fusion)],
  table.hline(stroke: 0.8pt),
)

Note: `PaintDumpster` and `salvadorc` / `Salvador Cantuarias Brañes` share a
commit email address and are almost certainly the same contributor under
different git configurations, making this the single largest contributor by
volume (35 commits) after `tompeace`. Heleri Koltsin's commit count nearly
tripled between the two revisions of this report, almost entirely from the
`feat/competition-bt` work merged in PR #16.

== Branch landscape and unmerged work

Twelve branches exist on the remote beyond `main`. Most (`feat/calibration`,
`Nav2`) have already been merged via pull request; several others
(`GPS_Fix_and_Task`, `feat/autonomy-bt`, `feat/cuda`, `feat/gazebo-motion`,
`foxglove`, `gazebo_debugg`, `piddebugg`, `Nav2-testing`) appear to be
exploratory or superseded and were not further examined for this report.

#callout("feat/competition-bt merged (PR #16, 2026-08-06)")[
  This branch was under active development at the time of the first revision
  of this report and has since merged into `main`, along with a documentation
  follow-up commit. It closes several items previously listed as open in
  @outstanding-work — docking is now wired into the behavior tree end to end,
  with an autonomous approach/hold/reverse state machine and a new
  `competition_manager` task-lifecycle package — but its review also surfaced
  new, genuinely open gaps (no GPS course for two of the five competition
  tasks, no automated test coverage for the new C++). See the new
  "Competition Behavior Tree / Task Orchestration" subsection below for the
  full picture.
]

// ---------------------------------------------------------------------------
= Outstanding Work <outstanding-work>
// ---------------------------------------------------------------------------

Items below are drawn from `TODOS.md`, cross-referenced against the commit
history and current source. Two items were found to be *stale* — marked done
in code but not updated in `TODOS.md` — and are flagged accordingly.

#callout("Blocking water test")[
  `TODOS.md` identifies two items as blocking hardware water testing, neither
  yet resolved:
  + *Rotate-to-heading configuration* — `nav2_params.yaml` currently enables
    in-place rotation (`rotate_to_heading_angular_vel: 0.5`, confirmed still
    present in the current config), pending confirmation of the Pixhawk's
    `FRAME_TYPE` setting via QGroundControl. This determines whether the
    boat is skid-steered or rudder-steered and changes which navigation mode
    is correct — not a tuning parameter.
  + *Stand-test dry run* — boat on a stand, FCU/RPi/thrusters connected,
    confirming MAVROS connection, sensor publishing, odometry updates under
    manual movement, correct thrust direction on a test waypoint, and
    abort-stops-thrusters-within-2s. Not yet marked complete.
]

== Navigation (Docking)

#table(
  columns: (30%, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Item], [Status]),
  table.hline(stroke: 0.5pt),
  [Multi-berth + occupancy detection], [#done_tag — verified via 27-case
    synthetic test suite and real-Gazebo run against `dockingWorldOccupied.sdf`],
  [Temporal filtering / tracking for `dock_detector_node`], [#open_tag —
    detection still runs per-scan only; the constant-velocity Kalman tracker
    already implemented in `geo_fusion_node.py` remains unreused here],
  [Wire docking into the behavior tree], [#done_tag (PR #16) —
    `docking_nodes.cpp` implements
    `WAITING_FOR_TARGET → ALIGNING → APPROACHING → FINAL_ENTRY → DOCKED`,
    wired into `simple_boat.xml` as the `DockingTask` subtree, selected via
    `competition_manager`],
  [Docking-approach path planning / maneuver], [#done_tag (PR #16) — a
    BT-internal proportional bearing/heading controller in
    `docking_nodes.cpp`, not a Nav2 goal sequence],
  [Mission-manager / lifecycle hookup for docking], [#done_tag (PR #16) —
    via the new `competition_manager` package;
    `/competition/set_task` + `/competition/start` select and launch a
    task, reporting completion via `/competition/complete`],
  [Add reacquisition robustness for near-symmetric multi-berth scenes],
    [#open_tag — new gap found while live-testing PR #16: a berth pick that
    flickers between two similarly-scored free berths can cause hard
    oscillation while `APPROACHING`; `docking_reacquire_timeout_sec`
    recovers from a lost target but not a flickering one],
  [Improve detection robustness/range (default spawn pose)], [#open_tag],
  [Tune detection parameters against real hardware LiDAR noise], [#open_tag —
    current defaults tuned against sim data only],
  [Resolve orphaned `opennav_docking` wiring in
    `navigation_no_collision.launch.py`], [#open_tag — dead code, not
    referenced by `njord.launch.py`],
  table.hline(stroke: 0.8pt),
)

== Competition Behavior Tree / Task Orchestration

`boat_bt` (BT.CPP 4 tree) plus the new `competition_manager` package
(task selection and lifecycle) landed via PR #16, reviewed against the
official Njord 2026 task specs (9.1 Maneuvering/Path Finding, 9.2 Collision
Avoidance, 9.3 Docking). Docking itself is covered in the table above.

#table(
  columns: (30%, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Item], [Status]),
  table.hline(stroke: 0.5pt),
  [Collision avoidance task subtree + adaptive bypass side], [#done_tag —
    `CollisionAvoidanceTask` now runs the real avoidance sequence (previously
    an `<AlwaysSuccess/>` stub); bypass side follows the obstacle's bearing
    instead of a hardcoded starboard default. Live-verified at three
    bearings. Caveat: still a reactive rule, not a COLREG/CPA classifier —
    no relative-velocity reasoning, no task-specific speed setpoint],
  [Maneuvering / Path Finding — no GPS course configured], [#open_tag —
    `competition_tasks/maneuvering.yaml` and `path_finding.yaml` both have
    `mission: waypoints: []`; `competition_manager` now rejects
    `/competition/start` for either with a clear error rather than silently
    doing nothing, but neither task can run yet. Owner: Sara],
  [Automated test coverage for `boat_bt` / `competition_manager`],
    [#open_tag — roughly 1,500 new C++ lines across the docking, collision,
    and cardinal-marker nodes plus the task/state machine ship with only
    boilerplate lint tests; no regression coverage, unlike perception's
    `test_dock_detector.py` precedent],
  [AR-tag / ArUco detection for docking], [#open_tag — competition spec 9.3
    frames AR-tags as the primary berth-identification method with
    LiDAR-shape detection as fallback; only the fallback is implemented],
  [Task 3.2 (parallel docking)], [#open_tag — only normal docking (3.1,
    2 m × 2 m berth) exists; no `CompetitionState` value, BT subtree, or
    task YAML for the 2 m × 4 m parallel variant],
  [Surprise task], [Intentional placeholder — explicit `<AlwaysSuccess/>`
    pending an official task definition],
  [`/perception/dock_targets` (multi-berth array) consumer], [#open_tag —
    the docking controller only subscribes to the singular
    `/perception/dock_target`; the multi-berth-aware array has no consumer],
  table.hline(stroke: 0.8pt),
)

== Sensor Data Processing Tests

All items in this `TODOS.md` section are manual verification checklist items
(confirming topic rates, frame IDs, and range filtering under
`use_sim:=true`) rather than code changes, so they cannot be confirmed or
refuted from the commit history alone. All remain unchecked in `TODOS.md`:
verifying `lidar_obstacle_node` output, `fusion_node` LiDAR passthrough,
`fusion_node` with YOLO active, EKF input rates, costmap obstacle-layer
response, and clean sensor-driver startup with no hardware attached.

== Perception / Fusion

#table(
  columns: (30%, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Item], [Status]),
  table.hline(stroke: 0.5pt),
  [Publish `CameraInfo` from `camera_driver`], [#done_tag],
  [Implement real late-fusion projection in `fusion_node`], [#stale_tag —
    `TODOS.md` describes this as an unimplemented stub (bearing estimate at a
    hardcoded 5 m range for all detections). Current
    `src/perception/perception/fusion_node.py` performs genuine TF-based
    projection (`_lidar_to_camera_tf`, `_project_calibrated` /
    `_project_nominal`) and segmentation-mask correlation, per its own
    docstring. The hardcoded 5 m distance is retained only as the documented
    fallback for YOLO detections with no LiDAR support in range — correct,
    intended behavior, not a limitation. This was resolved across several
    July commits (`ab2c9c7`, `99c34a1`, `fda32a8`, `63cb3af`) that were never
    reflected back into `TODOS.md`.],
  [Add in-memory object persistence to `fusion_node`], [#open_tag],
  table.hline(stroke: 0.8pt),
)

== Control

#table(
  columns: (30%, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Item], [Status]),
  table.hline(stroke: 0.5pt),
  [Close the speed loop in `pid_controller`], [#stale_tag — `TODOS.md`
    describes the speed PID as open-loop with no feedback sensor. Commit
    `79675e6` (2026-06-03) migrated `pid_controller.py` to the `simple-pid`
    library with a subscription to
    `/mavros/local_position/velocity_body`; the measured speed is confirmed
    passed as the PID process variable in the current source
    (`self._speed_pid(self._speed)`). Genuinely resolved, `TODOS.md` simply
    was not updated.],
  [Verify RC channel mapping in `actuator_driver`], [#open_tag — channel
    indices and scaling in `src/control/control/actuator_driver.py`
    (`CHAN_STEERING = 0`, `CHAN_THROTTLE = 2`, `RC_RANGE = 400`) remain
    unverified placeholders against the boat's actual wiring, confirmed
    unchanged in current source.],
  table.hline(stroke: 0.8pt),
)

== Navigation (Nav2 tuning)

Nav2 controller tuning for boat dynamics (evaluating MPPI vs. the current
Regulated Pure Pursuit controller; setting `desired_linear_vel`,
`lookahead_dist` from real on-water measurements) remains #open_tag — no
commits target this since the controller was first configured.

== Calibration

#table(
  columns: (30%, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Item], [Status]),
  table.hline(stroke: 0.5pt),
  [Camera intrinsic calibration], [#done_tag — real hardware intrinsics
    committed; reprojection error itself was not logged (caveat noted in
    `TODOS.md`)],
  [LiDAR-camera extrinsic calibration], [#done_tag — 2.51 px mean
    reprojection error, 7/12 RANSAC inliers],
  [Visually verify LiDAR-camera alignment in RViz2], [#open_tag — the
    numeric fit has not been visually confirmed end-to-end],
  [Live-test `fusion_node` / `geo_fusion_node` with calibrated extrinsic],
    [#open_tag — verified only against synthetic/unit test data so far],
  [Fix LiDAR mount yaw sign in URDF], [#done_tag — empirically verified
    (`5a61aae`)],
  [Fix URDF sensor heights to match physical hardware], [#open_tag — fix is
    written up in `TODOS.md` but not yet applied to
    `src/description/urdf/asket.urdf.xacro`],
  table.hline(stroke: 0.8pt),
)

== Infrastructure

Bind-mounting runtime config instead of baking it into the container image
remains #open_tag. Low priority: development-workflow convenience, not a
competition-readiness risk.

// ---------------------------------------------------------------------------
= Open Issues
// ---------------------------------------------------------------------------

Six issues are open on the GitHub tracker (#2, #3, #4, #5, #6, #10), dated
April -- June 2026. The software lead has directly confirmed (2026-08-01)
that all six are resolved by work already landed in the codebase and the
tracker itself is simply stale; they are summarized here for completeness
rather than as live action items.

#table(
  columns: (auto, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([\#], [Original ask]),
  table.hline(stroke: 0.5pt),
  [10], [Test-day software checklist: waypoint-file navigation A→B→C,
    re-orientation, GPS point acquisition.],
  [6],  [Build a digital URDF model of the boat with sensor/component
    positions relative to the hull.],
  [5],  [Create the URDF XML file linking the ROS network to the boat
    frame.],
  [4],  [Document and evaluate the PID controller nodes, including
    comparing a hand-rolled implementation against the `simple-pid`
    library.],
  [3],  [Implement decision nodes for buoy-obstacle avoidance and docking,
    coordinated with the path-planning layer.],
  [2],  [Integrate the standalone MATLAB GPS path-planning script into the
    ROS network, or replace it with Nav2.],
  table.hline(stroke: 0.8pt),
)

// ---------------------------------------------------------------------------
= Risk Assessment
// ---------------------------------------------------------------------------

The distinction between "verified in simulation" and "verified on real
hardware" is the dominant source of residual risk heading into competition.

#table(
  columns: (1fr, 1fr, 1fr),
  table.hline(stroke: 0.8pt),
  table.header([Capability], [Simulation], [Hardware]),
  table.hline(stroke: 0.5pt),
  [Full TF chain (`map → odom → base_link`)], [Verified], [Not verified],
  [Nav2 lifecycle activation and GPS waypoint navigation], [Verified
    (2026-05-17)], [Not verified],
  [Camera intrinsic calibration], [N/A], [Verified — real intrinsics
    committed],
  [LiDAR-camera extrinsic calibration], [N/A], [Numerically verified
    (2.51 px); visual RViz2 confirmation outstanding],
  [Dock detection (multi-berth + occupancy)], [Verified — synthetic suite
    + real-Gazebo run], [Not tested — sim-tuned parameters only],
  [Docking approach/hold/reverse + task lifecycle (PR #16)], [Verified —
    live-tested end to end in a synthetic-physics closed loop], [Not
    tested — no automated regression coverage either],
  [Maneuvering / Path Finding competition tasks], [Cannot run — no GPS
    course configured in either task's YAML], [Cannot run],
  [Collision avoidance task (adaptive bypass side)], [Verified at three
    obstacle bearings], [Not tested],
  [Speed/yaw PID control loop], [Not separately reported], [Closed-loop
    feedback wired to `/mavros/local_position/velocity_body`; gains
    untuned against real vehicle dynamics],
  [RC channel mapping (`actuator_driver`)], [N/A — bypassed in sim], [Not
    verified against actual thruster/ESC wiring],
  [Steering configuration (`FRAME_TYPE` / rotate-to-heading)], [N/A], [Not
    confirmed — blocking item],
  [Nav2 controller tuning (turning radius, speed limits)], [Configured with
    placeholder values], [Not validated on water],
  table.hline(stroke: 0.8pt),
)

Highest-priority risks, in order:

+ *`FRAME_TYPE` unresolved.* This is a configuration branch point, not a
  tunable parameter — an incorrect assumption here means the boat is
  configured for the wrong steering geometry entirely.
+ *RC channel mapping unverified against real wiring.* If steering/throttle
  channels are swapped or incorrectly scaled, first thruster arming could
  drive the boat in an unintended direction. This is precisely what the
  stand-test dry run is designed to catch and should not be skipped.
+ *Docking is now wired end to end on `main` (PR #16), but only
  simulation-verified.* Detection, approach, hold, and reverse all work in a
  synthetic-physics closed loop; none of it has run on the physical boat, and
  none of it has automated regression tests protecting it before travel.
+ *Two of five competition tasks (Maneuvering, Path Finding) cannot run at
  all yet* — no GPS course is configured in either task's YAML. This is a
  content gap, not a bug, but it means those tasks are not simply
  "unverified," they are non-functional until someone (owner: Sara per
  `TODOS.md`) adds real waypoints.
+ *No Nav2 parameter has been validated against real vehicle dynamics.*
  Turning radius, speed limits, and lookahead distance are all still
  simulation-derived placeholders.
+ *Perception parameters (dock detector, fusion) were tuned exclusively
  against simulated sensor data.* Expect on-site retuning against real
  LiDAR noise characteristics.
+ *~1,500 new lines of behavior-tree/task-orchestration C++ (PR #16) have no
  automated test coverage.* Any regression in docking or collision-avoidance
  logic between now and competition would likely only be caught by manual
  re-testing.
