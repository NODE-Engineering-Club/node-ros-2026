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
#let progress_tag = text(fill: rgb("#0969da"), weight: "bold")[IN PROGRESS (unmerged)]

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
  August 3, 2026
]

#v(1em)

// ---------------------------------------------------------------------------
// Abstract
// ---------------------------------------------------------------------------

#block(inset: (x: 1.2cm))[
  #text(weight: "bold")[Abstract.] Njord 2026 is a ROS 2 Jazzy autonomous surface
  vessel stack developed for the competition robot Asket. Over roughly four
  months of development (102 commits on `main` since April 2026, plus active
  work on unmerged feature branches), the team has built a full
  sensor-to-actuation pipeline — camera and LiDAR perception fused with GPS/IMU
  localization, a Nav2-based planner/controller, and a MAVROS bridge to an
  ArduPilot-driven Pixhawk flight controller — and validated it end-to-end in
  Gazebo simulation. Camera and LiDAR-camera calibration have been completed
  and verified on physical hardware. The most recent development cycle added a
  LiDAR-geometric dock detector capable of recognizing occupied and free
  berths, including in multi-berth scenes. Two blocking items remain before a
  water test can proceed: confirming the ArduPilot `FRAME_TYPE` steering
  configuration, and completing a stand-test dry run. Several `TODOS.md`
  entries describing outstanding work were found, on cross-reference against
  the commit history, to already be resolved in code but not yet marked as
  such — these are called out explicitly below. Substantial docking-autonomy
  work (behavior-tree integration, an autonomous docking controller, and
  competition task orchestration) is in progress on an unmerged branch,
  `feat/competition-bt`, and is not yet reflected in `main` or `TODOS.md`.
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

The `main` branch contains 102 commits spanning April 7 -- July 29, 2026.
Development proceeded in five broad phases:

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
    occupancy classification (most recent commit on `main`).],
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
  [14], [PaintDumpster], [URDF, camera/LiDAR calibration, dock detection],
  [14], [salvadorc], [Calibration tooling, fusion-node projection fixes],
  [12], [Jokar-man], [CI, Gazebo/MAVROS bridging, sim package],
  [9],  [Heleri Koltsin], [`boat_bt` behavior tree, mission manager],
  [6],  [auxenceIAAC], [Camera driver robustness],
  [5],  [Salvador Cantuarias Brañes], [Calibration PR merges],
  [3],  [Priyam Gulati], [PID controller closed-loop migration],
  [3],  [Sara], [Nav2 parameter fixes, waypoint flow],
  [2],  [Chakshu Chopra], [Foxglove bridge integration],
  [1],  [Kooshaaf], [`geo_fusion_node` (geo-referenced fusion)],
  table.hline(stroke: 0.8pt),
)

Note: `PaintDumpster` and `salvadorc` / `Salvador Cantuarias Brañes` share a
commit email address and are almost certainly the same contributor under
different git configurations, making this the single largest contributor by
volume (33 commits) after `tompeace`.

== Branch landscape and unmerged work

Twelve branches exist on the remote beyond `main`. Most (`feat/calibration`,
`Nav2`) have already been merged via pull request; several others
(`GPS_Fix_and_Task`, `feat/autonomy-bt`, `feat/cuda`, `feat/gazebo-motion`,
`foxglove`, `gazebo_debugg`, `piddebugg`, `Nav2-testing`) appear to be
exploratory or superseded and were not further examined for this report.

#callout("Active unmerged branch: feat/competition-bt")[
  This branch is *not merged into `main`* and its contents are *not reflected
  in `TODOS.md`*, but it directly addresses several items `TODOS.md` currently
  lists as open, including behavior-tree docking integration and the
  docking-approach maneuver (see @outstanding-work). Its seven commits — "Add
  CompetitionManager framework and task-aware Behavior Tree," "Add competition
  task orchestration and Nav2 integration," "Refactor behavior tree into
  task-oriented architecture," "Integrate dock target perception into
  behavior tree," "Add stateful autonomous docking controller," "Integrate
  docking completion with competition lifecycle," and "Improve collision risk
  selection" — indicate active, recent development (most recent activity
  concurrent with this report) toward wiring dock detection into the
  behavior tree, an autonomous docking approach maneuver, and a competition
  task/mission lifecycle. Recommend confirming this branch's status and merge
  timeline with the software lead directly, as it materially changes the
  "Outstanding Work" picture below if merged before competition.
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
  [Temporal filtering / tracking for `dock_detector_node`], [#open_tag],
  [Wire docking into the behavior tree], [#open_tag on `main`; #progress_tag
    on `feat/competition-bt` ("Integrate dock target perception into
    behavior tree")],
  [Docking-approach path planning / maneuver], [#open_tag on `main`;
    #progress_tag on `feat/competition-bt` ("Add stateful autonomous
    docking controller")],
  [Mission-manager / lifecycle hookup for docking], [#open_tag on `main`;
    #progress_tag on `feat/competition-bt` ("Integrate docking completion
    with competition lifecycle")],
  [Improve detection robustness/range (default spawn pose)], [#open_tag],
  [Tune detection parameters against real hardware LiDAR noise], [#open_tag —
    current defaults tuned against sim data only],
  [Resolve orphaned `opennav_docking` wiring in
    `navigation_no_collision.launch.py`], [#open_tag — dead code, not
    referenced by `njord.launch.py`],
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
+ *Docking is not competition-ready on `main`.* Detection is solid in
  simulation, but the approach maneuver, behavior-tree wiring, and mission
  hookup are either absent on `main` or present only on the unmerged
  `feat/competition-bt` branch. Whether that branch merges before travel is
  a material scheduling question.
+ *No Nav2 parameter has been validated against real vehicle dynamics.*
  Turning radius, speed limits, and lookahead distance are all still
  simulation-derived placeholders.
+ *Perception parameters (dock detector, fusion) were tuned exclusively
  against simulated sensor data.* Expect on-site retuning against real
  LiDAR noise characteristics.
