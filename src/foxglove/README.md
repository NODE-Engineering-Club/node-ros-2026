# Foxglove
Everything to do with the Graphic User Interface (GUI)

# ASKET 2.0 GUI — Mandatory Layout (`ASKET_GUI_mandatory.json`)

## What it is

A Foxglove Studio layout that shows the jury the minimum required information
about the ASV during a task run. Import it into Foxglove
(Layout menu → Import from file) while connected to the boat's foxglove_bridge
(`ws://<pi-ip>:8765`).

## Where these files live in the repo

1. `src/foxglove/ASKET_GUI_mandatory.json` — this layout.
2. `src/foxglove/README.md` — this file.
3. `src/foxglove/gui_telemetry_hw.py` — the telemetry helper node that feeds
   the gauges + status bar (see its own section below). It's a standalone
   script here, not inside a colcon package — run it directly with
   `python3`, not `ros2 run` (there's no package name for it).
4. `src/foxglove/gui_markers.py` — republishes `/obstacles/global` (cardinal
   markers/buoys, mission name/state) for the GUI's 3D and Map panels + the
   mission Indicators (see its own section below). Same standalone-script
   convention as `gui_telemetry_hw.py`.

## What it does

It arranges thirteen panels into a single jury-readable screen:

| Panel               | Shows                          | Topic it reads                 | Data source                     |
| ------------------- | ------------------------------ | ------------------------------ | ------------------------------- |
| LiDAR (3D)          | Live 2D LiDAR scan + detected cardinal markers/buoys, colored by class | `/lidar_driver/scan_raw` + `/gui/cardinal_markers_3d` | `lidar_driver` (real hardware) + `gui_markers` (from fusion) |
| RGB Front Camera    | Front camera feed              | `/front_camera_driver/image_raw` | `camera_driver` (real hardware) |
| Map                 | Boat position + planned-route markers + detected markers | `/gps_driver/gps_raw` (live trail, blue) + `/competition/waypoints/wp_*` and the docking missions' `*/points/*` (planned-route markers, amber) + `/gui/markers/marker_0..11` (detected markers, white — see Limitations for why these aren't color-coded here) | `imu_gps_driver` GPS + `competition_manager`/`mission_docking*` + `gui_markers` |
| Latitude / Longitude| Numeric GPS readout            | `/gps_driver/gps_raw`          | `imu_gps_driver` GPS            |
| ASV STATUS          | Auto / Remote / Standby / Out of control | `/vehicle/status`    | `gui_telemetry_hw` (from MAVROS) |
| Heading gauge       | Compass heading (deg)          | `/heading`                     | `gui_telemetry_hw` (from MAVROS) |
| COG gauge           | Course over ground (deg)       | `/cog`                         | `gui_telemetry_hw` (from MAVROS) — see Limitations, frame convention unverified on hardware |
| Speed gauge         | Speed over ground (kn)         | `/sog`                         | `gui_telemetry_hw` (from MAVROS) |
| Battery gauge       | Battery remaining (%)          | `/battery_percentage`          | `gui_telemetry_hw` (from MAVROS) |
| BMS (BQ76920)       | Pack voltage, per-cell voltage, temp, current, CHG/DSG, fault flags | `/diagnostics` | `bms_reader` (real hardware, already working) |
| Mission             | Current competition task name  | `/competition/status`          | `competition_manager`           |
| Mission State       | Current mission state (idle/running/succeeded/failed/aborted) | `/mission/status` | `mission_manager` |
| Mission Progress    | Raw mission status: current/total waypoint, state message | `/mission/status` | `mission_manager` |

The camera, LiDAR, GPS, map, and BMS panels read real, already-working
topics directly. The four gauges and the status bar read topics produced
by `gui_telemetry_hw.py`, which now exists (see below) but is
**UNVERIFIED against real hardware** — no bench/water test yet, see
Limitations for the one specific correctness risk (COG frame convention).
The map's waypoint-marker topics only exist while a task with waypoints is selected
(`competition_manager`'s `/competition/waypoints/wp_N`) or while
`mission_docking`/`mission_docking_parallel` is running (their own
`*/points/*` topics) — otherwise only the live GPS trail shows. There is
no single connected "ideal route" polyline published anywhere yet, only
individual waypoint pins in a different color from the live trail; a real
connected route line would need a new topic (e.g. a `nav_msgs/Path` built
from the task's waypoint list) that nothing currently publishes.
The cardinal-marker/buoy panels (3D + Map) and the three Mission panels
read topics produced by `gui_markers.py` / `competition_manager` /
`mission_manager` directly — see the "GUI Markers Bridge" section below,
**UNVERIFIED against real hardware**, sim-tested only so far.

## How to use it

1. Start the boat stack: `ros2 launch bringup njord.launch.py`.
2. Start the BMS reader: `ros2 run sensors bms_reader`.
3. Start the telemetry helper (it produces the gauge + status topics):
   `python3 src/foxglove/gui_telemetry_hw.py`.
4. Start the markers helper (it produces the cardinal-marker + mission
   topics): `python3 src/foxglove/gui_markers.py`.
5. In Foxglove, connect to `ws://<pi-ip>:8765` and import
   `src/foxglove/ASKET_GUI_mandatory.json`.
6. All thirteen panels should populate.

## Limitations

1. **`gui_telemetry_hw.py`'s COG (course-over-ground) reading is UNVERIFIED
   against real hardware.** It assumes MAVROS's usual ENU convention
   (x=East, y=North) applies to `/mavros/global_position/raw/gps_vel`. If COG
   ever reads a fixed 90° off the compass heading while driving straight,
   that's the signature of this actually being NED on this topic — swap
   east/north in `on_vel()`. Heading, speed, battery %, and status don't have
   this risk (more direct field reads).
2. **Everything `gui_telemetry_hw.py` produces depends on MAVROS being
   connected.** If `/mavros/state.connected` is false, the status bar reads
   OUT_OF_CONTROL and battery/heading are blank. Fix the FCU link first.
3. **Battery reads 0% until the ArduPilot battery monitor is configured.** 0%
   here means "not set up," not "empty." The BMS panel's own pack voltage is
   unaffected by this — it reads directly from the BQ76920, not through
   ArduPilot.
4. **The LiDAR panel needs a valid `base_link` TF frame.** If the TF tree does
   not publish `base_link`, the panel shows an empty grid.
5. **The camera topic is `/front_camera_driver/image_raw` only when launched via
   `njord.launch.py`** (which remaps it). Run the camera standalone and it
   publishes `/image_raw` instead.
6. **No single connected route-comparison line.** The map now plots individual
   planned-waypoint pins (amber) alongside the boat's live GPS trail (blue),
   but there's no connected "ideal route" polyline — see the note under
   "What it does" above.
7. **`gui_markers.py`'s class legend isn't documented anywhere else in the
   repo.** It's pulled straight from the vision model's embedded Ultralytics
   metadata (`strings models/yolo26n-seg-navier.onnx | grep names`):
   `{0: 'green', 1: 'red', 2: 'north', 3: 'east', 4: 'south', 5: 'west'}`.
   If the model is ever retrained/re-exported with a different class order,
   `CLASS_COLORS`/`CLASS_LABELS` in `gui_markers.py` need updating to match.
8. **The 2D Map panel's detected-marker dots (`/gui/markers/marker_0..11`)
   are a single fixed color (white), not per-class.** Foxglove's Map panel
   assigns one color per topic, set at layout-authoring time — it can't
   recolor per-message. The 3D LiDAR panel's `/gui/cardinal_markers_3d` is
   the color-accurate view (green/red buoys, colored cardinal marks); the
   map dots are redundant position confirmation only.
9. **A Map-panel marker slot's last position can linger briefly after that
   obstacle drops out of tracking**, since `sensor_msgs/NavSatFix` has no
   "clear" equivalent to `visualization_msgs/Marker`'s `DELETEALL` (which
   the 3D panel uses, so it's always current). Cosmetic only.
10. **`gui_markers.py` and its panels are sim-tested only, UNVERIFIED against
    real hardware.**

## Mandatory GUI requirements (Njord 2026, section 11.4)

The competition requires the GUI to be operable by someone with no software
background, and understandable by a jury member with no explanation. It must
display the following data:

| # | Required data                                            | Met by this layout |
| - | -------------------------------------------------------- | ------------------ |
| 1 | Camera feed / LIDAR                                       | Yes                |
| 2 | Latitude                                                 | Yes                |
| 3 | Longitude                                                | Yes                |
| 4 | Heading                                                  | Yes |
| 5 | Course over ground (COG) with trail, vs plot of the ideal route from GNSS points | Partial — COG value shown (frame convention unverified, see Limitations); live trail + planned-waypoint pins now on the map, but no single connected "ideal route" line |
| 6 | Speed over ground                                        | Yes |
| 7 | Battery life in %                                        | Yes (via `gui_telemetry_hw.py`) — plus the separate BMS panel's real pack voltage, independent of ArduPilot's battery monitor being configured |
| 8 | Status indicator (autonomous / remote / standby / out of control) | Yes |

Nice-to-have (not required):

1. Distance between ASV and next waypoint.
2. Battery life in Wh remaining.
3. ~~Any other parameters that help the jury understand the ASV.~~ Added: BMS
   pack voltage/per-cell voltage/temp/current panel; Mission/Mission
   State/Mission Progress panels; detected cardinal markers/buoys (3D +
   Map panels).


## GUI Telemetry Bridge (`gui_telemetry_hw`)

### What this adds

Feeds the mandatory Foxglove GUI (`ASKET_GUI_mandatory.json`) with **real boat
data**. It reads live values from the Pixhawk (via MAVROS) and republishes them
under the simple topic names the GUI gauges read. Nothing is simulated.

It fills the four gauges + status bar that would otherwise stay empty:

| GUI element        | Publishes            | Type                | Source (MAVROS)                       |
| ------------------ | -------------------- | ------------------- | ------------------------------------- |
| Heading gauge      | `/heading`           | `std_msgs/Float64`  | `/mavros/global_position/compass_hdg` |
| COG gauge          | `/cog`               | `std_msgs/Float64`  | `/mavros/global_position/raw/gps_vel` |
| SOG gauge          | `/sog`               | `std_msgs/Float64`  | `/mavros/global_position/raw/gps_vel` |
| Battery gauge      | `/battery_percentage`| `std_msgs/Float64`  | `/mavros/battery`                     |
| Status indicator   | `/vehicle/status`    | `std_msgs/String`   | `/mavros/state` (flight mode)         |

Status values: `AUTONOMOUS`, `REMOTE`, `STANDBY`, `OUT_OF_CONTROL`, derived
from ArduPilot Rover's mode string (Asket is `FRAME_CLASS=2`/Boat, which
runs Rover firmware) — `MANUAL`/`LEARNING`/`STEERING` → REMOTE,
`HOLD`/`INITIALISING` → STANDBY, `AUTO`/`GUIDED`/`RTL` → AUTONOMOUS (GUIDED
matters most: that's the mode `mission_manager`'s Nav2 goals actually run
the boat in), any other/unknown mode → STANDBY (safe default, never
falsely claims AUTONOMOUS), not connected → OUT_OF_CONTROL regardless of
mode. Verified against `mavros_msgs/State`'s real `MODE_APM_ROVER_*`
enum, not yet against the physical Pixhawk's reported mode strings.

### Requirements

1. The main stack running (`ros2 launch bringup njord.launch.py`), which starts MAVROS.
2. MAVROS connected to the Pixhawk (`/mavros/state.connected = true`).
3. `mavros_msgs` installed (already present, since MAVROS runs).

### How to run it

This is a standalone script, not part of a colcon package — there's no
`ros2 run` form for it, run it with `python3` directly:

```bash
python3 src/foxglove/gui_telemetry_hw.py
```

Run it in a second terminal after the main stack is up (or, on the deployed
Jetson, `podman exec -it njord bash -c "source /opt/ros/jazzy/setup.bash && source /opt/njord/setup.bash && python3 src/foxglove/gui_telemetry_hw.py"` — see the main README's SSH section). Leave it running.

### How to check it works

1. Confirm the five topics now exist:
   ```bash
   ros2 topic list | grep -E "/heading|/cog|/sog|/battery_percentage|/vehicle/status"
   ```
2. Confirm real values are flowing:
   ```bash
   ros2 topic echo /heading --once
   ros2 topic echo /vehicle/status --once
   ```
3. Open the GUI in Foxglove and import `ASKET_GUI_mandatory.json`. The four
   gauges should show needles and the status bar should show the current mode.

### Troubleshooting

1. **Status shows `OUT_OF_CONTROL` and gauges are empty**
   MAVROS is not talking to the Pixhawk. Check:
   ```bash
   ros2 topic echo /mavros/state --once
   ```
   If `connected: false`, fix the `fcu_url` in `njord.launch.py` (it is set to
   `tcp://localhost:5777`; on the real boat this must point at the Pixhawk
   serial device, e.g. `/dev/ttyACM0:57600`). Nothing here works until
   `connected: true`.

2. **Battery reads 0%**
   The ArduPilot battery monitor is not configured yet. `BatteryState.percentage`
   returns `-1` (unknown) until it is set up in QGC/params; the node maps that to
   0%. So 0% means "not configured," not "empty."

3. **Heading looks wrong**
   `/heading` is taken straight from the Pixhawk compass. If it is off, the fix
   is compass calibration in QGC, not this node.

4. **COG reads a fixed ~90° off the compass heading while driving straight**
   This is the signature of `/mavros/global_position/raw/gps_vel` actually
   being NED rather than the assumed ENU on this specific topic — see
   Limitations above. Swap `east`/`north` in `on_vel()`.

### Optional: run it automatically with the stack

It's a standalone script (no colcon package), so the simplest way to run it
alongside everything else is a second `ExecuteProcess`/`Node`-style entry in
`bringup/launch/njord.launch.py` invoking `python3` directly on this file's
path, or moving it into the `sensors` package as a proper console-script
entry point (matching `bms_reader`'s pattern) if it needs `mavros_msgs` as a
declared dependency. Not required for testing.

## GUI Markers Bridge (`gui_markers`)

### What this adds

Feeds the mandatory Foxglove GUI with cardinal-marker/buoy detections and
mission name/state/progress. Reads `/obstacles/global`
(`njord_msgs/ObstacleArray`, already fused + tracked by
`src/fusion/fusion/geo_fusion_node.py` from vision + lidar) and republishes
it as GUI-friendly topics. `/competition/status` and `/mission/status`
(already published by `competition_manager`/`mission_manager`) are read
directly by the layout's Indicator/RawMessages panels — `gui_markers.py`
doesn't touch those, only the obstacle republishing.

| GUI element                | Publishes                        | Type                              | Source                    |
| --------------------------- | --------------------------------- | ---------------------------------- | -------------------------- |
| 3D LiDAR panel markers      | `/gui/cardinal_markers_3d`        | `visualization_msgs/MarkerArray`   | `/obstacles/global`        |
| Map panel marker dots       | `/gui/markers/marker_0..11`       | `sensor_msgs/NavSatFix` (x12)      | `/obstacles/global`        |

Class legend (`CLASS_COLORS`/`CLASS_LABELS` in `gui_markers.py`), pulled
from the vision model's embedded metadata, not invented:
`{0: 'green', 1: 'red', 2: 'north', 3: 'east', 4: 'south', 5: 'west'}`.
Obstacles with `class_id == "unknown"` (lidar-only, no vision confirmation)
render gray, labelled "unknown".

The 3D markers use each obstacle's boat-relative `range_m`/`bearing_deg`
(already computed by the fusion node's tracker) to place a colored cylinder
+ text label directly in the `base_link` frame — no coordinate-frame
conversion needed. The Map-panel dots use each obstacle's absolute
`position` (WGS84), same as the existing waypoint pins.

### Requirements

1. The main stack running, with `/obstacles/global` publishing — i.e. both
   `src/vision/vision/node.py` and `src/fusion/fusion/geo_fusion_node.py`
   (or the sim equivalent) up and fusing detections.

### How to run it

Standalone script, same as `gui_telemetry_hw.py` — no `ros2 run` form:

```bash
python3 src/foxglove/gui_markers.py
```

### How to check it works

1. Confirm the topics exist once an obstacle is tracked:
   ```bash
   ros2 topic echo /gui/cardinal_markers_3d --once
   ros2 topic echo /gui/markers/marker_0 --once
   ```
2. Open the GUI in Foxglove: colored cylinders + labels should appear in
   the 3D LiDAR panel where LiDAR sees a tracked marker; white dots should
   appear at the same location on the Map panel; the Mission/Mission
   State/Mission Progress panels should update as a task runs.
