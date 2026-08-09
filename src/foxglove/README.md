# Foxglove
Everything to do with the Graphic User Interface (GUI)

# ASKET 2.0 GUI — Mandatory Layout (`ASKET_GUI_mandatory.json`)

## What it is

A Foxglove Studio layout that shows the jury the minimum required information
about the ASV during a task run. Import it into Foxglove
(Layout menu → Import from file) while connected to the boat's foxglove_bridge
(`ws://<pi-ip>:8765`).

## Where these files live in the repo

1. `gui/ASKET_GUI_mandatory.json` — this layout.
2. `gui/README_ASKET_GUI_mandatory.md` — this file.
3. `src/sensors/sensors/gui_telemetry_hw.py` — the telemetry helper node that
   feeds the gauges + status bar (see its own README).

## What it does

It arranges nine panels into a single jury-readable screen:

| Panel               | Shows                          | Topic it reads                 | Data source                     |
| ------------------- | ------------------------------ | ------------------------------ | ------------------------------- |
| LiDAR (3D)          | Live 2D LiDAR scan             | `/lidar_driver/scan_raw`       | `lidar_driver` (real hardware)  |
| RGB Front Camera    | Front camera feed              | `/front_camera_driver/image_raw` | `camera_driver` (real hardware) |
| Map                 | Boat position on a real map    | `/gps_driver/gps_raw`          | `imu_gps_driver` GPS            |
| Latitude / Longitude| Numeric GPS readout            | `/gps_driver/gps_raw`          | `imu_gps_driver` GPS            |
| ASV STATUS          | Auto / Remote / Standby / Out of control | `/vehicle/status`    | `gui_telemetry_hw` (from MAVROS)|
| Heading gauge       | Compass heading (deg)          | `/heading`                     | `gui_telemetry_hw` (from MAVROS)|
| COG gauge           | Course over ground (deg)       | `/cog`                         | `gui_telemetry_hw` (from MAVROS)|
| Speed gauge         | Speed over ground (kn)         | `/sog`                         | `gui_telemetry_hw` (from MAVROS)|
| Battery gauge       | Battery remaining (%)          | `/battery_percentage`          | `gui_telemetry_hw` (from MAVROS)|

The camera, LiDAR, and GPS panels read the boat's real driver topics directly.
The four gauges and the status bar read topics produced by the helper node
`gui_telemetry_hw.py`, which republishes real MAVROS data under those names.

## How to use it

1. Start the boat stack: `ros2 launch bringup njord.launch.py`.
2. Start the telemetry helper (it produces the gauge + status topics):
   `ros2 run sensors gui_telemetry_hw`.
3. In Foxglove, connect to `ws://<pi-ip>:8765` and import `gui/ASKET_GUI_mandatory.json`.
4. All nine panels should populate.

## Limitations

1. **The four gauges + status bar need `gui_telemetry_hw.py` running.** Without
   it, `/heading`, `/cog`, `/sog`, `/battery_percentage`, `/vehicle/status` do
   not exist and those panels stay empty.
2. **Everything depends on MAVROS being connected.** If `/mavros/state.connected`
   is false, the status bar reads OUT_OF_CONTROL and battery/heading are blank.
   Fix the FCU link first.
3. **Battery reads 0% until the ArduPilot battery monitor is configured.** 0%
   here means "not set up," not "empty."
4. **The LiDAR panel needs a valid `base_link` TF frame.** If the TF tree does
   not publish `base_link`, the panel shows an empty grid.
5. **The camera topic is `/front_camera_driver/image_raw` only when launched via
   `njord.launch.py`** (which remaps it). Run the camera standalone and it
   publishes `/image_raw` instead.
6. **No route comparison line.** The layout shows the boat's live position on the
   map, but does NOT draw the travelled track against the ideal GNSS route.

## Mandatory GUI requirements (Njord 2026, section 11.4)

The competition requires the GUI to be operable by someone with no software
background, and understandable by a jury member with no explanation. It must
display the following data:

| # | Required data                                            | Met by this layout |
| - | -------------------------------------------------------- | ------------------ |
| 1 | Camera feed / LIDAR                                       | Yes                |
| 2 | Latitude                                                 | Yes                |
| 3 | Longitude                                                | Yes                |
| 4 | Heading                                                  | Yes                |
| 5 | Course over ground (COG) with trail, vs plot of the ideal route from GNSS points | Partial — COG value shown; trail + ideal-route overlay not included in this layout |
| 6 | Speed over ground                                        | Yes                |
| 7 | Battery life in %                                        | Yes                |
| 8 | Status indicator (autonomous / remote / standby / out of control) | Yes       |

Nice-to-have (not required, not included):

1. Distance between ASV and next waypoint.
2. Battery life in Wh remaining.
3. Any other parameters that help the jury understand the ASV.


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

Status values: `AUTONOMOUS`, `REMOTE`, `STANDBY`, `OUT_OF_CONTROL`.

### Requirements

1. The main stack running (`ros2 launch bringup njord.launch.py`), which starts MAVROS.
2. MAVROS connected to the Pixhawk (`/mavros/state.connected = true`).
3. `mavros_msgs` installed (already present, since MAVROS runs).

### How to run it

Standalone (quickest, for testing):

```bash
python3 gui_telemetry_hw.py
```

Run it in a second terminal after the main stack is up. Leave it running.

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

### Optional: run it automatically with the stack

To launch it alongside everything else, add it as a node in
`bringup/launch/njord.launch.py` (and add `mavros_msgs` to the owning package's
`package.xml`). Not required for testing.
