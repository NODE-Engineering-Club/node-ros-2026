# Njord Mission Panel — Foxglove extension

A [Foxglove Studio](https://foxglove.dev/) panel that provides the operator
interface for the Njord 2026 ASV (**Asket**): a live map, GPS waypoint planning,
and mission control.

## What it does

- **Map view** — a Leaflet map (OpenStreetMap tiles) centred on the boat's live
  GPS position with a ~1 km view. The boat is shown as a marker and the map
  follows new GPS fixes.
- **Waypoint management** — enter waypoints by latitude/longitude (decimal
  degrees) with an optional label. Each waypoint appears as a numbered marker,
  connected in order by a mission-path polyline. Waypoints can be deleted
  individually or cleared all at once.
- **Mission control** — **Launch Mission** publishes the ordered waypoints to the
  boat; **Stop Mission** aborts a running mission.
- **Mission status** — a colored indicator reflects the live mission state:
  idle (gray), running (green, pulsing), completed (blue), aborted (red).

## Prerequisites

- **Foxglove Studio** (desktop app recommended for installing local extensions).
- **`foxglove_bridge` running on the boat.** The full stack launches it by
  default on port **8765** (`enable_foxglove:=true` in `bringup/njord.launch.py`).
  In Foxglove, connect via *Open connection → Foxglove WebSocket* to
  `ws://<boat-host>:8765` (e.g. `ws://boat.local:8765`).
- Publishing waypoints and calling the abort service require a **live, writable
  connection** (the Foxglove WebSocket bridge). These actions do not work against
  a recorded data source (MCAP/bag).
- Internet access for OpenStreetMap tiles. Without it the map background is blank,
  but the boat marker, waypoints, and path still render.

## Build

From this directory (`foxglove-mission-panel/`):

```bash
npm install        # install dependencies (first time only)
npm run build      # type-check + bundle into dist/
npm run package    # produce the installable .foxe file
```

`npm run package` writes a file like
`node-engineering-club.njord-mission-panel-0.1.0.foxe` in this directory.

> From the repository root you can run everything in one step — see
> [Building from the repo root](#building-from-the-repo-root).

## Install the `.foxe` in Foxglove Studio

**Option A — drag and drop:** open Foxglove Studio and drag the generated
`.foxe` file onto the application window.

**Option B — local install during development:** `npm run local-install` builds
and installs the extension into your local Foxglove (`~/.foxglove-studio/extensions`).
Restart Studio afterwards.

Once installed, add the panel: in a layout, click *Add panel* and choose
**Njord Mission**.

## ROS 2 interface

The panel talks to the boat through the `foxglove_bridge`. It uses these topics
and services:

| Direction | Name | Type | Purpose |
|---|---|---|---|
| Subscribe | `/gps_driver/gps_raw` | `sensor_msgs/msg/NavSatFix` | Boat position used to center the map and place the boat marker. |
| Subscribe | `/mission/status` | `std_msgs/msg/String` | Mission state: `idle` / `running` / `completed` / `aborted`. |
| Publish | `/mission/waypoints` | `geometry_msgs/msg/PoseArray` | Ordered mission waypoints. Geographic convention: `position.x = longitude`, `position.y = latitude`, `position.z = 0`, `header.frame_id = "wgs84"`. |
| Service call | `/mission/abort` | `std_srvs/srv/Trigger` | Stop / abort the active mission. |

The boat-side `mission_manager` node consumes `/mission/waypoints` (auto-starting
the mission), converts each pose to a `geographic_msgs/GeoPoint`, and drives Nav2.
It publishes `/mission/status` and serves `/mission/abort`.

## Development notes

- `src/MissionPanel.tsx` — the panel (UI, Leaflet lifecycle, ROS wiring).
- `src/ros/messages.ts` — ROS 2 message types and the waypoints → `PoseArray`
  serializer.
- `npm run lint` runs a type-only check (`tsc --noEmit`).
