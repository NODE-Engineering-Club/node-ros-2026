# asket_sim

Simulated data sources for the entire Asket stack.

**This is a permanent feature, not scaffolding.** It exists now because no
hardware is connected, and it will still be here afterwards: for development
without a boat, for regression tests, and eventually for replaying recorded
missions.

## What it simulates

| Source | Behaviour |
|---|---|
| Vessel | Follows a lawnmower pattern at survey speed, turn-rate limited, in a cross-current, so COG genuinely diverges from heading |
| Heading | Magnetometer model with a **throttle-dependent error** (the real failure mode on this hull), or a GNSS-compass model for comparison |
| Lidar | 360 degree scan against static obstacles, with roll-driven wave clutter and dropouts |
| Sonar | Synthetic seabed; emits angle/time-of-flight in the sensor frame, never XYZ |
| Battery | Cubic propulsion draw, endurance from a 60 s average |
| Pico | Mode state machine with a **confirmation delay**, plus a killswitch and RC channel 8 that software can observe but not move |
| Link | Quality falling off with distance from the shore station, with fading |

## The important design decision

The sonar is not injected into `omniscan_bridge` as an object. It is served as
**real Ping Protocol frames on a real UDP socket** by `fake_sonar_node`. So
`omniscan_bridge` is byte-for-byte identical in sim and in the field, and the
socket handling, resync and packet-loss accounting are genuinely exercised
rather than bypassed by a second code path that nobody tests.

## Running it

Whole system, no hardware:

```bash
ros2 launch asket_bringup sim.launch.py
```

Just the fake sonar, no ROS at all:

```bash
python3 -m asket_sim.core.fake_sonar_server --port 62312
```

A synthetic `sonar_raw.bin` for parser work:

```bash
python3 -m asket_sim.make_sonar_raw --duration 60 --out /tmp/sonar_raw.bin
python3 -m asket_sim.make_sonar_raw --duration 60 --corrupt --out /tmp/damaged.bin
```

## Injecting faults

The whole point. Faults are named; an unknown name is rejected loudly rather
than silently ignored.

```bash
ros2 topic pub --once /sim/inject_fault std_msgs/String '{data: "heading_invalid"}'
ros2 topic pub --once /sim/inject_fault std_msgs/String '{data: "link_degraded:30"}'   # 30 s
ros2 topic pub --once /sim/clear_fault  std_msgs/String '{data: "all"}'
ros2 topic echo /sim/faults
```

| Fault | What it proves |
|---|---|
| `link_loss`, `link_degraded` | The GUI stays usable and honest about data age on a bad link. With `--shape-link` the wire itself narrows, so this is a real test rather than a relabelling |
| `sonar_dropout`, `sonar_packet_loss` | Sonar health panel and alarms react |
| `clock_drift` | The operator is warned **during** the mission, before a whole dataset is quietly ruined |
| `heading_invalid` | Warning raised, and the compromised window is recorded per sample |
| `gnss_degraded` | Pre-flight check produces a plain-language, actionable message |
| `disk_full` | Recording alarms and stops cleanly |
| `rc_link_loss`, `battery_fault`, `lidar_stall` | Diagnostics and alarms |

## Testing

```bash
pytest src/asket_sim          # no ROS required
```
