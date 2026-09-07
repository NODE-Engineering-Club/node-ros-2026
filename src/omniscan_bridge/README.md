# omniscan_bridge

Cerulean Omniscan 3D → ROS 2. Speaks the Ping Protocol over Ethernet
(port 62312), publishes vessel-frame points and a health topic that tells you
what the sonar is actually doing.

The unit is the **AIO variant**: beamforming happens inside the sonar's own
Pi 5, so the Jetson receives processed data. One unit, so **one side only** —
which is why coverage gaps are easy to leave and hard to notice.

## Published

| Topic | Type | Rate |
|---|---|---|
| `/sonar/points` | `sensor_msgs/PointCloud2` | up to 20 Hz, vessel frame, **not georeferenced** |
| `/sonar/status` | `asket_interfaces/SonarStatus` | 1 Hz |
| `/sonar/attitude` | `sensor_msgs/Imu` | as received |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz |

## Services

* `~/set_ping_parameters` — range, gain, ping rate
* `~/start_pinging`, `~/stop_pinging`

## What it does not do

**It does not georeference.** Points come out in the vessel frame. Position and
heading are recorded alongside the raw stream by `mission_recorder` and merged
afterwards on `utc_msec` (brief, section 5). Fusing here would make the point
cloud depend on GNSS availability, so a fix dropout would corrupt the data
rather than merely annotating it.

## The parser

`core/ping_protocol.py` is the codec — frame layout, checksums, payloads. It has
no ROS dependency and no third-party dependency, and the binary layout is
written out explicitly in the module docstring because it is meant to be read.

`core/parser.py` is what survives a real link: partial frames held until
complete, corrupt frames rejected with resynchronisation on the next marker,
truncation counted, and ping-number gaps measured so `packet_loss_ratio` is a
measurement rather than a guess.

> ⚠️ The **frame** layout is the documented, stable Ping Protocol header. The
> **payload** layouts are transcribed from the architecture brief, which names
> the fields but not, in every case, their order and width. They are written as
> explicit `struct` tables so reconciling them with Cerulean's real definition
> is a one-table edit. The parser cross-checks the declared point count against
> the payload length and flags a mismatch, so a wrong assumption fails loudly
> rather than producing a plausible cloud of nonsense.
> **Validate against Cerulean sample data before the first field deployment.**
> Tracked as Q8 in `docs/open_questions.md`.

## The conversion

`core/geometry.py`. The sonar reports an angle and a time of flight; turning
that into a point is ours to do:

```
range = tof * speed_of_sound / 2
y     = range * sin(angle)        # sensor frame, lateral
z     = range * cos(angle)        # sensor frame, along the boresight
```

then mounting tilt → mounting yaw/pitch → lever arm from the GNSS antenna →
vessel attitude. Output is REP-103: x forward, y port, z up.

Two details worth knowing:

* The **speed of sound travels with every ping** and is used as reported.
  Assuming 1500 m/s when the sonar used 1520 puts a 20 m bottom 27 cm out,
  systematically, across the whole survey.
* The **lever arm** from the GNSS antenna to the transducer must be measured to
  the centimetre. It is a systematic offset that no post-processing will find.

## The clock

`clock_offset_ms` — the sonar's `utc_msec` against the Jetson's clock — matters
more than it looks. The whole post-mission fusion strategy rests on it. If it
drifts and nobody notices, the survey is not degraded, it is worthless, and
nobody finds out until the data is opened back home. So it is measured
continuously (as a **median**, so one late packet cannot cry wolf), surfaced in
`SonarStatus`, and escalated to an alarm in `/diagnostics`.

The bridge sends `SET_NTP_URL` at startup **and after every reconnect** — a
device that reboots mid-mission comes back with defaults, and silently
surveying at the wrong range for the second half is exactly the failure nobody
notices in time.

## Testing in sim

No sonar, no socket, no ROS:

```bash
pytest src/omniscan_bridge
```

With a fake device on a real socket:

```bash
python3 -m asket_sim.core.fake_sonar_server --port 62312 &
ros2 run omniscan_bridge omniscan_bridge_node --ros-args -p host:=127.0.0.1
```

Against a synthetic raw file:

```bash
python3 -m asket_sim.make_sonar_raw --duration 60 --corrupt --out /tmp/damaged.bin
```

`test/test_end_to_end.py` runs that whole path — synthetic stream in,
vessel-frame point clouds and health out — including the damaged-stream case.
