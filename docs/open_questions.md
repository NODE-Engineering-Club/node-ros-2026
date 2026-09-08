# Open questions

The architecture brief listed six; three more surfaced during implementation.

**Most are now answered.** Q3 and Q8 changed the design and are written up
below. What remains provisional is measurement — the survey area and the
mounting geometry — plus the `pico_bridge` interface.

Anything still standing on a provisional value is marked `PROVISIONAL` in the
config file that holds it, so it is greppable:

```bash
grep -rn PROVISIONAL src/
```

Nothing provisional is baked into code — it lives in YAML and can be changed
without a rebuild.

| # | Question | Status | Where |
|---|---|---|---|
| 1 | Survey area depth and profile in Namibia (harbour vs open coast) | **Stays provisional — will be tuned on site.** 30 m sonar range, 8/15 m lidar alarm/warn radii. Sonar range, gain and rate are adjustable **at runtime from the GUI**, not only at launch. | `src/asket_bringup/config/mission_defaults.yaml`, GUI sonar panel |
| 2 | Sonar mounting angle and lever arm from the GNSS antenna | **Must be physically measured; still unknown.** 35° down, offset (x 0.20 m aft, y 0.35 m starboard, z 0.15 m below waterline). The pre-flight check returns **WARN** while the file is still marked `PROVISIONAL`, so it cannot be deployed on defaults by accident. | `src/omniscan_bridge/config/mounting.yaml` |
| 3 | Can SonarView import an externally recorded trajectory to georeference a raw log? | **ANSWERED: no.** Position and heading must be inside the `.svlog` as `NMEA_WRAPPER` packets written at capture time; a log without them cannot be georeferenced or exported at all. Option B survives — the recorder still writes the two streams separately — and `mission_recorder.merge_svlog` merges them afterwards into a valid `.svlog` with `$GPGGA`/`$GPHDT` interleaved on `utc_msec`. Optional live interleaving is available as `write_live_svlog`. | `src/mission_recorder/core/svlog.py` |
| 4 | UM982 purchase confirmed? Antenna baseline length? | **Purchase decision pending; 1.0 m baseline confirmed as the right assumption.** The panel reads the *reported* accuracy from MAVROS whenever one is available rather than displaying a constant. | `src/asket_bringup/config/mission_defaults.yaml` |
| 5 | Mission duration target | **3 h confirmed** for disk sizing. Note that with a single sonar unit covering one side only, **battery endurance binds before disk does** — which is what the power panel's endurance-versus-survey comparison is for. | `src/asket_bringup/config/mission_defaults.yaml` |
| 6 | Shore-based or from a support vessel? | **Shore-based, confirmed.** Thresholds stay in YAML: a support-vessel deployment would have a very different link profile. | `src/gui_backend/config/link_profiles.yaml` |

## What the Cerulean documentation corrected (Q8)

The layouts transcribed from the brief were wrong in ways that would have
produced plausible-looking nonsense. All are now taken from the published
definitions.

| | Transcribed from the brief | Actually |
|---|---|---|
| `OS3D_POINT_SET` header | 20 bytes: `u32 ping, float sos, u64 utc, u32 count` | **80 bytes**, `num_points` is a `u16` at offset 8, and it carries `pwr_up_msec`, `version`, `device_number` and three power thresholds |
| `pt_type` | 0 = no return, 1 = bottom | **0 = unclassified, 1 = bottom, 2 = water column.** Only bottom points belong in bathymetry — a fish is not seabed |
| `ATTITUDE_REPORT` | `float pitch, float roll` | **An up vector** (`vec3`) in the device frame, plus `utc_msec`, `pwr_up_msec`, `channel_number`. Angles are derived from it |
| `END_PING_INFO` | `ping_number, num_results, duration` | **80 bytes**, including `ping_hz_realized` — the rate the device actually achieved, which is better than timing arrivals here |
| `OS3D_SET_PING_PARAMETERS` | `u32 range_mm, u8 gain, float rate_hz` | **36 bytes**: `start_m`/`end_m` floats, `gain_index` **i16 where −1 is auto**, `msec_per_ping` (a period, not a rate), and a `ping_enable` flag. `enable_atof_data` **must be true or no points are produced at all** |
| `SET_NTP_URL` (18) | Sent at startup to point the sonar at our NTP server | **The packet was removed.** NTP is configured on the device and defaults to an internet host. See `docs/SETUP.md` — this is a field setup step the bridge cannot perform |
| Frame layout | Transcribed from the brief | **Confirmed correct**, byte for byte |

Sources: the Cerulean Ping Protocol "Universal Packet Format", "Index of packet
types", and the Omniscan 3D API pages, fetched September 2026.

## Added during implementation

| # | Question | Why it matters | Interim handling |
|---|---|---|---|
| 7 | What message type does the existing `pico_bridge` publish, and on what topic? (`ros2 topic info -v` output pending) | `gui_backend` must read confirmed vessel mode from it, and the brief forbids modifying `pico_bridge`. | Logical streams are mapped to topics/types/adapters in `src/gui_backend/config/topics.yaml`. In sim we publish `asket_interfaces/msg/PicoStatus`; pointing the adapter at the real message is a config change plus one adapter function. |
| 8 | Exact byte layout of `OS3D_POINT_SET` (3104), `ATTITUDE_REPORT` (504) and `END_PING_INFO` (3010) payloads. | The brief described the fields but not their order or widths. | **ANSWERED from Cerulean's published documentation, and the transcribed guesses were wrong.** All layouts are now taken from docs.ceruleansonar.com and cited in `ping_protocol.py`. See the table below for what changed. Two things the docs do not state remain assumptions: **endianness** (little-endian assumed) and **`vec3`** (three `float`, the only reading consistent with the fixed payload sizes). Both would fail loudly rather than silently. |
| 9 | Which Jetson mount point(s) count as the "USB fast path" for export? | **A headless Jetson has no desktop session, so nothing auto-mounts and `/media/*` is empty.** Detection also has to test the actual mount rather than the directory's existence, or it will offer the eMMC. | udev rule and systemd unit in `deploy/`, documented in `docs/SETUP.md`; detection verifies the mount. |
