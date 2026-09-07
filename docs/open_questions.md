# Open questions

The architecture brief listed six. They are unanswered as of this
implementation. Rather than block the whole build, each has a **provisional**
value recorded in a config file, marked `PROVISIONAL` so it is greppable:

```bash
grep -rn PROVISIONAL src/
```

Nothing provisional is baked into code — it lives in YAML and can be changed
without a rebuild.

| # | Question | Provisional value | Where |
|---|---|---|---|
| 1 | Survey area depth and profile in Namibia (harbour vs open coast) — drives sonar range settings and lidar alarm distances | 30 m sonar range, 8/15 m lidar alarm/warn radii, seabed 12–35 m | `src/asket_bringup/config/mission_defaults.yaml` |
| 2 | Sonar mounting angle and lever arm from the GNSS antenna | 35° down, offset (x 0.20 m aft, y 0.35 m starboard, z 0.15 m below waterline) | `src/omniscan_bridge/config/mounting.yaml` |
| 3 | Can SonarView import an externally recorded trajectory to georeference a raw log? | Assumed **yes** (this is what Option B in the brief depends on). The recorder additionally writes `trajectory.jsonl` in a shape a fallback fusion script can consume, so a "no" answer costs a script, not a re-record. | `src/mission_recorder/README.md` |
| 4 | UM982 purchase confirmed? Antenna baseline length? | 1.0 m baseline assumed for the accuracy figure shown in the heading panel; the panel reads the *reported* accuracy when MAVROS provides one | `src/asket_bringup/config/mission_defaults.yaml` |
| 5 | Mission duration target | 3 h — drives the "estimated remaining recording time" and disk warnings | `src/asket_bringup/config/mission_defaults.yaml` |
| 6 | Shore-based or from a support vessel? | Shore-based; link profile thresholds tuned for a degrading WiFi link with 4G fallback | `src/gui_backend/config/link_profiles.yaml` |

## Added during implementation

| # | Question | Why it matters | Interim handling |
|---|---|---|---|
| 7 | What message type does the existing `pico_bridge` publish, and on what topic? | `gui_backend` must read confirmed vessel mode from it, and the brief forbids modifying `pico_bridge`. | Logical streams are mapped to topics/types/adapters in `src/gui_backend/config/topics.yaml`. In sim we publish `asket_interfaces/msg/PicoStatus`; pointing the adapter at the real message is a config change plus one adapter function. |
| 8 | Exact byte layout of `OS3D_POINT_SET` (3104), `ATTITUDE_REPORT` (504) and `END_PING_INFO` (3010) payloads. | The brief describes the fields but not their order, widths or units for every one. | The parser encodes the layout as an explicit, commented table in `src/omniscan_bridge/core/ping_protocol.py` (`_POINT_SET_HEADER`, etc.). If Cerulean's real layout differs it is a one-table change; the parser cross-checks the declared point count against the payload length and warns on mismatch, so a wrong layout fails loudly rather than silently producing plausible garbage. **Validate against Cerulean sample data before the first field deployment.** |
| 9 | Which Jetson mount point(s) count as the "USB fast path" for export? | Export detection must not offer a path that is actually the Jetson's own eMMC. | `export.fast_paths` in `src/mission_recorder/config/recorder.yaml`, default `/media/*`, `/mnt/usb*`. |
