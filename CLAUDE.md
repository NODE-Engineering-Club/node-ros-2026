# CLAUDE.md — Njord 2026

Working notes for anyone (human or agent) touching this repository.
Keep this file current as the architecture evolves.

## What this repository is

The NODE Engineering Club's ROS 2 Jazzy workspace for the **Asket** ASV. It now
holds two bodies of work in one colcon workspace:

- **The competition stack** — `description`, `sensors`, `perception`, `fusion`,
  `control`, `mission`, `vision`, `calibration`, `boat_bt`,
  `competition_manager`, `njord_msgs`, `bringup`. `README.md` is the reference
  for all of it and is the document to read first.
- **The mission GUI** — `asket_interfaces`, `asket_sim`, `omniscan_bridge`,
  `mission_recorder`, `system_test`, `gui_backend`, `asket_gui`. Built for the
  **Namibia seabed survey** with a Cerulean Omniscan 3D. Everything below in
  this file is about that half.

The GUI half was developed as a separate overlay and merged in with its history
intact. It is **additive** apart from two files in the competition stack: a
single `enable_gui` flag in `bringup/launch/njord.launch.py`, defaulting to
`false`, and a fix to `control/control/pico_bridge.py` which was filtering serial
lines for a prefix the firmware does not write. Both are described under
"Boundaries" and in `INTEGRATION_STATUS.md` §3.

## What it is *not*

- Not a sonar analysis tool. No point-cloud rendering, meshing or bathymetric
  processing in the GUI. That is SonarView's job, post-mission.
- Not part of the safety chain. See `docs/safety.md`.

## Layout

```
src/
  asket_interfaces/   msg/srv definitions shared by everything below
  asket_sim/          simulated data sources (stage 0) — a permanent feature
  omniscan_bridge/    Cerulean Ping Protocol -> ROS 2
  mission_recorder/   mission directories, trajectory/diagnostics/event logs, export
  system_test/        passive + active built-in test, GO/NO-GO verdict
  gui_backend/        FastAPI + WebSocket, subscription & bandwidth negotiation
  asket_gui/          React + MapLibre frontend, served by gui_backend
  asket_bringup/      launch files (`sim:=true` switches the whole system)
docs/                 architecture, safety rules, light tower codes, open questions
```

## The one structural rule

**Every package splits into a ROS-free `core/` and a thin ROS node.**

`src/<pkg>/<pkg>/core/*.py` must not import `rclpy` or any `*_msgs` package.
All the real logic lives there: the parser, the simulator physics, the mission
file writer, the diagnostics checks, the stream-rate policy. The
`src/<pkg>/<pkg>/*_node.py` files are adapters — subscribe, convert, call core,
publish.

Why: no hardware exists yet, ROS is not installed on every dev machine, and
Auxence wants to read this code rather than treat it as a black box. It also
means `pytest` at the repo root runs the entire test suite with zero ROS
dependencies.

```bash
pytest                                          # the GUI suite, no ROS needed
colcon build --symlink-install                  # the whole workspace
python3 -m flake8 --max-line-length=100 src/    # lint

cd src/asket_gui && npm install
npm run dev:mock                                # the GUI alone, nothing else installed
npm run build                                   # into gui_backend/gui_backend/static/

python3 -m gui_backend.core.app --sim                # GUI + simulated backend
python3 -m gui_backend.core.app --sim --shape-link   # ...on a genuinely narrow link
```

There are three ways to run this, in increasing order of what has to be
installed:

| | Needs | Use it for |
|---|---|---|
| `npm run dev:mock` | Node only | Developing and reviewing the interface |
| `python3 -m gui_backend.core.app --sim` | Node + Python | The real backend against simulated sources |
| `ros2 launch asket_bringup sim.launch.py` | the full workspace | The whole stack, still with no hardware |

Mock mode is **permanent**, not scaffolding — see `src/asket_gui/README.md`. It
swaps the socket, never the components, and two tests keep it honest: the
negotiation table is generated from `core/streams.py`, and the payload shapes
are compared across languages by running the JavaScript under Node.

## Boundaries

- Do **not** modify `pico_bridge`, `boat_bt`, or any other competition package.
  If a change there looks necessary, raise it — do not make it. The GUI's
  presence in this workspace must not cost the navigation team anything.
  - **One exception exists, on the `gui-integration` branch.** `pico_bridge` was
    filtering serial lines for `STATE` while the firmware writes `[STAT]`, so
    `/pico/status` published nothing at all. It was fixed on Auxence's explicit
    instruction, and the change is described in `INTEGRATION_STATUS.md` §3 and
    §2a. It is pending navigation-team review. The rule above still stands for
    everything else, this file included: raise it, do not make it.
- The only edit the GUI makes outside its own packages is the `enable_gui`
  launch flag in `bringup/launch/njord.launch.py`, **defaulting to false**.
  Somebody working on navigation is never made to start a web server.
- `gui_backend` must never block or starve `pico_bridge`. That node owns the
  serial link and runs a 20 Hz heartbeat. They stay separate processes.
  The firmware failsafes are **500 ms**, not 600, and the two are different
  events: SBUS lost outside AUTONOMOUS latches an e-stop and cuts ESC power;
  serial lost inside AUTONOMOUS holds the thrusters at neutral without cutting
  power or latching. A stalled GUI does not stop the boat; a lost transmitter
  does.
- Topic and message names for competition nodes are not hard-coded anywhere.
  `src/gui_backend/config/topics.yaml` maps a logical stream name to a topic,
  type and adapter — Q7, now answered against the real interface.
- `/pico/status` is a `std_msgs/String` of raw firmware `[STAT]` lines. The
  parser is isolated in `gui_backend/core/pico_state.py` and the format is now
  transcribed from `pico-node_v3.ino` rather than guessed. Watch the trap:
  `Mode:` and `Mode(Ch8):` are different fields carrying different units, and
  splitting the line on `:` conflates them.
- Anything mirrored from the firmware — the arbitration table, the SBUS
  thresholds, the timeouts, `ESTOP_FEEDBACK_ENABLED` — is checked against the
  sketch by `asket_common/test/test_firmware_arbitration_matches.py`, which
  compiles the firmware's own `arbitrate_mode()` with `g++` and compares all 24
  cases. Two ends of this wire have now disagreed about a format twice. Add to
  that test rather than adding an unchecked constant.

## Conventions

- ROS 2 Jazzy. Python unless performance dictates C++.
- `colcon build --symlink-install`.
- `ros2 run` may launch the image-baked copy under `/opt/njord`. When iterating
  on source, invoke `python3 /ros2_ws/src/...` directly.
- Podman resolves symlinks at launch: `/dev/pico` exists on the host, the
  container must reference the resolved `ttyACMn`.
- Frontend: React + MapLibre GL, Vite. Keep the dependency tree small — it must
  build offline on arm64.
- Every package has a README saying what it does and how to test it in sim.
- Times on the wire are **UTC milliseconds since the epoch**, named `*_utc_ms`.
  Every live value carries the timestamp of the sample it came from, never the
  time it was sent.

## Implementation status

| Stage | Scope | State |
|---|---|---|
| 0 | `asket_sim` simulation harness | done |
| 1 | Sonar parser + `omniscan_bridge` | done |
| 2 | `gui_backend` + GUI skeleton (map, vessel state, link, mode commands) | done |
| 3 | Lidar, power, sonar health panels | done |
| 4 | Recording, coverage, diagnostics, export | done |
| 5 | Degraded link handling | done |
| — | Motor active test **wired to the Pico** (the gate exists and is tested; connecting it is deliberately left for when somebody is standing next to the boat), obstacle tracking, survey planner, NMEA out, RTK | not started |

## Open questions

Six were listed in the architecture brief; three more surfaced during
implementation. All nine now have answers in `docs/open_questions.md`. Three
can only be closed by the hardware — the survey area (Q1), the mounting
geometry (Q2) and the `pico_bridge` interface (Q7) — and each is held by a
config file plus a pre-flight check that says so out loud, rather than by an
assumption buried in code. `README.md` lists what blocks the first deployment.

Anything implemented against a provisional value is marked `PROVISIONAL` in the
config file that holds it, so it is greppable:

```bash
grep -rn PROVISIONAL src/
```

**Q8 is closed**: every packet layout is taken from Cerulean's published
documentation, and several of the earlier transcriptions were wrong — including
one message (`SET_NTP_URL`) that does not exist. What changed is tabulated in
`docs/open_questions.md`. It is still unvalidated against a real device, which
is the first blocking item in `README.md`.

**Q3 is closed and it changed the design**: SonarView cannot import an external
trajectory, so `mission_recorder.merge_svlog` merges the recorded streams into a
valid `.svlog` afterwards. Option B survives — nothing about the onboard
recording changed.

**Q2 is answered by a check rather than by a number.** The mounting geometry
lives in `src/omniscan_bridge/config/mounting.yaml`, whose one significant line
is `measured:`. Until a human sets it true, the pre-flight returns an amber
WARN naming the file, on every run.

## Before deploying

`docs/SETUP.md` lists the steps that must be done on the hardware and cannot be
done from this repository. The first one — pointing the sonar's NTP at the
Jetson's GPS-disciplined server — is the most expensive to get wrong: its
default is an internet host, there is no internet in the field, and a wrong
sonar clock makes every mission un-georeferenceable.

## Handoff

`CONTEXT_PROJET_NJORD.md` at the repo root is the GUI section of the project
handoff document, meant to be merged into `node-ros-2026`'s copy.
