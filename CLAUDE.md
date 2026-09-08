# CLAUDE.md — Asket Mission GUI

Working notes for anyone (human or agent) touching this repository.
Keep this file current as the architecture evolves.

## What this repository is

The mission GUI and its supporting ROS 2 packages for the **Asket ASV**
(NODE Engineering Club), built for the **Namibia seabed survey** with a
Cerulean Omniscan 3D.

It is a **colcon workspace overlay**. The packages under `src/` are designed to
be built alongside the existing `node-ros-2026` workspace (which provides
`pico_bridge`, `boat_bt`, Nav2, MAVROS, `rplidar`, `balise_bridge`). Nothing
here modifies those packages — see "Boundaries" below.

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
pytest                                          # whole suite, no ROS needed
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

- Do **not** modify `pico_bridge`, `boat_bt`, or any other existing package.
  If a change there looks necessary, raise it — do not make it.
- Topic and message names for *existing* nodes are not hard-coded anywhere.
  `src/gui_backend/config/topics.yaml` maps a logical stream name to a topic,
  type and adapter. Reconciling it with the real `pico_bridge` message is
  tracked in `docs/open_questions.md` (Q7).

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
implementation. All nine are tracked, with the provisional values used in the
meantime, in `docs/open_questions.md`. Anything implemented against a
provisional value is marked `PROVISIONAL` in the config file that holds it, so
it is greppable:

```bash
grep -rn PROVISIONAL src/
```

The one to read first is **Q8**: the `OS3D_POINT_SET` payload layout is
transcribed from the brief, not from Cerulean documentation, and must be
validated against their sample data before the first field deployment. The
parser cross-checks the declared point count against the payload length so a
wrong assumption fails loudly rather than producing a plausible cloud of
nonsense — but it is still an assumption.

## Handoff

`CONTEXT_PROJET_NJORD.md` at the repo root is the GUI section of the project
handoff document, meant to be merged into `node-ros-2026`'s copy.
