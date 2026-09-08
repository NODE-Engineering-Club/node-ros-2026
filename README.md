# Asket Mission GUI — Namibia

Real-time mission cockpit for the NODE Engineering Club's **Asket** uncrewed
surface vessel, built for the Namibia seabed survey with a Cerulean Omniscan 3D.

It shows vessel state, navigation, obstacle awareness, sensor health, mission
recording and mode commands, in a browser on a laptop ashore, over a wireless
link of variable quality.

It is **not** a sonar analysis tool — no point clouds, no meshing, no
bathymetry. That is SonarView's job, post-mission.

It is **not part of the safety chain**. The hardware killswitch and RC channel 8
are sovereign; nothing here can override, delay or interfere with them. See
[`docs/safety.md`](docs/safety.md).

## Start here

```bash
pytest                                    # whole suite, no ROS required
ros2 launch asket_bringup sim.launch.py   # whole stack, no hardware required
```

Simulation is a first-class, permanent feature, not scaffolding — see
[`src/asket_sim/README.md`](src/asket_sim/README.md).

## Layout

| Package | Responsibility |
|---|---|
| [`asket_common`](src/asket_common) | Shared geometry, survey planning, heading maths |
| [`asket_interfaces`](src/asket_interfaces) | Messages and services |
| [`asket_sim`](src/asket_sim) | Simulated sources for everything, with injectable faults |
| [`omniscan_bridge`](src/omniscan_bridge) | Cerulean Ping Protocol → ROS 2 |
| [`mission_recorder`](src/mission_recorder) | Mission files, trajectory logging, export |
| [`system_test`](src/system_test) | Pre-flight built-in test, GO/NO-GO verdict |
| [`gui_backend`](src/gui_backend) | FastAPI + WebSocket, bandwidth negotiation |
| [`asket_gui`](src/asket_gui) | React + MapLibre frontend |
| [`asket_bringup`](src/asket_bringup) | Launch files and mission configuration |

Architecture notes for anyone working in here, human or agent:
[`CLAUDE.md`](CLAUDE.md).
Unanswered design questions and the provisional values standing in for them:
[`docs/open_questions.md`](docs/open_questions.md).

## Blocking before the first deployment

Not a wishlist. Each of these produces a survey that looks fine on the day and
is worthless afterwards, and none of them can be closed from a laptop. Full
procedure in [`docs/SETUP.md`](docs/SETUP.md).

| | What | Why it blocks |
|---|---|---|
| **1** | **Validate `OS3D_POINT_SET` against Cerulean sample data** (Q8) | The 80-byte header and the 16-byte point layout are taken from Cerulean's published documentation, but nothing here has ever been run against a real device. Two things the documentation does not state are assumptions: little-endian, and `vec3` as three floats. The parser cross-checks the declared point count against the payload length so a wrong layout fails loudly rather than producing a plausible cloud of nonsense — but it has not yet been proven right. Open one real `.svlog` or one captured stream through `omniscan_bridge` before the boat goes in the water. |
| **2** | **Point the sonar at a GPS-disciplined NTP server** on the Jetson | Cerulean removed the packet that used to set it; the device defaults to an internet host and there is no internet in the field. A drifted sonar clock makes every ping un-pairable with the trajectory. The pre-flight fails at 250 ms. |
| **3** | **Measure the mounting angle and the lever arm** (Q2) | A systematic offset that does not average out. Pre-flight warns, in amber, on every run until `mounting.yaml` says `measured: true`. |
| **4** | **Install the USB mount unit** (Q9) | A headless Jetson auto-mounts nothing, so export would silently have nowhere to go. `python3 -m mission_recorder.check_export_paths` answers this before the drive is needed. |
| **5** | **Copy the `.mbtiles` for the survey area** onto the Jetson | Without it the map degrades to a coordinate grid. Usable, but you will not see the coastline. |

Item 1 is the one to do first: it is the only one that cannot be fixed on the
beach.
