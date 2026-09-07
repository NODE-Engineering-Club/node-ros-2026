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
