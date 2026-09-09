# asket_bringup

Launch files and mission-wide configuration.

## Running

```bash
ros2 launch asket_bringup sim.launch.py             # no hardware needed
ros2 launch asket_bringup asket.launch.py           # real sources
ros2 launch asket_bringup asket.launch.py sim:=true # the same as sim.launch.py
```

`sim:=true` swaps the *sources* and nothing else. The sonar bridge, recorder,
diagnostics, backend and GUI are identical in both modes — in sim the sonar is
served as real Ping Protocol frames on a real UDP socket, so there is no second
code path to rot.

Real sources (MAVROS, rplidar, `pico_bridge`) are launched by `node-ros-2026`'s
own bringup. This workspace must not modify those packages, so `asket.launch.py`
deliberately starts nothing in the non-sim branch rather than duplicating them.

## Configuration

| File | Contents |
|---|---|
| `config/mission_defaults.yaml` | Survey box, sonar settings, alarm thresholds, data-age thresholds |
| `config/light_tower.yaml` | Light tower colour codes — the contract documented in `docs/light_tower.md` |

Values marked `PROVISIONAL` stand in for the unanswered questions in
`docs/open_questions.md`. They live in YAML rather than in code so that
answering one is an edit, not a rebuild:

```bash
grep -rn PROVISIONAL src/
```
