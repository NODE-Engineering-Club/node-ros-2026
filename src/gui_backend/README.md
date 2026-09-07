# gui_backend

FastAPI + WebSocket. Serves the API, the offline map tiles and the built
frontend from one process on the Jetson — there is no CDN in Namibia and no
second server to run.

## Running

Without ROS, against the simulator — this is the development path:

```bash
python3 -m gui_backend.core.app --sim --port 8080
python3 -m gui_backend.core.app --sim --time-scale 10   # survey faster than real time
python3 -m gui_backend.core.app --sim --tiles /data/maps/survey.mbtiles
```

With ROS:

```bash
ros2 run gui_backend gui_backend_node --ros-args \
  -p topics_config:=$(ros2 pkg prefix gui_backend)/share/gui_backend/config/topics.yaml
```

Either way the frontend must be built first:

```bash
cd src/asket_gui && npm install && npm run build
```

## The design decision that matters

**The backend never pushes anything by default.** Clients subscribe to named
streams at a requested rate and detail level; the server decides what it will
actually serve, and tells the client what it is getting and why.

Everything else follows from that: decimation is server-side and per client, so
two operators on different links can watch the same vessel at different rates;
a stream absent from a profile is refused with a reason rather than quietly
starved; and a subscription that is granted but produces nothing gets an
explicit `stream_unavailable` notice, because a silent empty panel reads as
"nothing is happening" rather than "that node is not running".

The protocol is documented in [`docs/ws_protocol.md`](../../docs/ws_protocol.md).

## Layout

| Module | Responsibility |
|---|---|
| `core/streams.py` | The stream registry and the profile policy. Adding a stream here is enough for it to be negotiable |
| `core/hub.py` | Clients, subscriptions, scheduling, decimation, backpressure |
| `core/payloads.py` | The wire shape of every message. Shared by both sources |
| `core/commands.py` | `pending` / `confirmed` / `failed`, and nothing else |
| `core/alarms.py` | The few conditions that cannot self-correct, each with a remedy |
| `core/link_profile.py` | Automatic profile selection, with hysteresis |
| `core/sim_source.py` | Data from `asket_sim`, in-process, no ROS |
| `core/ros_source.py` | Data from real topics |
| `core/adapters.py` | ROS messages → the shapes `payloads` expects |
| `core/tiles.py` | Offline MBTiles, read with stdlib `sqlite3` |
| `core/server.py` | The FastAPI app |

## Configuration

`config/topics.yaml` maps every logical source to a topic, a type and an
adapter. **No topic name or message type appears in the code.** That is not
tidiness: the real `pico_bridge` message is unknown to this repository and
`pico_bridge` must not be modified (`docs/open_questions.md` Q7), so pointing
the backend at it must be a config change plus at most one adapter function.

`config/link_profiles.yaml` holds the profile thresholds and the server's own
settings, including the path to the offline tile file.

## Testing in sim

```bash
pytest src/gui_backend        # no ROS, no browser, no hardware
```

The tests include the whole WebSocket path through the real FastAPI app: that
nothing is pushed before a subscription, that a client asking for too much is
told what it is getting, that a mode command goes `pending` → `confirmed` from
the vessel's own status, and that a slow client loses its oldest frames rather
than stalling the hub.
