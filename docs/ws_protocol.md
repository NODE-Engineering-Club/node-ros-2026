# WebSocket protocol

One endpoint, `/ws`. JSON frames both ways.

Two rules run through the whole thing:

* **The backend pushes nothing by default.** A client subscribes to named
  streams; anything it did not ask for it does not get. This is the defence
  against a GUI that works on the bench and collapses 200 m offshore.
* **The server decides, and says why.** A client can ask for 10 Hz lidar on an
  LTE-M link. It will be told no, in words, and the words end up on screen.
  An operator who cannot tell "nothing is happening" from "I am not being sent
  it" will eventually act on the wrong one.

## Client → server

```json
{"type": "subscribe", "streams": [{"name": "vessel", "rate_hz": 5, "detail": "full"}]}
{"type": "unsubscribe", "streams": ["lidar"]}
{"type": "set_profile", "profile": "full" | "reduced" | "minimal" | "auto"}
{"type": "command", "id": "c1", "name": "set_mode", "args": {"mode": "MANUAL"}}
{"type": "pong", "id": 7}
```

## Server → client

| Type | When | Carries |
|---|---|---|
| `hello` | once, on connect | protocol version, every stream and its limits, the active profile, alarm thresholds, whether the source is `sim` or `ros` |
| `subscribed` | after subscribe/unsubscribe, and on every profile change | per stream: `granted`, `rate_hz`, `detail`, and a **`reason`** when it is less than was asked for |
| `data` | at the negotiated rate | `stream`, `payload`, `detail`, `source_utc_ms`, `server_utc_ms` |
| `stream_unavailable` | once, ~5 s after a granted stream has produced nothing | `stream`, `reason` |
| `command_result` | on issue, then on resolution | `id`, `status` (`pending` / `confirmed` / `failed`), `detail` |
| `alarms` | on transition only | `raised`, `cleared`, `active` |
| `profile` | when the link profile changes | `profile`, `manual`, `reason` |
| `ping` | every `ping_interval_s` | `id`, `server_utc_ms` |
| `error` | malformed or unknown request | `message` |

## The two timestamps

Every `data` frame carries both, and they mean different things:

* `source_utc_ms` — when the value was **produced** by the sensor or the node.
* `server_utc_ms` — when the frame was **sent**.

The client renders age from `source_utc_ms` against an estimate of the
*server's* clock, not the laptop's. Skew is estimated as
`max(server_utc_ms − local receive time)` over a recent window: transmission
delay only ever makes a frame look older, so the maximum is the sample that
suffered the least delay and therefore the best estimate of true skew.

This matters because a laptop with a wrong clock would otherwise render every
value as either permanently fresh or permanently hours old, and the operator
would have no way to tell which.

## Commands

Three outcomes, and only three:

* `pending` — sent; the vessel has not confirmed
* `confirmed` — the vessel's own status now reports the requested state
* `failed` — it did not, within the timeout, or the request was rejected

There is deliberately no fourth outcome in which the UI optimistically shows
what was asked for. Confirmation always comes from observing the vessel
(safety rule 4), never from an acknowledgement that a request was received.

Mode commands additionally require two-step confirmation in the browser
(safety rule 5), and the soft ESTOP is named `cut_propulsion` and labelled
"Cut propulsion" — never "Emergency stop" (safety rule 3).

## Link profiles

| Profile | Carries |
|---|---|
| `full` | everything, at each stream's own ceiling |
| `reduced` | position, heading, mode, power, coverage at low rate, alarms |
| `minimal` | position, mode, battery, alarms only |

`reduced` **drops** lidar rather than decimating it: a 1 Hz lidar view invites an
operator to trust it for obstacle awareness, and obstacle avoidance is the
navigation stack's job anyway.

Selection is automatic from measured round-trip time and quality, with
hysteresis — fast to degrade, slow to recover — and a manual override that is
sticky until released. Automatic selection never overrides a human choice.

## Link shaping (simulation only)

`--shape-link` throttles the outbound socket to whatever the simulated bearer
currently is: serialisation delay from its capacity, half its round-trip time as
latency, and a loss rate derived from its quality.

Without it, a "degraded link" in sim means only that fewer streams are
subscribed while the wire stays a gigabit loopback — so the thing the whole
design exists to survive is never once exercised before Namibia. With it, an
LTE-M link really is 1 kB/s at 900 ms, and the beacon profile really does have
to fit inside it.

Shaping happens in the writer, not the hub: the hub keeps producing at the
negotiated rate, the client's bounded outbox fills, and the oldest frames are
discarded — which is what a real narrow link does to a stream nobody is
throttling. When it is on, the `link` payload carries a `shaping` object, so a
demonstration cannot be mistaken for reality.

It is off by default and `RosSource` never turns it on.
