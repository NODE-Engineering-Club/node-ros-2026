"""Stage 5: does the GUI stay usable, and honest, on a bad link?

These are the tests that decide whether the whole bandwidth-negotiation design
was worth building. Without shaping, "degraded link" in sim means only that
fewer streams are subscribed while the wire stays a gigabit loopback — which
never once exercises the thing the design exists to survive.
"""

import json

import pytest
from asket_sim.core.link import LINK_CHARACTERISTICS
from asket_sim.core.world import SimWorld, WorldConfig
from fastapi.testclient import TestClient
from gui_backend.core.hub import ClientSession, Hub
from gui_backend.core.server import create_app
from gui_backend.core.shaper import LinkShaper
from gui_backend.core.sim_source import SimSource
from gui_backend.core.streams import (
    PROFILE_FULL,
    PROFILE_MINIMAL,
    PROFILE_REDUCED,
    PROFILES,
    STREAMS,
    estimate_bytes_per_s,
    resolve,
)

# -- the shaper itself ----------------------------------------------------


def test_frames_queue_behind_each_other():
    """Queueing is what makes a narrow link feel narrow rather than merely late."""
    shaper = LinkShaper(enabled=True, capacity_bytes_per_s=1000.0, rtt_ms=0.0)
    first = shaper.delay_for(1000, now=0.0)
    second = shaper.delay_for(1000, now=0.0)
    assert first == pytest.approx(1.0, abs=0.01)
    assert second == pytest.approx(2.0, abs=0.01)


def test_latency_is_added_on_top_of_serialisation():
    shaper = LinkShaper(enabled=True, capacity_bytes_per_s=1_000_000.0, rtt_ms=200.0)
    assert shaper.delay_for(500, now=0.0) == pytest.approx(0.1, abs=0.01)


def test_a_disabled_shaper_costs_nothing():
    """It must be impossible for this to affect real hardware."""
    shaper = LinkShaper(enabled=False, capacity_bytes_per_s=1.0, rtt_ms=5000.0)
    assert shaper.delay_for(100_000) == 0.0
    assert shaper.should_drop() is False


def test_loss_is_applied_at_roughly_the_configured_rate():
    shaper = LinkShaper(enabled=True, loss_ratio=0.3, seed=7)
    dropped = sum(1 for _ in range(2000) if shaper.should_drop())
    assert 0.25 < dropped / 2000 < 0.35


# -- what each profile costs on its own bearer ----------------------------


@pytest.mark.parametrize(
    "profile,bearer",
    [(PROFILE_FULL, "wifi"), (PROFILE_REDUCED, "4g"), (PROFILE_MINIMAL, "ltem")],
)
def test_each_profile_fits_the_bearer_it_is_meant_for(profile, bearer):
    """The claim the whole design rests on: `minimal` has to fit down an LTE-M
    beacon link, or an operator who has lost WiFi has nothing at all."""
    _, capacity = LINK_CHARACTERISTICS[bearer]
    demand = estimate_bytes_per_s([resolve(n, None, None, profile) for n in STREAMS])
    assert demand < capacity * 0.8, (
        f"{profile} needs {demand:.0f} B/s but {bearer} carries {capacity:.0f} B/s"
    )


def test_the_beacon_profile_is_small_enough_to_be_worth_having():
    """A kilobyte a second is the budget. Position, mode and battery have to fit
    inside it with room for an alarm."""
    demand = estimate_bytes_per_s(
        [resolve(n, None, None, PROFILE_MINIMAL) for n in STREAMS]
    )
    assert demand < PROFILES[PROFILE_MINIMAL].budget_bytes_per_s
    assert demand < 200, "the beacon profile should be tiny, not merely small"


# -- end to end, through the real socket ----------------------------------


def make_client(profile, bearer, tick_hz=50.0):
    world = SimWorld(WorldConfig(), start_utc_ms=1_700_000_000_000)
    hub = Hub(SimSource(world, time_scale=10.0), tick_hz=tick_hz)
    hub.set_profile(profile)
    app = create_app(hub, static_dir=None, tiles_path=None,
                     ping_interval_s=0.2, shape_link=True)
    rtt, capacity = LINK_CHARACTERISTICS[bearer]
    return app, hub, rtt, capacity


def drive(client, hub, rtt, capacity, seconds, subscriptions):
    """Connect, subscribe, and collect whatever survives the shaped link."""
    received, bytes_in = [], 0
    with client.websocket_connect("/ws") as ws:
        ws.receive_json()
        for session in hub.clients.values():
            session.shaper.update(capacity, rtt, loss_ratio=0.0)

        ws.send_json({"type": "subscribe", "streams": subscriptions})
        for _ in range(seconds):
            try:
                message = ws.receive_json()
            except Exception:  # noqa: BLE001 - a closed socket ends the sample
                break
            if message["type"] == "ping":
                ws.send_json({"type": "pong", "id": message["id"]})
            received.append(message)
            bytes_in += len(json.dumps(message))
    return received, bytes_in


def test_position_still_arrives_on_a_beacon_link():
    """The one thing that must survive: where the boat is."""
    app, hub, rtt, capacity = make_client(PROFILE_MINIMAL, "ltem")
    subscriptions = [{"name": n} for n in ("vessel", "pico", "power", "alarms", "link")]
    with TestClient(app) as client:
        received, _ = drive(client, hub, rtt, capacity, 60, subscriptions)

    vessel = [m for m in received if m.get("stream") == "vessel"]
    assert vessel, "no position arrived at all on the beacon profile"
    assert vessel[-1]["payload"]["lat"] is not None


def test_a_beacon_link_carries_only_the_beacon_payload():
    """`minimal` must not be `full` with a smaller number in front of it: the
    payload itself has to shrink or it will not fit."""
    app, hub, rtt, capacity = make_client(PROFILE_MINIMAL, "ltem")
    with TestClient(app) as client:
        received, _ = drive(
            client, hub, rtt, capacity, 40,
            [{"name": "vessel"}, {"name": "lidar", "rate_hz": 5}],
        )

    vessel = [m for m in received if m.get("stream") == "vessel"]
    assert vessel
    payload = vessel[-1]["payload"]
    assert "lat" in payload and "heading_deg" in payload
    # The expensive fields are gone, not merely sent less often.
    assert "roll_deg" not in payload
    assert "num_sats" not in payload
    assert not [m for m in received if m.get("stream") == "lidar"]


def test_every_frame_is_still_honest_about_its_age_on_a_slow_link():
    """The whole point of stage 5. A slow link must produce OLD data that says
    it is old, never fresh-looking data that is wrong."""
    app, hub, rtt, capacity = make_client(PROFILE_REDUCED, "4g")
    with TestClient(app) as client:
        received, _ = drive(
            client, hub, rtt, capacity, 60,
            [{"name": "vessel", "rate_hz": 2}, {"name": "pico", "rate_hz": 1}],
        )

    frames = [m for m in received if m["type"] == "data"]
    assert frames
    for frame in frames:
        assert frame["source_utc_ms"] <= frame["server_utc_ms"], (
            "a frame claiming to have been produced after it was sent would make "
            "data age read as negative, i.e. fresher than now"
        )


def test_the_client_is_told_what_the_narrow_link_took_away():
    app, hub, rtt, capacity = make_client(PROFILE_MINIMAL, "ltem")
    with TestClient(app) as client:
        with client.websocket_connect("/ws") as ws:
            ws.receive_json()
            ws.send_json({
                "type": "subscribe",
                "streams": [{"name": "vessel", "rate_hz": 10}, {"name": "lidar"}],
            })
            confirmation = None
            for _ in range(20):
                message = ws.receive_json()
                if message["type"] == "subscribed":
                    confirmation = message
                    break

    assert confirmation is not None
    by_name = {s["name"]: s for s in confirmation["streams"]}
    assert not by_name["lidar"]["granted"]
    assert "not carried" in by_name["lidar"]["reason"]
    assert by_name["vessel"]["rate_hz"] < 10
    assert by_name["vessel"]["reason"]


def test_shaping_reports_itself_so_a_demo_cannot_be_mistaken_for_reality():
    """A shaped link is a simulation artefact. It has to be visible as one."""
    app, hub, rtt, capacity = make_client(PROFILE_FULL, "wifi")
    session = ClientSession("probe", shaper=LinkShaper(enabled=True))
    hub.add_client(session)
    hub.tick(1000.0)
    sample = hub.source.snapshot("link", "full")
    sample.payload.update(hub._link_context(session))
    assert sample.payload["shaping"]["enabled"] is True
