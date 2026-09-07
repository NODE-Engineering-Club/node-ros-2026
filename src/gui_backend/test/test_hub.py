"""The hub: nothing is pushed unless it was asked for, and never faster than
the link can carry."""

import pytest
from asket_sim.core.world import SimWorld, WorldConfig
from gui_backend.core.hub import ClientSession, Hub
from gui_backend.core.sim_source import SimSource
from gui_backend.core.streams import PROFILE_MINIMAL, PROFILE_REDUCED


@pytest.fixture
def hub():
    return Hub(SimSource(SimWorld(WorldConfig()), time_scale=10.0))


@pytest.fixture
def client(hub):
    session = ClientSession("test")
    hub.add_client(session)
    return session


def drain(session):
    out = []
    while not session.outbox.empty():
        out.append(session.outbox.get_nowait())
    return out


def run(hub, seconds, start=1000.0, step=0.05):
    t = start
    for _ in range(int(seconds / step)):
        t += step
        hub.tick(t)
    return t


def test_a_client_with_no_subscriptions_receives_no_data(hub, client):
    run(hub, 3.0)
    assert [m for m in drain(client) if m["type"] == "data"] == []


def test_subscribing_delivers_that_stream_and_only_that_stream(hub, client):
    hub.subscribe(client, [{"name": "vessel", "rate_hz": 5.0}])
    run(hub, 2.0)
    streams = {m["stream"] for m in drain(client) if m["type"] == "data"}
    assert streams == {"vessel"}


def test_the_negotiated_rate_is_the_rate_actually_delivered(hub, client):
    hub.subscribe(client, [{"name": "vessel", "rate_hz": 2.0}])
    drain(client)
    run(hub, 10.0)
    count = len([m for m in drain(client) if m["type"] == "data"])
    assert 16 <= count <= 24     # 2 Hz for 10 s, allowing for tick alignment


def test_a_client_asking_for_too_much_is_told_what_it_is_getting(hub, client):
    reply = hub.subscribe(client, [{"name": "vessel", "rate_hz": 50.0}])
    entry = reply["streams"][0]
    assert entry["granted"] and entry["rate_hz"] == 10.0
    assert "maximum" in entry["reason"]


def test_every_frame_carries_both_timestamps(hub, client):
    """source_utc_ms is when the value was produced; server_utc_ms is when it
    was sent. Data age depends entirely on keeping them apart."""
    hub.subscribe(client, [{"name": "vessel", "rate_hz": 5.0}])
    drain(client)
    run(hub, 1.0)
    frames = [m for m in drain(client) if m["type"] == "data"]
    assert frames
    for frame in frames:
        assert "source_utc_ms" in frame and "server_utc_ms" in frame
        assert frame["source_utc_ms"] <= frame["server_utc_ms"]


def test_unsubscribing_stops_the_stream(hub, client):
    hub.subscribe(client, [{"name": "vessel", "rate_hz": 5.0}])
    run(hub, 1.0)
    hub.unsubscribe(client, ["vessel"])
    drain(client)
    run(hub, 2.0)
    assert [m for m in drain(client) if m["type"] == "data"] == []


def test_forcing_a_profile_re_resolves_and_tells_the_client(hub, client):
    hub.subscribe(client, [{"name": "vessel", "rate_hz": 10.0},
                           {"name": "lidar", "rate_hz": 10.0}])
    drain(client)
    hub.set_profile(PROFILE_REDUCED)

    messages = [m for m in drain(client) if m["type"] == "subscribed"]
    assert messages
    by_name = {s["name"]: s for s in messages[-1]["streams"]}
    assert by_name["vessel"]["rate_hz"] == 2.0
    assert not by_name["lidar"]["granted"]


def test_a_stream_dropped_by_a_profile_stops_being_sent(hub, client):
    hub.subscribe(client, [{"name": "lidar", "rate_hz": 5.0}])
    run(hub, 1.0)
    hub.set_profile(PROFILE_MINIMAL)
    drain(client)
    run(hub, 3.0)
    assert [m for m in drain(client) if m.get("stream") == "lidar"] == []


def test_a_manual_profile_survives_automatic_selection(hub, client):
    """Auto-selection quietly overriding a human decision at the worst moment is
    how people learn to distrust automation."""
    hub.set_profile(PROFILE_MINIMAL)
    run(hub, 30.0)
    assert hub.selector.profile == PROFILE_MINIMAL
    assert hub.selector.manual


def test_releasing_a_manual_override_returns_to_automatic(hub, client):
    hub.set_profile(PROFILE_MINIMAL)
    assert hub.selector.manual
    hub.set_profile(None)
    assert not hub.selector.manual
    # Whether it then *recovers* depends on the link, which by this point in the
    # simulated survey is genuinely degrading as the vessel goes offshore. The
    # recovery logic itself is pinned down in test_link_profile.py.
    run(hub, 5.0)


def test_on_change_streams_are_not_resent_unchanged(hub, client):
    hub.subscribe(client, [{"name": "plan"}])
    run(hub, 5.0)
    plans = [m for m in drain(client) if m.get("stream") == "plan"]
    assert len(plans) == 1


def test_a_slow_client_loses_its_oldest_frames_not_the_newest(hub, client):
    """A stalled browser must never be able to stall the vessel's telemetry."""
    hub.subscribe(client, [{"name": "vessel", "rate_hz": 10.0}])
    run(hub, 60.0)                       # never drained
    assert client.dropped > 0
    assert client.outbox.qsize() <= ClientSession.QUEUE_LIMIT

    newest = None
    while not client.outbox.empty():
        newest = client.outbox.get_nowait()
    assert newest is not None


def test_a_mode_command_is_pending_then_confirmed_by_the_vessel(hub, client):
    result = hub.issue_command("set_mode", {"mode": "AUTONOMOUS"})
    assert result["status"] == "pending"

    run(hub, 3.0)
    results = [m for m in drain(client) if m["type"] == "command_result"]
    assert results and results[-1]["status"] == "confirmed"
    assert hub.source.state()["pico"]["mode"] == "AUTONOMOUS"


def test_an_invalid_mode_fails_at_once_rather_than_timing_out(hub, client):
    result = hub.issue_command("set_mode", {"mode": "SIDEWAYS"})
    assert result["status"] == "failed"


def test_cutting_propulsion_is_confirmed_by_mode_and_latch_together(hub, client):
    hub.issue_command("set_mode", {"mode": "AUTONOMOUS"})
    run(hub, 3.0)
    drain(client)

    assert hub.issue_command("cut_propulsion", {})["status"] == "pending"
    run(hub, 3.0)
    state = hub.source.state()["pico"]
    assert state["mode"] == "ESTOP" and state["estop_latched"]


def test_alarms_are_broadcast_on_transition_only(hub, client):
    drain(client)
    hub.issue_command("inject_fault", {"fault": "heading_invalid"})
    run(hub, 2.0)

    messages = [m for m in drain(client) if m["type"] == "alarms"]
    assert len(messages) == 1
    assert "heading_invalid" in [a["key"] for a in messages[0]["raised"]]

    run(hub, 3.0)
    assert [m for m in drain(client) if m["type"] == "alarms"] == []

    hub.issue_command("clear_fault", {"fault": "heading_invalid"})
    run(hub, 2.0)
    cleared = [m for m in drain(client) if m["type"] == "alarms"]
    assert cleared and "heading_invalid" in cleared[0]["cleared"]


def test_hello_describes_everything_a_client_needs_to_start(hub):
    session = ClientSession("hello")
    message = hub.add_client(session)
    assert message["type"] == "hello"
    assert "vessel" in message["streams"]
    assert message["profiles"] == ["full", "reduced", "minimal"]
    assert "alarm_thresholds" in message


def test_the_link_stream_carries_server_side_facts_the_source_cannot_know(hub, client):
    hub.subscribe(client, [{"name": "link", "rate_hz": 2.0}])
    drain(client)
    run(hub, 2.0)
    frames = [m for m in drain(client) if m.get("stream") == "link"]
    assert frames
    payload = frames[-1]["payload"]
    assert payload["profile"] in ("full", "reduced", "minimal")
    assert payload["connected_clients"] == 1
    assert payload["active_link"]


def test_a_granted_stream_that_produces_nothing_says_so(hub, client):
    """A subscription accepted and then silently empty is the worst of both
    worlds: the client believes it is being fed, and the operator reads the
    blank panel as 'nothing is happening'."""
    hub.subscribe(client, [{"name": "diagnostics", "rate_hz": 1.0}])
    drain(client)
    run(hub, 8.0)

    notices = [m for m in drain(client) if m["type"] == "stream_unavailable"]
    assert len(notices) == 1, "said once, not every tick"
    assert notices[0]["stream"] == "diagnostics"
    assert "not be running" in notices[0]["reason"]


def test_a_stream_that_does_produce_data_never_reports_unavailable(hub, client):
    hub.subscribe(client, [{"name": "vessel", "rate_hz": 2.0}])
    drain(client)
    run(hub, 10.0)
    assert [m for m in drain(client) if m["type"] == "stream_unavailable"] == []
