"""The synthetic sonar_raw.bin, and the fake sonar on a real socket."""

import socket
import time

import pytest
from asket_sim.core.fake_sonar_server import FakeSonarServer, FrameCollector
from asket_sim.core.raw_stream import corrupt_stream, generate_raw_stream
from asket_sim.core.world import SimWorld, WorldConfig
from omniscan_bridge.core import ping_protocol as pp


def make_world(**sonar_kwargs):
    cfg = WorldConfig()
    for k, v in sonar_kwargs.items():
        setattr(cfg.sonar, k, v)
    return SimWorld(cfg, start_utc_ms=1_700_000_000_000)


def test_generated_stream_decodes_back_to_what_went_in():
    data, stats = generate_raw_stream(make_world(ping_rate_hz=5.0), duration_s=4.0)
    assert stats.pings >= 15
    frames = FrameCollector().feed(data)
    point_sets = [f for f in frames if f.message_id == pp.MSG_OS3D_POINT_SET]
    assert len(point_sets) == stats.pings

    first = pp.decode_point_set(point_sets[0].payload)
    assert first.ping_number == 1
    assert not first.length_mismatch
    assert first.speed_of_sound == pytest.approx(1500.0)
    assert len(first.points) == make_world().cfg.sonar.points_per_ping


def test_stream_contains_the_three_inbound_message_types():
    data, _ = generate_raw_stream(make_world(), duration_s=4.0)
    ids = {f.message_id for f in FrameCollector().feed(data)}
    assert ids == {
        pp.MSG_OS3D_POINT_SET,
        pp.MSG_END_PING_INFO,
        pp.MSG_ATTITUDE_REPORT,
    }


def test_timestamps_advance_monotonically():
    """Post-mission fusion pairs on these. Out-of-order would break it."""
    data, _ = generate_raw_stream(make_world(), duration_s=6.0)
    stamps = [
        pp.decode_point_set(f.payload).utc_msec
        for f in FrameCollector().feed(data)
        if f.message_id == pp.MSG_OS3D_POINT_SET
    ]
    assert stamps == sorted(stamps)
    assert len(set(stamps)) == len(stamps)


def test_corrupt_helper_actually_damages_the_stream():
    data, _ = generate_raw_stream(make_world(), duration_s=4.0)
    damaged = corrupt_stream(data, drop_ranges=[(1500, 1573)], flip_bytes=[3200])
    assert len(damaged) == len(data) - 73
    assert damaged != data[: len(damaged)]


# -- the fake device on a real socket ------------------------------------


@pytest.fixture
def server():
    srv = FakeSonarServer(
        make_world(ping_rate_hz=20.0),
        host="127.0.0.1",
        port=0,            # let the OS pick, so tests never collide
        time_scale=20.0,   # run the world fast so the test is quick
        serve_discovery=False,
    )
    srv.start()
    yield srv
    srv.stop()


def _client(server):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(3.0)
    # Any packet registers us as a client; a parameter set is the natural one.
    sock.sendto(pp.encode_set_ping_parameters(25.0, 3, 10.0), ("127.0.0.1", server.bound_port))
    return sock


def test_the_bridge_can_receive_real_frames_over_udp(server):
    sock = _client(server)
    collector = FrameCollector()
    frames = []
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline and not any(
        f.message_id == pp.MSG_OS3D_POINT_SET for f in frames
    ):
        try:
            frames += collector.feed(sock.recv(65535))
        except socket.timeout:
            break
    sock.close()
    assert any(f.message_id == pp.MSG_OS3D_POINT_SET for f in frames)


def test_set_ping_parameters_is_applied_by_the_device(server):
    sock = _client(server)
    time.sleep(0.4)
    sock.close()
    assert server.world.cfg.sonar.range_setting_m == pytest.approx(25.0)
    assert server.world.cfg.sonar.gain == 3
    assert server.world.cfg.sonar.ping_rate_hz == pytest.approx(10.0)


def test_set_ntp_url_is_recorded_so_the_bridge_can_verify_it(server):
    """Everything in section 5 depends on this having actually been sent."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.sendto(pp.encode_set_ntp_url("192.168.2.1"), ("127.0.0.1", server.bound_port))
    deadline = time.monotonic() + 2.0
    while server.ntp_url is None and time.monotonic() < deadline:
        time.sleep(0.02)
    sock.close()
    assert server.ntp_url == "192.168.2.1"
