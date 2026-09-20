"""Synthetic sonar stream in, vessel-frame points out.

The stage 1 acceptance criterion, run without a sonar, a socket or ROS.
"""

import pytest
from asket_sim.core.raw_stream import corrupt_stream, generate_raw_stream
from asket_sim.core.world import SimWorld, WorldConfig
from omniscan_bridge.core import ping_protocol as pp
from omniscan_bridge.core.geometry import SonarMounting, point_set_to_vessel_frame
from omniscan_bridge.core.parser import PingParser
from omniscan_bridge.core.status import SonarHealthTracker


def synthetic_stream(duration_s=8.0, **sonar_kwargs):
    cfg = WorldConfig()
    cfg.sonar.ping_rate_hz = 5.0
    for k, v in sonar_kwargs.items():
        setattr(cfg.sonar, k, v)
    world = SimWorld(cfg, start_utc_ms=1_700_000_000_000)
    return generate_raw_stream(world, duration_s)


def run(stream, mounting=None):
    """Parse a stream the way the ROS node does, and return the point clouds."""
    parser = PingParser()
    tracker = SonarHealthTracker()
    mounting = mounting or SonarMounting()
    clouds, now = [], 0.0

    for frame, message in parser.feed_and_decode(stream):
        if isinstance(message, pp.PointSet):
            now += 0.2
            valid = sum(1 for p in message.points if p.pt_type != 0)
            tracker.on_point_set(
                message.utc_msec, len(message.points), valid,
                message.speed_of_sound, now_monotonic=now,
                now_utc_ms=message.utc_msec,
            )
            clouds.append(
                point_set_to_vessel_frame(
                    message, mounting,
                    roll_deg=tracker.roll_deg, pitch_deg=tracker.pitch_deg,
                )
            )
        elif isinstance(message, pp.AttitudeReport):
            tracker.on_attitude(message.pitch_deg, message.roll_deg)

    return parser, tracker, clouds, now


def test_a_clean_synthetic_stream_produces_plausible_point_clouds():
    stream, stats = synthetic_stream()
    parser, tracker, clouds, now = run(stream)

    assert len(clouds) == stats.pings
    assert parser.stats.checksum_errors == 0
    assert parser.stats.bytes_discarded == 0
    assert parser.stats.packet_loss_ratio == 0.0

    non_empty = [c for c in clouds if c]
    assert len(non_empty) > len(clouds) * 0.9

    for cloud in non_empty:
        for x, y, z, power in cloud:
            assert z < 0.0, "every detection must be below the waterline"
            assert y < 0.5, "a starboard install must not paint the port side"
            assert 0.0 <= power <= 1.0


def test_depths_match_the_synthetic_seabed():
    stream, _ = synthetic_stream(duration_s=4.0)
    _, _, clouds, _ = run(stream)
    depths = [-z for cloud in clouds for _, _, z, _ in cloud]
    # SeabedConfig defaults to ~18 m with a gentle slope and small ripples.
    assert 12.0 < sum(depths) / len(depths) < 26.0


def test_health_matches_the_stream_that_produced_it():
    stream, stats = synthetic_stream(duration_s=6.0)
    parser, tracker, _, now = run(stream)
    health = tracker.health(
        connected=True,
        packet_loss_ratio=parser.stats.packet_loss_ratio,
        packets_parsed=parser.stats.frames_parsed,
        checksum_errors=parser.stats.checksum_errors,
        bytes_discarded=parser.stats.bytes_discarded,
        seconds_since_data=0.1,
        now_monotonic=now,
    )
    assert health.points_per_ping == 256
    assert 0 < health.valid_points_per_ping <= 256
    assert health.speed_of_sound == pytest.approx(1500.0)
    assert health.clock_ok
    assert health.packets_parsed == stats.frames


def test_a_damaged_stream_still_yields_most_of_the_survey():
    """Bytes lost and a corrupted frame: the parser must resynchronise, and the
    damage must show up in the counters rather than as a silent hole."""
    clean, _ = synthetic_stream(duration_s=8.0)
    damaged = corrupt_stream(clean, drop_ranges=[(5000, 5600)], flip_bytes=[20_000, 60_000])

    clean_parser, _, clean_clouds, _ = run(clean)
    parser, _, clouds, _ = run(damaged)

    assert parser.stats.bytes_discarded > 0
    assert parser.stats.checksum_errors >= 1
    assert len(clouds) >= len(clean_clouds) - 6
    assert parser.stats.packet_loss_ratio > 0.0


def test_clock_drift_in_the_stream_is_detected_downstream():
    """The fault injector drifts the sonar clock; the health tracker must see it
    from the bytes alone, with no other hint."""
    cfg = WorldConfig()
    cfg.sonar.ping_rate_hz = 5.0
    world = SimWorld(cfg, start_utc_ms=1_700_000_000_000)
    world.inject_fault("clock_drift")
    stream, _ = generate_raw_stream(world, 30.0)

    parser = PingParser()
    tracker = SonarHealthTracker()
    for _, message in parser.feed_and_decode(stream):
        if isinstance(message, pp.PointSet):
            # The Jetson's own clock, which is the honest reference.
            wall = message.utc_msec - _drift_at(message)
            tracker.on_point_set(message.utc_msec, len(message.points), 1,
                                 message.speed_of_sound, 0.0, wall)

    assert not tracker.health(True, 0, 0, 0, 0, 0.1, 0.0).clock_ok


def _drift_at(point_set):
    """The world drifts the sonar clock 40 ms per simulated second."""
    elapsed_s = (point_set.utc_msec - 1_700_000_000_000) / 1000.0
    return int(40.0 * elapsed_s / (1.0 + 40.0 / 1000.0))
