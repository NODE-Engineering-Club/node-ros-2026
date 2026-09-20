"""Merging a recorded mission into a SonarView-readable .svlog.

SonarView cannot import an external trajectory: position and heading have to be
inside the log, as NMEA_WRAPPER packets. A log without them cannot be
georeferenced or exported at all — so if this merge is wrong, the survey is
unusable and nobody finds out until somebody opens it back home.
"""

import json

import pytest
from asket_sim.core.raw_stream import generate_raw_stream
from asket_sim.core.world import SimWorld, WorldConfig
from mission_recorder.core import svlog
from omniscan_bridge.core import ping_protocol as pp
from omniscan_bridge.core.parser import PingParser

UTC = 1_700_000_000_000


# -- NMEA ------------------------------------------------------------------


def test_checksum_matches_the_reference_sentence():
    """The canonical GGA example, whose checksum is documented as 47."""
    body = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,"
    assert svlog.nmea_checksum(body) == "47"


def test_gga_carries_position_in_nmea_degrees_and_minutes():
    sample = svlog.TrajectorySample(
        utc_ms=UTC, lat=-22.9576, lon=14.5053, heading_deg=20.0,
        heading_valid=True, num_sats=14, hdop=0.8,
    )
    sentence = svlog.gga_sentence(sample)
    assert sentence.startswith("$GPGGA,")
    fields = sentence.split(",")
    # -22.9576 deg = 22 deg 57.456 min South.
    assert fields[2].startswith("2257.456")
    assert fields[3] == "S"
    # 14.5053 deg = 14 deg 30.318 min East, three-digit degrees for longitude.
    assert fields[4].startswith("01430.318")
    assert fields[5] == "E"
    assert fields[7] == "14"          # satellites
    assert svlog.nmea_checksum(sentence[1:].split("*")[0]) == sentence.split("*")[1]


def test_heading_is_a_separate_sentence():
    """GGA has no heading field. Letting SonarView infer one from successive
    positions would substitute course over ground for heading, and in a
    cross-current those are different numbers."""
    assert "GPHDT" in svlog.hdt_sentence(123.45)
    assert "123.45" in svlog.hdt_sentence(123.45)
    assert "1.00" in svlog.hdt_sentence(361.0)      # wrapped, not clipped


def test_southern_and_western_hemispheres_are_signed_correctly():
    south_west = svlog.TrajectorySample(UTC, -33.9, -18.4, 0.0, True)
    fields = svlog.gga_sentence(south_west).split(",")
    assert fields[3] == "S"
    assert fields[5] == "W"


# -- interpolation ---------------------------------------------------------


def samples(*rows):
    return [
        svlog.TrajectorySample(utc_ms=t, lat=lat, lon=lon, heading_deg=hdg,
                               heading_valid=valid)
        for t, lat, lon, hdg, valid in rows
    ]


def test_position_is_interpolated_between_samples():
    track = samples((UTC, -22.0, 14.0, 10.0, True), (UTC + 1000, -22.001, 14.001, 20.0, True))
    middle = svlog.interpolate(track, UTC + 500)
    assert middle.lat == pytest.approx(-22.0005)
    assert middle.lon == pytest.approx(14.0005)
    assert middle.heading_deg == pytest.approx(15.0)


def test_heading_interpolates_the_short_way_round():
    """359 to 001 must cross north, not sweep backwards through south."""
    track = samples((UTC, -22.0, 14.0, 359.0, True), (UTC + 1000, -22.0, 14.0, 1.0, True))
    middle = svlog.interpolate(track, UTC + 500)
    assert middle.heading_deg == pytest.approx(0.0, abs=1e-6)


def test_a_ping_outside_the_trajectory_is_not_extrapolated():
    """A confident position for seabed nobody knows the location of would look
    exactly like the good data around it."""
    track = samples((UTC, -22.0, 14.0, 10.0, True), (UTC + 1000, -22.001, 14.001, 20.0, True))
    assert svlog.interpolate(track, UTC - 1) is None
    assert svlog.interpolate(track, UTC + 1001) is None


def test_a_long_gap_is_not_bridged():
    track = samples((UTC, -22.0, 14.0, 10.0, True), (UTC + 60_000, -22.01, 14.01, 20.0, True))
    assert svlog.interpolate(track, UTC + 30_000, max_gap_ms=2000) is None


def test_heading_validity_does_not_survive_interpolation_across_an_invalid_sample():
    track = samples((UTC, -22.0, 14.0, 10.0, True), (UTC + 1000, -22.001, 14.001, 20.0, False))
    assert svlog.interpolate(track, UTC + 500).heading_valid is False


# -- the merge -------------------------------------------------------------


@pytest.fixture
def mission(tmp_path):
    """A synthetic mission: real Ping Protocol bytes plus a trajectory."""
    config = WorldConfig()
    config.sonar.ping_rate_hz = 5.0
    world = SimWorld(config, start_utc_ms=UTC)

    raw, stats = generate_raw_stream(world, duration_s=20.0)
    (tmp_path / "sonar_raw.bin").write_bytes(raw)

    # A trajectory that brackets the sonar stream, as the recorder writes it.
    replay = SimWorld(WorldConfig(), start_utc_ms=UTC)
    lines = []
    for step in range(0, 2200):
        replay.step(0.01)
        if step % 10:
            continue
        vessel = replay.vessel.sample(replay.utc_ms)
        lines.append(json.dumps({
            "utc_ms": replay.utc_ms, "lat": vessel.lat, "lon": vessel.lon, "alt": 0.0,
            "heading_deg": vessel.heading_deg, "heading_source": "magnetometer",
            "heading_valid": True, "cog_deg": vessel.cog_deg, "sog_ms": vessel.sog_ms,
            "roll_deg": vessel.roll_deg, "pitch_deg": vessel.pitch_deg,
            "gnss_fix_type": 3, "num_sats": 14, "hdop": 0.8,
        }))
    (tmp_path / "trajectory.jsonl").write_text("\n".join(lines) + "\n")
    return tmp_path, stats


def parse(path):
    parser = PingParser()
    return [(f, parser.decode(f)) for f in parser.feed(path.read_bytes())]


def test_the_merged_log_parses_back_with_positions(mission):
    """The round trip the whole correction exists for."""
    directory, _ = mission
    output = directory / "merged.svlog"
    stats = svlog.merge_to_svlog(
        directory / "sonar_raw.bin", directory / "trajectory.jsonl", output
    )

    assert stats.pings > 50
    assert stats.gga_written == stats.pings
    assert stats.hdt_written == stats.pings
    assert stats.unplaceable_pings == 0

    decoded = parse(output)
    nmea = [m for f, m in decoded if f.message_id == pp.MSG_NMEA_WRAPPER]
    point_sets = [m for f, m in decoded if isinstance(m, pp.PointSet)]

    assert len(point_sets) == stats.pings
    assert len([s for s in nmea if s.startswith("$GPGGA")]) == stats.pings
    assert len([s for s in nmea if s.startswith("$GPHDT")]) == stats.pings


def test_the_positions_in_the_log_are_the_positions_the_vessel_had(mission):
    """Not just present — right. A merged log with plausible-but-wrong
    positions is the worst possible outcome, because it looks fine."""
    directory, _ = mission
    output = directory / "merged.svlog"
    svlog.merge_to_svlog(directory / "sonar_raw.bin", directory / "trajectory.jsonl", output)

    track = svlog.load_trajectory(directory / "trajectory.jsonl")
    decoded = parse(output)

    checked = 0
    pending = None
    for frame, message in decoded:
        if frame.message_id == pp.MSG_NMEA_WRAPPER and message.startswith("$GPGGA"):
            pending = message
        elif isinstance(message, pp.PointSet) and pending is not None:
            expected = svlog.interpolate(track, message.utc_msec)
            assert expected is not None
            fields = pending.split(",")
            minutes = float(fields[2][2:])
            degrees = float(fields[2][:2])
            lat = -(degrees + minutes / 60.0)     # southern hemisphere
            assert lat == pytest.approx(expected.lat, abs=2e-6)   # ~20 cm
            checked += 1
            pending = None
    assert checked > 50


def test_navigation_is_written_before_the_ping_it_describes(mission):
    """A reader carrying the most recent position forward must already have one
    by the time the ping arrives."""
    directory, _ = mission
    output = directory / "merged.svlog"
    svlog.merge_to_svlog(directory / "sonar_raw.bin", directory / "trajectory.jsonl", output)

    seen_position = False
    for frame, message in parse(output):
        if frame.message_id == pp.MSG_NMEA_WRAPPER and message.startswith("$GPGGA"):
            seen_position = True
        elif isinstance(message, pp.PointSet):
            assert seen_position, "a ping arrived before any position"


def test_merged_log_preserves_the_sonar_stream_byte_for_byte(mission):
    """The sonar half has to reach SonarView exactly as the device produced it."""
    directory, _ = mission
    output = directory / "merged.svlog"
    svlog.merge_to_svlog(directory / "sonar_raw.bin", directory / "trajectory.jsonl", output)

    original = (directory / "sonar_raw.bin").read_bytes()
    merged = output.read_bytes()

    # Strip the navigation packets back out; what is left must be the input.
    stripped = bytearray()
    for frame, raw, _ in svlog.iter_frames(merged):
        if frame.message_id != pp.MSG_NMEA_WRAPPER:
            stripped += raw
    assert bytes(stripped) == original


def test_an_invalid_heading_is_omitted_rather_than_written(mission):
    """The trajectory records validity per sample precisely so this decision
    can be made rather than guessed. A heading we did not trust must not go
    into the log as though we did."""
    directory, _ = mission
    trajectory = directory / "trajectory.jsonl"
    records = [json.loads(line) for line in trajectory.read_text().splitlines()]
    for record in records[len(records) // 2:]:
        record["heading_valid"] = False
    trajectory.write_text("\n".join(json.dumps(r) for r in records) + "\n")

    output = directory / "merged.svlog"
    stats = svlog.merge_to_svlog(directory / "sonar_raw.bin", trajectory, output)

    assert stats.heading_invalid_samples > 0
    assert stats.hdt_written < stats.gga_written
    assert "heading was invalid" in svlog.describe(stats)


def test_merging_a_mission_directory_names_the_output_after_it(mission):
    directory, _ = mission
    path, stats = svlog.merge_mission(directory)
    assert path.name == f"{directory.name}.svlog"
    assert path.is_file()
    assert stats.pings > 0


def test_a_mission_with_no_trajectory_fails_loudly(tmp_path):
    (tmp_path / "sonar_raw.bin").write_bytes(b"")
    with pytest.raises(FileNotFoundError):
        svlog.merge_mission(tmp_path)


def test_the_summary_warns_about_pings_that_could_not_be_placed():
    stats = svlog.MergeStats(pings=100, unplaceable_pings=12, gga_written=88)
    message = svlog.describe(stats)
    assert "12 pings" in message
    assert "will not be georeferenced" in message


# -- live interleaving -----------------------------------------------------


def test_a_live_svlog_is_written_alongside_the_raw_stream(tmp_path):
    """Belt and braces: if the trajectory is lost or nobody runs the merge, the
    survey is still openable. A survey that cannot be opened did not happen."""
    from mission_recorder.core.mission import (
        LIVE_SVLOG_NAME,
        SONAR_RAW_NAME,
        MissionRecorder,
        RecorderConfig,
    )

    config = WorldConfig()
    config.sonar.ping_rate_hz = 5.0
    world = SimWorld(config, start_utc_ms=UTC)

    recorder = MissionRecorder(
        RecorderConfig(missions_root=tmp_path, write_live_svlog=True)
    )
    recorder.start("live", UTC)

    from asket_sim.core.raw_stream import encode_sim_ping

    for step in range(600):
        world.step(0.05)
        vessel = world.vessel.sample(world.utc_ms)
        recorder.write_trajectory({
            "utc_ms": world.utc_ms, "lat": vessel.lat, "lon": vessel.lon, "alt": 0.0,
            "heading_deg": vessel.heading_deg, "heading_source": "magnetometer",
            "heading_valid": True, "cog_deg": vessel.cog_deg, "sog_ms": vessel.sog_ms,
            "roll_deg": vessel.roll_deg, "pitch_deg": vessel.pitch_deg,
            "gnss_fix_type": 3, "num_sats": 14, "hdop": 0.8,
        })
        for ping in world.take_pings():
            recorder.write_sonar(encode_sim_ping(ping))
    recorder.stop(world.utc_ms)

    directory = tmp_path / [p.name for p in tmp_path.iterdir()][0]
    live = directory / LIVE_SVLOG_NAME
    assert live.is_file()

    decoded = parse(live)
    point_sets = [m for f, m in decoded if isinstance(m, pp.PointSet)]
    positions = [
        m for f, m in decoded
        if f.message_id == pp.MSG_NMEA_WRAPPER and m.startswith("$GPGGA")
    ]
    assert len(point_sets) > 20
    assert len(positions) == len(point_sets)

    # And the raw stream is untouched by any of it.
    raw_frames = [m for f, m in parse(directory / SONAR_RAW_NAME) if isinstance(m, pp.PointSet)]
    assert len(raw_frames) == len(point_sets)
    assert not any(
        f.message_id == pp.MSG_NMEA_WRAPPER for f, _ in parse(directory / SONAR_RAW_NAME)
    ), "the raw file must stay exactly as the device produced it"


def test_live_interleaving_never_splits_a_sonar_frame(tmp_path):
    """Sonar bytes arrive in arbitrary chunks. Injecting navigation at a chunk
    boundary would cut a ping in half and corrupt the log."""
    from mission_recorder.core.svlog import LiveSvlogWriter, TrajectorySample

    config = WorldConfig()
    config.sonar.ping_rate_hz = 5.0
    raw, _ = generate_raw_stream(SimWorld(config, start_utc_ms=UTC), 10.0)

    output = tmp_path / "live.svlog"
    with output.open("wb") as handle:
        writer = LiveSvlogWriter(handle)
        # Feed it in awkward little pieces, straddling every frame boundary.
        for offset in range(0, len(raw), 37):
            writer.set_position(
                TrajectorySample(UTC + offset, -22.9, 14.5, 20.0, True, 14, 0.8)
            )
            writer.feed_sonar(raw[offset : offset + 37])

    parser = PingParser()
    frames = parser.feed(output.read_bytes())
    assert parser.stats.checksum_errors == 0
    assert parser.stats.bytes_discarded == 0, "a frame was split"
    assert len([f for f in frames if f.message_id == pp.MSG_OS3D_POINT_SET]) > 30
