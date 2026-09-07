"""Sonar simulation: sensor-frame output, and the range/depth relationship."""

import math

from asket_sim.core.sonar import (
    PT_TYPE_BOTTOM,
    PT_TYPE_NONE,
    SeabedConfig,
    SonarSim,
    SonarSimConfig,
)


def test_emits_angle_and_time_of_flight_not_coordinates():
    """The sonar knows nothing about where it is. It reports angles and times."""
    ping = SonarSim().ping(1000, 0.0, 0.0, 0.0)
    assert ping is not None
    p = next(p for p in ping.points if p.pt_type == PT_TYPE_BOTTOM)
    assert -math.pi < p.angle_rad < math.pi
    assert 0.0 < p.tof_s < 1.0


def test_range_recovered_from_tof_matches_the_seabed():
    seabed = SeabedConfig(base_depth_m=20.0, slope_north=0.0, slope_east=0.0,
                          ripple_amplitude_m=0.0)
    cfg = SonarSimConfig(mounting_tilt_deg=0.0, aperture_deg=0.0, points_per_ping=1,
                         dropout_probability=0.0, range_noise_m=0.0, range_setting_m=50.0)
    ping = SonarSim(seabed, cfg).ping(0, 0.0, 0.0, 0.0)
    p = ping.points[0]
    recovered = p.tof_s * ping.speed_of_sound_ms / 2.0
    assert abs(recovered - 20.0) < 0.01   # straight down, flat bottom


def test_beams_beyond_the_range_setting_return_nothing():
    """The range setting is the reason the coverage overlay can have gaps."""
    seabed = SeabedConfig(base_depth_m=25.0, slope_north=0.0, slope_east=0.0,
                          ripple_amplitude_m=0.0)
    short = SonarSim(seabed, SonarSimConfig(range_setting_m=10.0, dropout_probability=0.0))
    long = SonarSim(seabed, SonarSimConfig(range_setting_m=60.0, dropout_probability=0.0))
    n_short = sum(1 for p in short.ping(0, 0, 0, 0).points if p.pt_type == PT_TYPE_BOTTOM)
    n_long = sum(1 for p in long.ping(0, 0, 0, 0).points if p.pt_type == PT_TYPE_BOTTOM)
    assert n_short == 0
    assert n_long > 100


def test_ping_number_advances_even_when_a_ping_is_dropped():
    """A dropped packet must look like a gap, not like a pause — otherwise the
    bridge's packet-loss estimate has nothing to detect."""
    sim = SonarSim()
    sim.ping(0, 0, 0, 0)
    sim.drop_next = 1
    assert sim.ping(0, 0, 0, 0) is None
    third = sim.ping(0, 0, 0, 0)
    assert third.ping_number == 3


def test_not_pinging_yields_nothing():
    sim = SonarSim()
    sim.pinging = False
    assert sim.ping(0, 0, 0, 0) is None


def test_clock_offset_is_applied_to_the_reported_timestamp():
    """Post-mission fusion pairs the sonar stream and the trajectory on this
    timestamp. If it drifts, the whole dataset is quietly ruined."""
    sim = SonarSim(config=SonarSimConfig(clock_offset_ms=750))
    ping = sim.ping(1_700_000_000_000, 0, 0, 0)
    assert ping.utc_ms == 1_700_000_000_750
