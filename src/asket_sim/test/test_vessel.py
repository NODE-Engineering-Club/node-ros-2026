"""Vessel simulation: does it produce a plausible, honest survey?"""

import math

from asket_common.geo import LocalOrigin, angular_difference
from asket_common.survey import SurveyPlan
from asket_sim.core.vessel import HEADING_SOURCE_GNSS, VesselConfig, VesselSim


def make_sim(**cfg_kwargs):
    plan = SurveyPlan(
        origin=LocalOrigin(-22.9576, 14.5053),
        heading_deg=20.0,
        line_length_m=150.0,
        line_spacing_m=20.0,
        num_lines=3,
    )
    return VesselSim(plan, VesselConfig(**cfg_kwargs))


def run(sim, seconds, dt=0.1):
    for _ in range(int(seconds / dt)):
        sim.step(dt)


def test_completes_the_pattern():
    sim = make_sim()
    run(sim, 900)
    assert sim.finished
    # 3 lines of 150 m plus two 20 m turns, give or take the turning circles.
    assert 450 <= sim.distance_travelled <= 620


def test_visits_every_waypoint_in_order():
    sim = make_sim()
    seen = []
    for _ in range(9000):
        sim.step(0.1)
        if not seen or seen[-1] != sim.wp_index:
            seen.append(sim.wp_index)
    assert seen == sorted(seen), "waypoints must not be revisited or skipped"
    assert seen[-1] >= len(sim.plan.waypoints) - 1


def test_magnetometer_error_grows_with_throttle():
    """The failure mode that motivates the GNSS compass, reproduced."""
    sim = make_sim(mag_throttle_gain_deg=20.0, mag_noise_deg=0.0, mag_bias_deg=0.0)
    run(sim, 60)

    sim.throttle = 0.0
    low = abs(angular_difference(sim.sample(0).heading_deg, sim.true_heading))
    sim.throttle = 1.0
    high = abs(angular_difference(sim.sample(0).heading_deg, sim.true_heading))
    assert high > low + 5.0


def test_gnss_compass_is_an_order_of_magnitude_better():
    mag = make_sim()
    gnss = make_sim(heading_source=HEADING_SOURCE_GNSS)
    run(mag, 120)
    run(gnss, 120)

    mag_err = abs(angular_difference(mag.sample(0).heading_deg, mag.true_heading))
    gnss_err = abs(angular_difference(gnss.sample(0).heading_deg, gnss.true_heading))
    assert gnss_err < mag_err


def test_current_makes_cog_diverge_from_heading():
    """With a cross-current, heading and COG must not agree — that divergence is
    the diagnostic the GUI shows, so the simulator has to actually produce it."""
    sim = make_sim(current_speed_ms=0.5, current_bearing_deg=110.0)
    run(sim, 120)
    s = sim.sample(0)
    assert s.sog_ms > 0.5
    assert abs(angular_difference(s.true_heading_deg, s.cog_deg)) > 3.0


def test_heading_invalid_is_reported_not_hidden():
    sim = make_sim()
    run(sim, 30)
    sim.set_heading_valid(False)
    s = sim.sample(0)
    assert s.heading_valid is False
    assert s.heading_source == "none"
    assert math.isnan(s.heading_accuracy_deg)
