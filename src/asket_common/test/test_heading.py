"""Heading is the dominant error source. These tests pin down what we claim."""

import math

from asket_common.heading import (
    DIVERGENCE_WARN_DEG,
    SOURCE_GNSS_COMPASS,
    SOURCE_MAGNETOMETER,
    SOURCE_NONE,
    evaluate_heading,
)


def test_one_degree_is_about_ninety_centimetres_at_fifty_metres():
    """The number that makes this whole module worth having."""
    est = evaluate_heading(0.0, SOURCE_GNSS_COMPASS, 0.0, 2.0, reported_accuracy_deg=1.0)
    assert abs(est.position_error_at_m(50.0) - 0.87) < 0.02


def test_magnetometer_error_costs_metres_where_the_compass_costs_centimetres():
    mag = evaluate_heading(10.0, SOURCE_MAGNETOMETER, 10.0, 2.0)
    gnss = evaluate_heading(10.0, SOURCE_GNSS_COMPASS, 10.0, 2.0)
    assert mag.position_error_at_m(50.0) > 5.0
    assert gnss.position_error_at_m(50.0) < 0.25


def test_divergence_is_meaningless_at_rest():
    """Sitting still with a 40 degree 'divergence' is not a fault."""
    est = evaluate_heading(100.0, SOURCE_MAGNETOMETER, 140.0, sog_ms=0.05)
    assert not est.divergence_meaningful
    assert not est.divergence_suspicious


def test_divergence_at_speed_is_flagged():
    est = evaluate_heading(100.0, SOURCE_MAGNETOMETER, 140.0, sog_ms=1.5)
    assert est.divergence_meaningful
    assert abs(est.divergence_deg + 40.0) < 1e-9
    assert est.divergence_suspicious


def test_small_divergence_is_not_flagged():
    est = evaluate_heading(100.0, SOURCE_MAGNETOMETER, 100.0 - DIVERGENCE_WARN_DEG / 2,
                           sog_ms=1.5)
    assert not est.divergence_suspicious


def test_invalid_source_produces_no_heading_and_no_false_confidence():
    est = evaluate_heading(None, SOURCE_NONE, 90.0, 1.5)
    assert not est.valid
    assert est.source == SOURCE_NONE
    assert math.isnan(est.accuracy_deg)
    assert math.isnan(est.position_error_at_m(50.0))


def test_a_source_marked_invalid_is_not_trusted_even_with_a_value():
    """The heading_invalid fault must not leave a plausible number on screen."""
    est = evaluate_heading(123.0, SOURCE_GNSS_COMPASS, 120.0, 1.5, source_valid=False)
    assert not est.valid
    assert est.source == SOURCE_NONE


def test_a_reported_accuracy_beats_the_nominal_one():
    est = evaluate_heading(10.0, SOURCE_MAGNETOMETER, 10.0, 2.0, reported_accuracy_deg=2.5)
    assert est.accuracy_deg == 2.5
