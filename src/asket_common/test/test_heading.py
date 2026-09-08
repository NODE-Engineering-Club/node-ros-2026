"""Heading is the dominant error source. These tests pin down what we claim."""

import math

from asket_common.heading import (
    DIVERGENCE_WARN_DEG,
    NOMINAL_ACCURACY_DEG,
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


def test_an_assumed_accuracy_is_marked_as_assumed():
    """A figure the receiver reported and one we picked for its class of
    hardware are worth different amounts, so they must not display alike
    (docs/open_questions.md Q4)."""
    reported = evaluate_heading(10.0, SOURCE_GNSS_COMPASS, 10.0, 2.0,
                                reported_accuracy_deg=0.9)
    assumed = evaluate_heading(10.0, SOURCE_GNSS_COMPASS, 10.0, 2.0)
    assert reported.accuracy_reported and reported.accuracy_deg == 0.9
    assert not assumed.accuracy_reported
    assert assumed.accuracy_deg == NOMINAL_ACCURACY_DEG[SOURCE_GNSS_COMPASS]


def test_a_degraded_accuracy_from_the_receiver_is_never_replaced_by_the_nominal():
    """A UM982 that has lost one antenna reports a much worse figure. Silently
    substituting 0.2 degrees would hide the failure worth seeing."""
    est = evaluate_heading(10.0, SOURCE_GNSS_COMPASS, 10.0, 2.0,
                           reported_accuracy_deg=14.0)
    assert est.accuracy_deg == 14.0
    assert est.accuracy_reported
    # 14 degrees at 50 m is over 12 m of seabed error. That is the point.
    assert est.position_error_at_m(50.0) > 12.0


def test_a_nan_from_the_receiver_counts_as_no_report_not_as_a_measurement():
    est = evaluate_heading(10.0, SOURCE_MAGNETOMETER, 10.0, 2.0,
                           reported_accuracy_deg=float("nan"))
    assert not est.accuracy_reported
    assert est.accuracy_deg == NOMINAL_ACCURACY_DEG[SOURCE_MAGNETOMETER]


def test_an_invalid_heading_reports_no_accuracy_at_all():
    est = evaluate_heading(None, SOURCE_NONE, 10.0, 2.0, reported_accuracy_deg=0.2)
    assert not est.accuracy_reported
    assert math.isnan(est.accuracy_deg)
