import math

from asket_common.geo import LocalOrigin, distance_m
from asket_common.survey import (
    SIDE_PORT,
    SIDE_STARBOARD,
    SurveyPlan,
    swath_half_width_m,
    swath_polygon,
)

ORIGIN = LocalOrigin(-22.9576, 14.5053)


def plan(**kw):
    kw.setdefault("heading_deg", 0.0)
    kw.setdefault("line_length_m", 200.0)
    kw.setdefault("line_spacing_m", 25.0)
    kw.setdefault("num_lines", 4)
    return SurveyPlan(origin=ORIGIN, **kw)


def test_lawnmower_alternates_direction():
    p = plan()
    assert len(p.waypoints) == 8
    first_leg = p.waypoints[1].north_m - p.waypoints[0].north_m
    second_leg = p.waypoints[3].north_m - p.waypoints[2].north_m
    assert first_leg > 0 > second_leg


def test_lines_are_spaced_as_asked():
    p = plan(line_spacing_m=25.0)
    assert abs(p.waypoints[2].east_m - p.waypoints[0].east_m - 25.0) < 1e-9


def test_lines_follow_the_survey_heading():
    p = plan(heading_deg=90.0, line_length_m=100.0)
    a, b = p.waypoints[0], p.waypoints[1]
    assert abs(b.east_m - a.east_m - 100.0) < 1e-6
    assert abs(b.north_m - a.north_m) < 1e-6


def test_only_the_along_line_legs_count_as_survey():
    p = plan()
    on = [w for w in p.waypoints if w.on_survey]
    assert len(on) == 4
    assert p.total_survey_distance_m() == 800.0
    assert p.total_track_distance_m() == 800.0 + 3 * 25.0


def test_geographic_segments_have_the_right_length():
    p = plan(line_length_m=200.0)
    segs = p.line_segments_geo()
    assert len(segs) == 4
    (lat1, lon1), (lat2, lon2) = segs[0]
    e1, n1 = ORIGIN.to_enu(lat1, lon1)
    e2, n2 = ORIGIN.to_enu(lat2, lon2)
    assert abs(distance_m((e1, n1), (e2, n2)) - 200.0) < 0.5


def test_swath_is_bounded_by_range_in_deep_water():
    # 30 m range, 35 deg tilt, plenty of depth: range is the limit.
    assert abs(swath_half_width_m(30.0, 35.0, 100.0) - 30.0 * math.sin(math.radians(35))) < 1e-6


def test_swath_is_bounded_by_depth_in_shallow_water():
    """In shallow water the seabed intercepts the beam before the range does.
    Painting the range-limited width there would claim coverage we do not have."""
    shallow = swath_half_width_m(30.0, 35.0, 5.0)
    deep = swath_half_width_m(30.0, 35.0, 100.0)
    assert shallow < deep
    assert abs(shallow - 5.0 * math.tan(math.radians(35))) < 1e-6


def test_swath_is_painted_on_the_configured_side():
    """One sonar unit means one side. Painting the wrong one hides every gap."""
    stbd = swath_polygon(0.0, 0.0, heading_deg=0.0, half_width_m=20.0, side=SIDE_STARBOARD)
    port = swath_polygon(0.0, 0.0, heading_deg=0.0, half_width_m=20.0, side=SIDE_PORT)
    assert stbd[1][0] > 19.9      # east of the vessel when heading north
    assert port[1][0] < -19.9


def test_nadir_gap_is_left_unpainted():
    near, far = swath_polygon(0, 0, 0.0, 20.0, SIDE_STARBOARD, inner_gap_m=6.0)
    assert abs(near[0] - 6.0) < 1e-9
    assert abs(far[0] - 20.0) < 1e-9
