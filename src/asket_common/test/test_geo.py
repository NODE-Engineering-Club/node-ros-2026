import math

from asket_common.geo import (
    LocalOrigin,
    angular_difference,
    bearing_to_enu,
    distance_m,
    enu_bearing_deg,
    wrap180,
    wrap360,
)


def test_wrapping():
    assert wrap180(190) == -170
    assert wrap180(-190) == 170
    assert wrap360(-10) == 350
    assert wrap360(370) == 10


def test_angular_difference_takes_the_short_way_round():
    assert angular_difference(10, 350) == 20
    assert angular_difference(350, 10) == -20


def test_enu_round_trip():
    origin = LocalOrigin(-22.9576, 14.5053)
    for east, north in [(0, 0), (500, -1200), (-3000, 2500)]:
        lat, lon = origin.to_geo(east, north)
        back = origin.to_enu(lat, lon)
        assert abs(back[0] - east) < 1e-6
        assert abs(back[1] - north) < 1e-6


def test_a_degree_of_latitude_is_about_111_km():
    origin = LocalOrigin(0.0, 0.0)
    _, north = origin.to_enu(1.0, 0.0)
    assert 110_000 < north < 112_000


def test_longitude_scale_shrinks_towards_the_pole():
    equator = LocalOrigin(0.0, 0.0).to_enu(0.0, 1.0)[0]
    namibia = LocalOrigin(-22.9576, 0.0).to_enu(-22.9576, 1.0)[0]
    assert namibia < equator
    assert abs(namibia / equator - math.cos(math.radians(22.9576))) < 1e-6


def test_bearings_are_compass_bearings():
    assert abs(enu_bearing_deg(0, 10) - 0.0) < 1e-9      # north
    assert abs(enu_bearing_deg(10, 0) - 90.0) < 1e-9     # east
    assert abs(enu_bearing_deg(0, -10) - 180.0) < 1e-9   # south
    assert abs(enu_bearing_deg(-10, 0) - 270.0) < 1e-9   # west


def test_bearing_round_trip():
    for bearing in (0, 45, 137, 289, 359):
        e, n = bearing_to_enu(bearing, 100.0)
        assert abs(enu_bearing_deg(e, n) - bearing) < 1e-9
        assert abs(distance_m((0, 0), (e, n)) - 100.0) < 1e-9
