"""Geodetic helpers.

Scope is deliberately tiny. A survey box is a few kilometres across, so a local
tangent-plane (equirectangular) approximation about a mission origin is accurate
to well under the errors we actually care about — heading error alone is
~90 cm at 50 m range (see docs/safety.md and the brief, section 6). Bringing in
pyproj or geographiclib to chase millimetres we cannot measure would add an
arm64 build dependency for no benefit.

Conventions used throughout this repository:

* **Geographic** — WGS84 latitude/longitude in degrees, altitude in metres.
* **Local ENU** — east / north / up in metres, relative to a mission origin.
* **Heading** — compass degrees, 0 = true north, increasing clockwise.
  (ROS yaw is counter-clockwise from +x; the conversion happens only at the ROS
  boundary, never in the middle of the logic.)
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: Mean Earth radius, metres. WGS84 semi-major would be overkill here.
EARTH_RADIUS_M = 6_371_008.8


def wrap180(deg: float) -> float:
    """Wrap an angle to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def wrap360(deg: float) -> float:
    """Wrap an angle to [0, 360)."""
    return deg % 360.0


def angular_difference(a_deg: float, b_deg: float) -> float:
    """Signed smallest difference ``a - b``, in [-180, 180).

    Used for heading-vs-COG divergence, which is the single most useful
    heading-quality diagnostic we have (brief, section 6).
    """
    return wrap180(a_deg - b_deg)


@dataclass(frozen=True)
class LocalOrigin:
    """A tangent-plane origin. Fix one per mission and never move it."""

    lat_deg: float
    lon_deg: float

    @property
    def _metres_per_deg_lat(self) -> float:
        return math.pi * EARTH_RADIUS_M / 180.0

    @property
    def _metres_per_deg_lon(self) -> float:
        return self._metres_per_deg_lat * math.cos(math.radians(self.lat_deg))

    def to_enu(self, lat_deg: float, lon_deg: float) -> tuple[float, float]:
        """Geographic -> local east/north metres."""
        east = (lon_deg - self.lon_deg) * self._metres_per_deg_lon
        north = (lat_deg - self.lat_deg) * self._metres_per_deg_lat
        return east, north

    def to_geo(self, east_m: float, north_m: float) -> tuple[float, float]:
        """Local east/north metres -> geographic."""
        lat = self.lat_deg + north_m / self._metres_per_deg_lat
        lon = self.lon_deg + east_m / self._metres_per_deg_lon
        return lat, lon


def enu_bearing_deg(east_m: float, north_m: float) -> float:
    """Compass bearing of an ENU vector, degrees clockwise from north."""
    return wrap360(math.degrees(math.atan2(east_m, north_m)))


def bearing_to_enu(bearing_deg: float, distance_m: float) -> tuple[float, float]:
    """Inverse of :func:`enu_bearing_deg`: a compass bearing to an ENU offset."""
    rad = math.radians(bearing_deg)
    return distance_m * math.sin(rad), distance_m * math.cos(rad)


def distance_m(a: tuple[float, float], b: tuple[float, float]) -> float:
    """Planar distance between two ENU points."""
    return math.hypot(b[0] - a[0], b[1] - a[1])
