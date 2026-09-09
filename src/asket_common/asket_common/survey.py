"""Survey line geometry and swath coverage.

Shared by three consumers that must agree exactly:

* ``asket_sim`` drives the simulated vessel along these lines;
* ``gui_backend`` publishes them to the map as the planned pattern;
* the coverage overlay paints the swath actually achieved against them.

If these three ever disagree the coverage overlay lies, and with a single
sonar unit covering one side only, a lying coverage overlay is the most
expensive bug in the system — you find the gap back in Windhoek.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

from .geo import LocalOrigin, bearing_to_enu, wrap360

#: Which side of the hull the single Omniscan looks at. One unit means one side;
#: see the brief, section 4.
SIDE_STARBOARD = "starboard"
SIDE_PORT = "port"


@dataclass(frozen=True)
class Waypoint:
    """A point on the planned track, in local ENU metres."""

    east_m: float
    north_m: float
    #: Index of the survey line this waypoint belongs to; ``-1`` for turns.
    line_index: int = -1
    #: True when the leg *arriving* at this waypoint is on-survey (recording
    #: useful swath) rather than a turn.
    on_survey: bool = False


@dataclass
class SurveyPlan:
    """A lawnmower pattern over a rectangular box."""

    origin: LocalOrigin
    #: Compass bearing of the survey lines (the long axis of the box).
    heading_deg: float
    line_length_m: float
    line_spacing_m: float
    num_lines: int
    #: East/north of the start of the first line, in local ENU.
    start_east_m: float = 0.0
    start_north_m: float = 0.0
    #: Which side the sonar looks at; determines where the swath is painted.
    sonar_side: str = SIDE_STARBOARD
    #: Half-width of the ensonified swath on that side, metres. Derived from
    #: range and mounting angle by :func:`swath_half_width_m`.
    swath_width_m: float = 25.0

    waypoints: list[Waypoint] = field(default_factory=list)

    def __post_init__(self) -> None:
        if not self.waypoints:
            self.waypoints = self._build()

    def _build(self) -> list[Waypoint]:
        """Boustrophedon: up one line, across the spacing, down the next."""
        along_e, along_n = bearing_to_enu(self.heading_deg, 1.0)
        # Perpendicular, 90 deg clockwise from the line direction.
        across_e, across_n = bearing_to_enu(wrap360(self.heading_deg + 90.0), 1.0)

        points: list[Waypoint] = []
        for i in range(self.num_lines):
            offset = i * self.line_spacing_m
            base_e = self.start_east_m + across_e * offset
            base_n = self.start_north_m + across_n * offset
            far_e = base_e + along_e * self.line_length_m
            far_n = base_n + along_n * self.line_length_m

            # Reverse every other line so the vessel does not fly back to the
            # start of each one.
            a, b = ((base_e, base_n), (far_e, far_n))
            if i % 2 == 1:
                a, b = b, a

            # The leg that *arrives* at `a` is the turn from the previous line.
            points.append(Waypoint(a[0], a[1], line_index=i, on_survey=False))
            points.append(Waypoint(b[0], b[1], line_index=i, on_survey=True))
        return points

    # -- geographic views -------------------------------------------------

    def line_segments_geo(self) -> list[list[tuple[float, float]]]:
        """The survey lines as [[ (lat, lon), (lat, lon) ], ...] for the map."""
        segs = []
        for i in range(0, len(self.waypoints) - 1, 2):
            a, b = self.waypoints[i], self.waypoints[i + 1]
            segs.append(
                [
                    self.origin.to_geo(a.east_m, a.north_m),
                    self.origin.to_geo(b.east_m, b.north_m),
                ]
            )
        return segs

    def total_survey_distance_m(self) -> float:
        """On-survey distance only; turns are not survey."""
        return self.line_length_m * self.num_lines

    def total_track_distance_m(self) -> float:
        """Everything the vessel actually has to drive, turns included."""
        turns = max(0, self.num_lines - 1) * self.line_spacing_m
        return self.total_survey_distance_m() + turns


def swath_half_width_m(range_m: float, mounting_tilt_deg: float, depth_m: float) -> float:
    """Lateral reach of the swath on the ensonified side.

    A fan tilted ``mounting_tilt_deg`` from vertical and reaching ``range_m``
    puts its far edge at ``range_m * sin(tilt)`` laterally — but only if the
    seabed is deep enough to still be there. In shallow water the seabed
    intercepts the beam first and the swath is bounded by geometry instead.

    Returns the lesser of the two, which is the honest number to paint.
    """
    tilt = math.radians(mounting_tilt_deg)
    by_range = range_m * math.sin(tilt)
    if math.cos(tilt) <= 1e-6:
        return by_range
    # Slant range needed to reach the seabed at this tilt.
    slant_to_seabed = depth_m / math.cos(tilt)
    by_depth = min(range_m, slant_to_seabed) * math.sin(tilt)
    return max(0.0, min(by_range, by_depth))


def swath_polygon(
    east_m: float,
    north_m: float,
    heading_deg: float,
    half_width_m: float,
    side: str,
    inner_gap_m: float = 0.0,
) -> list[tuple[float, float]]:
    """The ENU quad of swath ensonified at one instant, as 2 offsets.

    Returns the near and far lateral edge points for this vessel position. The
    coverage layer stitches consecutive pairs into a ribbon; doing it that way
    rather than accumulating a polygon means a gap in the data becomes a gap in
    the ribbon, which is exactly what we need an operator to see.

    ``inner_gap_m`` is the nadir gap: directly under the hull a tilted fan sees
    nothing.
    """
    sign = 1.0 if side == SIDE_STARBOARD else -1.0
    across_e, across_n = bearing_to_enu(wrap360(heading_deg + 90.0 * sign), 1.0)
    near = (east_m + across_e * inner_gap_m, north_m + across_n * inner_gap_m)
    far = (east_m + across_e * half_width_m, north_m + across_n * half_width_m)
    return [near, far]
