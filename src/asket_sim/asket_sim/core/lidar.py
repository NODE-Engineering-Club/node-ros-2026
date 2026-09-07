"""Simulated RPLiDAR S2L.

Produces a 360 degree scan against a handful of static obstacles, with the two
degradations that matter for the GUI:

* **wave noise** — the scanner is on a small hull that rolls, so beams tilt into
  the water and return short. This is the reason the lidar panel has a
  raw/filtered toggle: during debugging you must be able to tell whether the
  sensor or the filter is at fault (brief, section 7.4).
* **dropouts** — no return at all on some beams, which must render as absence,
  not as a range of zero.

Ranges are in metres in the vessel body frame, angle 0 = dead ahead, increasing
clockwise (starboard positive), matching the heading convention used everywhere
else in this repository.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field


@dataclass
class Obstacle:
    """A circular obstacle in local ENU metres — a buoy, a moored boat, a rock."""

    east_m: float
    north_m: float
    radius_m: float
    name: str = ""


@dataclass
class LidarConfig:
    num_beams: int = 720          # 0.5 degree resolution
    max_range_m: float = 30.0
    min_range_m: float = 0.2
    range_noise_m: float = 0.02
    rotation_hz: float = 10.0
    #: Probability a beam returns nothing at all, before wave effects.
    dropout_probability: float = 0.01
    #: Roll above which beams start hitting water and returning short.
    wave_clutter_roll_deg: float = 4.0
    wave_clutter_probability: float = 0.06
    #: Hull silhouette half-dimensions, metres. Drawn by the GUI, and used to
    #: reject returns from the boat's own structure.
    hull_length_m: float = 1.6
    hull_beam_m: float = 0.9


@dataclass
class LidarScan:
    utc_ms: int
    angle_min_deg: float
    angle_increment_deg: float
    #: ``None`` where the beam returned nothing. Never 0.0 — a zero range is a
    #: measurement, an absent return is not.
    ranges_m: list[float | None]
    filtered_m: list[float | None]
    rotation_hz: float
    points_per_revolution: int

    def nearest(self, use_filtered: bool = True) -> tuple[float, float] | None:
        """(range_m, bearing_deg) of the closest return, or None if the scan is empty."""
        src = self.filtered_m if use_filtered else self.ranges_m
        best = None
        for i, r in enumerate(src):
            if r is None:
                continue
            if best is None or r < best[0]:
                best = (r, self.angle_min_deg + i * self.angle_increment_deg)
        return best


class LidarSim:
    def __init__(
        self,
        obstacles: list[Obstacle] | None = None,
        config: LidarConfig | None = None,
        seed: int = 2,
    ) -> None:
        self.obstacles = obstacles or []
        self.cfg = config or LidarConfig()
        self._rng = random.Random(seed)

    def scan(
        self,
        utc_ms: int,
        east_m: float,
        north_m: float,
        heading_deg: float,
        roll_deg: float = 0.0,
    ) -> LidarScan:
        cfg = self.cfg
        inc = 360.0 / cfg.num_beams
        raw: list[float | None] = []

        clutter_p = cfg.wave_clutter_probability * max(
            0.0, abs(roll_deg) - cfg.wave_clutter_roll_deg
        ) / max(1e-6, cfg.wave_clutter_roll_deg)

        for i in range(cfg.num_beams):
            bearing_body = i * inc
            bearing_world = (heading_deg + bearing_body) % 360.0
            r = self._raycast(east_m, north_m, bearing_world)

            if r is not None:
                r += self._rng.gauss(0.0, cfg.range_noise_m)

            # Wave clutter: a short spurious return, indistinguishable from a
            # real obstacle until you look at its temporal consistency.
            if self._rng.random() < clutter_p:
                r = self._rng.uniform(0.5, 4.0)

            if r is not None and (r < cfg.min_range_m or r > cfg.max_range_m):
                r = None
            if self._rng.random() < cfg.dropout_probability:
                r = None
            raw.append(r)

        return LidarScan(
            utc_ms=utc_ms,
            angle_min_deg=0.0,
            angle_increment_deg=inc,
            ranges_m=raw,
            filtered_m=filter_scan(raw, inc),
            rotation_hz=cfg.rotation_hz + self._rng.gauss(0.0, 0.05),
            points_per_revolution=sum(1 for r in raw if r is not None),
        )

    def _raycast(self, ox: float, oy: float, bearing_deg: float) -> float | None:
        """Distance to the first obstacle along a bearing, or None."""
        rad = math.radians(bearing_deg)
        dx, dy = math.sin(rad), math.cos(rad)  # east, north components
        best: float | None = None
        for obs in self.obstacles:
            # Ray-circle intersection.
            fx = ox - obs.east_m
            fy = oy - obs.north_m
            b = 2.0 * (fx * dx + fy * dy)
            c = fx * fx + fy * fy - obs.radius_m * obs.radius_m
            disc = b * b - 4.0 * c
            if disc < 0.0:
                continue
            sq = math.sqrt(disc)
            for t in ((-b - sq) / 2.0, (-b + sq) / 2.0):
                if t > 0.0 and (best is None or t < best):
                    best = t
        return best


def filter_scan(
    ranges: list[float | None],
    angle_increment_deg: float,
    min_cluster: int = 3,
    max_jump_m: float = 0.8,
) -> list[float | None]:
    """Drop isolated returns.

    A real obstacle subtends several consecutive beams. A single beam with
    neighbours far away (or absent) is wave clutter. Keeping this crude and
    legible is the point — the operator can compare it against the raw set and
    see exactly what was removed.
    """
    n = len(ranges)
    keep: list[float | None] = [None] * n
    i = 0
    while i < n:
        if ranges[i] is None:
            i += 1
            continue
        j = i
        while (
            j + 1 < n
            and ranges[j + 1] is not None
            and abs(ranges[j + 1] - ranges[j]) < max_jump_m
        ):
            j += 1
        if (j - i + 1) >= min_cluster:
            for k in range(i, j + 1):
                keep[k] = ranges[k]
        i = j + 1
    return keep
