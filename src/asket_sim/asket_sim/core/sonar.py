"""Simulated Cerulean Omniscan 3D.

Generates the quantity the real sonar actually produces: an array of
``(angle, time-of-flight, power, type)`` tuples in the **sensor frame**. It does
not produce XYZ. That conversion is ours to do and lives in
``omniscan_bridge.core.geometry``; keeping the simulator on the far side of that
boundary means the conversion is genuinely exercised rather than bypassed.

The seabed is a synthetic surface: a base depth, a gentle cross-slope, and a
sand-ripple term. Enough structure that a coverage overlay and a range setting
have something to be right or wrong about.

Geometry, per ping, for one beam at sensor angle ``theta``:

    total angle from vertical = theta + mounting_tilt + vessel_roll
    slant range               = depth / cos(total)
    lateral offset            = depth * tan(total)

Depth varies laterally, so the lateral offset is solved by two fixed-point
iterations — more than enough at these gradients.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

#: ``pt_type`` values, matching Cerulean's OS3D_POINT_SET definition:
#: 0 = unclassified, 1 = bottom, 2 = water column. A beam with no detection is
#: reported as unclassified, which is what the real device does — it is not a
#: separate "no return" code.
PT_TYPE_UNCLASSIFIED = 0
PT_TYPE_BOTTOM = 1
PT_TYPE_WATER_COLUMN = 2


@dataclass
class SeabedConfig:
    """Synthetic seabed. Depths are PROVISIONAL — see docs/open_questions.md Q1."""

    base_depth_m: float = 18.0
    #: Depth change per metre of northing (a gentle offshore slope).
    slope_north: float = 0.012
    slope_east: float = -0.004
    ripple_amplitude_m: float = 0.35
    ripple_wavelength_m: float = 14.0
    min_depth_m: float = 4.0
    max_depth_m: float = 60.0

    def depth_at(self, east_m: float, north_m: float) -> float:
        d = (
            self.base_depth_m
            + self.slope_north * north_m
            + self.slope_east * east_m
            + self.ripple_amplitude_m
            * math.sin(2.0 * math.pi * east_m / self.ripple_wavelength_m)
            * math.cos(2.0 * math.pi * north_m / (self.ripple_wavelength_m * 1.7))
        )
        return max(self.min_depth_m, min(self.max_depth_m, d))


@dataclass
class SonarSimConfig:
    ping_rate_hz: float = 5.0
    points_per_ping: int = 256
    #: Total across-track aperture of the fan, degrees.
    aperture_deg: float = 60.0
    #: Downward tilt of the fan's centre from vertical. PROVISIONAL, Q2.
    mounting_tilt_deg: float = 35.0
    range_setting_m: float = 30.0
    gain: int = 4
    speed_of_sound_ms: float = 1500.0
    #: Proportion of beams that fail to detect bottom in good conditions.
    dropout_probability: float = 0.02
    range_noise_m: float = 0.03
    #: Simulated sonar clock error against the Jetson, milliseconds. Nonzero
    #: values are how we test that the operator gets warned before a whole
    #: dataset is quietly ruined.
    clock_offset_ms: int = 0


@dataclass
class SonarPoint:
    angle_rad: float
    tof_s: float
    power: float
    pt_type: int


@dataclass
class PingSet:
    """One ``OS3D_POINT_SET`` worth of data, before it is put on the wire."""

    ping_number: int
    utc_ms: int
    speed_of_sound_ms: float
    points: list[SonarPoint] = field(default_factory=list)


class SonarSim:
    def __init__(
        self,
        seabed: SeabedConfig | None = None,
        config: SonarSimConfig | None = None,
        seed: int = 3,
    ) -> None:
        self.seabed = seabed or SeabedConfig()
        self.cfg = config or SonarSimConfig()
        self._rng = random.Random(seed)
        self.ping_number = 0
        #: What the host last commanded via ``ping_enable``. Kept separate from
        #: :attr:`pinging` because a simulated dropout must model the sonar
        #: failing, not the operator's command being forgotten — otherwise
        #: clearing the fault would leave it stopped, or injecting one would
        #: look like the command had never been sent.
        self.ping_enabled_by_command = True
        self.pinging = True
        #: Ping numbers deliberately skipped, so packet-loss estimation has
        #: something real to detect.
        self.drop_next = 0

    def ping(
        self,
        utc_ms: int,
        east_m: float,
        north_m: float,
        heading_deg: float,
        roll_deg: float = 0.0,
        side_sign: float = 1.0,
    ) -> PingSet | None:
        """Produce one ping, or ``None`` if the sonar is not currently pinging.

        ``side_sign`` is +1 for a starboard-looking install, -1 for port.
        """
        if not self.pinging:
            return None

        self.ping_number += 1
        if self.drop_next > 0:
            # Emitted nothing, but the ping number still advanced: exactly what
            # a lost packet looks like downstream.
            self.drop_next -= 1
            return None

        cfg = self.cfg
        n = cfg.points_per_ping
        half = math.radians(cfg.aperture_deg / 2.0)
        tilt = math.radians(cfg.mounting_tilt_deg) * side_sign
        roll = math.radians(roll_deg)

        # Across-track unit vector in ENU, on the ensonified side.
        across = math.radians((heading_deg + 90.0 * side_sign) % 360.0)
        ax, ay = math.sin(across), math.cos(across)

        points: list[SonarPoint] = []
        for i in range(n):
            theta = -half + (2.0 * half) * i / max(1, n - 1)
            total = theta + tilt + roll

            if abs(total) >= math.radians(88.0):
                points.append(SonarPoint(theta, 0.0, 0.0, PT_TYPE_UNCLASSIFIED))
                continue

            # Two fixed-point iterations for the laterally varying depth.
            depth = self.seabed.depth_at(east_m, north_m)
            for _ in range(2):
                lateral = depth * math.tan(total)
                depth = self.seabed.depth_at(
                    east_m + ax * lateral, north_m + ay * lateral
                )

            slant = depth / math.cos(total) + self._rng.gauss(0.0, cfg.range_noise_m)

            if slant > cfg.range_setting_m or self._rng.random() < cfg.dropout_probability:
                points.append(SonarPoint(theta, 0.0, 0.0, PT_TYPE_UNCLASSIFIED))
                continue

            # Power: spreading loss, plus a grazing-angle term. Steeper
            # incidence backscatters more, which is why nadir is bright.
            grazing = max(0.05, math.cos(total))
            power = min(1.0, (8.0 / max(1.0, slant)) * grazing) * self._rng.uniform(0.8, 1.0)

            tof = 2.0 * slant / cfg.speed_of_sound_ms
            points.append(SonarPoint(theta, tof, power, PT_TYPE_BOTTOM))

        return PingSet(
            ping_number=self.ping_number,
            utc_ms=utc_ms + cfg.clock_offset_ms,
            speed_of_sound_ms=cfg.speed_of_sound_ms,
            points=points,
        )

    def attitude(self, roll_deg: float, pitch_deg: float) -> tuple[float, float]:
        """Sonar-internal attitude report, with its own small bias and noise."""
        return (
            pitch_deg + self._rng.gauss(0.0, 0.1),
            roll_deg + self._rng.gauss(0.0, 0.1),
        )
