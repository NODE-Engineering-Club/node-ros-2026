"""Simulated vessel dynamics.

Deliberately simple: this is not a hydrodynamics model, it is a source of
*plausible and honest* telemetry so that everything downstream — the map, the
coverage overlay, the recorder, the link decimation — can be built and regression
tested with no hardware present.

What it does model, because these are the things the GUI has to get right:

* a skid-steer hull following a lawnmower pattern at survey speed;
* a **cross-current**, so course-over-ground diverges from heading in a way an
  operator must be able to see;
* a **heading error whose size depends on throttle**, which is exactly the
  magnetometer failure mode on this hull (brief, section 6), and a
  configurable switch to a GNSS-compass error model for comparison;
* wave-driven roll and pitch;
* GNSS quality that can be degraded on demand.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

from asket_common.geo import (
    angular_difference,
    bearing_to_enu,
    enu_bearing_deg,
    wrap180,
    wrap360,
)
from asket_common.survey import SurveyPlan

HEADING_SOURCE_MAG = "magnetometer"
HEADING_SOURCE_GNSS = "gnss_compass"


@dataclass
class VesselConfig:
    survey_speed_ms: float = 1.5
    turn_speed_ms: float = 0.8
    max_turn_rate_dps: float = 25.0
    #: How far ahead the controller aims. Larger = smoother, sloppier lines.
    lookahead_m: float = 6.0
    waypoint_radius_m: float = 4.0

    #: Cross-current, as a compass bearing the water is flowing *towards*.
    current_bearing_deg: float = 210.0
    current_speed_ms: float = 0.25

    heading_source: str = HEADING_SOURCE_MAG
    #: Constant magnetic deviation on this hull, degrees.
    mag_bias_deg: float = 3.0
    #: Extra error per unit throttle — the skid-steer motor-current effect.
    mag_throttle_gain_deg: float = 6.0
    mag_noise_deg: float = 0.8
    #: A dual-antenna GNSS compass at a 1 m baseline. PROVISIONAL, see
    #: docs/open_questions.md Q4.
    gnss_compass_noise_deg: float = 0.2

    wave_height_m: float = 0.4
    wave_period_s: float = 4.5

    #: Nominal GNSS quality when healthy.
    num_sats: int = 14
    hdop: float = 0.8
    fix_type: int = 3  # 3 = 3D fix, matching the usual MAVLink enumeration


@dataclass
class VesselSample:
    utc_ms: int
    lat: float
    lon: float
    alt: float
    east_m: float
    north_m: float
    heading_deg: float
    heading_source: str
    heading_valid: bool
    heading_accuracy_deg: float
    true_heading_deg: float
    cog_deg: float
    sog_ms: float
    roll_deg: float
    pitch_deg: float
    gnss_fix_type: int
    num_sats: int
    hdop: float
    throttle: float
    waypoint_index: int
    on_survey: bool
    distance_travelled_m: float

    @property
    def divergence_deg(self) -> float:
        """Heading minus COG. In a straight line on calm water these agree."""
        return angular_difference(self.heading_deg, self.cog_deg)


class VesselSim:
    def __init__(
        self,
        plan: SurveyPlan,
        config: VesselConfig | None = None,
        seed: int = 1,
    ) -> None:
        self.plan = plan
        self.cfg = config or VesselConfig()
        self._rng = random.Random(seed)

        first = plan.waypoints[0]
        self.east = first.east_m
        self.north = first.north_m
        # Start pointing at the second waypoint so the first line is not a
        # 180 degree pirouette.
        second = plan.waypoints[1]
        self.true_heading = enu_bearing_deg(
            second.east_m - first.east_m, second.north_m - first.north_m
        )
        self.speed = 0.0
        self.t = 0.0
        self.wp_index = 1
        self.distance_travelled = 0.0
        self.throttle = 0.0
        self.finished = False

        self._heading_valid = True
        self._mag_walk = 0.0

    # -- fault hooks ------------------------------------------------------

    def set_heading_valid(self, valid: bool) -> None:
        """Injectable fault: heading goes invalid mid-survey.

        Data recorded during this window is compromised, which is the whole
        reason the trajectory records validity per sample.
        """
        self._heading_valid = valid

    def set_heading_source(self, source: str) -> None:
        self.cfg.heading_source = source

    # -- simulation -------------------------------------------------------

    def step(self, dt: float) -> None:
        self.t += dt
        if self.finished:
            # Coast to a stop rather than teleporting to zero, and keep the
            # reported velocity consistent with it so COG does not freeze at
            # whatever it happened to be on the last leg.
            self.speed = max(0.0, self.speed - 0.5 * dt)
            self.throttle = 0.0
            v_e, v_n = bearing_to_enu(self.true_heading, self.speed)
            c_e, c_n = bearing_to_enu(
                self.cfg.current_bearing_deg, self.cfg.current_speed_ms
            )
            self.east += (v_e + c_e) * dt
            self.north += (v_n + c_n) * dt
            self._last_velocity = (v_e + c_e, v_n + c_n)
            return

        target = self.plan.waypoints[self.wp_index]
        to_e = target.east_m - self.east
        to_n = target.north_m - self.north
        dist = math.hypot(to_e, to_n)

        if dist < self.cfg.waypoint_radius_m:
            self.wp_index += 1
            if self.wp_index >= len(self.plan.waypoints):
                self.finished = True
                return
            target = self.plan.waypoints[self.wp_index]
            to_e = target.east_m - self.east
            to_n = target.north_m - self.north
            dist = math.hypot(to_e, to_n)

        desired_heading = enu_bearing_deg(to_e, to_n)
        error = wrap180(desired_heading - self.true_heading)

        # Turn-rate limited heading control.
        max_step = self.cfg.max_turn_rate_dps * dt
        self.true_heading = wrap360(
            self.true_heading + max(-max_step, min(max_step, error))
        )

        # Slow down for the turns; a skid-steer hull cannot hold survey speed
        # through a 180.
        want = (
            self.cfg.survey_speed_ms
            if abs(error) < 20.0
            else self.cfg.turn_speed_ms
        )
        self.speed += (want - self.speed) * min(1.0, dt * 0.8)

        # Throttle proxy: forward demand plus differential for the turn. This is
        # what drives both battery draw and the magnetometer error.
        self.throttle = min(
            1.0, self.speed / self.cfg.survey_speed_ms * 0.6 + abs(error) / 180.0
        )

        # Move: own velocity plus current.
        v_e, v_n = bearing_to_enu(self.true_heading, self.speed)
        c_e, c_n = bearing_to_enu(self.cfg.current_bearing_deg, self.cfg.current_speed_ms)
        de = (v_e + c_e) * dt
        dn = (v_n + c_n) * dt
        self.east += de
        self.north += dn
        self.distance_travelled += math.hypot(de, dn)

        self._last_velocity = (v_e + c_e, v_n + c_n)

    # -- observation ------------------------------------------------------

    def sample(self, utc_ms: int) -> VesselSample:
        cfg = self.cfg
        v_e, v_n = getattr(self, "_last_velocity", (0.0, 0.0))
        sog = math.hypot(v_e, v_n)
        cog = enu_bearing_deg(v_e, v_n) if sog > 0.05 else self.true_heading

        if cfg.heading_source == HEADING_SOURCE_GNSS:
            err = self._rng.gauss(0.0, cfg.gnss_compass_noise_deg)
            accuracy = cfg.gnss_compass_noise_deg
        else:
            # A slow random walk on top of the throttle-proportional term: the
            # error is not white, which is why averaging it away does not work.
            self._mag_walk = 0.98 * self._mag_walk + self._rng.gauss(0.0, 0.15)
            err = (
                cfg.mag_bias_deg
                + cfg.mag_throttle_gain_deg * self.throttle
                + self._mag_walk
                + self._rng.gauss(0.0, cfg.mag_noise_deg)
            )
            accuracy = cfg.mag_bias_deg + cfg.mag_throttle_gain_deg * self.throttle

        heading = wrap360(self.true_heading + err)

        phase = 2.0 * math.pi * self.t / cfg.wave_period_s
        roll = cfg.wave_height_m * 12.0 * math.sin(phase) + self._rng.gauss(0, 0.3)
        pitch = cfg.wave_height_m * 4.0 * math.sin(phase * 0.7 + 1.1) + self._rng.gauss(0, 0.2)

        lat, lon = self.plan.origin.to_geo(self.east, self.north)
        wp = self.plan.waypoints[min(self.wp_index, len(self.plan.waypoints) - 1)]

        return VesselSample(
            utc_ms=utc_ms,
            lat=lat,
            lon=lon,
            alt=0.0,
            east_m=self.east,
            north_m=self.north,
            heading_deg=heading,
            heading_source=cfg.heading_source if self._heading_valid else "none",
            heading_valid=self._heading_valid,
            heading_accuracy_deg=accuracy if self._heading_valid else float("nan"),
            true_heading_deg=self.true_heading,
            cog_deg=cog,
            sog_ms=sog,
            roll_deg=roll,
            pitch_deg=pitch,
            gnss_fix_type=cfg.fix_type,
            num_sats=cfg.num_sats,
            hdop=cfg.hdop,
            throttle=self.throttle,
            waypoint_index=self.wp_index,
            on_survey=wp.on_survey and not self.finished,
            distance_travelled_m=self.distance_travelled,
        )
