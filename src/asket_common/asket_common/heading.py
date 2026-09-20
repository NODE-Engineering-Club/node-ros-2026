"""Heading quality assessment.

Heading is the dominant error source in the whole survey: **1 degree is about
90 cm at 50 m range**. This module is small, and it is shared by
``gui_backend``, ``mission_recorder`` and ``system_test`` deliberately — all
three must agree on whether the heading was trustworthy at a given instant, or
the recorded validity flag will not mean what the GUI showed.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .geo import angular_difference

SOURCE_GNSS_COMPASS = "gnss_compass"
#: The Njord stack's EKF yaw, from /odometry/filtered. A fusion of IMU, GNSS and
#: whatever else robot_localization is configured with — better than a bare
#: magnetometer, and not as good as a dual-antenna compass.
SOURCE_EKF = "ekf"
SOURCE_MAGNETOMETER = "magnetometer"
SOURCE_COG = "cog"
SOURCE_NONE = "none"

#: Typical accuracy by source, degrees, used **only** when the source reports
#: none of its own. The device's figure always wins: a UM982 that has lost one
#: antenna reports a degraded accuracy, and substituting the nominal 0.2 would
#: hide exactly the failure worth seeing (docs/open_questions.md Q4).
#:
#: The magnetometer figure is the upper end of the 2-10 degree range the hull
#: actually shows; the GNSS compass figure assumes the 1 m baseline
#: (PROVISIONAL, docs/open_questions.md Q4).
NOMINAL_ACCURACY_DEG = {
    SOURCE_GNSS_COMPASS: 0.2,
    # PROVISIONAL: the EKF publishes a pose covariance and that is the figure to
    # use once somebody confirms robot_localization is filling it in rather than
    # leaving the default. Until then this is a guess at a fused heading, and
    # the panel marks it "assumed" because it is not a measurement.
    SOURCE_EKF: 3.0,
    SOURCE_MAGNETOMETER: 10.0,
    SOURCE_COG: 5.0,
    SOURCE_NONE: float("nan"),
}

#: Below this speed, course over ground is noise and comparing it with heading
#: says nothing. Sitting still with a 40 degree "divergence" is not a fault.
MIN_SPEED_FOR_DIVERGENCE_MS = 0.6

#: Sustained divergence above this, at speed, means the heading is wrong or the
#: current is strong. Either way the operator wants to know.
DIVERGENCE_WARN_DEG = 15.0


@dataclass
class HeadingEstimate:
    heading_deg: float
    source: str
    valid: bool
    accuracy_deg: float
    cog_deg: float
    sog_ms: float
    divergence_deg: float
    divergence_meaningful: bool
    #: True when ``accuracy_deg`` came from the device. False means it is the
    #: nominal figure for the source — a guess about this class of hardware,
    #: not a measurement of this one, and the GUI labels it as such.
    accuracy_reported: bool = False

    @property
    def divergence_suspicious(self) -> bool:
        return (
            self.valid
            and self.divergence_meaningful
            and abs(self.divergence_deg) > DIVERGENCE_WARN_DEG
        )

    def position_error_at_m(self, range_m: float) -> float:
        """Across-track seabed error this heading accuracy implies at a range.

        The number that makes heading matter. At 50 m, one degree is 87 cm.
        """
        if math.isnan(self.accuracy_deg):
            return float("nan")
        return range_m * math.tan(math.radians(self.accuracy_deg))


def evaluate_heading(
    heading_deg: float | None,
    source: str,
    cog_deg: float,
    sog_ms: float,
    reported_accuracy_deg: float | None = None,
    source_valid: bool = True,
) -> HeadingEstimate:
    """Combine a heading and a velocity into an estimate with provenance."""
    valid = source_valid and heading_deg is not None and source != SOURCE_NONE
    hdg = heading_deg if heading_deg is not None else float("nan")

    accuracy = reported_accuracy_deg
    reported = accuracy is not None and not (
        isinstance(accuracy, float) and math.isnan(accuracy)
    )
    if not reported:
        accuracy = NOMINAL_ACCURACY_DEG.get(source, float("nan"))

    meaningful = valid and sog_ms >= MIN_SPEED_FOR_DIVERGENCE_MS
    divergence = angular_difference(hdg, cog_deg) if valid else float("nan")

    return HeadingEstimate(
        heading_deg=hdg,
        source=source if valid else SOURCE_NONE,
        valid=valid,
        accuracy_deg=accuracy if valid else float("nan"),
        cog_deg=cog_deg,
        sog_ms=sog_ms,
        divergence_deg=divergence,
        divergence_meaningful=meaningful,
        accuracy_reported=bool(reported and valid),
    )
