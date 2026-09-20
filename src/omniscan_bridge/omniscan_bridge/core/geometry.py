"""Sensor frame to vessel frame.

The sonar reports an **angle** and a **time of flight**. It knows nothing about
where it is or which way it points. Turning that into a point on the vessel is
our job, and this is where it happens.

Georeferencing is deliberately *not* here. The bridge publishes vessel-frame
points; position and heading are recorded alongside the raw stream and merged
afterwards (brief, section 5). Mixing the two would couple sonar output to GNSS
availability, and a dropout would then corrupt the point cloud instead of merely
annotating it.

Frames
------

**Sensor frame**, as the brief defines it::

    range = tof * speed_of_sound / 2
    y     = range * sin(angle)      # lateral
    z     = range * cos(angle)      # along the sensor boresight

so ``angle = 0`` is the boresight and ``z`` grows away from the transducer.

**Vessel frame** is ROS REP-103: **x forward, y port, z up**, origin at the
GNSS antenna. That origin choice matters — the lever arm from the antenna to the
transducer must be measured to the centimetre and put in the config, because it
is a systematic offset that no amount of post-processing will discover for you
(docs/open_questions.md Q2).

Order of operations
-------------------

1. sensor angle/tof -> sensor-frame ``(y, z)``
2. rotate by the **mounting tilt** about the vessel's forward axis
3. rotate by the **mounting yaw and pitch**, if the transducer is not square
4. translate by the **lever arm** from the GNSS antenna
5. rotate by the **vessel attitude** (roll, pitch) at the instant of the ping

Steps 2-4 are fixed installation geometry. Step 5 changes every ping.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .ping_protocol import BOTTOM_TYPES, PointSet


@dataclass
class SonarMounting:
    """Where the transducer is and which way it looks.

    All values PROVISIONAL until measured — see docs/open_questions.md Q2.
    """

    #: Downward tilt of the fan's centre from vertical, degrees. Positive tilts
    #: towards starboard, matching a starboard-looking install.
    tilt_deg: float = 35.0
    #: Rotation about the vertical axis, degrees, if the head is not square to
    #: the hull. Positive is clockwise seen from above.
    yaw_deg: float = 0.0
    #: Nose-up pitch of the head, degrees.
    pitch_deg: float = 0.0

    #: Lever arm from the GNSS antenna to the transducer, in vessel frame
    #: (x forward, y port, z up), metres.
    lever_x_m: float = -0.20
    lever_y_m: float = -0.35   # negative y = starboard
    lever_z_m: float = -0.15   # below the antenna

    #: Which side the fan looks at. Only used for sanity checks and for the
    #: coverage overlay; the tilt sign is what actually steers the geometry.
    side: str = "starboard"


def range_from_tof(tof_s: float, speed_of_sound_ms: float) -> float:
    """Two-way time of flight to one-way range, in metres.

    The factor of two is the whole reason ``speed_of_sound`` travels with every
    ping: assuming 1500 m/s when the sonar used 1520 puts a 20 m bottom 27 cm
    out, systematically, across the entire survey.
    """
    return tof_s * speed_of_sound_ms / 2.0


def sensor_to_vessel(
    angle_rad: float,
    range_m: float,
    mounting: SonarMounting,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
) -> tuple[float, float, float]:
    """One beam's detection, from sensor angle/range to a vessel-frame point."""
    # 1. Sensor frame, exactly as the brief defines it.
    y_s = range_m * math.sin(angle_rad)   # lateral
    z_s = range_m * math.cos(angle_rad)   # along the boresight

    # 2. Mounting tilt, about the vessel's forward axis. With zero tilt the
    #    boresight points straight down, so the sensor's +z maps to vessel -z.
    tilt = math.radians(mounting.tilt_deg)
    #    Lateral, positive to starboard, and depth, positive downwards.
    lateral_stbd = y_s * math.cos(tilt) + z_s * math.sin(tilt)
    depth = z_s * math.cos(tilt) - y_s * math.sin(tilt)

    # Into REP-103: x forward, y port, z up.
    x = 0.0
    y = -lateral_stbd
    z = -depth

    # 3. Fixed mounting yaw and pitch of the head itself.
    if mounting.yaw_deg:
        x, y = _rotate_z(x, y, math.radians(-mounting.yaw_deg))
    if mounting.pitch_deg:
        x, z = _rotate_pitch(x, z, math.radians(mounting.pitch_deg))

    # 4. Lever arm from the GNSS antenna. Applied before vessel attitude,
    #    because the transducer rotates with the hull about the antenna.
    x += mounting.lever_x_m
    y += mounting.lever_y_m
    z += mounting.lever_z_m

    # 5. Vessel attitude at the instant of the ping.
    if roll_deg:
        y, z = _rotate_x(y, z, math.radians(roll_deg))
    if pitch_deg:
        x, z = _rotate_pitch(x, z, math.radians(pitch_deg))

    return x, y, z


def _rotate_x(y: float, z: float, roll: float) -> tuple[float, float]:
    """Roll: rotation about the forward axis, starboard-down positive."""
    return y * math.cos(roll) - z * math.sin(roll), y * math.sin(roll) + z * math.cos(roll)


def _rotate_pitch(x: float, z: float, pitch: float) -> tuple[float, float]:
    """Pitch: rotation about the port axis, nose-up positive."""
    return x * math.cos(pitch) + z * math.sin(pitch), -x * math.sin(pitch) + z * math.cos(pitch)


def _rotate_z(x: float, y: float, yaw: float) -> tuple[float, float]:
    """Yaw: rotation about the up axis, counter-clockwise positive."""
    return x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw)


def point_set_to_vessel_frame(
    point_set: PointSet,
    mounting: SonarMounting,
    roll_deg: float = 0.0,
    pitch_deg: float = 0.0,
    max_range_m: float | None = None,
    speed_of_sound_override: float | None = None,
) -> list[tuple[float, float, float, float]]:
    """A whole ping, as ``(x, y, z, power)`` in the vessel frame.

    Only ``bottom_points`` are kept. An *unclassified* point is one the sonar
    could not place, and a *water column* point is a fish, a thermocline or a
    wake — putting either into the bathymetry would place features in the
    seabed that are not on the bottom. Beams with no detection are dropped
    rather than emitted at range zero, where they would sit on the hull and
    read as a boulder underneath the boat.

    ``speed_of_sound_override`` exists only for testing against a known value;
    in normal operation the sonar's own reported figure is used, because it is
    the one it actually applied.
    """
    sos = speed_of_sound_override or point_set.speed_of_sound
    if not sos or sos <= 0.0:
        return []

    out: list[tuple[float, float, float, float]] = []
    for p in point_set.points:
        if p.pt_type not in BOTTOM_TYPES:
            continue
        rng = range_from_tof(p.tof_s, sos)
        if rng <= 0.0 or (max_range_m is not None and rng > max_range_m):
            continue
        x, y, z = sensor_to_vessel(p.angle_rad, rng, mounting, roll_deg, pitch_deg)
        out.append((x, y, z, p.power))
    return out


def swath_extent(points: list[tuple[float, float, float, float]]) -> tuple[float, float, float]:
    """``(nearest_lateral, farthest_lateral, mean_depth)`` for one ping.

    Used by the coverage overlay: it needs to know how wide the swath actually
    was on this ping, not how wide the range setting says it could have been.
    Empty pings return zeros, which the overlay renders as a gap.
    """
    if not points:
        return 0.0, 0.0, 0.0
    laterals = [abs(p[1]) for p in points]
    depths = [-p[2] for p in points]
    return min(laterals), max(laterals), sum(depths) / len(depths)
