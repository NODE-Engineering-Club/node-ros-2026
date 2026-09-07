"""Sonar health accounting.

Everything the sonar health panel shows, computed here so it can be tested
without a sonar, a socket or ROS.

The field that matters most is the least obvious one. ``clock_offset_ms`` is the
difference between the sonar's ``utc_msec`` and the Jetson's clock. The whole
post-mission fusion strategy — record the raw sonar stream and the trajectory
separately, pair them afterwards on that timestamp — rests on it staying small.
If it drifts and nobody notices, the survey is not degraded, it is **worthless**,
and nobody finds out until the data is opened back home. So it is measured
continuously and surfaced as an alarm, not buried in a debug topic.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

#: Beyond this the operator is warned; beyond the alarm threshold the data
#: should be considered compromised. Mirrors mission_defaults.yaml.
CLOCK_OFFSET_WARN_MS = 250
CLOCK_OFFSET_ALARM_MS = 1000


@dataclass
class SonarHealth:
    """A snapshot of everything the panel and the diagnostics need."""

    connected: bool = False
    actual_ping_rate_hz: float = 0.0
    commanded_ping_rate_hz: float = 0.0
    points_per_ping: int = 0
    valid_points_per_ping: int = 0
    speed_of_sound: float = 0.0
    range_setting_m: float = 0.0
    gain_setting: int = 0
    packet_loss_ratio: float = 0.0
    clock_offset_ms: int = 0
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    packets_parsed: int = 0
    checksum_errors: int = 0
    bytes_discarded: int = 0
    seconds_since_data: float = float("inf")
    ntp_url_sent: str = ""

    @property
    def clock_ok(self) -> bool:
        return abs(self.clock_offset_ms) < CLOCK_OFFSET_WARN_MS

    @property
    def clock_compromised(self) -> bool:
        """True when data recorded now cannot be trusted to georeference."""
        return abs(self.clock_offset_ms) >= CLOCK_OFFSET_ALARM_MS

    @property
    def ping_rate_ok(self) -> bool:
        """Actual within 20% of commanded. Divergence is a real symptom: the
        sonar reduces its rate on its own when the range setting demands it."""
        if self.commanded_ping_rate_hz <= 0.0:
            return True
        ratio = self.actual_ping_rate_hz / self.commanded_ping_rate_hz
        return ratio > 0.8


class SonarHealthTracker:
    """Rolling measurement of sonar behaviour.

    Rates are measured over a window rather than derived from the commanded
    setting: the point of this panel is to show what the sonar is *doing*, which
    is not always what it was told.
    """

    def __init__(self, window_s: float = 5.0) -> None:
        self.window_s = window_s
        self._ping_times: deque[float] = deque()
        self._clock_offsets: deque[int] = deque(maxlen=20)

        self.points_per_ping = 0
        self.valid_points_per_ping = 0
        self.speed_of_sound = 0.0
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.commanded_ping_rate_hz = 0.0
        self.range_setting_m = 0.0
        self.gain_setting = 0
        self.ntp_url_sent = ""

    # -- inputs -----------------------------------------------------------

    def on_point_set(
        self,
        utc_msec: int,
        num_points: int,
        num_valid: int,
        speed_of_sound: float,
        now_monotonic: float | None = None,
        now_utc_ms: int | None = None,
    ) -> None:
        now = now_monotonic if now_monotonic is not None else time.monotonic()
        self._ping_times.append(now)
        self._trim(now)

        self.points_per_ping = num_points
        self.valid_points_per_ping = num_valid
        self.speed_of_sound = speed_of_sound

        # A ping with a zero timestamp means the sonar's clock is unset, which
        # is a different fault from a drifting clock and must not be averaged
        # into the offset as if it were an 1.7-trillion-millisecond error.
        if utc_msec > 0:
            wall = now_utc_ms if now_utc_ms is not None else int(time.time() * 1000)
            self._clock_offsets.append(int(utc_msec - wall))

    def on_attitude(self, pitch_deg: float, roll_deg: float) -> None:
        self.pitch_deg = pitch_deg
        self.roll_deg = roll_deg

    def on_parameters_commanded(self, range_m: float, gain: int, ping_rate_hz: float) -> None:
        self.range_setting_m = range_m
        self.gain_setting = gain
        self.commanded_ping_rate_hz = ping_rate_hz

    # -- outputs ----------------------------------------------------------

    def _trim(self, now: float) -> None:
        while self._ping_times and now - self._ping_times[0] > self.window_s:
            self._ping_times.popleft()

    def ping_rate_hz(self, now_monotonic: float | None = None) -> float:
        now = now_monotonic if now_monotonic is not None else time.monotonic()
        self._trim(now)
        if len(self._ping_times) < 2:
            return 0.0
        span = self._ping_times[-1] - self._ping_times[0]
        return (len(self._ping_times) - 1) / span if span > 0 else 0.0

    def clock_offset_ms(self) -> int:
        """Median offset. A median rather than a mean because one late packet
        must not be able to raise a clock alarm on its own."""
        if not self._clock_offsets:
            return 0
        ordered = sorted(self._clock_offsets)
        return ordered[len(ordered) // 2]

    def health(
        self,
        connected: bool,
        packet_loss_ratio: float,
        packets_parsed: int,
        checksum_errors: int,
        bytes_discarded: int,
        seconds_since_data: float,
        now_monotonic: float | None = None,
    ) -> SonarHealth:
        return SonarHealth(
            connected=connected,
            actual_ping_rate_hz=self.ping_rate_hz(now_monotonic),
            commanded_ping_rate_hz=self.commanded_ping_rate_hz,
            points_per_ping=self.points_per_ping,
            valid_points_per_ping=self.valid_points_per_ping,
            speed_of_sound=self.speed_of_sound,
            range_setting_m=self.range_setting_m,
            gain_setting=self.gain_setting,
            packet_loss_ratio=packet_loss_ratio,
            clock_offset_ms=self.clock_offset_ms(),
            pitch_deg=self.pitch_deg,
            roll_deg=self.roll_deg,
            packets_parsed=packets_parsed,
            checksum_errors=checksum_errors,
            bytes_discarded=bytes_discarded,
            seconds_since_data=seconds_since_data,
            ntp_url_sent=self.ntp_url_sent,
        )
