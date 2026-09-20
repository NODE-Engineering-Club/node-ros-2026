"""Automatic link profile selection.

Picks ``full`` / ``reduced`` / ``minimal`` from measured link behaviour, with
two properties that matter more than the thresholds themselves:

**Hysteresis.** A profile that flaps between ``full`` and ``reduced`` every few
seconds is worse than one that is simply wrong, because every change re-runs the
client's subscriptions and the operator watches panels appear and vanish. So
degrading is fast and recovering is slow: a link that has just come back has not
yet proved anything.

**Manual override is sticky.** If an operator forces a profile, it stays forced
until they release it. Auto-selection quietly overriding a human decision at the
worst possible moment is exactly the behaviour that makes people stop trusting
automatic anything.

Selection uses *measured* round-trip time and delivery, not the bearer's name.
A "WiFi" link at 300 m with 40% loss is a 4G link as far as the GUI is
concerned, and the profile that keeps the operator informed is the one that
matches what the link is doing.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .streams import PROFILE_FULL, PROFILE_MINIMAL, PROFILE_ORDER, PROFILE_REDUCED


@dataclass
class ProfileThresholds:
    """Tuned for a degrading WiFi link with cellular fallback (PROVISIONAL, Q6)."""

    #: Above this round-trip time, drop out of `full`.
    full_max_rtt_ms: float = 150.0
    #: Above this, drop to `minimal`.
    reduced_max_rtt_ms: float = 600.0
    #: Below this measured quality, drop out of `full`.
    full_min_quality: float = 0.35
    reduced_min_quality: float = 0.1

    #: Seconds a better link must hold before we believe it.
    upgrade_hold_s: float = 8.0
    #: Seconds a worse link must hold before we act. Short: being slow to
    #: degrade means the operator watches a frozen screen instead.
    downgrade_hold_s: float = 2.0


@dataclass
class ProfileSelector:
    thresholds: ProfileThresholds = field(default_factory=ProfileThresholds)
    profile: str = PROFILE_FULL
    manual: bool = False
    _candidate: str | None = field(default=None, init=False)
    _candidate_since: float | None = field(default=None, init=False)
    #: Why the current profile is what it is. Shown in the link panel.
    reason: str = "starting up"

    # -- operator control -------------------------------------------------

    def force(self, profile: str) -> None:
        """Pin a profile until :meth:`release` is called."""
        if profile not in PROFILE_ORDER:
            raise ValueError(f"unknown profile {profile!r}")
        self.profile = profile
        self.manual = True
        self._candidate = None
        self._candidate_since = None
        self.reason = "set manually by the operator"

    def release(self) -> None:
        self.manual = False
        self._candidate = None
        self._candidate_since = None
        self.reason = "returned to automatic selection"

    # -- measurement ------------------------------------------------------

    def _target(self, rtt_ms: float, quality: float, connected: bool) -> tuple[str, str]:
        t = self.thresholds
        if not connected:
            return PROFILE_MINIMAL, "no link"
        if rtt_ms > t.reduced_max_rtt_ms or quality < t.reduced_min_quality:
            return PROFILE_MINIMAL, f"round trip {rtt_ms:.0f} ms, quality {quality:.2f}"
        if rtt_ms > t.full_max_rtt_ms or quality < t.full_min_quality:
            return PROFILE_REDUCED, f"round trip {rtt_ms:.0f} ms, quality {quality:.2f}"
        return PROFILE_FULL, f"round trip {rtt_ms:.0f} ms, quality {quality:.2f}"

    def update(
        self,
        rtt_ms: float,
        quality: float,
        now_s: float,
        connected: bool = True,
    ) -> bool:
        """Feed a measurement. Returns True if the active profile changed."""
        if self.manual:
            return False

        target, why = self._target(rtt_ms, quality, connected)
        if target == self.profile:
            self._candidate = None
            self._candidate_since = None
            return False

        current_index = PROFILE_ORDER.index(self.profile)
        target_index = PROFILE_ORDER.index(target)

        if self._candidate != target:
            candidate_index = (
                PROFILE_ORDER.index(self._candidate) if self._candidate else None
            )
            improving = (
                candidate_index is not None
                and candidate_index < current_index
                and target_index < current_index
            )
            if improving:
                # The link is wandering between two profiles that are both
                # better than the one we are stuck on. Hold the clock and take
                # the more conservative of the two, rather than restarting the
                # timer every few seconds and never recovering at all.
                self._candidate = PROFILE_ORDER[max(candidate_index, target_index)]
                target = self._candidate
                target_index = PROFILE_ORDER.index(target)
            else:
                self._candidate = target
                self._candidate_since = now_s
                return False

        worse = target_index > current_index
        hold = (
            self.thresholds.downgrade_hold_s if worse else self.thresholds.upgrade_hold_s
        )
        if now_s - (self._candidate_since or now_s) < hold:
            return False

        self.profile = target
        self.reason = why
        self._candidate = None
        self._candidate_since = None
        return True

    def to_dict(self) -> dict:
        return {"profile": self.profile, "manual": self.manual, "reason": self.reason}
