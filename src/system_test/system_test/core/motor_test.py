"""The one active test, and the guard rails around it.

**Motor tests never run automatically.** Safety rule 6. The requirement is not
a preference and not a UI convention: a thruster spinning up while somebody has
a hand in the water is the worst outcome this whole repository can produce.

Three gates, all of which must be passed, in order:

1. an explicit request — never a scheduled run, never part of the boot-time
   pre-flight;
2. an operator acknowledgement that **the vessel is out of the water or
   securely moored**, given as a specific token, not a boolean that could
   default to true;
3. a short, low-power pulse only.

The implementation here is the *gate*. Actually driving the thrusters is
``pico_bridge``'s job, and this repository does not modify it — see
``docs/open_questions.md``. So this module validates consent and produces the
command envelope; wiring it to the Pico is deliberately left for when a human
is standing next to the boat.
"""

from __future__ import annotations

import time
from dataclasses import dataclass

#: The operator must type or click this exact phrase. Not a checkbox: a
#: checkbox can be clicked by muscle memory, and a default-true boolean in a
#: config file can arm this without anybody deciding to.
CONSENT_PHRASE = "VESSEL CLEAR OF WATER"

#: Hard ceilings. Not defaults — ceilings. A request above these is refused.
MAX_THROTTLE = 0.15
MAX_DURATION_S = 1.5

#: A consent token is good for one test, briefly.
CONSENT_VALIDITY_S = 60.0


@dataclass
class MotorTestRequest:
    thruster: int
    throttle: float
    duration_s: float
    consent_phrase: str
    consent_utc_ms: int


@dataclass
class MotorTestDecision:
    allowed: bool
    reason: str
    #: The command actually permitted, clamped. ``None`` when refused.
    command: dict | None = None


def evaluate_request(
    request: MotorTestRequest,
    now_utc_ms: int,
    armed: bool,
    in_water: bool | None = None,
) -> MotorTestDecision:
    """Decide whether a motor test may proceed.

    Refuses by default. Every path that returns ``allowed=True`` has passed
    every gate explicitly.
    """
    if request.consent_phrase.strip().upper() != CONSENT_PHRASE:
        return MotorTestDecision(
            False,
            "Refused: the operator has not confirmed the vessel is out of the "
            f'water or securely moored. Type "{CONSENT_PHRASE}" to confirm.',
        )

    age_s = (now_utc_ms - request.consent_utc_ms) / 1000.0
    if age_s < 0 or age_s > CONSENT_VALIDITY_S:
        return MotorTestDecision(
            False,
            f"Refused: that confirmation is {age_s:.0f} s old. Confirm again "
            "immediately before the test.",
        )

    if in_water:
        return MotorTestDecision(
            False,
            "Refused: the vessel reports it is in the water. Recover it or "
            "moor it securely first.",
        )

    if not armed:
        return MotorTestDecision(
            False,
            "Refused: the vessel is not armed, so the Pico would ignore the "
            "command. Arm it deliberately, with the propellers clear.",
        )

    if not 0 < request.throttle <= MAX_THROTTLE:
        return MotorTestDecision(
            False,
            f"Refused: throttle must be above 0 and at most {MAX_THROTTLE:.2f}. "
            "This is a brief low-power pulse, not a run-up.",
        )

    if not 0 < request.duration_s <= MAX_DURATION_S:
        return MotorTestDecision(
            False,
            f"Refused: duration must be above 0 and at most {MAX_DURATION_S:.1f} s.",
        )

    return MotorTestDecision(
        True,
        f"Allowed: thruster {request.thruster}, {request.throttle:.2f} throttle "
        f"for {request.duration_s:.1f} s.",
        command={
            "thruster": request.thruster,
            "throttle": round(request.throttle, 3),
            "duration_s": round(request.duration_s, 2),
            "issued_utc_ms": now_utc_ms,
        },
    )


def new_consent(now_utc_ms: int | None = None) -> dict:
    """What the GUI sends alongside a request once the operator has confirmed."""
    return {
        "consent_phrase": CONSENT_PHRASE,
        "consent_utc_ms": now_utc_ms if now_utc_ms is not None else int(time.time() * 1000),
    }
