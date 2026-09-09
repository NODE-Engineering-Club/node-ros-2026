"""Mode arbitration between the RC transmitter and a software request.

**The RC transmitter is sovereign.** Software may only ever request a state at
least as restrictive as what channel 8 currently allows. This module is the one
place that decides what that means, and it is a pure function of its inputs so
the whole decision table can be tested without a boat.

This is the Python mirror of ``arbitrate_mode()`` in the Pico firmware
(``firmware/pico-node_v3/``). The firmware is the authority — it is what
actually drives the relay — but the two implement the same table, and
``test_mode_arbitration.py`` covers every RC x request combination so a change
on one side that is not mirrored on the other shows up as a failing test rather
than as a boat that does not stop.

Ordering, least to most permissive::

    ESTOP  <  MANUAL  <  AUTONOMOUS

The effective mode is the **more restrictive** of (RC channel 8, active software
request). A software request can therefore only ever take authority away.

Why a request can be rejected rather than silently clamped
----------------------------------------------------------

Silently clamping would leave the GUI showing a button press that appeared to
work while the vessel did something else. Every request that cannot be honoured
comes back with :attr:`Arbitration.reason` naming what blocked it, so the
operator is told "the transmitter is in MANUAL" rather than being left to guess.

The software request expires
----------------------------

A held request is not permanent: see :data:`SOFTWARE_REQUEST_TIMEOUT_S`. If the
GUI stops refreshing it — because it crashed, or the link dropped — the request
lapses and the effective mode returns to whatever RC says. Without that, a GUI
that died while holding an ESTOP request would lock the vessel out until somebody
power-cycled the Pico.

A **latched** e-stop is a different thing and deliberately does not expire. See
``trigger_estop()`` in the firmware: it is cleared only by the operator cycling
the arm switch or selecting ESTOP on channel 8.
"""

from __future__ import annotations

from dataclasses import dataclass

MODE_ESTOP = "ESTOP"
MODE_MANUAL = "MANUAL"
MODE_AUTONOMOUS = "AUTONOMOUS"

#: Least to most permissive. The whole rule is an ordering comparison.
MODE_RANK = {MODE_ESTOP: 0, MODE_MANUAL: 1, MODE_AUTONOMOUS: 2}

#: Firmware's ``OperationMode`` enum, as it appears in the ``[STAT]`` line.
#: ``enum OperationMode { MODE_ESTOP = 1, MODE_MANUAL = 2, MODE_AUTONOMOUS = 3 }``
FIRMWARE_MODE_NUMBERS = {1: MODE_ESTOP, 2: MODE_MANUAL, 3: MODE_AUTONOMOUS}

#: **The single constant that answers "may the GUI request AUTONOMOUS?"**
#:
#: ``False`` (default) — downward-only. The GUI may request MANUAL or ESTOP and
#: nothing else. To return to AUTONOMOUS the operator stops refreshing the
#: request and lets it expire, which hands authority back to channel 8.
#:
#: ``True`` — the GUI may also request AUTONOMOUS, which matters if missions are
#: ever to be started from the GUI. It is still clamped by RC: requesting
#: AUTONOMOUS while channel 8 is in MANUAL is refused either way.
#:
#: Mirrored by ``SOFTWARE_UPWARD_REQUESTS_ALLOWED`` in the firmware. Changing it
#: in one place and not the other is a drift bug; the tests check both values.
SOFTWARE_UPWARD_REQUESTS_ALLOWED = False

#: Seconds a software request survives without being refreshed. Mirrors
#: ``SOFTWARE_REQUEST_TIMEOUT_MS`` in the firmware.
SOFTWARE_REQUEST_TIMEOUT_S = 5.0

# -- rejection reasons, as sent back over serial and shown in the GUI --------

REASON_RC_CLAMP = "rc_clamp"
REASON_UPWARD_DISABLED = "upward_disabled"
REASON_UNKNOWN_MODE = "unknown_mode"
REASON_UNKNOWN_COMMAND = "unknown_command"

#: Firmware acknowledgements carry only the code — a Pico has no business
#: shipping English sentences over a serial link at 20 Hz. The wording lives
#: here, so the GUI and the logs say the same thing about the same refusal.
REASON_TEXT = {
    REASON_RC_CLAMP: (
        "refused: RC channel 8 is more restrictive than that. Software can only "
        "ever restrict, never extend — move the transmitter first."
    ),
    REASON_UPWARD_DISABLED: (
        "refused: this build does not let software request AUTONOMOUS. Release "
        "the software clamp and channel 8 decides."
    ),
    REASON_UNKNOWN_MODE: "refused: the firmware did not recognise that mode.",
    REASON_UNKNOWN_COMMAND: "refused: the firmware did not recognise that command.",
}


def reason_text(code: str) -> str:
    """An operator-facing sentence for a firmware reason code.

    An unknown code is quoted rather than dropped: a refusal we cannot explain
    is still a refusal, and hiding it would leave the button looking merely slow.
    """
    return REASON_TEXT.get(code, f"refused by the vessel ({code})")


@dataclass(frozen=True)
class Arbitration:
    """What arbitration decided, and why.

    :param effective: the mode the vessel should actually be in.
    :param accepted: whether the software request was taken up. ``True`` when
        there was no request to judge.
    :param reason: machine-readable rejection reason, or ``""`` when accepted.
    :param detail: one sentence for the operator, or ``""`` when accepted.
    :param clamped_by_rc: the request was accepted but RC is holding the vessel
        more restrictive than it asked for. The GUI shows this so an operator
        can see the transmitter, not a fault, is what they are looking at.
    """

    effective: str
    accepted: bool
    reason: str = ""
    detail: str = ""
    clamped_by_rc: bool = False


def more_restrictive(a: str, b: str) -> str:
    """The lower of two modes in the permissiveness ordering."""
    return a if MODE_RANK[a] <= MODE_RANK[b] else b


def arbitrate(
    rc_mode: str,
    request: str | None,
    *,
    allow_upward: bool = SOFTWARE_UPWARD_REQUESTS_ALLOWED,
) -> Arbitration:
    """Decide the effective mode from the RC switch and a software request.

    Pure: no clock, no I/O, no globals beyond the module constants that are
    passed in explicitly. Every branch is covered by the decision-table test.

    :param rc_mode: what channel 8 currently selects. Sovereign.
    :param request: the active software request, or ``None`` if there is none
        (never sent, or expired).
    :param allow_upward: see :data:`SOFTWARE_UPWARD_REQUESTS_ALLOWED`.
    """
    if rc_mode not in MODE_RANK:
        # An unreadable RC mode is not a licence to pick one. Fail safe.
        return Arbitration(
            effective=MODE_ESTOP,
            accepted=False,
            reason=REASON_UNKNOWN_MODE,
            detail=f"RC mode {rc_mode!r} is not a mode; falling back to ESTOP",
        )

    if request is None:
        return Arbitration(effective=rc_mode, accepted=True)

    if request not in MODE_RANK:
        return Arbitration(
            effective=rc_mode,
            accepted=False,
            reason=REASON_UNKNOWN_MODE,
            detail=f"{request!r} is not a mode",
        )

    # ESTOP is always available, from any state, whatever the configuration.
    # It is the one request that can only ever remove authority.
    if request == MODE_ESTOP:
        return Arbitration(effective=MODE_ESTOP, accepted=True)

    if request == MODE_AUTONOMOUS and not allow_upward:
        return Arbitration(
            effective=rc_mode,
            accepted=False,
            reason=REASON_UPWARD_DISABLED,
            detail=(
                "this build does not allow software to request AUTONOMOUS "
                "(SOFTWARE_UPWARD_REQUESTS_ALLOWED is off)"
            ),
        )

    if MODE_RANK[request] > MODE_RANK[rc_mode]:
        return Arbitration(
            effective=rc_mode,
            accepted=False,
            reason=REASON_RC_CLAMP,
            detail=(
                f"RC channel 8 is in {rc_mode}; software cannot request "
                f"{request}, which is less restrictive"
            ),
        )

    effective = more_restrictive(rc_mode, request)
    return Arbitration(
        effective=effective,
        accepted=True,
        clamped_by_rc=effective != request,
    )


def requestable_modes(*, allow_upward: bool = SOFTWARE_UPWARD_REQUESTS_ALLOWED) -> list[str]:
    """Modes the GUI is allowed to offer as buttons.

    The GUI reads this rather than hard-coding a list, so flipping the constant
    changes the interface too instead of leaving a button that always fails.
    """
    if allow_upward:
        return [MODE_ESTOP, MODE_MANUAL, MODE_AUTONOMOUS]
    return [MODE_ESTOP, MODE_MANUAL]
