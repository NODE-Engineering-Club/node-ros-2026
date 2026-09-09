"""Command lifecycle.

This module exists to enforce two safety rules that are easy to state and easy
to violate by accident:

**Rule 4 — displayed state is always confirmed state.** A command never updates
what the GUI shows. Only the *status stream*, reporting what the Pico says the
vessel is actually doing, does that.

**Rule 5 — mode commands require two-step confirmation and Pico-confirmed
feedback.** The second step happens in the browser; the confirmation happens
here, by watching the status stream until it agrees, or timing out and saying so.

So a command has three outcomes and only three:

* ``pending``   — sent, waiting for the Pico
* ``confirmed`` — the Pico now reports the requested state
* ``failed``    — it did not, within the timeout, and the operator is told

There is no fourth outcome where the UI optimistically shows what was asked for.
That is the failure mode this module exists to prevent: an operator reading
"AUTONOMOUS" off a screen while the boat is still in MANUAL.

The soft ESTOP is named ``cut_propulsion`` here and labelled "Cut propulsion" in
the UI. Never "Emergency stop" — the hardware killswitch and RC channel 8 are
the emergency stop, and no operator should ever come to rely on a
software latch as one.
"""

from __future__ import annotations

import uuid
from dataclasses import dataclass, field
from typing import Callable

STATUS_PENDING = "pending"
STATUS_CONFIRMED = "confirmed"
STATUS_FAILED = "failed"

CMD_SET_MODE = "set_mode"
CMD_CUT_PROPULSION = "cut_propulsion"
CMD_SET_PING_PARAMETERS = "set_ping_parameters"
CMD_START_MISSION = "start_mission"
CMD_STOP_MISSION = "stop_mission"
CMD_EXPORT_MISSION = "export_mission"
CMD_DELETE_MISSION = "delete_mission"
CMD_LIST_MISSIONS = "list_missions"
CMD_RUN_SYSTEM_TEST = "run_system_test"
CMD_SET_PROFILE = "set_profile"
CMD_INJECT_FAULT = "inject_fault"
CMD_CLEAR_FAULT = "clear_fault"

#: Commands that change what the vessel is doing. These get the two-step
#: confirmation in the UI and the confirmation watch here.
MODE_COMMANDS = {CMD_SET_MODE, CMD_CUT_PROPULSION}

#: Default time to wait for the Pico. Generous: a confirmation that arrives at
#: 2.9 s is still a confirmation, and a spurious "failed" that is really a slow
#: link teaches an operator to ignore the failure message.
DEFAULT_TIMEOUT_S = 3.0

#: The mode a propulsion cut asks for, for matching acknowledgements.
MODE_ESTOP_NAME = "ESTOP"


@dataclass
class Command:
    id: str
    name: str
    args: dict
    issued_utc_ms: int
    status: str = STATUS_PENDING
    detail: str = ""
    #: Returns True once the vessel state shows the command took effect.
    confirm: Callable[[dict], bool] | None = field(default=None, repr=False)
    timeout_s: float = DEFAULT_TIMEOUT_S
    resolved_utc_ms: int | None = None

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "name": self.name,
            "args": self.args,
            "status": self.status,
            "detail": self.detail,
            "issued_utc_ms": self.issued_utc_ms,
            "resolved_utc_ms": self.resolved_utc_ms,
        }


class CommandManager:
    """Tracks in-flight commands and confirms them against observed state."""

    def __init__(self, timeout_s: float = DEFAULT_TIMEOUT_S) -> None:
        self.timeout_s = timeout_s
        self._pending: dict[str, Command] = {}
        #: Resolved commands, newest last. Bounded; this is a UI aid, not a log.
        self.history: list[Command] = []
        self._history_limit = 50

    def issue(
        self,
        name: str,
        args: dict,
        now_utc_ms: int,
        confirm: Callable[[dict], bool] | None = None,
        timeout_s: float | None = None,
        command_id: str | None = None,
    ) -> Command:
        """Register a command as sent. Returns it in the ``pending`` state.

        Note what this does *not* do: it does not touch any displayed value.
        """
        cmd = Command(
            id=command_id or uuid.uuid4().hex[:12],
            name=name,
            args=dict(args),
            issued_utc_ms=now_utc_ms,
            confirm=confirm,
            timeout_s=timeout_s if timeout_s is not None else self.timeout_s,
        )
        if confirm is None:
            # Commands with nothing to observe (a profile change, a fault
            # injection) are settled immediately — but by an explicit decision
            # here, not by defaulting to optimism.
            cmd.status = STATUS_CONFIRMED
            cmd.detail = "applied"
            cmd.resolved_utc_ms = now_utc_ms
            self._archive(cmd)
        else:
            self._pending[cmd.id] = cmd
        return cmd

    def fail(self, cmd: Command, detail: str, now_utc_ms: int) -> Command:
        """Mark a command failed at the point of sending."""
        self._pending.pop(cmd.id, None)
        cmd.status = STATUS_FAILED
        cmd.detail = detail
        cmd.resolved_utc_ms = now_utc_ms
        self._archive(cmd)
        return cmd

    def update(self, observed_state: dict, now_utc_ms: int) -> list[Command]:
        """Resolve pending commands against observed state.

        Called every time fresh vessel state arrives. Returns the commands that
        changed status, so the client can be told.
        """
        changed: list[Command] = []
        ack = observed_state.get("pico_command_ack") or None

        for cmd_id in list(self._pending):
            cmd = self._pending[cmd_id]

            # A refusal is not a slow confirmation. Fail immediately with the
            # vessel's own reason rather than making the operator wait out the
            # timeout for a message that says nothing about why.
            rejection = _rejection_for(cmd, ack)
            if rejection is not None:
                del self._pending[cmd_id]
                cmd.status = STATUS_FAILED
                cmd.detail = rejection
                cmd.resolved_utc_ms = now_utc_ms
                self._archive(cmd)
                changed.append(cmd)
                continue

            if cmd.confirm and cmd.confirm(observed_state):
                del self._pending[cmd_id]
                cmd.status = STATUS_CONFIRMED
                cmd.detail = _confirmation_detail(cmd, observed_state)
                cmd.resolved_utc_ms = now_utc_ms
                self._archive(cmd)
                changed.append(cmd)
                continue

            elapsed_s = (now_utc_ms - cmd.issued_utc_ms) / 1000.0
            if elapsed_s > cmd.timeout_s:
                del self._pending[cmd_id]
                cmd.status = STATUS_FAILED
                cmd.detail = (
                    f"no confirmation from the vessel after {cmd.timeout_s:g} s — "
                    "the vessel has NOT changed state"
                )
                cmd.resolved_utc_ms = now_utc_ms
                self._archive(cmd)
                changed.append(cmd)
        return changed

    @property
    def pending(self) -> list[Command]:
        return list(self._pending.values())

    def _archive(self, cmd: Command) -> None:
        self.history.append(cmd)
        del self.history[: max(0, len(self.history) - self._history_limit)]


def _confirmation_detail(cmd: Command, observed_state: dict) -> str:
    """"Confirmed" is not always the whole story.

    A mode request can be accepted, arbitrated and confirmed in the status line
    while the propellers still cannot turn, because arming is channel 7 and
    nothing in software can raise it. Reporting only "confirmed by the vessel"
    there is the button appearing to succeed — the operator sees a green line
    and a boat that does not move, and cannot tell which of the two to believe.

    So the confirmation carries the reason as well. A propulsion cut is exempt:
    a disarmed vessel is what that command was *for*, and appending "propulsion
    cannot start" to it would read as a fault instead of as success.
    """
    confirmed = "confirmed by the vessel"
    if cmd.name != CMD_SET_MODE:
        return confirmed

    block = (observed_state.get("pico") or {}).get("arming_block")
    if not block:
        return confirmed
    return f"{confirmed} — but {block}"


def _rejection_for(cmd: Command, ack: dict | None) -> str | None:
    """The reason this command was refused, or ``None`` if it was not.

    Matched on the mode, not just on time: two commands can be in flight, and
    failing the wrong one would report a refusal against a command the vessel
    never objected to.

    An ack from *before* the command was issued is ignored — it is the answer to
    an earlier press, and reusing it would fail a command the firmware has not
    even seen yet.
    """
    if not ack or ack.get("accepted") is not False:
        return None
    if cmd.name not in MODE_COMMANDS:
        return None
    if int(ack.get("utc_ms") or 0) < cmd.issued_utc_ms:
        return None

    wanted = (
        MODE_ESTOP_NAME if cmd.name == CMD_CUT_PROPULSION
        else str(cmd.args.get("mode", "")).upper()
    )
    acked = (ack.get("mode") or "").upper()
    # AUTO and AUTONOMOUS are the same mode under two spellings.
    if acked == "AUTO":
        acked = "AUTONOMOUS"
    if wanted == "AUTO":
        wanted = "AUTONOMOUS"
    if acked and wanted and acked != wanted:
        return None

    from asket_common.mode_arbitration import reason_text

    return reason_text(str(ack.get("reason") or ""))


# -- confirmation predicates ---------------------------------------------
#
# Each returns True when the observed vessel state shows the command took
# effect. They read the `pico` payload, which is the Pico's own report — never
# a local echo of what was requested.


def mode_confirmed(mode_name: str):
    def check(state: dict) -> bool:
        return (state.get("pico") or {}).get("mode") == mode_name

    return check


def propulsion_cut_confirmed(state: dict) -> bool:
    """The vessel reporting ESTOP is the confirmation.

    This used to also require ``estop_latched``. The firmware latches, but it
    does **not** print the latch in its status line — so requiring it here meant
    a propulsion cut could never be confirmed from real hardware, only from the
    simulator, and every real press would have timed out reporting that the
    vessel had NOT changed state while it sat there with its relay open.

    So the latch corroborates when it is reported and is not required when it is
    absent. ``None`` means "not sent"; only an explicit ``False`` contradicts the
    mode, and that combination is a real disagreement worth failing on.
    """
    pico = state.get("pico") or {}
    if pico.get("mode") != "ESTOP":
        return False
    return pico.get("estop_latched") is not False


def recording_confirmed(should_be_recording: bool):
    def check(state: dict) -> bool:
        mission = state.get("mission") or {}
        is_recording = mission.get("state") == "RECORDING"
        return is_recording == should_be_recording

    return check


def ping_parameters_confirmed(range_m: float, gain: int, rate_hz: float, tolerance=0.01):
    """The sonar is the authority on its own settings, exactly as the Pico is on
    the vessel's mode. A commanded value that never took effect must show as
    failed, not as applied."""

    def check(state: dict) -> bool:
        sonar = state.get("sonar") or {}
        return (
            abs((sonar.get("range_setting_m") or -1) - range_m) < tolerance
            and sonar.get("gain_setting") == gain
            and abs((sonar.get("commanded_ping_rate_hz") or -1) - rate_hz) < tolerance
        )

    return check
