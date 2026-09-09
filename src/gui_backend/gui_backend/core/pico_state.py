"""Parsing the Pico's ``[STAT]`` line.

``/pico/status`` is a ``std_msgs/String``. ``pico_bridge`` reads lines off the
serial link and republishes the status lines verbatim, unparsed, so the GUI has
to read text and this module is the only place in the codebase that does.

The format, from firmware v3
============================

``pico-node_v3.ino`` emits one line every 250 ms::

    [STAT] Mode:2 Armed:Y Relay:ON Thr(Ch3):991 Yaw(Ch4):991 Arm(Ch7):172 Mode(Ch8):172

===================  ====================================================
Field                Values
===================  ====================================================
``Mode``             ``1`` ESTOP, ``2`` MANUAL, ``3`` AUTONOMOUS
``Armed``            ``Y`` / ``N``
``Relay``            ``ON`` / ``OFF``
``Thr(Ch3)``         raw SBUS, 172-1811, centre 991
``Yaw(Ch4)``         raw SBUS
``Arm(Ch7)``         raw SBUS
``Mode(Ch8)``        raw SBUS
===================  ====================================================

**The parsing trap.** ``Mode`` appears twice: once as the firmware's own mode
enum, once as ``Mode(Ch8)``, the raw channel it derives that from. Splitting the
line on ``:`` and taking the last ``Mode`` conflates a mode number (1-3) with a
raw SBUS count (172-1811), which silently turns ESTOP into a plausible-looking
value. This parser matches whole keys exactly — ``Mode`` and ``Mode(Ch8)`` are
different keys — and :func:`parse_state_line` is tested against that specific
confusion.

Why ``Mode(Ch8)`` matters as much as ``Mode``
---------------------------------------------

``Mode`` is what the firmware settled on. ``Mode(Ch8)`` is what the operator's
transmitter is asking for. When software holds a restricting request the two
differ, and an operator needs to see *which* is holding the vessel down — a
button they pressed, or the switch in their hand. Both are reported.

Verified, and checked at runtime
--------------------------------

:data:`FORMAT_VERIFIED` is ``True``: the format above is transcribed from the
firmware source, not guessed. That flag alone is a promise about the past,
though, so it is not the whole check — ``system_test`` also verifies that lines
actually arriving still parse. A future firmware change that alters the format
fails loudly there instead of quietly producing nulls.

**Nothing here fabricates a value.** A field the line does not carry comes back
``None``, travels to the GUI as ``null``, and renders as "not sent" — never as
zero, and never as a fault. An unrecognised line yields ``parsed=False`` with
the raw text kept, which the GUI shows as "unrecognised" rather than inventing a
mode. Silently mapping an unknown line onto MANUAL would be the most dangerous
thing this file could do.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, field

#: True: the field names and value ranges below are transcribed from
#: ``pico-node_v3.ino``, not guessed. The pre-flight still checks that live
#: lines parse — see ``system_test.core.checks``.
FORMAT_VERIFIED = True

MODE_ESTOP = "ESTOP"
MODE_MANUAL = "MANUAL"
MODE_AUTONOMOUS = "AUTONOMOUS"
MODE_UNKNOWN = "UNKNOWN"

#: ``enum OperationMode { MODE_ESTOP = 1, MODE_MANUAL = 2, MODE_AUTONOMOUS = 3 }``
#:
#: Note these are 1-based. ``asket_sim`` and ``payloads.MODE_NAMES`` use a
#: 0-based enum of their own; the two are deliberately converted by name rather
#: than by number, because an off-by-one between them would turn ESTOP into
#: MANUAL. Nothing in this file emits a raw number downstream.
FIRMWARE_MODE_NUMBERS = {1: MODE_ESTOP, 2: MODE_MANUAL, 3: MODE_AUTONOMOUS}

# -- SBUS calibration, mirrored from the firmware ---------------------------
#
# Changing these here does not change the boat. They exist so the GUI can show
# the operator what their transmitter is doing, and so it can derive the RC-
# selected mode the same way the firmware does.

SBUS_MIN = 172
SBUS_MID = 991
SBUS_MAX = 1811

#: ``Ch8 < MODE_LOW_MAX`` is ESTOP; below ``MODE_MID_MAX`` MANUAL; else AUTONOMOUS.
MODE_LOW_MAX = 700
MODE_MID_MAX = 1400
#: ``Ch7 > ARM_THRESHOLD`` is the arm switch held high.
ARM_THRESHOLD = 1000

#: The ESC arming window: after the relay closes the firmware holds both
#: thrusters at neutral for this long. ``ESC_ARM_DELAY_MS`` in the firmware.
ESC_ARM_DELAY_MS = 2000

#: Mirrors ``ESTOP_FEEDBACK_ENABLED`` in the firmware, which is **0**: the
#: GPIO20 divider trace is cut for bench testing, so ``check_power_feedback()``
#: compiles to nothing and *nothing verifies that ESC power actually dropped
#: when the relay was commanded open*.
#:
#: A compile-time flag cannot be read off the wire — the firmware announces it
#: once in its boot banner and never again — so it is mirrored here and the
#: mirror is checked against the sketch by
#: ``asket_common/test/test_firmware_arbitration_matches.py``. The pre-flight
#: turns it into a standing warning.
ESTOP_FEEDBACK_ENABLED = False

#: Exact key spellings, as the firmware prints them. Whole-key matching is what
#: keeps ``Mode`` and ``Mode(Ch8)`` apart.
_KEYS = {
    "Mode": "mode_number",
    "Armed": "armed",
    "Relay": "relay_on",
    "Thr(Ch3)": "ch_throttle",
    "Yaw(Ch4)": "ch_yaw",
    "Arm(Ch7)": "ch_arm",
    "Mode(Ch8)": "ch_mode",
}

_SBUS_FIELDS = ("ch_throttle", "ch_yaw", "ch_arm", "ch_mode")

#: ``Key:Value`` where the key may itself contain parentheses. Anchored on
#: whitespace boundaries so a value can never be mistaken for a key.
_FIELD_RE = re.compile(r"(?P<key>[A-Za-z][A-Za-z0-9]*(?:\([A-Za-z0-9]+\))?):(?P<value>\S+)")

_STATUS_PREFIXES = ("[STAT]", "STATE")

_TRUE = {"y", "yes", "1", "true", "on", "armed"}
_FALSE = {"n", "no", "0", "false", "off", "disarmed"}


@dataclass
class PicoState:
    """What the GUI needs from the Pico.

    Everything is optional. ``None`` means the line did not carry it, and the
    GUI renders that as "not sent" rather than as a zero or a fault.
    """

    #: False when the line could not be understood. The raw text is kept so the
    #: operator, and whoever fixes the parser, can see what actually arrived.
    parsed: bool = False
    raw: str = ""

    #: The mode the firmware settled on, by name.
    mode: str | None = None
    #: The same, as the raw enum value the line carried.
    mode_number: int | None = None
    armed: bool | None = None
    #: The single ESC-power relay on GPIO21. Not a list: there is one.
    relay_on: bool | None = None

    #: Raw SBUS counts, exactly as the line carried them.
    ch_throttle: int | None = None
    ch_yaw: int | None = None
    ch_arm: int | None = None
    ch_mode: int | None = None

    #: Keys seen that this parser does not recognise. Reported rather than
    #: dropped: an unknown key is usually the field that matters.
    unknown_keys: list[str] = field(default_factory=list)

    # -- values the firmware does not report -------------------------------
    #
    # Kept as explicit ``None`` rather than removed, because downstream code
    # distinguishes "absent" from "false" and the GUI renders the two very
    # differently. The firmware has no battery sense and no RC-link flag of its
    # own; RC link health is inferred from status lines arriving at all.
    rc_link_ok: bool | None = None
    battery_voltage: float | None = None
    battery_current: float | None = None

    @property
    def format_verified(self) -> bool:
        return FORMAT_VERIFIED

    @property
    def rc_mode(self) -> str | None:
        """The mode the operator's channel 8 is selecting, derived the same way
        the firmware derives it. ``None`` if the line did not carry Ch8."""
        return rc_mode_from_sbus(self.ch_mode)

    @property
    def rc_arm_high(self) -> bool | None:
        """Whether the arm switch (Ch7) is held high."""
        if self.ch_arm is None:
            return None
        return self.ch_arm > ARM_THRESHOLD

    @property
    def rc_channel8_raw_pct(self) -> int | None:
        """Channel 8 as a percentage of its travel, for the existing GUI field."""
        return sbus_to_pct(self.ch_mode)

    @property
    def software_clamp_active(self) -> bool | None:
        """True when the firmware is in a more restrictive mode than channel 8
        alone would give. That is a software request holding the vessel down —
        the operator should see it as a clamp, not as a fault."""
        rc = self.rc_mode
        if rc is None or self.mode is None:
            return None
        from asket_common.mode_arbitration import MODE_RANK

        if self.mode not in MODE_RANK or rc not in MODE_RANK:
            return None
        return MODE_RANK[self.mode] < MODE_RANK[rc]


def sbus_to_pct(value: int | None) -> int | None:
    """A raw SBUS count as a percentage of travel, clamped to 0-100."""
    if value is None:
        return None
    span = SBUS_MAX - SBUS_MIN
    pct = round((value - SBUS_MIN) * 100.0 / span)
    return max(0, min(100, int(pct)))


def rc_mode_from_sbus(value: int | None) -> str | None:
    """Channel 8 to a mode, using the firmware's own thresholds."""
    if value is None:
        return None
    if value < MODE_LOW_MAX:
        return MODE_ESTOP
    if value < MODE_MID_MAX:
        return MODE_MANUAL
    return MODE_AUTONOMOUS


def is_status_line(line: str) -> bool:
    """Whether this is a periodic status line, under either prefix.

    ``[STAT]`` is what firmware v3 emits. ``STATE`` is accepted because earlier
    notes in this repository described that prefix and a future firmware may yet
    use it; accepting both costs nothing and a prefix mismatch is exactly the
    drift that stopped ``/pico/status`` publishing at all.
    """
    if not isinstance(line, str):
        return False
    body = line.strip()
    return any(body.startswith(p) for p in _STATUS_PREFIXES)


def parse_state_line(line: str) -> PicoState:
    """Turn one raw status line into a :class:`PicoState`.

    **The only function in this repository that knows the wire format.**

    Never raises, on anything a serial link can produce. Never invents: a field
    that cannot be read stays ``None``.
    """
    state = PicoState(raw=line if isinstance(line, str) else "")
    if not isinstance(line, str):
        return state

    body = line.strip()
    matched_prefix = next((p for p in _STATUS_PREFIXES if body.startswith(p)), None)
    if matched_prefix is None:
        return state
    body = body[len(matched_prefix):].strip()
    if not body:
        # A prefix with nothing after it is a well-formed line carrying nothing.
        state.parsed = True
        return state

    seen_any = False
    consumed_spans: list[tuple[int, int]] = []

    for match in _FIELD_RE.finditer(body):
        key = match.group("key")
        value = match.group("value")
        consumed_spans.append(match.span())

        name = _KEYS.get(key)
        if name is None:
            state.unknown_keys.append(key)
            continue
        if _assign(state, name, value):
            seen_any = True

    # Anything that was not a Key:Value pair at all. Reported, not dropped.
    leftover = body
    for start, end in reversed(consumed_spans):
        leftover = leftover[:start] + leftover[end:]
    for token in leftover.split():
        state.unknown_keys.append(token)

    state.parsed = seen_any
    return state


def _assign(state: PicoState, name: str, value: str) -> bool:
    """Set one field. Returns False if the value could not be read as its type,
    leaving the field ``None`` — a field we could not read is a field we do not
    have, and that is what the GUI must be told."""
    if name == "mode_number":
        try:
            number = int(value)
        except ValueError:
            state.unknown_keys.append(f"Mode:{value}")
            return False
        state.mode_number = number
        # An unknown number is reported as unknown, never guessed at.
        state.mode = FIRMWARE_MODE_NUMBERS.get(number)
        if state.mode is None:
            state.unknown_keys.append(f"Mode:{value}")
        return True

    if name in ("armed", "relay_on"):
        low = value.strip().lower()
        if low in _TRUE:
            setattr(state, name, True)
        elif low in _FALSE:
            setattr(state, name, False)
        else:
            return False
        return True

    if name in _SBUS_FIELDS:
        try:
            setattr(state, name, int(value))
        except ValueError:
            return False
        return True

    return False


# -- event lines -----------------------------------------------------------
#
# The firmware also prints one-off event lines. They are not status, they are
# transitions, and they are the record of what the vessel did and why — which is
# what belongs in the mission event log rather than in a panel.

EVENT_MODE_ESTOP = "mode_estop"
EVENT_ARMED = "armed"
EVENT_DISARMED = "disarmed"
EVENT_ESTOP_TRIGGERED = "estop_triggered"
EVENT_INVALID_COMMAND = "invalid_command"
EVENT_COMMAND_ACK = "command_ack"

#: Ordered: the first pattern that matches wins, so more specific patterns come
#: first. Each yields ``(kind, detail)``.
_EVENT_PATTERNS: tuple[tuple[re.Pattern, str], ...] = (
    (re.compile(r"^>>>\s*MODE:\s*(?P<detail>.+)$"), EVENT_MODE_ESTOP),
    (re.compile(r"^>>>\s*ARMED:\s*(?P<detail>.+)$"), EVENT_ARMED),
    (re.compile(r"^>>>\s*DISARMED\s*\((?P<detail>[^)]*)\)"), EVENT_DISARMED),
    (re.compile(r"^\[E-STOP\]\s*(?P<detail>.*)$"), EVENT_ESTOP_TRIGGERED),
    (re.compile(r"^\[ACK\]\s*(?P<detail>.*)$"), EVENT_COMMAND_ACK),
    (re.compile(r"^Invalid cmd:\s*(?P<detail>.*)$"), EVENT_INVALID_COMMAND),
)


@dataclass(frozen=True)
class PicoEvent:
    kind: str
    detail: str
    raw: str


def parse_event_line(line: str) -> PicoEvent | None:
    """Recognise a firmware event line, or return ``None``.

    Used to feed ``events.jsonl``. Status lines are deliberately not events:
    they arrive four times a second and would drown the log.
    """
    if not isinstance(line, str):
        return None
    body = line.strip()
    if not body:
        return None
    for pattern, kind in _EVENT_PATTERNS:
        match = pattern.match(body)
        if match:
            return PicoEvent(kind=kind, detail=match.group("detail").strip(), raw=body)
    return None


# -- the ESC arming window -------------------------------------------------


class RelayEdgeTracker:
    """Times the relay's OFF->ON edge across successive status lines.

    The firmware knows when the relay closed (``relay_on_ms``) but does not
    print it, and the status line carries only ``Relay:ON``/``OFF``. So the edge
    has to be observed rather than read, which is what this does: it watches the
    boolean and remembers when it last went up.

    Stateful but not mysterious — no clock of its own, no I/O. The caller passes
    the time, so the whole thing is testable by calling it with a made-up one.

    Returns milliseconds since the relay closed, or ``None`` when the relay is
    open or no edge has been seen yet. ``None`` means "we do not know", and the
    GUI renders no arming window rather than a wrong one.
    """

    def __init__(self) -> None:
        self._previous: bool | None = None
        self._closed_at_utc_ms: int | None = None

    def observe(self, relay_on: bool | None, now_utc_ms: int) -> int | None:
        if relay_on is None:
            # A line that did not carry the relay tells us nothing either way;
            # hold what we had rather than forgetting it.
            return self._since(now_utc_ms)

        if relay_on and self._previous is not True:
            self._closed_at_utc_ms = now_utc_ms       # OFF (or unknown) -> ON
        elif not relay_on:
            self._closed_at_utc_ms = None             # open: no window at all

        self._previous = relay_on
        return self._since(now_utc_ms)

    def _since(self, now_utc_ms: int) -> int | None:
        if self._closed_at_utc_ms is None:
            return None
        return max(0, int(now_utc_ms - self._closed_at_utc_ms))


# -- command acknowledgements ----------------------------------------------

_ACK_RE = re.compile(
    r"^\[ACK\]\s+(?P<subject>.+?)\s+(?P<result>accepted|rejected)(?:\s+(?P<reason>\S+))?$"
)


@dataclass(frozen=True)
class PicoAck:
    """One ``[ACK]`` line: what the firmware did with a command, and why not.

    ``subject`` is the command as the firmware echoed it (``MODE MANUAL``,
    ``ESTOP``, ``REQUEST``). ``reason`` is a machine code, empty when accepted;
    :func:`asket_common.mode_arbitration.reason_text` turns it into a sentence.
    """

    subject: str
    accepted: bool
    reason: str
    raw: str

    @property
    def mode(self) -> str | None:
        """The mode this ack is about, if it is a mode command."""
        if self.subject.startswith("MODE "):
            return self.subject[len("MODE "):].strip() or None
        if self.subject == "ESTOP":
            return MODE_ESTOP
        return None


def parse_ack_line(line: str) -> PicoAck | None:
    """Read one acknowledgement, or return ``None`` if it is not one."""
    if not isinstance(line, str):
        return None
    match = _ACK_RE.match(line.strip())
    if not match:
        return None
    return PicoAck(
        subject=match.group("subject").strip(),
        accepted=match.group("result") == "accepted",
        reason=(match.group("reason") or "").strip(),
        raw=line.strip(),
    )
