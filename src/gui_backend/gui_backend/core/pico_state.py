"""Parsing the Pico's ``STATE`` line.

``/pico/status`` is a ``std_msgs/String``. ``pico_bridge`` reads lines from the
serial link and republishes any line starting with ``STATE`` **verbatim**,
without parsing it. So the GUI has to read text, and this module is the only
place in the codebase that does.

One format, on purpose
======================

This parser accepts ``STATE key=value`` and nothing else. That is the format
``pico-node_v4`` emits (``firmware/pico-node_v4/``), and earlier firmwares are
not supported.

Accepting several formats was considered and rejected. Two accepted formats
means one of them is rarely exercised, and a parser branch nobody runs is a
parser branch nobody notices breaking — which is exactly how the previous
mismatch survived: ``pico_bridge`` filtered for ``STATE``, the flashed firmware
emitted ``[STAT]``, every line was dropped silently, and the downlink kept
working so nothing looked wrong. A line this parser cannot read now comes back
``parsed=False`` and the pre-flight fails loudly.

Version, and why it is checked
==============================

Every line carries ``ver=`` as its **first** field, so an unrecognised version
can be rejected before a single field is misread. :data:`PROTOCOL_VERSION` is
what this parser understands; anything else sets
:attr:`PicoState.version_mismatch` and the pre-flight check **fails**.

That is deliberately stricter than a warning. A version this parser does not
know is a firmware whose field meanings are unknown, and a plausible-looking
vessel panel built from misread fields is worse than no panel at all.

Nothing here fabricates a value. A field the line does not contain comes back
``None``, travels to the GUI as ``null``, and renders as "not sent" — never as
zero, and never as a fault.

Capturing a real line
---------------------

On the Jetson, with the Pico connected::

    ros2 topic echo /pico/status --field data

Paste a few lines into ``test_pico_state.py``, check them against
:func:`parse_state_line`, and set :data:`FORMAT_VERIFIED` to ``True``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

#: The firmware protocol version this parser understands. Must match
#: ``FW_VERSION`` in ``firmware/pico-node_v4/pico-node_v4.ino``.
PROTOCOL_VERSION = 4

#: Set to True only when a real line off real hardware has been captured and
#: checked against this parser. Reading the firmware source is not the same as
#: reading its output: the pre-flight surfaces this to the operator until
#: somebody has actually looked.
FORMAT_VERIFIED = False

MODE_ESTOP = "ESTOP"
MODE_MANUAL = "MANUAL"
MODE_AUTONOMOUS = "AUTONOMOUS"

#: ``enum OperationMode { MODE_ESTOP = 1, MODE_MANUAL = 2, MODE_AUTONOMOUS = 3 }``
#:
#: These are the *firmware's* numbers. They are deliberately not the GUI's
#: numbers (``asket_interfaces/PicoStatus`` counts from 0), and the two are kept
#: apart by translating through the names here rather than by assuming an
#: offset. An off-by-one between those two scales would silently turn MANUAL
#: into AUTONOMOUS on screen.
_MODE_INTS = {1: MODE_ESTOP, 2: MODE_MANUAL, 3: MODE_AUTONOMOUS}

#: Word forms, accepted alongside the integers. The firmware sends integers;
#: a human typing into a serial terminal to reproduce a bug sends words.
_MODE_WORDS = {
    "ESTOP": MODE_ESTOP,
    "E-STOP": MODE_ESTOP,
    "MANUAL": MODE_MANUAL,
    "AUTO": MODE_AUTONOMOUS,
    "AUTONOMOUS": MODE_AUTONOMOUS,
}

#: Raw SBUS counts. The scale is 11 bits, clamped by the protocol to 172..1811
#: with 991 at centre — **not** a percentage, and every threshold below is on
#: this scale. ``MODE_LOW_MAX`` and ``ARM_THRESHOLD`` are the firmware's own
#: constants; if you change them there, change them here.
SBUS_MIN = 172
SBUS_MID = 991
SBUS_MAX = 1811

#: Below this on channel 8, the firmware forces ``MODE_ESTOP``. This is the
#: threshold the GUI alarms on, and it is the firmware's ``MODE_LOW_MAX``.
CH8_ESTOP_MAX = 700

#: Above this on channel 7, the firmware arms. The firmware's ``ARM_THRESHOLD``.
CH7_ARM_MIN = 1000

_TRUE = {"1", "true", "yes", "on"}
_FALSE = {"0", "false", "no", "off"}

#: Every key ``pico-node_v4`` emits, and the attribute it lands on. A key that
#: is not here goes to ``unknown_keys`` and the pre-flight reports it: an
#: unrecognised key usually means the firmware moved and this file did not.
_FIELDS = {
    "ver": "firmware_version",
    "mode": "mode",
    "armed": "armed",
    "relay": "relay_closed",
    "wantauto": "mode_requested_auto",
    "link": "link_live",
    "estoplatch": "estop_latched",
    "thr": "rc_throttle_raw",
    "yaw": "rc_yaw_raw",
    "ch7": "rc_channel7_raw",
    "ch8": "rc_channel8_raw",
    "sbusok": "sbus_frames_ok",
    "sbusbad": "sbus_frames_bad",
    "sbusfs": "sbus_failsafe",
    "sbuslost": "sbus_frame_lost",
}

_BOOL_FIELDS = {
    "armed", "relay_closed", "mode_requested_auto", "link_live",
    "estop_latched", "sbus_failsafe", "sbus_frame_lost",
}
_INT_FIELDS = {
    "firmware_version", "rc_throttle_raw", "rc_yaw_raw", "rc_channel7_raw",
    "rc_channel8_raw", "sbus_frames_ok", "sbus_frames_bad",
}

#: A line is only called parsed when these are present. One understood field is
#: not enough: ``armed=1`` alone used to mark a line good, which meant a line
#: whose mode had been misread still reported ``parsed=True`` and the operator
#: saw a confident-looking panel with an unknown mode in it.
_ESSENTIAL = ("mode", "armed")


@dataclass
class PicoState:
    """What the GUI needs from the Pico.

    Everything is optional. ``None`` means the line did not carry it, and the
    GUI renders that as "not sent" rather than as a zero or a fault.
    """

    #: False when the line could not be understood, or was understood but did
    #: not carry :data:`_ESSENTIAL`. The raw text is kept either way.
    parsed: bool = False
    raw: str = ""

    #: True when a ``ver=`` was present and is not :data:`PROTOCOL_VERSION`.
    #: Distinct from ``firmware_version is None``, which means the line carried
    #: no version at all — an even older firmware, or a corrupted line.
    version_mismatch: bool = False
    firmware_version: int | None = None

    mode: str | None = None
    armed: bool | None = None
    relay_closed: bool | None = None
    estop_latched: bool | None = None

    #: What the Jetson last asked for, which is **not** what it got. Shown
    #: beside ``mode`` so "I asked for AUTO and it is still MANUAL" is legible
    #: as a refusal by channel 8 rather than as a lost command.
    mode_requested_auto: bool | None = None
    #: The Pico's view of the Jetson's heartbeat. Zero here while pico_bridge
    #: is running means the link, not the software.
    link_live: bool | None = None

    rc_throttle_raw: int | None = None
    rc_yaw_raw: int | None = None
    rc_channel7_raw: int | None = None
    rc_channel8_raw: int | None = None

    sbus_frames_ok: int | None = None
    sbus_frames_bad: int | None = None
    sbus_failsafe: bool | None = None
    sbus_frame_lost: bool | None = None

    #: Keys seen in the line that this parser does not recognise. Reported
    #: rather than dropped: an unknown key is usually the field that matters.
    unknown_keys: list[str] = field(default_factory=list)

    @property
    def format_verified(self) -> bool:
        return FORMAT_VERIFIED

    @property
    def rc_link_ok(self) -> bool | None:
        """The receiver's own failsafe bit, inverted.

        This is a *report*, not an inference: ``sbusfs`` is bit 3 of the SBUS
        flags byte, set by the receiver itself when it has lost the
        transmitter.

        Its blind spot is worth stating, because it is the dangerous direction.
        The bit describes the **last decoded frame**. If frames stop arriving
        altogether it simply stops updating, and this property keeps returning
        True from a frame that may be seconds old. What covers that case is the
        firmware, not this: its 500 ms SBUS timeout forces ``MODE_ESTOP``, so a
        dead transmitter shows up in ``mode`` within half a second. Read the
        two together, and read both against the age of the sample.
        """
        if self.sbus_failsafe is None:
            return None
        return not self.sbus_failsafe

    @property
    def ch8_asserting_estop(self) -> bool | None:
        """Whether channel 8 is below the firmware's ESTOP threshold.

        The comparison lives here, once, on the raw SBUS scale. It used to live
        in the GUI as ``pct < 25`` against a field misnamed ``_raw_pct`` that
        actually held a raw count — so it never fired, including at the bottom
        of the travel, which is the one position it existed to catch.
        """
        if self.rc_channel8_raw is None:
            return None
        return self.rc_channel8_raw < CH8_ESTOP_MAX


def parse_state_line(line: str) -> PicoState:
    """Turn one raw ``STATE ...`` line into a :class:`PicoState`.

    **The only function in this repository that knows the wire format.**

    Never raises. Never invents. An unparseable line comes back with
    ``parsed=False`` and nothing filled in.
    """
    state = PicoState(raw=line if isinstance(line, str) else "")
    if not isinstance(line, str):
        return state

    body = line.strip()
    if not body.upper().startswith("STATE"):
        return state
    body = body[len("STATE"):]
    # "STATEMENT ..." must not be read as a STATE line carrying "MENT".
    if body and not body[:1].isspace():
        return state
    body = body.strip()
    if not body:
        # "STATE" with nothing after it is well formed and carries nothing,
        # which is not the same as parsed.
        return state

    for token in body.split():
        key, sep, value = token.partition("=")
        if not sep:
            state.unknown_keys.append(token)
            continue
        name = _FIELDS.get(key.strip().lower())
        if name is None:
            state.unknown_keys.append(key.strip())
            continue
        _assign(state, name, value.strip())

    if state.firmware_version is not None:
        state.version_mismatch = state.firmware_version != PROTOCOL_VERSION

    state.parsed = all(getattr(state, n) is not None for n in _ESSENTIAL)
    return state


def _assign(state: PicoState, name: str, value: str) -> None:
    """Set one field, or leave it ``None`` and record why.

    A value that cannot be read as its type is a value we do not have, and that
    is what the GUI must be told — never a zero, never a default.
    """
    if name == "mode":
        mode = None
        if value.isdigit():
            mode = _MODE_INTS.get(int(value))
        if mode is None:
            mode = _MODE_WORDS.get(value.upper())
        if mode is None:
            state.unknown_keys.append(f"mode={value}")
            return
        state.mode = mode
        return

    if name in _BOOL_FIELDS:
        low = value.lower()
        if low in _TRUE:
            setattr(state, name, True)
        elif low in _FALSE:
            setattr(state, name, False)
        else:
            state.unknown_keys.append(f"{name}={value}")
        return

    if name in _INT_FIELDS:
        try:
            setattr(state, name, int(value))
        except ValueError:
            state.unknown_keys.append(f"{name}={value}")
        return

    state.unknown_keys.append(f"{name}={value}")
