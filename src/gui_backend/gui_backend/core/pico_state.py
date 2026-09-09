"""Parsing the Pico's ``STATE`` line.

``/pico/status`` is a ``std_msgs/String``. ``pico_bridge`` reads lines from the
serial link and republishes any line starting with ``STATE`` **verbatim**,
without parsing it (see ``control/control/pico_bridge.py``). So the GUI has to
read text, and this module is the only place in the codebase that does.

PROVISIONAL — nobody here has seen a real STATE line
====================================================

The exact format lives in Pico firmware v3 and has not been captured. Every
field name below is a **guess at a plausible shape**, not a transcription, and
this module is written so that replacing the guess is a small, local change:

* :data:`FORMAT_VERIFIED` is ``False``. Flip it to ``True`` only once a real
  line has been captured and the parser checked against it. The pre-flight
  check reads that flag and warns on every run until it is set.
* :func:`parse_state_line` is the *only* function that knows the wire format.
  Everything downstream consumes :class:`PicoState`, which is stable.
* Nothing here fabricates a value. A field the line does not contain comes back
  ``None``, travels to the GUI as ``null``, and renders as "not sent" — never
  as zero, and never as a fault.

**The parser never raises and never guesses.** An unrecognised line yields a
``PicoState`` with ``parsed=False`` and the raw text kept, which the GUI shows
as "unrecognised" rather than inventing a mode. Silently mapping an unknown
line onto MANUAL would be the most dangerous thing this file could do: the
panel would claim to know a confirmed vessel state that nobody confirmed.

Capturing the real format
-------------------------

On the Jetson, with the Pico connected::

    ros2 topic echo /pico/status --field data

Paste a few lines into ``test_pico_state.py`` as a new test, adjust
:data:`_FIELD_ALIASES` or :func:`parse_state_line` to match, and set
:data:`FORMAT_VERIFIED` to ``True``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

#: Set to True only when a real STATE line has been captured and this parser
#: checked against it. The pre-flight check surfaces this to the operator.
FORMAT_VERIFIED = False

#: Modes the Pico is known to have, from pico_bridge's own interface: it accepts
#: only "MODE AUTO" and "MODE MANUAL". There is no software ESTOP path.
MODE_MANUAL = "MANUAL"
MODE_AUTONOMOUS = "AUTONOMOUS"
MODE_UNKNOWN = "UNKNOWN"

#: How a mode word in the line maps to the GUI's vocabulary. "AUTO" is what
#: pico_bridge sends; "AUTONOMOUS" is what the GUI calls it.
_MODE_WORDS = {
    "AUTO": MODE_AUTONOMOUS,
    "AUTONOMOUS": MODE_AUTONOMOUS,
    "MANUAL": MODE_MANUAL,
    "MAN": MODE_MANUAL,
}

#: PROVISIONAL: plausible key spellings for the same quantity. Extend rather
#: than rewrite when the real format is known.
_FIELD_ALIASES = {
    "mode": ("mode", "m"),
    "armed": ("armed", "arm", "a"),
    "rc_link_ok": ("rc", "rc_ok", "rclink"),
    "rc_channel8_raw_pct": ("ch8", "ch8_pct", "kill_pct"),
    "battery_voltage": ("vbat", "v", "volt", "voltage"),
    "battery_current": ("ibat", "i", "amps", "current"),
}

_TRUE = {"1", "true", "yes", "on", "ok", "armed"}
_FALSE = {"0", "false", "no", "off", "disarmed"}


@dataclass
class PicoState:
    """What the GUI needs from the Pico, however the line happens to spell it.

    Everything is optional. A ``None`` means the line did not carry it, and the
    GUI is built to render that as "not sent" rather than as a zero or a fault.
    """

    #: False when the line could not be understood. The raw text is kept so the
    #: operator (and whoever fixes the parser) can see what actually arrived.
    parsed: bool = False
    raw: str = ""

    mode: str | None = None
    armed: bool | None = None
    rc_link_ok: bool | None = None
    rc_channel8_raw_pct: int | None = None
    battery_voltage: float | None = None
    battery_current: float | None = None
    #: Keys seen in the line that this parser does not recognise. Reported
    #: rather than dropped: an unknown key is usually the field that matters.
    unknown_keys: list[str] = field(default_factory=list)

    @property
    def format_verified(self) -> bool:
        return FORMAT_VERIFIED


def parse_state_line(line: str) -> PicoState:
    """Turn one raw ``STATE ...`` line into a :class:`PicoState`.

    **The only function in this repository that knows the wire format.**

    It reads ``key=value`` pairs, which is the most common shape for a line like
    this and costs nothing if the real firmware turns out to use something else
    — replacing the body is a contained change. Bare words are also matched
    against the known mode names, so ``STATE MANUAL ARMED`` is understood as
    well as ``STATE mode=MANUAL armed=1``.

    Never raises. Never invents. An unparseable line comes back with
    ``parsed=False`` and nothing filled in.
    """
    state = PicoState(raw=line)
    if not isinstance(line, str):
        return state

    body = line.strip()
    if not body.upper().startswith("STATE"):
        return state
    body = body[len("STATE"):].strip()
    if not body:
        # "STATE" with nothing after it is a well-formed line carrying nothing.
        state.parsed = True
        return state

    lookup = {alias: name for name, aliases in _FIELD_ALIASES.items() for alias in aliases}
    seen_any = False

    for token in body.replace(",", " ").split():
        if "=" in token or ":" in token:
            sep = "=" if "=" in token else ":"
            key, _, value = token.partition(sep)
            name = lookup.get(key.strip().lower())
            if name is None:
                state.unknown_keys.append(key.strip())
                continue
            if _assign(state, name, value.strip()):
                seen_any = True
        else:
            # A bare word: the only thing we can safely recognise is a mode.
            word = token.strip().upper()
            if word in _MODE_WORDS:
                state.mode = _MODE_WORDS[word]
                seen_any = True
            elif word in ("ARMED", "DISARMED"):
                state.armed = word == "ARMED"
                seen_any = True
            else:
                state.unknown_keys.append(token.strip())

    state.parsed = seen_any
    return state


def _assign(state: PicoState, name: str, value: str) -> bool:
    """Set one field. Returns False if the value could not be read as its type,
    leaving the field ``None`` — a field we could not read is a field we do not
    have, and that is what the GUI must be told."""
    if name == "mode":
        mode = _MODE_WORDS.get(value.upper())
        if mode is None:
            state.unknown_keys.append(f"mode={value}")
            return False
        state.mode = mode
        return True

    if name in ("armed", "rc_link_ok"):
        low = value.lower()
        if low in _TRUE:
            setattr(state, name, True)
        elif low in _FALSE:
            setattr(state, name, False)
        else:
            return False
        return True

    if name == "rc_channel8_raw_pct":
        try:
            state.rc_channel8_raw_pct = int(round(float(value)))
        except ValueError:
            return False
        return True

    try:
        setattr(state, name, float(value))
    except ValueError:
        return False
    return True
