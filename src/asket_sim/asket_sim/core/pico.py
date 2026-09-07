"""Simulated Pico.

The Pico is the authority on what the vessel is *actually* doing. The GUI must
never display a requested mode — only a confirmed one (docs/safety.md rule 4).
This simulator therefore models the thing that makes that rule non-trivial: a
mode request takes time to take effect, can be rejected, and can be lost.

It also models the two things that are *not* ours to control: the hardware
killswitch and RC channel 8. Software can observe them. Software cannot move
them. Attempting to do so from here is a bug, and there is a test for it.
"""

from __future__ import annotations

import random
from dataclasses import dataclass

MODE_ESTOP = 0
MODE_MANUAL = 1
MODE_AUTONOMOUS = 2

MODE_NAMES = {MODE_ESTOP: "ESTOP", MODE_MANUAL: "MANUAL", MODE_AUTONOMOUS: "AUTONOMOUS"}
MODE_VALUES = {v: k for k, v in MODE_NAMES.items()}


@dataclass
class PicoConfig:
    #: How long the Pico takes to act on a mode request and report back.
    confirm_delay_s: float = 0.6
    #: Probability a mode request is simply lost, so the GUI's timeout path is
    #: exercised rather than assumed.
    request_loss_probability: float = 0.0
    num_relays: int = 4
    num_escs: int = 2


@dataclass
class PicoSample:
    utc_ms: int
    mode: int
    armed: bool
    estop_latched: bool
    relay_states: list[bool]
    esc_status: list[int]
    rc_link_ok: bool
    rc_channel8_raw_pct: int
    hardware_killswitch_engaged: bool


class PicoSim:
    def __init__(self, config: PicoConfig | None = None, seed: int = 4) -> None:
        self.cfg = config or PicoConfig()
        self._rng = random.Random(seed)
        self.mode = MODE_MANUAL
        self.armed = False
        self.estop_latched = False
        self.rc_link_ok = True
        #: The sovereign channel. 0 = kill asserted. Only the operator's
        #: transmitter moves this; nothing in software may write it.
        self.rc_channel8_raw_pct = 100
        self.hardware_killswitch_engaged = False
        self.relay_states = [False] * self.cfg.num_relays
        self.esc_status = [0] * self.cfg.num_escs
        self._pending: tuple[int, float] | None = None
        self._t = 0.0

    # -- command surface --------------------------------------------------

    def request_mode(self, mode: int) -> bool:
        """Queue a mode change. Returns whether the request was *accepted*.

        Acceptance is not confirmation. Confirmation is a later ``sample()``
        reporting the new mode.
        """
        if mode not in MODE_NAMES:
            return False
        if self._rng.random() < self.cfg.request_loss_probability:
            return True  # accepted, then silently lost — the nastiest case
        self._pending = (mode, self._t + self.cfg.confirm_delay_s)
        return True

    # -- hardware-side events, not commandable from software --------------

    def set_hardware_killswitch(self, engaged: bool) -> None:
        """Operator pressed the physical killswitch. Simulation input only."""
        self.hardware_killswitch_engaged = engaged
        if engaged:
            self.mode = MODE_ESTOP
            self.estop_latched = True
            self.armed = False
            self._pending = None

    def set_rc_channel8(self, pct: int) -> None:
        """Operator moved the RC kill channel. Simulation input only."""
        self.rc_channel8_raw_pct = max(0, min(100, pct))
        if self.rc_channel8_raw_pct < 25:
            self.mode = MODE_ESTOP
            self.estop_latched = True
            self.armed = False
            self._pending = None

    def set_rc_link(self, ok: bool) -> None:
        self.rc_link_ok = ok

    # -- simulation -------------------------------------------------------

    def step(self, dt: float) -> None:
        self._t += dt
        if self._pending and self._t >= self._pending[1]:
            mode, _ = self._pending
            self._pending = None
            # Hardware wins: no software request can leave ESTOP while the
            # killswitch or the RC channel is asserting it.
            blocked = self.hardware_killswitch_engaged or self.rc_channel8_raw_pct < 25
            if blocked and mode != MODE_ESTOP:
                return
            self.mode = mode
            self.estop_latched = mode == MODE_ESTOP
            self.armed = mode != MODE_ESTOP
            self.relay_states = [self.armed] * self.cfg.num_relays
            self.esc_status = [0 if self.armed else 1] * self.cfg.num_escs

    def sample(self, utc_ms: int) -> PicoSample:
        return PicoSample(
            utc_ms=utc_ms,
            mode=self.mode,
            armed=self.armed,
            estop_latched=self.estop_latched,
            relay_states=list(self.relay_states),
            esc_status=list(self.esc_status),
            rc_link_ok=self.rc_link_ok,
            rc_channel8_raw_pct=self.rc_channel8_raw_pct,
            hardware_killswitch_engaged=self.hardware_killswitch_engaged,
        )
