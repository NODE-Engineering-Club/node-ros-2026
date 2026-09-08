"""Simulated shore link.

Bandwidth negotiation is the single biggest architectural risk in this system: a
GUI that works on the bench and collapses 200 m offshore (brief, section 9).
That risk is only retired if the degraded case can be produced on demand, so the
link is simulated as a first-class source rather than assumed to be perfect.

Quality falls off with distance from the shore station, with fading on top, and
can be forced to 4G or LTE-M grade — or dropped entirely — by fault injection.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass

LINK_ETHERNET = "ethernet"
LINK_WIFI = "wifi"
LINK_4G = "4g"
LINK_LTEM = "ltem"
LINK_NONE = "none"

#: Representative characteristics of each bearer: (rtt_ms, usable bytes/s).
LINK_CHARACTERISTICS = {
    LINK_ETHERNET: (2.0, 10_000_000.0),
    LINK_WIFI: (25.0, 800_000.0),
    LINK_4G: (90.0, 120_000.0),
    LINK_LTEM: (900.0, 1_000.0),
    LINK_NONE: (float("inf"), 0.0),
}


@dataclass
class LinkConfig:
    #: Where the shore station is, in local ENU metres.
    station_east_m: float = 0.0
    station_north_m: float = -50.0
    #: Range at which WiFi has fully given out.
    wifi_range_m: float = 400.0
    #: Whether a cellular fallback exists at this site. PROVISIONAL, Q6.
    cellular_available: bool = True

    #: Fading, as an Ornstein-Uhlenbeck process: a standard deviation and a
    #: time constant, rather than a per-step noise amplitude.
    #:
    #: Parameterised this way on purpose. A per-step amplitude makes the amount
    #: of fading depend on how often the simulator happens to be stepped, so
    #: changing the tick rate silently changes the weather. These two numbers
    #: mean what they say at any step size.
    fade_std: float = 0.06
    fade_time_constant_s: float = 12.0


@dataclass
class LinkSample:
    utc_ms: int
    active_link: str
    quality: float          # 0..1
    rtt_ms: float
    capacity_bytes_per_s: float
    distance_m: float


class LinkSim:
    """Simulated shore link.

    ``step()`` advances the fading; ``sample()`` is **pure**. That separation is
    not stylistic. An earlier version advanced the fade inside ``sample()``, and
    since the backend samples the link several times per tick — once for the
    stream, once for alarms, once for profile selection — the "slow" fade was
    being advanced sixty times a second instead of once. Quality swung between
    0.26 and 0.99 on a stationary vessel, the link flipped between WiFi and
    LTE-M, and the profile selector could never hold a candidate long enough to
    recover. A read that changes what it reads is a bug waiting to happen.
    """

    def __init__(self, config: LinkConfig | None = None, seed: int = 5) -> None:
        self.cfg = config or LinkConfig()
        self._rng = random.Random(seed)
        self._fade = 0.0

    def step(self, dt: float) -> None:
        """Advance the fading by ``dt`` seconds.

        Ornstein-Uhlenbeck, discretised exactly, so the steady-state spread is
        ``fade_std`` whatever step size the caller uses.
        """
        cfg = self.cfg
        if dt <= 0.0 or cfg.fade_time_constant_s <= 0.0:
            return
        decay = math.exp(-dt / cfg.fade_time_constant_s)
        kick = cfg.fade_std * math.sqrt(max(0.0, 1.0 - decay * decay))
        self._fade = decay * self._fade + self._rng.gauss(0.0, kick)

    def sample(
        self,
        utc_ms: int,
        east_m: float,
        north_m: float,
        forced: str | None = None,
    ) -> LinkSample:
        cfg = self.cfg
        dist = math.hypot(east_m - cfg.station_east_m, north_m - cfg.station_north_m)

        wifi_q = max(0.0, 1.0 - (dist / cfg.wifi_range_m) ** 1.6) + self._fade
        wifi_q = max(0.0, min(1.0, wifi_q))

        if forced is not None:
            link = forced
            quality = 0.0 if link == LINK_NONE else max(0.15, wifi_q)
        elif wifi_q > 0.25:
            link = LINK_WIFI
            quality = wifi_q
        elif cfg.cellular_available and wifi_q > 0.02:
            link = LINK_4G
            # Derived from the fade rather than drawn fresh, so that sampling
            # twice in a row cannot report two different links.
            quality = 0.55 + 0.15 * self._fade / max(1e-6, cfg.fade_std)
            quality = max(0.2, min(0.85, quality))
        elif cfg.cellular_available:
            link = LINK_LTEM
            quality = 0.2
        else:
            link = LINK_NONE
            quality = 0.0

        base_rtt, capacity = LINK_CHARACTERISTICS[link]
        # Degrading quality shows up as latency long before it shows up as an
        # outage, which is why the profile selector watches RTT.
        rtt = base_rtt * (1.0 + 2.0 * (1.0 - quality) ** 2)
        return LinkSample(
            utc_ms=utc_ms,
            active_link=link,
            quality=quality,
            rtt_ms=rtt,
            capacity_bytes_per_s=capacity * max(0.05, quality),
            distance_m=dist,
        )
