"""Link shaping: make the simulated link actually slow.

Without this, "degraded link" in sim only means *fewer streams are subscribed*.
The wire itself stays a loopback socket at gigabit speed, so the thing the whole
bandwidth-negotiation design exists to survive — a link that is genuinely narrow
and genuinely late — is never once exercised before Namibia.

So in sim the outbound side of each client is shaped to the bearer the simulated
link reports:

* **Serialisation delay.** A frame of N bytes occupies ``N / capacity`` seconds
  of the link. Frames queue behind each other, which is what actually makes a
  narrow link feel narrow.
* **Latency.** Half the round-trip time is added to each frame's arrival.
* **Loss.** A configurable fraction of frames never arrive. The client must
  cope, which for the append-only streams means noticing the gap and resyncing
  rather than splicing a hole.

Shaping applies only to what the server sends. The measured round-trip time the
profile selector runs on therefore reflects the shaping, which is the point: the
selector sees what the operator's laptop sees.

Never enabled against real hardware. There is one flag, it defaults to off, and
:class:`RosSource` never sets it.
"""

from __future__ import annotations

import random
import time
from dataclasses import dataclass, field

#: Frames smaller than this still cost this much to put on the wire — headers,
#: framing, and the fact that no real link sends a 40-byte packet for free.
MIN_FRAME_BYTES = 120


@dataclass
class ShaperStats:
    frames_sent: int = 0
    frames_dropped: int = 0
    bytes_sent: int = 0
    total_delay_s: float = 0.0

    @property
    def mean_delay_ms(self) -> float:
        return (self.total_delay_s / self.frames_sent * 1000.0) if self.frames_sent else 0.0


@dataclass
class LinkShaper:
    """Per-client outbound shaping. One instance per WebSocket."""

    enabled: bool = False
    #: Usable bytes per second. Updated from the simulated link as the vessel
    #: moves, so the link degrades with distance exactly as it would offshore.
    capacity_bytes_per_s: float = 800_000.0
    rtt_ms: float = 25.0
    loss_ratio: float = 0.0
    seed: int | None = None

    _rng: random.Random = field(init=False)
    _busy_until: float = field(default=0.0, init=False)
    stats: ShaperStats = field(default_factory=ShaperStats, init=False)

    def __post_init__(self) -> None:
        self._rng = random.Random(self.seed)

    def update(self, capacity_bytes_per_s: float, rtt_ms: float, loss_ratio: float = 0.0) -> None:
        self.capacity_bytes_per_s = max(1.0, capacity_bytes_per_s)
        self.rtt_ms = max(0.0, rtt_ms)
        self.loss_ratio = min(0.9, max(0.0, loss_ratio))

    def should_drop(self) -> bool:
        if not self.enabled or self.loss_ratio <= 0.0:
            return False
        if self._rng.random() < self.loss_ratio:
            self.stats.frames_dropped += 1
            return True
        return False

    def delay_for(self, size_bytes: int, now: float | None = None) -> float:
        """Seconds to wait before this frame arrives.

        Frames queue: a frame sent while the link is still busy with the
        previous one waits for it. That queueing is the whole reason a narrow
        link feels narrow rather than merely late.
        """
        if not self.enabled:
            return 0.0
        now = now if now is not None else time.monotonic()
        size = max(MIN_FRAME_BYTES, size_bytes)

        start = max(now, self._busy_until)
        serialisation_s = size / self.capacity_bytes_per_s
        self._busy_until = start + serialisation_s

        arrival = self._busy_until + (self.rtt_ms / 2000.0)
        delay = max(0.0, arrival - now)

        self.stats.frames_sent += 1
        self.stats.bytes_sent += size
        self.stats.total_delay_s += delay
        return delay

    def to_dict(self) -> dict:
        return {
            "enabled": self.enabled,
            "capacity_bytes_per_s": round(self.capacity_bytes_per_s),
            "rtt_ms": round(self.rtt_ms, 1),
            "loss_ratio": round(self.loss_ratio, 3),
            "frames_sent": self.stats.frames_sent,
            "frames_dropped": self.stats.frames_dropped,
            "mean_delay_ms": round(self.stats.mean_delay_ms, 1),
        }
