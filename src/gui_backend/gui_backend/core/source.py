"""Where the backend gets its data.

One interface, two implementations:

* :class:`~gui_backend.core.sim_source.SimSource` drives everything from
  ``asket_sim``, in-process, with no ROS. This is what runs on a laptop.
* :class:`~gui_backend.core.ros_source.RosSource` subscribes to real topics.

The interface is narrow on purpose. Anything the backend can do through it is
something both modes support, so a feature cannot accidentally come to depend on
being in one mode or the other.

The topic names and message types a ``RosSource`` uses are **not** in the code.
They live in ``config/topics.yaml``, because the existing ``pico_bridge``
message is unknown to this repository and `pico_bridge` must not be modified
(docs/open_questions.md Q7). Pointing the backend at the real message is a
config change plus one adapter function.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


@dataclass
class Sample:
    """One stream's current value.

    ``source_utc_ms`` is when the value was *produced*, never when it was sent.
    Everything about honest data age depends on that distinction.
    """

    stream: str
    source_utc_ms: int
    payload: dict


@dataclass
class CommandOutcome:
    accepted: bool
    detail: str = ""


class DataSource(Protocol):
    """What the hub needs from a source of vessel data."""

    def step(self, now_s: float) -> None:
        """Advance to wall-clock ``now_s``. A no-op for a ROS source."""

    def now_utc_ms(self) -> int:
        """The source's idea of the current time, in UTC milliseconds."""

    def snapshot(self, stream: str, detail: str) -> Sample | None:
        """Current value of a stream, or ``None`` if there isn't one yet."""

    def state(self) -> dict:
        """Everything at full detail, for alarms and command confirmation.

        Keyed by stream name. This is what command confirmation predicates read,
        so it must reflect what the *vessel* reports and never what was asked
        for.
        """

    def send_command(self, name: str, args: dict) -> CommandOutcome:
        """Send a command onward. Acceptance is not confirmation."""

    def describe(self) -> dict:
        """Static facts a client needs once: mode, survey plan, thresholds."""
