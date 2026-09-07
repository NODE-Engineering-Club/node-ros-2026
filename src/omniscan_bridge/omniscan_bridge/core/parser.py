"""Incremental, resynchronising Ping Protocol parser.

Separate from :mod:`omniscan_bridge.core.ping_protocol`, which only knows how to
turn bytes into messages and back. This module knows how to survive a real link:

* **partial frames** — a frame split across two reads is held until complete;
* **corruption** — a bad checksum discards that frame and resynchronises on the
  next plausible start marker, rather than desynchronising for good;
* **truncation** — bytes lost from the middle of the stream are counted, so a
  framing problem shows up as a number in the sonar health panel instead of as
  a mysterious absence of data;
* **gaps and reordering** — ping numbers are tracked, so packet loss is
  measured rather than guessed, and a late ping is not mistaken for a new one.

None of it blocks and none of it allocates without bound. ``feed()`` is pure:
give it bytes, get back messages. The socket lives elsewhere.
"""

from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass

from . import ping_protocol as pp

#: Largest payload we will ever wait for. The length field is a uint16, so the
#: protocol permits up to 65535, but the largest frame this device actually
#: sends is a point set: 20 bytes of header plus 16 per point. At 2048 points
#: that is under 33 KiB, so anything past 40 KiB is a corrupt length field, not
#: a frame. Bounding it here means a corrupt length resynchronises immediately
#: instead of stalling the stream waiting for bytes that will never arrive.
DEFAULT_MAX_PAYLOAD_LENGTH = 40_000

#: A real frame completes within milliseconds. If one has been incomplete for
#: longer than this, its length field was corrupt in a way the bound above
#: cannot catch, and we resynchronise rather than waiting indefinitely.
DEFAULT_STALL_TIMEOUT_S = 2.0


@dataclass
class ParserStats:
    """Everything the sonar health panel needs to explain what it is seeing."""

    frames_parsed: int = 0
    point_sets: int = 0
    checksum_errors: int = 0
    #: Bytes thrown away while hunting for a valid frame. Non-zero means the
    #: stream is being damaged somewhere; it is never normal.
    bytes_discarded: int = 0
    #: Frames whose declared point count disagreed with the payload length.
    #: A persistent non-zero value means our layout assumption is wrong — see
    #: docs/open_questions.md Q8 — not that the sonar is faulty.
    length_mismatches: int = 0
    pings_seen: int = 0
    pings_missing: int = 0
    pings_out_of_order: int = 0
    decode_errors: int = 0

    @property
    def packet_loss_ratio(self) -> float:
        """Fraction of expected pings that never arrived, 0..1."""
        expected = self.pings_seen + self.pings_missing
        return self.pings_missing / expected if expected else 0.0


class PingParser:
    """Feed it bytes, take messages out.

    >>> parser = PingParser()
    >>> frames = parser.feed(some_bytes)
    """

    #: Never buffer more than this while waiting for a frame to complete. A
    #: corrupt length field must not be able to make us hold megabytes.
    MAX_BUFFER = 4 * 1024 * 1024

    def __init__(
        self,
        loss_window: int = 200,
        max_payload_length: int = DEFAULT_MAX_PAYLOAD_LENGTH,
        stall_timeout_s: float = DEFAULT_STALL_TIMEOUT_S,
    ) -> None:
        self._buf = bytearray()
        self._now = 0.0
        self.max_payload_length = max_payload_length
        self.stall_timeout_s = stall_timeout_s
        #: When the frame currently at the head of the buffer was first seen
        #: incomplete. ``None`` when nothing is pending.
        self._pending_since: float | None = None
        self.stats = ParserStats()
        self._last_ping_number: int | None = None
        #: Recent ping numbers, so a ping arriving late is recognised as late
        #: rather than counted as a fresh one.
        self._recent_pings: deque[int] = deque(maxlen=loss_window)

    # -- framing ----------------------------------------------------------

    def feed(self, data: bytes, now: float | None = None) -> list[pp.Frame]:
        """Consume bytes, return every complete, checksum-valid frame in them.

        ``now`` is injectable so the stall guard can be tested without sleeping.
        """
        self._now = now if now is not None else time.monotonic()
        self._buf += data
        if len(self._buf) > self.MAX_BUFFER:
            # Keep the tail: the newest bytes are the ones most likely to
            # contain a recoverable frame boundary.
            dropped = len(self._buf) - self.MAX_BUFFER
            del self._buf[:dropped]
            self.stats.bytes_discarded += dropped

        frames: list[pp.Frame] = []
        while True:
            frame = self._next_frame()
            if frame is None:
                return frames
            frames.append(frame)

    def _next_frame(self) -> pp.Frame | None:
        buf = self._buf
        while True:
            start = buf.find(pp.START_BYTES)
            if start < 0:
                # No marker at all. Keep the last byte: it might be the 'B' of a
                # marker split across this read and the next.
                keep = 1 if buf.endswith(b"B") else 0
                discarded = len(buf) - keep
                if discarded > 0:
                    self.stats.bytes_discarded += discarded
                    del buf[: len(buf) - keep]
                return None

            if start > 0:
                # Everything before a start marker is rubbish by definition.
                self.stats.bytes_discarded += start
                del buf[:start]

            if len(buf) < pp.HEADER_SIZE:
                return None  # header still incomplete

            _, length, msg_id, src, dst = pp._HEADER_FMT.unpack_from(buf, 0)
            total = pp.HEADER_SIZE + length + pp.CHECKSUM_SIZE

            if length > self.max_payload_length:
                # A length no real frame can have. Corrupt: resynchronise now
                # rather than blocking the stream on bytes that never come.
                self._resync()
                continue

            if len(buf) < total:
                # Incomplete. Normal — unless it stays that way, which means the
                # length field was corrupt within the plausible range.
                if self._pending_since is None:
                    self._pending_since = self._now
                elif self._now - self._pending_since > self.stall_timeout_s:
                    self._resync()
                    self._pending_since = None
                    continue
                return None

            self._pending_since = None

            body = bytes(buf[: pp.HEADER_SIZE + length])
            (declared,) = pp._CHECKSUM_FMT.unpack_from(buf, pp.HEADER_SIZE + length)

            if declared != pp.checksum(body):
                self.stats.checksum_errors += 1
                self._resync()
                continue

            del buf[:total]
            self.stats.frames_parsed += 1
            return pp.Frame(msg_id, src, dst, body[pp.HEADER_SIZE :])

    def _resync(self) -> None:
        """Drop the current start marker and hunt for the next one."""
        self.stats.bytes_discarded += len(pp.START_BYTES)
        del self._buf[: len(pp.START_BYTES)]

    # -- decoding ---------------------------------------------------------

    def decode(self, frame: pp.Frame):
        """Decode a frame's payload, updating statistics.

        Returns ``None`` for message types we do not consume — which is not an
        error; the device sends more than we need.
        """
        try:
            message = pp.decode_payload(frame)
        except pp.PingProtocolError:
            self.stats.decode_errors += 1
            return None

        if isinstance(message, pp.PointSet):
            self.stats.point_sets += 1
            if message.length_mismatch:
                self.stats.length_mismatches += 1
            self._account_ping(message.ping_number)
        return message

    def feed_and_decode(self, data: bytes) -> list[tuple[pp.Frame, object]]:
        """Convenience: framing and decoding in one call."""
        return [(f, self.decode(f)) for f in self.feed(data)]

    # -- ping accounting --------------------------------------------------

    def _account_ping(self, ping_number: int) -> None:
        """Track gaps and reordering in the ping sequence.

        The sonar increments ``ping_number`` whether or not the packet reaches
        us, so a gap is real evidence of loss. That makes ``packet_loss_ratio``
        a measurement rather than an estimate — which matters, because it is the
        number that tells an operator whether the survey is being recorded
        properly or quietly holed.
        """
        last = self._last_ping_number
        if last is None:
            self.stats.pings_seen += 1
            self._last_ping_number = ping_number
            self._recent_pings.append(ping_number)
            return

        if ping_number in self._recent_pings:
            self.stats.pings_out_of_order += 1
            return  # a duplicate; do not double-count it

        if ping_number < last:
            # Either a late arrival or the sonar restarted its counter.
            if last - ping_number > self._recent_pings.maxlen:
                self._reset_sequence(ping_number)
            else:
                self.stats.pings_out_of_order += 1
                # It filled a gap we had already counted as missing.
                self.stats.pings_missing = max(0, self.stats.pings_missing - 1)
                self.stats.pings_seen += 1
                self._recent_pings.append(ping_number)
            return

        gap = ping_number - last - 1
        if gap > 0:
            self.stats.pings_missing += gap
        self.stats.pings_seen += 1
        self._last_ping_number = ping_number
        self._recent_pings.append(ping_number)

    def _reset_sequence(self, ping_number: int) -> None:
        """The sonar restarted. Start counting again rather than reporting a
        loss of several thousand pings that never existed."""
        self._last_ping_number = ping_number
        self._recent_pings.clear()
        self._recent_pings.append(ping_number)
        self.stats.pings_seen += 1

    def reset(self) -> None:
        """Called on reconnection. Keeps cumulative counters, drops stream state."""
        self._buf.clear()
        self._pending_since = None
        self._last_ping_number = None
        self._recent_pings.clear()
