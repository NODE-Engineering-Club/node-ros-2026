"""Merge a recorded mission into a SonarView-readable ``.svlog``.

Why this exists
---------------

The original plan (Option B) was to record the raw sonar stream and a
timestamped trajectory as two files and let SonarView pair them afterwards.
**SonarView cannot do that.** Position and heading have to live *inside* the
log, as ``NMEA_WRAPPER`` packets written at capture time; a log with no position
packets cannot be georeferenced or exported at all.

So Option B survives, with one addition: we still record the two files
separately during the mission — which keeps the recorder simple, keeps the raw
stream unmodified, and keeps SonarView off the boat — and then **merge** them
into a valid ``.svlog`` afterwards. Nothing about the onboard recording changes.

What a ``.svlog`` is
--------------------

A concatenation of Cerulean Ping Protocol packets, in time order, with no file
header. So the merge is a matter of interleaving: walk the sonar stream, and
before each ping emit the navigation packets that belong to the moment just
before it.

What is emitted
---------------

Per the Cerulean Ping Protocol documentation, ``NMEA_WRAPPER`` (packet 109)
carries "variable length string to be interpreted as NMEA 0183 sentence". Two
sentences carry what a survey needs:

* ``$GPGGA`` — time, latitude, longitude, fix quality, satellites, HDOP.
* ``$GPHDT`` — true heading.

Heading gets its own sentence on purpose. ``$GPGGA`` has no heading field, and
letting SonarView infer it from successive positions would substitute course
over ground for heading — which on a vessel in a cross-current is a different
number, and the difference is the dominant error in the whole survey.

Samples whose heading was **not valid** are emitted without a ``$GPHDT``. The
trajectory records validity per sample precisely so that this decision can be
made here rather than guessed: a heading we did not trust must not be written
into the log as though we did.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator

from omniscan_bridge.core import ping_protocol as pp
from omniscan_bridge.core.parser import PingParser

from .mission import SONAR_RAW_NAME, TRAJECTORY_NAME


@dataclass
class TrajectorySample:
    utc_ms: int
    lat: float
    lon: float
    heading_deg: float | None
    heading_valid: bool
    num_sats: int = 0
    hdop: float = 0.0
    gnss_fix_type: int = 3
    alt: float = 0.0

    @classmethod
    def from_record(cls, record: dict) -> "TrajectorySample":
        return cls(
            utc_ms=int(record["utc_ms"]),
            lat=float(record["lat"]),
            lon=float(record["lon"]),
            heading_deg=(
                float(record["heading_deg"])
                if record.get("heading_deg") is not None
                else None
            ),
            heading_valid=bool(record.get("heading_valid")),
            num_sats=int(record.get("num_sats") or 0),
            hdop=float(record.get("hdop") or 0.0),
            gnss_fix_type=int(record.get("gnss_fix_type") or 0),
            alt=float(record.get("alt") or 0.0),
        )


@dataclass
class MergeStats:
    pings: int = 0
    frames_copied: int = 0
    gga_written: int = 0
    hdt_written: int = 0
    #: Samples skipped because the heading was not trustworthy at that instant.
    heading_invalid_samples: int = 0
    #: Pings that fell outside the trajectory and could not be placed.
    unplaceable_pings: int = 0
    bytes_written: int = 0
    first_utc_ms: int = 0
    last_utc_ms: int = 0

    def to_dict(self) -> dict:
        return {
            "pings": self.pings,
            "frames_copied": self.frames_copied,
            "gga_written": self.gga_written,
            "hdt_written": self.hdt_written,
            "heading_invalid_samples": self.heading_invalid_samples,
            "unplaceable_pings": self.unplaceable_pings,
            "bytes_written": self.bytes_written,
            "first_utc_ms": self.first_utc_ms,
            "last_utc_ms": self.last_utc_ms,
        }


# --------------------------------------------------------------------------
# NMEA 0183
# --------------------------------------------------------------------------


def nmea_checksum(body: str) -> str:
    """XOR of every character between ``$`` and ``*``, as two hex digits."""
    value = 0
    for char in body:
        value ^= ord(char)
    return f"{value:02X}"


def _sentence(body: str) -> str:
    return f"${body}*{nmea_checksum(body)}"


def _degrees_to_nmea(value: float, is_latitude: bool) -> tuple[str, str]:
    """Decimal degrees to NMEA ``ddmm.mmmm`` plus hemisphere.

    Six decimal places on the minutes: about 0.2 mm, comfortably finer than
    anything the GNSS can offer, so the format is never the limiting factor.
    """
    hemisphere = ("N" if value >= 0 else "S") if is_latitude else ("E" if value >= 0 else "W")
    magnitude = abs(value)
    degrees = int(magnitude)
    minutes = (magnitude - degrees) * 60.0
    width = 2 if is_latitude else 3
    return f"{degrees:0{width}d}{minutes:09.6f}", hemisphere


def _utc_to_hhmmss(utc_ms: int) -> str:
    seconds_of_day = (utc_ms // 1000) % 86400
    hours, remainder = divmod(seconds_of_day, 3600)
    minutes, seconds = divmod(remainder, 60)
    return f"{hours:02d}{minutes:02d}{seconds:02d}.{utc_ms % 1000:03d}"


def gga_sentence(sample: TrajectorySample) -> str:
    """``$GPGGA`` — time, position, fix quality, satellites, HDOP."""
    lat, north_south = _degrees_to_nmea(sample.lat, True)
    lon, east_west = _degrees_to_nmea(sample.lon, False)
    # Fix quality: 1 = GPS fix, 0 = invalid. We do not claim differential or
    # RTK; if that ever becomes true it must come from the receiver, not here.
    quality = 1 if sample.gnss_fix_type >= 3 else 0
    return _sentence(
        "GPGGA,"
        f"{_utc_to_hhmmss(sample.utc_ms)},"
        f"{lat},{north_south},{lon},{east_west},"
        f"{quality},{sample.num_sats:02d},{sample.hdop:.1f},"
        f"{sample.alt:.1f},M,0.0,M,,"
    )


def hdt_sentence(heading_deg: float) -> str:
    """``$GPHDT`` — true heading."""
    return _sentence(f"GPHDT,{heading_deg % 360.0:.2f},T")


# --------------------------------------------------------------------------
# Trajectory
# --------------------------------------------------------------------------


def load_trajectory(path: Path | str) -> list[TrajectorySample]:
    """Read ``trajectory.jsonl``, skipping records that cannot be used.

    A record without a position is not a position; it is dropped rather than
    written as zero, which would put the vessel in the Gulf of Guinea.
    """
    samples: list[TrajectorySample] = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            record = json.loads(line)
        except ValueError:
            continue
        if record.get("lat") is None or record.get("lon") is None:
            continue
        try:
            samples.append(TrajectorySample.from_record(record))
        except (KeyError, TypeError, ValueError):
            continue
    samples.sort(key=lambda s: s.utc_ms)
    return samples


def interpolate(
    samples: list[TrajectorySample], utc_ms: int, max_gap_ms: int = 2000
) -> TrajectorySample | None:
    """The trajectory at an instant, or ``None`` if it cannot be placed.

    Returns ``None`` rather than extrapolating when the instant falls outside
    the trajectory, or inside a gap longer than ``max_gap_ms``. Extrapolating
    across a GNSS dropout would produce a confident position for a stretch of
    seabed nobody actually knows the location of — and it would look exactly
    like the good data around it.

    Heading is interpolated the short way round, so 359 to 001 crosses through
    north rather than sweeping backwards through south.
    """
    if not samples:
        return None
    if utc_ms < samples[0].utc_ms or utc_ms > samples[-1].utc_ms:
        return None

    low, high = 0, len(samples) - 1
    while low < high - 1:
        middle = (low + high) // 2
        if samples[middle].utc_ms <= utc_ms:
            low = middle
        else:
            high = middle

    before, after = samples[low], samples[high]
    span = after.utc_ms - before.utc_ms
    if span > max_gap_ms:
        return None
    if span <= 0:
        return before

    fraction = (utc_ms - before.utc_ms) / span
    heading = None
    heading_valid = before.heading_valid and after.heading_valid
    if heading_valid and before.heading_deg is not None and after.heading_deg is not None:
        delta = ((after.heading_deg - before.heading_deg + 540.0) % 360.0) - 180.0
        heading = (before.heading_deg + delta * fraction) % 360.0

    return TrajectorySample(
        utc_ms=utc_ms,
        lat=before.lat + (after.lat - before.lat) * fraction,
        lon=before.lon + (after.lon - before.lon) * fraction,
        heading_deg=heading,
        heading_valid=heading_valid,
        num_sats=before.num_sats,
        hdop=before.hdop,
        gnss_fix_type=min(before.gnss_fix_type, after.gnss_fix_type),
        alt=before.alt + (after.alt - before.alt) * fraction,
    )


# --------------------------------------------------------------------------
# The merge
# --------------------------------------------------------------------------


def iter_frames(data: bytes) -> Iterator[tuple[pp.Frame, bytes, object]]:
    """Walk a raw stream, yielding ``(frame, wire bytes, decoded)``.

    The wire bytes are reconstructed from the frame rather than sliced out of
    the input, and that is byte-identical by construction: a frame is
    ``header || payload || checksum(header || payload)``, the parser only
    yields frames whose checksum already verified, and the header holds nothing
    but the length, the message id and the two device ids — all of which we
    have. ``test_merged_log_preserves_the_sonar_stream_byte_for_byte`` holds
    that true, because the sonar half of the log has to reach SonarView exactly
    as the device produced it.

    Rubbish between frames is dropped, which is the point: a ``.svlog`` has to
    be a clean sequence of packets, and the parser's counters record how much
    was discarded.
    """
    parser = PingParser()
    for offset in range(0, len(data), 65536):
        for frame in parser.feed(data[offset : offset + 65536]):
            raw = pp.encode_frame(
                frame.message_id, frame.payload, frame.src_device_id, frame.dst_device_id
            )
            yield frame, raw, parser.decode(frame)


def merge_to_svlog(
    sonar_raw: Path | str,
    trajectory: Path | str,
    output: Path | str,
    max_gap_ms: int = 2000,
) -> MergeStats:
    """Interleave navigation into a raw sonar stream and write a ``.svlog``.

    Navigation is emitted **before** each ping, so a reader that carries the
    most recent position forward has one by the time the ping arrives.
    """
    stats = MergeStats()
    samples = load_trajectory(trajectory)
    data = Path(sonar_raw).read_bytes()
    out = Path(output)
    out.parent.mkdir(parents=True, exist_ok=True)

    last_emitted_utc = 0
    with out.open("wb") as handle:
        for frame, raw, message in iter_frames(data):
            if isinstance(message, pp.PointSet) and message.utc_msec > 0:
                stats.pings += 1
                placed = interpolate(samples, message.utc_msec, max_gap_ms)
                if placed is None:
                    stats.unplaceable_pings += 1
                elif message.utc_msec != last_emitted_utc:
                    last_emitted_utc = message.utc_msec
                    handle.write(pp.encode_nmea_wrapper(gga_sentence(placed)))
                    stats.gga_written += 1
                    if placed.heading_valid and placed.heading_deg is not None:
                        handle.write(pp.encode_nmea_wrapper(hdt_sentence(placed.heading_deg)))
                        stats.hdt_written += 1
                    else:
                        # Recorded, not silently dropped: the operator needs to
                        # know which part of the survey has no heading in it.
                        stats.heading_invalid_samples += 1

                if stats.first_utc_ms == 0:
                    stats.first_utc_ms = message.utc_msec
                stats.last_utc_ms = message.utc_msec

            handle.write(raw)
            stats.frames_copied += 1

    stats.bytes_written = out.stat().st_size
    return stats


class LiveSvlogWriter:
    """Interleave navigation into a ``.svlog`` *while* the mission records.

    The post-mission merge is the primary path; this is belt and braces. If the
    trajectory file is lost or corrupted, or nobody remembers to run the merge,
    the survey is still openable — and a survey that cannot be opened is a
    survey that did not happen.

    It costs one extra copy of the sonar stream on disk, which is why it is a
    config flag rather than the default.

    **Frame-aware on purpose.** Sonar bytes arrive in arbitrary chunks that
    split frames, so injecting an NMEA packet at a chunk boundary would cut a
    ping in half and corrupt the log. Frames are therefore reassembled here and
    only whole ones are written, with navigation inserted between them.
    """

    def __init__(self, handle) -> None:
        self._handle = handle
        self._parser = PingParser()
        self._pending: TrajectorySample | None = None
        self._last_emitted_utc = 0
        self.stats = MergeStats()

    def set_position(self, sample: TrajectorySample) -> None:
        """Record the newest fix, to be written before the next ping."""
        self._pending = sample

    def feed_sonar(self, data: bytes) -> None:
        for frame in self._parser.feed(data):
            message = self._parser.decode(frame)
            if isinstance(message, pp.PointSet):
                self.stats.pings += 1
                self._emit_navigation()
                if self.stats.first_utc_ms == 0:
                    self.stats.first_utc_ms = message.utc_msec
                self.stats.last_utc_ms = message.utc_msec

            self._handle.write(
                pp.encode_frame(
                    frame.message_id, frame.payload,
                    frame.src_device_id, frame.dst_device_id,
                )
            )
            self.stats.frames_copied += 1

    def _emit_navigation(self) -> None:
        sample = self._pending
        if sample is None or sample.utc_ms == self._last_emitted_utc:
            return
        self._last_emitted_utc = sample.utc_ms

        self._handle.write(pp.encode_nmea_wrapper(gga_sentence(sample)))
        self.stats.gga_written += 1
        if sample.heading_valid and sample.heading_deg is not None:
            self._handle.write(pp.encode_nmea_wrapper(hdt_sentence(sample.heading_deg)))
            self.stats.hdt_written += 1
        else:
            self.stats.heading_invalid_samples += 1


def merge_mission(
    mission_dir: Path | str,
    output: Path | str | None = None,
    max_gap_ms: int = 2000,
) -> tuple[Path, MergeStats]:
    """Merge a whole mission directory. Returns ``(path, stats)``."""
    mission = Path(mission_dir)
    sonar = mission / SONAR_RAW_NAME
    trajectory = mission / TRAJECTORY_NAME
    if not sonar.is_file():
        raise FileNotFoundError(f"{sonar} not found")
    if not trajectory.is_file():
        raise FileNotFoundError(f"{trajectory} not found")

    target = Path(output) if output else mission / f"{mission.name}.svlog"
    return target, merge_to_svlog(sonar, trajectory, target, max_gap_ms)


def describe(stats: MergeStats) -> str:
    """A line an operator can act on."""
    if stats.pings == 0:
        return "No pings found in the sonar stream — nothing to merge."

    parts = [
        f"{stats.pings} pings, {stats.gga_written} positions, "
        f"{stats.hdt_written} headings, {stats.bytes_written / 1024**2:.1f} MB"
    ]
    if stats.unplaceable_pings:
        share = stats.unplaceable_pings / stats.pings * 100
        parts.append(
            f"WARNING: {stats.unplaceable_pings} pings ({share:.0f}%) fall outside the "
            "trajectory and have no position. They are in the log but will not be "
            "georeferenced."
        )
    if stats.heading_invalid_samples:
        parts.append(
            f"WARNING: {stats.heading_invalid_samples} pings were recorded while the "
            "heading was invalid. No heading was written for them rather than a "
            "heading we did not trust."
        )
    return " ".join(parts)
