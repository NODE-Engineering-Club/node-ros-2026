"""Synthetic ``sonar_raw.bin`` generator.

Produces exactly what ``mission_recorder`` writes to disk during a real mission:
the unmodified Ping Protocol byte stream, as it arrived. That makes it the
fixture for two different things at once —

* the parser can be developed and regression-tested with no sonar present;
* the post-mission fusion path (raw stream + ``trajectory.jsonl``, paired on
  ``utc_msec``) can be rehearsed before anyone flies to Namibia.

The stream interleaves the three inbound message types in the order the real
device sends them: a point set, then its end-of-ping metadata, with attitude
reports arriving asynchronously in between.

Import direction note: ``asket_sim`` depends on ``omniscan_bridge`` for the
codec, not the other way round. The encoder and the decoder must share one
layout table or they will drift apart, and a simulator that agrees with a buggy
parser is worse than no simulator at all.
"""

from __future__ import annotations

from dataclasses import dataclass

from omniscan_bridge.core import ping_protocol as pp

from .sonar import PingSet as SimPingSet
from .world import SimWorld


def encode_sim_ping(ping: SimPingSet) -> bytes:
    """Convert a simulator ping into an on-the-wire ``OS3D_POINT_SET`` frame."""
    return pp.encode_point_set(
        pp.PointSet(
            ping_number=ping.ping_number,
            speed_of_sound=ping.speed_of_sound_ms,
            utc_msec=ping.utc_ms,
            points=[
                pp.Point(p.angle_rad, p.tof_s, p.power, p.pt_type) for p in ping.points
            ],
        )
    )


@dataclass
class RawStreamStats:
    frames: int = 0
    pings: int = 0
    bytes_written: int = 0
    first_utc_ms: int = 0
    last_utc_ms: int = 0


def generate_raw_stream(
    world: SimWorld,
    duration_s: float,
    dt: float = 0.05,
    attitude_every_n_pings: int = 2,
) -> tuple[bytes, RawStreamStats]:
    """Run ``world`` forward and return the sonar bytes it would have produced."""
    out = bytearray()
    stats = RawStreamStats()
    steps = int(duration_s / dt)

    for _ in range(steps):
        world.step(dt)
        ping = world.take_ping()
        if ping is None:
            continue

        out += encode_sim_ping(ping)
        stats.frames += 1
        stats.pings += 1
        if stats.first_utc_ms == 0:
            stats.first_utc_ms = ping.utc_ms
        stats.last_utc_ms = ping.utc_ms

        valid = sum(1 for p in ping.points if p.pt_type != 0)
        out += pp.encode_end_ping_info(
            pp.EndPingInfo(
                ping_number=ping.ping_number,
                num_results=valid,
                ping_duration_s=1.0 / max(0.1, world.cfg.sonar.ping_rate_hz),
            )
        )
        stats.frames += 1

        if attitude_every_n_pings and stats.pings % attitude_every_n_pings == 0:
            snap = world.snapshot()
            out += pp.encode_attitude_report(
                pp.AttitudeReport(snap.sonar_pitch_deg, snap.sonar_roll_deg)
            )
            stats.frames += 1

    stats.bytes_written = len(out)
    return bytes(out), stats


def corrupt_stream(
    data: bytes,
    drop_ranges: list[tuple[int, int]] | None = None,
    flip_bytes: list[int] | None = None,
) -> bytes:
    """Damage a stream the way a real link does, for parser robustness tests.

    ``drop_ranges`` excises byte ranges (a dropped TCP segment; the parser must
    resynchronise). ``flip_bytes`` corrupts single bytes (the checksum must
    catch it).
    """
    buf = bytearray(data)
    for index in flip_bytes or []:
        if 0 <= index < len(buf):
            buf[index] ^= 0xFF
    for start, end in sorted(drop_ranges or [], reverse=True):
        del buf[start:end]
    return bytes(buf)
