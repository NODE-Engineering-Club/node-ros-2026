"""CLI: write a synthetic ``sonar_raw.bin``.

    python3 -m asket_sim.make_sonar_raw --duration 60 --out /tmp/sonar_raw.bin

Deliberately runnable without ROS: the point of this file is to give the parser
something to chew on before any hardware exists.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from asket_sim.core.raw_stream import corrupt_stream, generate_raw_stream
from asket_sim.core.world import SimWorld, WorldConfig


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, default=Path("sonar_raw.bin"))
    ap.add_argument("--duration", type=float, default=60.0, help="seconds of survey")
    ap.add_argument("--dt", type=float, default=0.05)
    ap.add_argument("--ping-rate", type=float, default=5.0)
    ap.add_argument("--points-per-ping", type=int, default=256)
    ap.add_argument("--range", type=float, default=30.0, dest="range_m")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument(
        "--corrupt",
        action="store_true",
        help="excise a byte range and flip a byte, to exercise parser resync",
    )
    args = ap.parse_args(argv)

    cfg = WorldConfig(seed=args.seed)
    cfg.sonar.ping_rate_hz = args.ping_rate
    cfg.sonar.points_per_ping = args.points_per_ping
    cfg.sonar.range_setting_m = args.range_m

    world = SimWorld(cfg, start_utc_ms=1_700_000_000_000)
    data, stats = generate_raw_stream(world, args.duration, args.dt)

    if args.corrupt and len(data) > 4000:
        data = corrupt_stream(data, drop_ranges=[(1500, 1573)], flip_bytes=[3200])

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_bytes(data)

    print(
        f"wrote {args.out} — {len(data)} bytes, {stats.pings} pings, "
        f"{stats.frames} frames, utc {stats.first_utc_ms}..{stats.last_utc_ms}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
