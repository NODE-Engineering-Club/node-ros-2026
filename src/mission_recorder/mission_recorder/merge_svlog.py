"""CLI: merge a recorded mission into a SonarView-readable ``.svlog``.

    python3 -m mission_recorder.merge_svlog /data/missions/namibia_2026...
    python3 -m mission_recorder.merge_svlog --all /data/missions

This is the **primary** path for getting a survey into SonarView. It runs after
the mission, on a laptop or on the Jetson, and needs nothing but the mission
directory — no ROS, no vessel, no sonar.

Why it is needed at all: SonarView cannot import an external trajectory.
Position and heading have to be inside the log as NMEA_WRAPPER packets, and a
log without them cannot be georeferenced or exported. Recording the two streams
separately and merging afterwards keeps the onboard recorder simple and keeps
SonarView off the boat, which is the whole point of Option B.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from mission_recorder.core.mission import list_missions
from mission_recorder.core.svlog import describe, merge_mission


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("path", type=Path, help="a mission directory, or a missions root with --all")
    ap.add_argument("--all", action="store_true", help="merge every mission under PATH")
    ap.add_argument("--out", type=Path, default=None, help="output file (single mission only)")
    ap.add_argument(
        "--max-gap-ms",
        type=int,
        default=2000,
        help=(
            "Longest trajectory gap to interpolate across. Pings inside a "
            "longer gap are written without a position rather than given an "
            "extrapolated one, because a confident position for seabed nobody "
            "knows the location of looks exactly like good data."
        ),
    )
    ap.add_argument("--force", action="store_true", help="overwrite an existing .svlog")
    args = ap.parse_args(argv)

    missions = (
        [Path(m.path) for m in list_missions(args.path)] if args.all else [args.path]
    )
    if not missions:
        print(f"No missions found under {args.path}", file=sys.stderr)
        return 1

    failures = 0
    for mission in missions:
        target = args.out if (args.out and not args.all) else mission / f"{mission.name}.svlog"
        if target.exists() and not args.force:
            print(f"{mission.name}: {target.name} already exists (use --force)")
            continue
        try:
            path, stats = merge_mission(mission, target, max_gap_ms=args.max_gap_ms)
        except (FileNotFoundError, OSError) as exc:
            print(f"{mission.name}: FAILED — {exc}", file=sys.stderr)
            failures += 1
            continue
        print(f"{mission.name}: {path}")
        print(f"  {describe(stats)}")
        # A merge that placed nothing is a failure even though it wrote a file.
        if stats.pings and stats.gga_written == 0:
            print(
                "  ERROR: no positions were written. The log will not be "
                "georeferenceable. Check that trajectory.jsonl covers the same "
                "time span as the sonar stream.",
                file=sys.stderr,
            )
            failures += 1

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
