"""CLI: what can a mission actually be exported to, and why not?

    python3 -m mission_recorder.check_export_paths

Run this on the Jetson after plugging a drive in. It answers the question that
otherwise costs an hour on a beach: *"the export button offers nothing — is the
drive broken, is the software broken, or did nobody install the mount unit?"*

It tests the **mount**, not the directory. A headless Jetson has no desktop
session, so nothing auto-mounts a USB stick; ``/media/usb`` left behind by a
previous mount is an empty folder on the Jetson's own disk, and copying a
mission there fills the disk the missions already live on.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from mission_recorder.core.export import (
    DEFAULT_FAST_PATH_GLOBS,
    NO_FAST_PATH_MESSAGE,
    inspect_destinations,
)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--missions-root", type=Path, default=Path("/data/missions"))
    ap.add_argument("--glob", action="append", dest="globs", default=None,
                    help="extra path pattern to consider (repeatable)")
    ap.add_argument("--allow-non-mount", action="store_true",
                    help="accept plain directories too — for testing only")
    args = ap.parse_args(argv)

    globs = list(DEFAULT_FAST_PATH_GLOBS) + (args.globs or [])
    accepted, rejected = inspect_destinations(
        globs=globs,
        exclude_roots=[str(args.missions_root), "/"],
        require_mount=not args.allow_non_mount,
    )

    print(f"Missions root: {args.missions_root}")
    print(f"Searched: {', '.join(globs)}\n")

    if accepted:
        print(f"Usable export destinations ({len(accepted)}):")
        for destination in accepted:
            print(
                f"  {destination.path}"
                f"  —  {destination.free_bytes / 1024**3:.1f} GB free"
                f" of {destination.total_bytes / 1024**3:.1f} GB"
            )
            hours = destination.free_bytes / (3.5 * 1024**3)
            print(f"      about {hours:.1f} hours of survey at 3.5 GB/h")
    else:
        print("No usable export destinations.")
        print(f"  {NO_FAST_PATH_MESSAGE}")

    if rejected:
        print(f"\nRejected ({len(rejected)}):")
        for rejection in rejected:
            print(f"  {rejection.path}\n      {rejection.reason}")

    if not accepted and not rejected:
        print(
            "\nNothing was even a candidate. On a headless Jetson nothing "
            "auto-mounts a USB drive — install the udev rule or the mount unit "
            "from deploy/, then re-run. See docs/SETUP.md."
        )

    return 0 if accepted else 1


if __name__ == "__main__":
    sys.exit(main())
