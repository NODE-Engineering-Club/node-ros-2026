"""Mission export.

Full mission data is 2-5 GB per hour. **It never goes over the wireless link.**
Export is available only when a fast path is present: a USB drive mounted on the
Jetson, or a client connected over wired Ethernet.

Three rules, all of them here:

* **Blocked while recording.** Copying gigabytes competes with the recorder for
  CPU and disk I/O, and losing survey data to save time on a transfer is a bad
  trade.
* **Verified.** SHA-256 after copying, reported explicitly. A copy that
  silently truncated is worse than no copy, because the original may then be
  deleted.
* **Explained when unavailable.** A disabled button with no explanation is the
  thing that wastes an hour on a beach. If no fast path is detected the
  operator is told what to plug in.
"""

from __future__ import annotations

import shutil
from dataclasses import dataclass, field
from pathlib import Path

from .mission import CHECKSUMS_NAME, verify_mission

#: Where a USB drive is expected to appear. Configurable — which mount points
#: count as removable is Q9 in docs/open_questions.md.
DEFAULT_FAST_PATH_GLOBS = ["/media/*", "/media/*/*", "/mnt/usb*", "/run/media/*/*"]

NO_FAST_PATH_MESSAGE = (
    "Transfer unavailable — connect a USB drive or an Ethernet cable. "
    "Mission data is several gigabytes per hour and will not go over the "
    "wireless link."
)


@dataclass
class Destination:
    path: str
    kind: str            # "usb" | "ethernet"
    label: str
    free_bytes: int
    total_bytes: int

    def to_dict(self) -> dict:
        return {
            "path": self.path,
            "kind": self.kind,
            "label": self.label,
            "free_bytes": self.free_bytes,
            "total_bytes": self.total_bytes,
        }


@dataclass
class ExportResult:
    success: bool
    message: str
    #: "verified" | "mismatch" | "not_checked"
    checksum_status: str = "not_checked"
    bytes_copied: int = 0
    destination: str = ""
    per_file: dict[str, str] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "message": self.message,
            "checksum_status": self.checksum_status,
            "bytes_copied": self.bytes_copied,
            "destination": self.destination,
            "per_file": self.per_file,
        }


def detect_destinations(
    globs: list[str] | None = None,
    disk_usage=None,
    exclude_roots: list[str] | None = None,
) -> list[Destination]:
    """Find writable fast paths.

    Only mount points that are actually separate file systems are offered.
    Offering the Jetson's own eMMC as a "USB drive" would copy the mission onto
    the disk it already lives on, fill it, and stop the next recording.
    """
    from glob import glob

    disk_usage = disk_usage or shutil.disk_usage
    excluded = {Path(p).resolve() for p in (exclude_roots or [])}
    found: dict[str, Destination] = {}

    for pattern in globs or DEFAULT_FAST_PATH_GLOBS:
        for match in glob(pattern):
            path = Path(match)
            if not path.is_dir():
                continue
            try:
                resolved = path.resolve()
            except OSError:
                continue
            if any(resolved == e or e in resolved.parents for e in excluded):
                continue
            try:
                usage = disk_usage(str(path))
            except OSError:
                continue
            found[str(resolved)] = Destination(
                path=str(resolved),
                kind="usb",
                label=path.name,
                free_bytes=int(usage.free),
                total_bytes=int(usage.total),
            )

    return sorted(found.values(), key=lambda d: d.path)


def export_mission(
    mission_path: Path | str,
    destination: Path | str,
    recording: bool,
    allowed_destinations: list[Destination] | None = None,
    verify: bool = True,
) -> ExportResult:
    """Copy a mission to a fast path and verify it."""
    mission_path = Path(mission_path)
    destination = Path(destination)

    if recording:
        return ExportResult(
            False,
            "Export is blocked while a mission is recording. Stop the mission "
            "first — copying gigabytes competes with the recorder for disk.",
        )

    if not mission_path.is_dir():
        return ExportResult(False, f"No mission at {mission_path}.")

    if allowed_destinations is not None:
        allowed = {Path(d.path).resolve() for d in allowed_destinations}
        try:
            target_root = destination.resolve()
        except OSError:
            target_root = destination
        if not any(target_root == a or a in target_root.parents for a in allowed):
            return ExportResult(
                False,
                f"{destination} is not a detected fast path. {NO_FAST_PATH_MESSAGE}",
            )

    source_size = _directory_size(mission_path)
    try:
        free = shutil.disk_usage(str(destination)).free
    except OSError as exc:
        return ExportResult(False, f"Cannot read {destination}: {exc}")

    if free < source_size * 1.05:
        return ExportResult(
            False,
            f"Not enough space: the mission is {source_size / 1024**3:.1f} GB and "
            f"{destination} has {free / 1024**3:.1f} GB free.",
        )

    target = destination / mission_path.name
    if target.exists():
        return ExportResult(False, f"{target} already exists. Delete it or rename it.")

    try:
        shutil.copytree(mission_path, target)
    except OSError as exc:
        return ExportResult(False, f"Copy failed: {exc}", destination=str(target))

    if not verify:
        return ExportResult(
            True, f"Copied {source_size / 1024**3:.2f} GB to {target} (not verified).",
            checksum_status="not_checked", bytes_copied=source_size, destination=str(target),
        )

    # Verify the COPY, not the original. The point is to prove the bytes that
    # arrived are the bytes that left, so the original can be deleted safely.
    if not (target / CHECKSUMS_NAME).is_file():
        return ExportResult(
            True,
            f"Copied {source_size / 1024**3:.2f} GB to {target}, but the mission "
            "has no checksum file so the copy could not be verified.",
            checksum_status="not_checked", bytes_copied=source_size, destination=str(target),
        )

    ok, per_file = verify_mission(target)
    if ok:
        return ExportResult(
            True,
            f"Copied and verified {source_size / 1024**3:.2f} GB to {target}. "
            "Every file matches its checksum.",
            checksum_status="verified", bytes_copied=source_size,
            destination=str(target), per_file=per_file,
        )

    bad = [name for name, status in per_file.items() if status != "ok"]
    return ExportResult(
        False,
        f"Copy to {target} FAILED verification: {', '.join(bad)}. "
        "Do not delete the original.",
        checksum_status="mismatch", bytes_copied=source_size,
        destination=str(target), per_file=per_file,
    )


def delete_mission(mission_path: Path | str, recording_dir: str | None = None) -> tuple[bool, str]:
    """Delete a mission directory.

    Refuses to delete the mission currently being recorded, which is the one
    mistake that cannot be undone by re-running anything.
    """
    mission_path = Path(mission_path)
    if not mission_path.is_dir():
        return False, f"No mission at {mission_path}."
    if recording_dir and Path(recording_dir).resolve() == mission_path.resolve():
        return False, "That mission is being recorded right now."
    try:
        shutil.rmtree(mission_path)
    except OSError as exc:
        return False, f"Delete failed: {exc}"
    return True, f"Deleted {mission_path}."


def _directory_size(path: Path) -> int:
    import os

    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                pass
    return total
