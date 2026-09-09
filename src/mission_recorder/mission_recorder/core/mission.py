"""Mission files.

The core problem this module exists to solve: **the sonar knows nothing about
where it is.** It records angles and times. Position and heading come from the
vessel and must be recorded *alongside* the sonar stream so the two can be
merged afterwards.

The approach (Option B, decided with the team) is to record the raw sonar stream
and a timestamped trajectory as two separate files and pair them afterwards on
``utc_msec``. SonarView does not have to run during the mission.

That makes two things non-negotiable here:

* **Clock discipline.** Everything rests on the sonar's ``utc_msec`` being
  comparable with ours. The recorder logs the clock offset with every
  diagnostics snapshot so that, if the worst happens, the damage is at least
  visible in the recording rather than invisible in the data.
* **An interrupted mission must still be readable.** ``manifest.json`` is
  written at start *and* updated at stop. A mission that ends because the
  battery died leaves a directory that describes itself.

Layout, per the brief::

    /data/missions/<name>_<UTC timestamp>/
        manifest.json          metadata, versions, config snapshot
        sonar_raw.bin          raw Ping Protocol stream, unmodified
        trajectory.jsonl       one record per sample, ~10 Hz
        diagnostics.jsonl      health snapshots
        events.jsonl           mode changes, alarms, operator actions
        rosbag/                full rosbag2 (optional)
        checksums.sha256
"""

from __future__ import annotations

import hashlib
import json
import os
import re
import shutil
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

STATE_IDLE = "IDLE"
STATE_RECORDING = "RECORDING"
STATE_STOPPING = "STOPPING"
STATE_ERROR = "ERROR"

MANIFEST_NAME = "manifest.json"
SONAR_RAW_NAME = "sonar_raw.bin"
TRAJECTORY_NAME = "trajectory.jsonl"
DIAGNOSTICS_NAME = "diagnostics.jsonl"
EVENTS_NAME = "events.jsonl"
CHECKSUMS_NAME = "checksums.sha256"
LIVE_SVLOG_NAME = "live.svlog"
ROSBAG_DIR = "rosbag"

#: Files whose integrity is checked on export. The rosbag is excluded: it is
#: optional, it is a directory, and rosbag2 has its own metadata.
CHECKSUM_FILES = [SONAR_RAW_NAME, TRAJECTORY_NAME, DIAGNOSTICS_NAME, EVENTS_NAME]

#: Fields of one trajectory record. `heading_source` and `heading_valid` are
#: essential and not decoration: the trajectory must record not just the heading
#: but how trustworthy it was at that instant, so a window of compromised data
#: can be identified afterwards rather than guessed at.
TRAJECTORY_FIELDS = [
    "utc_ms", "lat", "lon", "alt", "heading_deg", "heading_source", "heading_valid",
    "cog_deg", "sog_ms", "roll_deg", "pitch_deg", "gnss_fix_type", "num_sats", "hdop",
]

_SAFE_NAME = re.compile(r"[^A-Za-z0-9_.-]+")


def sanitise_name(name: str) -> str:
    """Make an operator-typed name safe to use as a directory name.

    Names are typed on a laptop on a beach, sometimes with a glove on. Anything
    that could escape the missions directory or confuse a shell is replaced.
    """
    cleaned = _SAFE_NAME.sub("_", (name or "").strip()).strip("._")
    return cleaned[:64] or "mission"


def mission_dir_name(name: str, utc_ms: int) -> str:
    stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime(utc_ms / 1000.0))
    return f"{sanitise_name(name)}_{stamp}"


def sha256_file(path: Path, chunk_size: int = 1024 * 1024) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(chunk_size), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass
class MissionStatus:
    state: str = STATE_IDLE
    name: str = ""
    mission_dir: str = ""
    started_utc_ms: int = 0
    elapsed_s: float = 0.0
    bytes_written: int = 0
    disk_free_bytes: int = 0
    disk_total_bytes: int = 0
    estimated_remaining_s: float = 0.0
    sonar_bytes: int = 0
    trajectory_records: int = 0
    events: int = 0
    error_message: str = ""

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class RecorderConfig:
    missions_root: Path = Path("/data/missions")
    #: Stop recording rather than fill the disk completely. A truncated file
    #: system takes the whole Jetson down, not just the recording.
    min_free_bytes: int = 1 * 1024**3
    #: Warn the operator at this much remaining.
    low_free_bytes: int = 5 * 1024**3
    #: Flush to disk this often. A mission that ends with the power being cut
    #: should lose a second, not a minute.
    flush_interval_s: float = 1.0
    record_rosbag: bool = False
    #: Also write a SonarView-readable log as the mission runs, with navigation
    #: already interleaved. The post-mission merge is the primary path; this is
    #: belt and braces for the day the trajectory file is lost or nobody
    #: remembers to run it. Costs a second copy of the sonar stream on disk.
    write_live_svlog: bool = False


class MissionRecorder:
    """Writes one mission directory. ROS-free, so it can be tested properly."""

    def __init__(self, config: RecorderConfig | None = None, disk_usage=None) -> None:
        self.cfg = config or RecorderConfig()
        #: Injectable so tests and the simulator can drive the disk full.
        self._disk_usage = disk_usage or shutil.disk_usage

        self.status = MissionStatus()
        self._dir: Path | None = None
        self._files: dict[str, object] = {}
        self._last_flush = 0.0
        self._manifest: dict = {}
        self._rate_samples: list[tuple[float, int]] = []
        self._live_svlog = None

    # -- lifecycle --------------------------------------------------------

    def start(
        self,
        name: str,
        utc_ms: int,
        config_snapshot: dict | None = None,
        record_rosbag: bool | None = None,
    ) -> MissionStatus:
        if self.status.state == STATE_RECORDING:
            self.status.error_message = "a mission is already recording"
            return self.status

        root = Path(self.cfg.missions_root)
        try:
            root.mkdir(parents=True, exist_ok=True)
        except OSError as exc:
            return self._fail(f"cannot create {root}: {exc}")

        free = self.disk_free_bytes()
        if free <= self.cfg.min_free_bytes:
            return self._fail(
                f"only {free / 1024**3:.1f} GB free — not enough to start a mission"
            )

        directory = root / mission_dir_name(name, utc_ms)
        try:
            directory.mkdir(parents=True, exist_ok=False)
            if record_rosbag if record_rosbag is not None else self.cfg.record_rosbag:
                (directory / ROSBAG_DIR).mkdir(exist_ok=True)
        except OSError as exc:
            return self._fail(f"cannot create {directory}: {exc}")

        self._dir = directory
        try:
            self._files = {
                SONAR_RAW_NAME: (directory / SONAR_RAW_NAME).open("wb"),
                TRAJECTORY_NAME: (directory / TRAJECTORY_NAME).open("w", encoding="utf-8"),
                DIAGNOSTICS_NAME: (directory / DIAGNOSTICS_NAME).open("w", encoding="utf-8"),
                EVENTS_NAME: (directory / EVENTS_NAME).open("w", encoding="utf-8"),
            }
        except OSError as exc:
            return self._fail(f"cannot open mission files: {exc}")

        if self.cfg.write_live_svlog:
            from .svlog import LiveSvlogWriter

            try:
                handle = (directory / LIVE_SVLOG_NAME).open("wb")
            except OSError as exc:
                return self._fail(f"cannot open {LIVE_SVLOG_NAME}: {exc}")
            self._files[LIVE_SVLOG_NAME] = handle
            self._live_svlog = LiveSvlogWriter(handle)

        self._manifest = {
            "name": sanitise_name(name),
            "started_utc_ms": utc_ms,
            "stopped_utc_ms": None,
            "complete": False,
            "schema_version": 1,
            "trajectory_fields": TRAJECTORY_FIELDS,
            "config": config_snapshot or {},
            "files": {
                "sonar_raw": SONAR_RAW_NAME,
                "trajectory": TRAJECTORY_NAME,
                "diagnostics": DIAGNOSTICS_NAME,
                "events": EVENTS_NAME,
            },
            "notes": (
                "Pair sonar_raw.bin with trajectory.jsonl on utc_ms. The sonar "
                "records angle and time of flight only; position and heading "
                "come from this trajectory. heading_valid marks samples whose "
                "heading cannot be trusted."
            ),
        }
        # Written at start as well as at stop: an interrupted mission must still
        # leave a readable, self-describing directory.
        self._write_manifest()

        self.status = MissionStatus(
            state=STATE_RECORDING,
            name=self._manifest["name"],
            mission_dir=str(directory),
            started_utc_ms=utc_ms,
            disk_free_bytes=free,
            disk_total_bytes=self.disk_total_bytes(),
        )
        self.write_event(utc_ms, "mission_started", {"name": self._manifest["name"]})
        return self.status

    def stop(self, utc_ms: int) -> MissionStatus:
        if self.status.state != STATE_RECORDING:
            return self.status

        self.status.state = STATE_STOPPING
        self.write_event(utc_ms, "mission_stopped", {})

        for handle in self._files.values():
            try:
                handle.flush()
                os.fsync(handle.fileno())
                handle.close()
            except OSError:
                pass
        self._files.clear()
        live_stats = self._live_svlog.stats if self._live_svlog else None
        self._live_svlog = None

        self._manifest["stopped_utc_ms"] = utc_ms
        self._manifest["complete"] = True
        self._manifest["duration_s"] = (utc_ms - self.status.started_utc_ms) / 1000.0
        self._manifest["counts"] = {
            "sonar_bytes": self.status.sonar_bytes,
            "trajectory_records": self.status.trajectory_records,
            "events": self.status.events,
        }
        if live_stats is not None:
            self._manifest["live_svlog"] = live_stats.to_dict()
            self._manifest["files"]["live_svlog"] = LIVE_SVLOG_NAME
        self._write_manifest()
        self.write_checksums()

        self.status.state = STATE_IDLE
        finished, self._dir = self._dir, None
        self.status.mission_dir = str(finished) if finished else ""
        return self.status

    def _fail(self, message: str) -> MissionStatus:
        self.status.state = STATE_ERROR
        self.status.error_message = message
        return self.status

    # -- writing ----------------------------------------------------------

    @property
    def recording(self) -> bool:
        return self.status.state == STATE_RECORDING

    def write_sonar(self, data: bytes) -> None:
        """The raw Ping Protocol stream, byte for byte as it arrived.

        Unmodified on purpose: anything we reinterpret here is something a
        post-mission tool cannot reinterpret differently later.
        """
        if not self.recording or not data:
            return
        handle = self._files.get(SONAR_RAW_NAME)
        if handle is None:
            return
        try:
            handle.write(data)
        except OSError as exc:
            self._fail(f"sonar write failed: {exc}")
            return
        self.status.sonar_bytes += len(data)
        self.status.bytes_written += len(data)

        if self._live_svlog is not None:
            # Never let the optional file take down the mandatory one.
            try:
                self._live_svlog.feed_sonar(data)
            except OSError as exc:
                self._live_svlog = None
                self.write_event(0, "live_svlog_failed", {"error": str(exc)})

    def write_trajectory(self, record: dict) -> None:
        if not self.recording:
            return
        # Fixed field order, and every field present even when null: a
        # post-mission script should not have to guess whether a missing key
        # means "not measured" or "not recorded".
        ordered = {key: record.get(key) for key in TRAJECTORY_FIELDS}
        self._write_line(TRAJECTORY_NAME, ordered)
        self.status.trajectory_records += 1

        if self._live_svlog is not None and ordered.get("lat") is not None:
            from .svlog import TrajectorySample

            try:
                self._live_svlog.set_position(TrajectorySample.from_record(ordered))
            except (KeyError, TypeError, ValueError):
                pass

    def write_diagnostics(self, record: dict) -> None:
        if not self.recording:
            return
        self._write_line(DIAGNOSTICS_NAME, record)

    def write_event(self, utc_ms: int, kind: str, detail: dict | None = None) -> None:
        """Mode changes, alarms, operator actions.

        Written even during STOPPING, so the stop event itself is recorded.
        """
        if self.status.state not in (STATE_RECORDING, STATE_STOPPING):
            return
        self._write_line(
            EVENTS_NAME, {"utc_ms": utc_ms, "kind": kind, "detail": detail or {}}
        )
        self.status.events += 1

    def _write_line(self, filename: str, record: dict) -> None:
        handle = self._files.get(filename)
        if handle is None:
            return
        try:
            line = json.dumps(record, separators=(",", ":"), allow_nan=False) + "\n"
        except ValueError:
            # NaN or Infinity: not valid JSON, and a reader that accepts it and
            # one that rejects it would disagree about the same file.
            line = json.dumps(_scrub(record), separators=(",", ":")) + "\n"
        try:
            handle.write(line)
        except OSError as exc:
            self._fail(f"write to {filename} failed: {exc}")
            return
        self.status.bytes_written += len(line)

    def tick(self, now_s: float, utc_ms: int) -> MissionStatus:
        """Housekeeping: flush, refresh disk figures, stop before the disk fills."""
        if not self.recording:
            self.status.disk_free_bytes = self.disk_free_bytes()
            self.status.disk_total_bytes = self.disk_total_bytes()
            return self.status

        self.status.elapsed_s = (utc_ms - self.status.started_utc_ms) / 1000.0
        self.status.disk_free_bytes = self.disk_free_bytes()
        self.status.disk_total_bytes = self.disk_total_bytes()
        self.status.estimated_remaining_s = self._estimate_remaining_s(now_s)

        if now_s - self._last_flush >= self.cfg.flush_interval_s:
            self._last_flush = now_s
            for handle in self._files.values():
                try:
                    handle.flush()
                except OSError:
                    pass

        if self.status.disk_free_bytes <= self.cfg.min_free_bytes:
            # Stop cleanly rather than fill the disk. A full file system takes
            # the whole Jetson down, not just the recording, and a mission that
            # stops with a valid manifest is salvageable.
            self.write_event(
                utc_ms, "disk_full",
                {"free_bytes": self.status.disk_free_bytes, "action": "stopped recording"},
            )
            self.stop(utc_ms)
            self._fail(
                f"stopped: only {self.status.disk_free_bytes / 1024**2:.0f} MB left"
            )
        return self.status

    def _estimate_remaining_s(self, now_s: float) -> float:
        """Recording time left at the rate we are actually writing."""
        self._rate_samples.append((now_s, self.status.bytes_written))
        del self._rate_samples[: max(0, len(self._rate_samples) - 60)]
        if len(self._rate_samples) < 2:
            return float("inf")
        (t0, b0), (t1, b1) = self._rate_samples[0], self._rate_samples[-1]
        if t1 <= t0 or b1 <= b0:
            return float("inf")
        rate = (b1 - b0) / (t1 - t0)
        usable = max(0, self.status.disk_free_bytes - self.cfg.min_free_bytes)
        return usable / rate if rate > 0 else float("inf")

    # -- disk -------------------------------------------------------------

    def disk_free_bytes(self) -> int:
        try:
            return int(self._disk_usage(str(self.cfg.missions_root)).free)
        except OSError:
            return 0

    def disk_total_bytes(self) -> int:
        try:
            return int(self._disk_usage(str(self.cfg.missions_root)).total)
        except OSError:
            return 0

    # -- manifest and checksums -------------------------------------------

    def _write_manifest(self) -> None:
        if self._dir is None:
            return
        path = self._dir / MANIFEST_NAME
        try:
            # Write and rename, so an interrupted write cannot leave a manifest
            # that is neither the old one nor the new one.
            temporary = path.with_suffix(".json.tmp")
            temporary.write_text(json.dumps(self._manifest, indent=2), encoding="utf-8")
            temporary.replace(path)
        except OSError as exc:
            self._fail(f"cannot write manifest: {exc}")

    def write_checksums(self) -> dict[str, str]:
        if self._dir is None:
            return {}
        digests = {}
        for filename in CHECKSUM_FILES:
            path = self._dir / filename
            if path.is_file():
                digests[filename] = sha256_file(path)
        try:
            (self._dir / CHECKSUMS_NAME).write_text(
                "".join(f"{d}  {n}\n" for n, d in sorted(digests.items())),
                encoding="utf-8",
            )
        except OSError:
            pass
        return digests


def _scrub(value):
    """Replace NaN and Infinity with None, recursively. JSON has neither."""
    import math

    if isinstance(value, float):
        return None if (math.isnan(value) or math.isinf(value)) else value
    if isinstance(value, dict):
        return {k: _scrub(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_scrub(v) for v in value]
    return value


# -- reading missions back ------------------------------------------------


@dataclass
class MissionSummary:
    name: str
    path: str
    started_utc_ms: int
    stopped_utc_ms: int | None
    complete: bool
    size_bytes: int
    duration_s: float
    manifest: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)


def directory_size(path: Path) -> int:
    total = 0
    for root, _, files in os.walk(path):
        for name in files:
            try:
                total += (Path(root) / name).stat().st_size
            except OSError:
                pass
    return total


def list_missions(root: Path | str) -> list[MissionSummary]:
    """Every mission directory under ``root``, newest first.

    A directory with an unreadable or missing manifest is still listed, marked
    incomplete. Hiding it would hide exactly the mission most likely to need
    attention.
    """
    root = Path(root)
    if not root.is_dir():
        return []

    summaries = []
    for directory in sorted(p for p in root.iterdir() if p.is_dir()):
        manifest = {}
        manifest_path = directory / MANIFEST_NAME
        if manifest_path.is_file():
            try:
                manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                manifest = {}
        summaries.append(
            MissionSummary(
                name=manifest.get("name", directory.name),
                path=str(directory),
                started_utc_ms=int(manifest.get("started_utc_ms", 0)),
                stopped_utc_ms=manifest.get("stopped_utc_ms"),
                complete=bool(manifest.get("complete", False)),
                size_bytes=directory_size(directory),
                duration_s=float(manifest.get("duration_s", 0.0)),
                manifest=manifest,
            )
        )
    summaries.sort(key=lambda s: s.started_utc_ms, reverse=True)
    return summaries


def verify_mission(path: Path | str) -> tuple[bool, dict[str, str]]:
    """Re-hash a mission's files against its ``checksums.sha256``.

    Returns ``(ok, per-file status)``. A mission with no checksum file is
    reported as ``not_checked`` rather than as passing.
    """
    path = Path(path)
    checksum_path = path / CHECKSUMS_NAME
    if not checksum_path.is_file():
        return False, {"checksums.sha256": "missing"}

    results: dict[str, str] = {}
    ok = True
    for line in checksum_path.read_text(encoding="utf-8").splitlines():
        expected, _, filename = line.partition("  ")
        if not filename:
            continue
        target = path / filename
        if not target.is_file():
            results[filename] = "missing"
            ok = False
            continue
        actual = sha256_file(target)
        results[filename] = "ok" if actual == expected else "mismatch"
        ok = ok and results[filename] == "ok"
    return ok, results
