"""Mission files: layout, interruption, disk exhaustion, integrity."""

import json
from pathlib import Path

import pytest
from mission_recorder.core.mission import (
    CHECKSUMS_NAME,
    EVENTS_NAME,
    MANIFEST_NAME,
    SONAR_RAW_NAME,
    STATE_ERROR,
    STATE_IDLE,
    TRAJECTORY_FIELDS,
    TRAJECTORY_NAME,
    MissionRecorder,
    RecorderConfig,
    list_missions,
    mission_dir_name,
    sanitise_name,
    verify_mission,
)

UTC = 1_700_000_000_000


class FakeDisk:
    """A disk we can run out of on demand."""

    def __init__(self, free=100 * 1024**3, total=512 * 1024**3):
        self.free = free
        self.total = total

    def __call__(self, _path):
        return type("Usage", (), {"free": self.free, "total": self.total, "used": 0})()


@pytest.fixture
def recorder(tmp_path):
    disk = FakeDisk()
    rec = MissionRecorder(
        RecorderConfig(missions_root=tmp_path, min_free_bytes=1024**3), disk_usage=disk
    )
    rec.fake_disk = disk
    return rec


def trajectory_record(utc=UTC, **overrides):
    record = {
        "utc_ms": utc, "lat": -22.9576, "lon": 14.5053, "alt": 0.0,
        "heading_deg": 20.0, "heading_source": "magnetometer", "heading_valid": True,
        "cog_deg": 18.0, "sog_ms": 1.5, "roll_deg": 1.0, "pitch_deg": 0.2,
        "gnss_fix_type": 3, "num_sats": 14, "hdop": 0.8,
    }
    record.update(overrides)
    return record


# -- naming ---------------------------------------------------------------


def test_operator_typed_names_cannot_escape_the_missions_directory():
    """Typed on a laptop on a beach, sometimes with a glove on."""
    assert sanitise_name("../../etc/passwd") == "etc_passwd"
    assert sanitise_name("walvis bay line 3") == "walvis_bay_line_3"
    assert sanitise_name("") == "mission"
    assert "/" not in sanitise_name("a/b")


def test_directory_name_carries_a_utc_timestamp():
    assert mission_dir_name("survey", UTC) == "survey_20231114T221320Z"


# -- the happy path -------------------------------------------------------


def test_a_mission_produces_the_documented_layout(recorder, tmp_path):
    recorder.start("namibia", UTC, config_snapshot={"sonar": {"range_m": 30.0}})
    recorder.write_sonar(b"BR" + b"\x00" * 100)
    recorder.write_trajectory(trajectory_record())
    recorder.write_diagnostics({"utc_ms": UTC, "clock_offset_ms": 12})
    recorder.stop(UTC + 5000)

    directory = Path(recorder.status.mission_dir)
    for name in (MANIFEST_NAME, SONAR_RAW_NAME, TRAJECTORY_NAME, EVENTS_NAME, CHECKSUMS_NAME):
        assert (directory / name).is_file(), f"{name} missing"

    manifest = json.loads((directory / MANIFEST_NAME).read_text())
    assert manifest["complete"] is True
    assert manifest["duration_s"] == 5.0
    assert manifest["config"]["sonar"]["range_m"] == 30.0
    assert manifest["trajectory_fields"] == TRAJECTORY_FIELDS


def test_the_raw_sonar_stream_is_stored_byte_for_byte(recorder):
    """Anything we reinterpret here is something a post-mission tool cannot
    reinterpret differently later."""
    payload = bytes(range(256)) * 4
    recorder.start("raw", UTC)
    recorder.write_sonar(payload)
    recorder.stop(UTC + 1000)
    assert (Path(recorder.status.mission_dir) / SONAR_RAW_NAME).read_bytes() == payload


def test_trajectory_records_heading_provenance_per_sample(recorder):
    """The trajectory must record not just the heading but how trustworthy it
    was at that instant, so a compromised window can be identified afterwards
    rather than guessed at."""
    recorder.start("heading", UTC)
    recorder.write_trajectory(trajectory_record(utc=UTC))
    recorder.write_trajectory(
        trajectory_record(utc=UTC + 100, heading_valid=False, heading_source="none")
    )
    recorder.stop(UTC + 1000)

    lines = (Path(recorder.status.mission_dir) / TRAJECTORY_NAME).read_text().splitlines()
    records = [json.loads(line) for line in lines]
    assert [r["heading_valid"] for r in records] == [True, False]
    assert records[1]["heading_source"] == "none"
    # Fixed field order, every field present, so a reader never has to guess
    # whether a missing key means "not measured" or "not recorded".
    assert list(records[0]) == TRAJECTORY_FIELDS


def test_nan_never_reaches_the_file(recorder):
    """JSON has no NaN. Some parsers accept it and some reject it, so a file
    containing it means two readers disagree about the same mission."""
    recorder.start("nan", UTC)
    recorder.write_trajectory(trajectory_record(hdop=float("nan"), sog_ms=float("inf")))
    recorder.stop(UTC + 1000)

    text = (Path(recorder.status.mission_dir) / TRAJECTORY_NAME).read_text()
    assert "NaN" not in text and "Infinity" not in text
    record = json.loads(text)
    assert record["hdop"] is None


# -- interruption ---------------------------------------------------------


def test_an_interrupted_mission_still_describes_itself(recorder):
    """A mission that ends because the battery died must leave a readable,
    self-describing directory."""
    recorder.start("interrupted", UTC)
    recorder.write_trajectory(trajectory_record())
    # No stop() — simulate the power being cut.

    directory = Path(recorder.status.mission_dir)
    manifest = json.loads((directory / MANIFEST_NAME).read_text())
    assert manifest["name"] == "interrupted"
    assert manifest["started_utc_ms"] == UTC
    assert manifest["complete"] is False
    assert manifest["stopped_utc_ms"] is None
    assert "utc_ms" in manifest["notes"] or "utc_ms" in str(manifest)


def test_an_interrupted_mission_is_still_listed(recorder, tmp_path):
    """Hiding it would hide exactly the mission most likely to need attention."""
    recorder.start("interrupted", UTC)
    recorder.write_trajectory(trajectory_record())

    missions = list_missions(tmp_path)
    assert len(missions) == 1
    assert missions[0].complete is False


def test_a_mission_with_an_unreadable_manifest_is_still_listed(tmp_path):
    directory = tmp_path / "broken_20231114T221320Z"
    directory.mkdir()
    (directory / MANIFEST_NAME).write_text("{ this is not json")

    missions = list_missions(tmp_path)
    assert len(missions) == 1
    assert missions[0].complete is False
    assert missions[0].name == "broken_20231114T221320Z"


# -- disk -----------------------------------------------------------------


def test_recording_stops_cleanly_before_the_disk_fills(recorder):
    """A full file system takes the whole Jetson down, not just the recording.
    A mission that stops with a valid manifest is salvageable."""
    recorder.start("diskfull", UTC)
    recorder.write_trajectory(trajectory_record())
    directory = Path(recorder.status.mission_dir)

    recorder.fake_disk.free = 100 * 1024**2      # below min_free_bytes
    recorder.tick(now_s=1.0, utc_ms=UTC + 1000)

    assert recorder.status.state == STATE_ERROR
    assert "MB left" in recorder.status.error_message
    manifest = json.loads((directory / MANIFEST_NAME).read_text())
    assert manifest["complete"] is True, "the mission must still be readable"

    events = [json.loads(line) for line in (directory / EVENTS_NAME).read_text().splitlines()]
    assert any(e["kind"] == "disk_full" for e in events)


def test_a_mission_will_not_start_without_room(recorder):
    recorder.fake_disk.free = 100 * 1024**2
    status = recorder.start("nospace", UTC)
    assert status.state == STATE_ERROR
    assert "not enough" in status.error_message


def test_remaining_recording_time_is_estimated_from_the_actual_write_rate(recorder):
    recorder.start("estimate", UTC)
    recorder.fake_disk.free = 10 * 1024**3 + 1024**3     # 10 GB usable
    for i in range(10):
        recorder.write_sonar(b"x" * 1_000_000)           # 1 MB per tick
        recorder.tick(now_s=float(i), utc_ms=UTC + i * 1000)

    remaining = recorder.status.estimated_remaining_s
    assert 5_000 < remaining < 20_000                    # ~10 GB at ~1 MB/s


# -- integrity ------------------------------------------------------------


def test_checksums_are_written_and_verify(recorder):
    recorder.start("verify", UTC)
    recorder.write_sonar(b"sonar bytes")
    recorder.write_trajectory(trajectory_record())
    recorder.stop(UTC + 1000)

    ok, per_file = verify_mission(recorder.status.mission_dir)
    assert ok
    assert per_file[SONAR_RAW_NAME] == "ok"


def test_a_corrupted_file_fails_verification(recorder):
    recorder.start("corrupt", UTC)
    recorder.write_sonar(b"sonar bytes")
    recorder.stop(UTC + 1000)

    path = Path(recorder.status.mission_dir) / SONAR_RAW_NAME
    path.write_bytes(b"different!!")

    ok, per_file = verify_mission(recorder.status.mission_dir)
    assert not ok
    assert per_file[SONAR_RAW_NAME] == "mismatch"


def test_a_mission_without_checksums_is_not_checked_rather_than_passing(tmp_path):
    directory = tmp_path / "unchecked"
    directory.mkdir()
    ok, per_file = verify_mission(directory)
    assert not ok
    assert per_file[CHECKSUMS_NAME] == "missing"


# -- state ----------------------------------------------------------------


def test_a_second_mission_cannot_start_over_the_first(recorder):
    recorder.start("first", UTC)
    status = recorder.start("second", UTC + 1000)
    assert "already recording" in status.error_message
    assert status.name == "first"


def test_writes_are_ignored_when_not_recording(recorder):
    recorder.write_sonar(b"nowhere")
    recorder.write_trajectory(trajectory_record())
    assert recorder.status.state == STATE_IDLE
    assert recorder.status.bytes_written == 0


def test_missions_are_listed_newest_first(recorder, tmp_path):
    for i, name in enumerate(["oldest", "middle", "newest"]):
        recorder.start(name, UTC + i * 86_400_000)
        recorder.stop(UTC + i * 86_400_000 + 1000)
    assert [m.name for m in list_missions(tmp_path)] == ["newest", "middle", "oldest"]
