"""Export: blocked while recording, verified after copying, explained when not
available."""

import shutil
from pathlib import Path

import pytest
from mission_recorder.core.export import (
    NO_FAST_PATH_MESSAGE,
    Destination,
    delete_mission,
    detect_destinations,
    export_mission,
)
from mission_recorder.core.mission import (
    SONAR_RAW_NAME,
    MissionRecorder,
    RecorderConfig,
)

UTC = 1_700_000_000_000


@pytest.fixture
def mission(tmp_path):
    recorder = MissionRecorder(RecorderConfig(missions_root=tmp_path / "missions"))
    recorder.start("survey", UTC)
    recorder.write_sonar(b"sonar bytes" * 100)
    recorder.write_trajectory(
        {"utc_ms": UTC, "lat": -22.9, "lon": 14.5, "heading_valid": True}
    )
    recorder.stop(UTC + 1000)
    return Path(recorder.status.mission_dir)


@pytest.fixture
def usb(tmp_path):
    drive = tmp_path / "media" / "ASKET_USB"
    drive.mkdir(parents=True)
    return drive


def allowed(path):
    usage = shutil.disk_usage(str(path))
    return [Destination(str(Path(path).resolve()), "usb", "usb", usage.free, usage.total)]


def test_export_is_blocked_while_recording(mission, usb):
    """Copying gigabytes competes with the recorder for disk. Losing survey
    data to save time on a transfer is a bad trade."""
    result = export_mission(mission, usb, recording=True)
    assert not result.success
    assert "blocked while a mission is recording" in result.message
    assert not any(usb.iterdir())


def test_a_good_export_is_verified(mission, usb):
    result = export_mission(mission, usb, recording=False, allowed_destinations=allowed(usb))
    assert result.success
    assert result.checksum_status == "verified"
    assert "verified" in result.message
    assert (usb / mission.name / SONAR_RAW_NAME).is_file()


def test_a_truncated_copy_is_caught_and_the_operator_told_not_to_delete(mission, usb):
    """A copy that silently truncated is worse than no copy, because the
    original may then be deleted."""
    result = export_mission(mission, usb, recording=False, allowed_destinations=allowed(usb))
    assert result.success

    # Damage the copy and re-verify it the way a second export would.
    (usb / mission.name / SONAR_RAW_NAME).write_bytes(b"truncated")
    shutil.rmtree(usb / mission.name)

    # Now damage the SOURCE and export again: the copy inherits the damage and
    # fails its own checksum.
    (mission / SONAR_RAW_NAME).write_bytes(b"truncated")
    result = export_mission(mission, usb, recording=False, allowed_destinations=allowed(usb))
    assert not result.success
    assert result.checksum_status == "mismatch"
    assert "Do not delete the original" in result.message


def test_an_arbitrary_destination_is_refused_with_advice(mission, usb, tmp_path):
    """A disabled button with no explanation is the thing that wastes an hour
    on a beach."""
    elsewhere = tmp_path / "somewhere_else"
    elsewhere.mkdir()
    result = export_mission(
        mission, elsewhere, recording=False, allowed_destinations=allowed(usb)
    )
    assert not result.success
    assert "not a detected fast path" in result.message
    assert "USB drive or an Ethernet cable" in result.message


def test_the_no_fast_path_message_says_what_to_plug_in():
    assert "USB drive" in NO_FAST_PATH_MESSAGE
    assert "Ethernet" in NO_FAST_PATH_MESSAGE
    assert "wireless" in NO_FAST_PATH_MESSAGE


def test_exporting_twice_refuses_rather_than_overwriting(mission, usb):
    export_mission(mission, usb, recording=False, allowed_destinations=allowed(usb))
    again = export_mission(mission, usb, recording=False, allowed_destinations=allowed(usb))
    assert not again.success
    assert "already exists" in again.message


def test_export_refuses_when_the_destination_is_too_small(mission, usb, monkeypatch):
    tiny = type("Usage", (), {"free": 10, "total": 1000, "used": 990})()
    monkeypatch.setattr(shutil, "disk_usage", lambda _p: tiny)
    result = export_mission(mission, usb, recording=False)
    assert not result.success
    assert "Not enough space" in result.message


def test_the_jetsons_own_disk_is_never_offered_as_a_destination(tmp_path):
    """Offering the eMMC as a 'USB drive' would copy the mission onto the disk
    it already lives on, fill it, and stop the next recording."""
    media = tmp_path / "media"
    (media / "usb1").mkdir(parents=True)
    (media / "internal").mkdir(parents=True)

    found = detect_destinations(
        globs=[str(media / "*")],
        exclude_roots=[str(media / "internal")],
    )
    assert [d.label for d in found] == ["usb1"]


def test_no_destinations_when_nothing_is_plugged_in(tmp_path):
    assert detect_destinations(globs=[str(tmp_path / "nothing" / "*")]) == []


def test_deleting_the_mission_being_recorded_is_refused(mission):
    """The one mistake that cannot be undone by re-running anything."""
    ok, message = delete_mission(mission, recording_dir=str(mission))
    assert not ok
    assert "being recorded right now" in message
    assert mission.is_dir()


def test_deleting_a_finished_mission_works(mission):
    ok, message = delete_mission(mission, recording_dir=None)
    assert ok
    assert not mission.exists()
