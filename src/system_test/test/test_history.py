"""Trends: gradual degradation shows up before outright failure."""

from system_test.core.checks import CheckResult, PASS, Report
from system_test.core.history import PreflightHistory


def report(run_utc_ms, value, status=PASS):
    return Report(
        go=True, summary="GO", run_utc_ms=run_utc_ms,
        items=[CheckResult("disk.speed", "Disk write speed", status,
                           f"{value} MB/s", "", value, "MB/s")],
    )


def test_runs_are_stored_and_read_back(tmp_path):
    history = PreflightHistory(tmp_path / "system_test.jsonl")
    for i in range(3):
        history.append(report(1000 + i, 100.0))
    assert len(history.runs()) == 3
    assert history.summary()["runs"] == 3


def test_a_check_drifting_the_wrong_way_is_flagged_while_it_still_passes():
    """A corroding connector does not fail on the day it fails. It spends a
    month getting worse while every pre-flight still says GO."""
    import tempfile
    from pathlib import Path

    with tempfile.TemporaryDirectory() as directory:
        history = PreflightHistory(Path(directory) / "h.jsonl")
        for i, speed in enumerate([120, 118, 115, 100, 88, 80, 74, 70, 66]):
            history.append(report(1000 + i, float(speed)))

        degrading = history.degrading_checks()
        assert [t.check_id for t in degrading] == ["disk.speed"]
        assert history.summary()["degrading"] == ["disk.speed"]


def test_a_stable_check_is_not_flagged(tmp_path):
    history = PreflightHistory(tmp_path / "h.jsonl")
    for i, speed in enumerate([120, 118, 121, 119, 120, 122, 118, 120]):
        history.append(report(1000 + i, float(speed)))
    assert history.degrading_checks() == []


def test_too_few_runs_is_not_a_trend(tmp_path):
    history = PreflightHistory(tmp_path / "h.jsonl")
    for i, speed in enumerate([120, 60]):
        history.append(report(1000 + i, float(speed)))
    assert history.degrading_checks() == []


def test_history_that_cannot_be_written_does_not_break_the_check(tmp_path):
    """A pre-flight that cannot write its history is still a valid pre-flight.
    Bookkeeping must never block the check itself."""
    blocked = tmp_path / "a_file"
    blocked.write_text("not a directory")
    history = PreflightHistory(blocked / "h.jsonl")
    history.append(report(1000, 100.0))          # must not raise
    assert history.runs() == []


def test_a_corrupt_line_is_skipped_not_fatal(tmp_path):
    path = tmp_path / "h.jsonl"
    history = PreflightHistory(path)
    history.append(report(1000, 100.0))
    with path.open("a") as handle:
        handle.write("{ not json\n")
    history.append(report(1001, 99.0))
    assert len(history.runs()) == 2
