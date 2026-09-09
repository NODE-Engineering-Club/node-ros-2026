"""Pre-flight history.

Every run is timestamped and stored, because **gradual degradation shows up as
a trend before it becomes an outright failure**. A connector that is slowly
corroding does not fail on the day it fails; it spends a month getting worse
while every individual pre-flight still says GO.

Stored as JSON lines under the missions disk. Small — a few hundred bytes a run
— so keeping months of it costs nothing.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

from .checks import FAIL, PASS, SKIPPED, WARN, Report

HISTORY_NAME = "system_test.jsonl"

#: Keep this many runs. At one or two per outing that is years.
MAX_RUNS = 2000


@dataclass
class Trend:
    check_id: str
    name: str
    #: Oldest first: (run_utc_ms, measured_value or None, status).
    points: list[tuple[int, float | None, str]]

    @property
    def degrading(self) -> bool:
        """True when a numeric measurement has moved consistently the wrong way.

        Deliberately crude — a straight comparison of the first and last thirds
        with a requirement that the recent runs are not simply noisy. The point
        is to raise an eyebrow, not to do statistics.
        """
        values = [v for _, v, _ in self.points if v is not None]
        if len(values) < 6:
            return False
        third = max(2, len(values) // 3)
        early = sum(values[:third]) / third
        late = sum(values[-third:]) / third
        if math.isclose(early, 0.0, abs_tol=1e-9):
            return False
        return abs(late - early) / abs(early) > 0.25


class PreflightHistory:
    # Named for what it is rather than 'TestHistory', which pytest tries to
    # collect as a test class and then warns about having a constructor.
    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)

    def append(self, report: Report) -> None:
        record = {
            "run_utc_ms": report.run_utc_ms,
            "go": report.go,
            "summary": report.summary,
            "items": {
                item.id: {
                    "status": item.status,
                    "value": None
                    if isinstance(item.measured_value, float) and math.isnan(item.measured_value)
                    else item.measured_value,
                }
                for item in report.items
            },
        }
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, separators=(",", ":")) + "\n")
        except OSError:
            # A pre-flight that cannot write its history is still a valid
            # pre-flight. Never let bookkeeping block the check itself.
            return
        self._trim()

    def _trim(self) -> None:
        try:
            lines = self.path.read_text(encoding="utf-8").splitlines()
        except OSError:
            return
        if len(lines) <= MAX_RUNS:
            return
        try:
            self.path.write_text("\n".join(lines[-MAX_RUNS:]) + "\n", encoding="utf-8")
        except OSError:
            pass

    def runs(self, limit: int = 50) -> list[dict]:
        try:
            lines = self.path.read_text(encoding="utf-8").splitlines()
        except OSError:
            return []
        out = []
        for line in lines[-limit:]:
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
        return out

    def trend(self, check_id: str, name: str = "", limit: int = 50) -> Trend:
        points = []
        for run in self.runs(limit):
            item = (run.get("items") or {}).get(check_id)
            if item is None:
                continue
            points.append((int(run.get("run_utc_ms", 0)), item.get("value"), item.get("status")))
        return Trend(check_id, name or check_id, points)

    def degrading_checks(self, limit: int = 50) -> list[Trend]:
        """Checks that are drifting even though they still pass.

        This is the whole reason the history exists.
        """
        ids: list[str] = []
        for run in self.runs(limit):
            for check_id in (run.get("items") or {}):
                if check_id not in ids:
                    ids.append(check_id)
        return [t for t in (self.trend(i, limit=limit) for i in ids) if t.degrading]

    def summary(self, limit: int = 20) -> dict:
        runs = self.runs(limit)
        if not runs:
            return {"runs": 0}
        statuses = [status for run in runs for status in (run.get("items") or {}).values()]
        return {
            "runs": len(runs),
            "last_run_utc_ms": runs[-1].get("run_utc_ms", 0),
            "go_rate": sum(1 for r in runs if r.get("go")) / len(runs),
            "counts": {
                s: sum(1 for st in statuses if st.get("status") == s)
                for s in (PASS, WARN, FAIL, SKIPPED)
            },
            "degrading": [t.check_id for t in self.degrading_checks(limit)],
        }
