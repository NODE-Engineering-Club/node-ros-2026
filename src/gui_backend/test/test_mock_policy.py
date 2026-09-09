"""The frontend mock must negotiate exactly as the real backend does.

If the two drift, the mock stops being a preview of this system and becomes a
demonstration of a system that does not exist — which is worse than no mock,
because it is reviewed and believed.

The JSON is generated, not transcribed. This test is what makes that true.
"""

import json
from pathlib import Path

import pytest
from gui_backend.tools.export_stream_policy import build

# .../src/gui_backend/test/this_file.py -> parents[2] is the workspace src/
POLICY_PATH = (
    Path(__file__).resolve().parents[2]
    / "asket_gui" / "src" / "lib" / "mock" / "streamPolicy.json"
)


def test_the_checked_in_policy_matches_the_python():
    if not POLICY_PATH.is_file():
        pytest.skip(f"{POLICY_PATH} not present")
    on_disk = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
    assert on_disk == build(), (
        "src/asket_gui/src/lib/mock/streamPolicy.json is out of date. Regenerate it:\n"
        "  python3 -m gui_backend.tools.export_stream_policy "
        "--out src/asket_gui/src/lib/mock/streamPolicy.json"
    )


def test_the_export_carries_everything_the_mock_needs_to_negotiate():
    exported = build()
    assert exported["profile_order"] == ["full", "reduced", "minimal"]
    for name, spec in exported["streams"].items():
        assert spec["max_rate_hz"] >= spec["default_rate_hz"], name
        assert spec["typical_bytes"] > 0, name
    for name, profile in exported["profiles"].items():
        assert profile["policies"], name
        assert profile["budget_bytes_per_s"] > 0, name
