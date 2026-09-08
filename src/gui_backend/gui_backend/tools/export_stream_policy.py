"""Export the stream registry and profile policy as JSON, for the frontend mock.

The mock has to negotiate exactly as the real backend does, or it is a
demonstration of a system that does not exist. Rather than transcribing the
policy into JavaScript by hand and letting the two drift, the JSON is generated
from the Python and checked in, and a test fails if they disagree.

    python3 -m gui_backend.tools.export_stream_policy \
        --out src/asket_gui/src/lib/mock/streamPolicy.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from gui_backend.core.streams import PROFILE_ORDER, PROFILES, STREAMS


def build() -> dict:
    return {
        "_comment": (
            "GENERATED from gui_backend/core/streams.py by "
            "gui_backend/tools/export_stream_policy.py. Do not edit by hand — "
            "test_mock_policy.py fails if this drifts from the Python."
        ),
        "streams": {
            name: {
                "description": spec.description,
                "default_rate_hz": spec.default_rate_hz,
                "max_rate_hz": spec.max_rate_hz,
                "critical": spec.critical,
                "on_change_only": spec.on_change_only,
                "typical_bytes": spec.typical_bytes,
            }
            for name, spec in STREAMS.items()
        },
        "profile_order": list(PROFILE_ORDER),
        "profiles": {
            name: {
                "description": profile.description,
                "budget_bytes_per_s": profile.budget_bytes_per_s,
                "policies": {
                    stream: {"max_rate_hz": policy.max_rate_hz, "detail": policy.detail}
                    for stream, policy in profile.policies.items()
                },
            }
            for name, profile in PROFILES.items()
        },
    }


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(build(), indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(f"wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
