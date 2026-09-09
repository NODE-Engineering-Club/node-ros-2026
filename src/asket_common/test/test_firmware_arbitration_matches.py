"""The firmware's arbitration and the Python mirror must agree, cell for cell.

The Pico firmware cannot be run here. But ``arbitrate_mode()`` in the sketch is
deliberately *pure* — no globals, no clock, no I/O — so the C++ can be lifted out
of the ``.ino``, compiled with the host compiler, and driven through the same 24
combinations as :mod:`asket_common.mode_arbitration`.

That is what this test does. It is not a substitute for bench validation, and it
proves nothing about SBUS, relays or timing. What it does prove is the one thing
that bit this project before: **the two sides of a wire agreeing on a format.**
A change to the firmware table that is not mirrored in Python (or the reverse)
fails here rather than on the water.

If ``g++`` is unavailable the test skips rather than passing quietly — a check
that silently does nothing is worse than no check.
"""

import re
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest

from asket_common.mode_arbitration import (
    MODE_AUTONOMOUS,
    MODE_ESTOP,
    MODE_MANUAL,
    arbitrate,
)

FIRMWARE = (
    Path(__file__).resolve().parents[3] / "firmware" / "pico-node_v3" / "pico-node_v3.ino"
)

#: Firmware enum values. Mirrored in mode_arbitration.FIRMWARE_MODE_NUMBERS.
NUMBER = {MODE_ESTOP: 1, MODE_MANUAL: 2, MODE_AUTONOMOUS: 3}
NAME = {v: k for k, v in NUMBER.items()}
REQUEST_NONE = 0

pytestmark = pytest.mark.skipif(
    shutil.which("g++") is None, reason="no C++ compiler to check the firmware against"
)


def _extract(source: str, start: str, end: str) -> str:
    """Lift one span out of the sketch, so the test reads the real firmware
    rather than a copy of it that could drift on its own."""
    i = source.index(start)
    j = source.index(end, i)
    return source[i:j]


def _build_harness(tmp_path: Path) -> Path:
    assert FIRMWARE.is_file(), f"firmware sketch not found at {FIRMWARE}"
    source = FIRMWARE.read_text()

    enum = _extract(source, "enum OperationMode", ";") + ";"
    decision = _extract(source, "struct ModeDecision", "};") + "};"
    func = _extract(source, "ModeDecision arbitrate_mode(", "\n}\n") + "\n}\n"

    request_none = re.search(r"const int REQUEST_NONE = (\d+);", source)
    assert request_none, "REQUEST_NONE not found in the sketch"

    upward = re.search(r"#define SOFTWARE_UPWARD_REQUESTS_ALLOWED (\d+)", source)
    assert upward, "SOFTWARE_UPWARD_REQUESTS_ALLOWED not found in the sketch"

    program = textwrap.dedent(
        """
        #include <cstdio>
        %(enum)s
        const int REQUEST_NONE = %(none)s;
        %(decision)s
        %(func)s

        int main() {
          int modes[3] = { MODE_ESTOP, MODE_MANUAL, MODE_AUTONOMOUS };
          int requests[4] = { REQUEST_NONE, MODE_ESTOP, MODE_MANUAL, MODE_AUTONOMOUS };
          for (int u = 0; u < 2; u++) {
            for (int i = 0; i < 3; i++) {
              for (int j = 0; j < 4; j++) {
                ModeDecision d = arbitrate_mode((OperationMode)modes[i],
                                                requests[j], u == 1);
                printf("%%d %%d %%d %%d %%d %%s\\n",
                       u, modes[i], requests[j],
                       (int)d.effective, d.accepted ? 1 : 0,
                       d.reason[0] ? d.reason : "-");
              }
            }
          }
          return 0;
        }
        """
    ) % {
        "enum": enum,
        "decision": decision,
        "func": func,
        "none": request_none.group(1),
    }

    src = tmp_path / "arbitration_harness.cpp"
    src.write_text(program)
    exe = tmp_path / "arbitration_harness"
    subprocess.run(
        ["g++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-o", str(exe), str(src)],
        check=True,
        capture_output=True,
    )
    return exe


@pytest.fixture(scope="module")
def firmware_rows(tmp_path_factory):
    exe = _build_harness(tmp_path_factory.mktemp("firmware"))
    out = subprocess.run([str(exe)], check=True, capture_output=True, text=True).stdout

    rows = {}
    for line in out.strip().splitlines():
        upward, rc, request, effective, accepted, reason = line.split()
        key = (
            NAME[int(rc)],
            NAME[int(request)] if int(request) != REQUEST_NONE else None,
            upward == "1",
        )
        rows[key] = (
            NAME[int(effective)],
            accepted == "1",
            "" if reason == "-" else reason,
        )
    return rows


def test_the_harness_covers_every_combination(firmware_rows):
    assert len(firmware_rows) == 24


def test_firmware_agrees_with_python_on_every_cell(firmware_rows):
    """The whole point of this file."""
    mismatches = []
    for key, (effective, accepted, reason) in sorted(firmware_rows.items(), key=str):
        rc, request, allow_upward = key
        expected = arbitrate(rc, request, allow_upward=allow_upward)
        got = (effective, accepted, reason)
        want = (expected.effective, expected.accepted, expected.reason)
        if got != want:
            mismatches.append(f"  rc={rc} req={request} upward={allow_upward}: "
                              f"firmware={got} python={want}")
    assert not mismatches, (
        "firmware and Python arbitration disagree:\n" + "\n".join(mismatches)
    )


def test_firmware_never_makes_the_vessel_more_permissive(firmware_rows):
    """Read straight off the compiled firmware, not off the Python."""
    from asket_common.mode_arbitration import MODE_RANK

    for (rc, _request, _upward), (effective, _a, _r) in firmware_rows.items():
        assert MODE_RANK[effective] <= MODE_RANK[rc]


def test_firmware_always_accepts_estop(firmware_rows):
    for (rc, request, upward), (effective, accepted, _r) in firmware_rows.items():
        if request == MODE_ESTOP:
            assert accepted is True, f"rc={rc} upward={upward}"
            assert effective == MODE_ESTOP


def test_the_two_upward_constants_have_the_same_default():
    """Flipping one and not the other is a drift bug that would not show up as a
    crash — the boat would just refuse, or accept, the wrong thing."""
    from asket_common.mode_arbitration import SOFTWARE_UPWARD_REQUESTS_ALLOWED

    source = FIRMWARE.read_text()
    match = re.search(r"#define SOFTWARE_UPWARD_REQUESTS_ALLOWED (\d+)", source)
    assert match
    assert bool(int(match.group(1))) is SOFTWARE_UPWARD_REQUESTS_ALLOWED


def test_the_two_request_timeouts_agree():
    from asket_common.mode_arbitration import SOFTWARE_REQUEST_TIMEOUT_S

    source = FIRMWARE.read_text()
    match = re.search(
        r"const unsigned long SOFTWARE_REQUEST_TIMEOUT_MS = (\d+);", source
    )
    assert match
    assert int(match.group(1)) / 1000.0 == SOFTWARE_REQUEST_TIMEOUT_S


# -- other constants mirrored out of the firmware --------------------------
#
# Each of these exists in Python only because it cannot be read off the wire.
# That makes every one a drift risk, so every one is checked against the sketch.


def test_the_estop_feedback_flag_matches_the_firmware():
    """If somebody enables the feedback in firmware and not here, the pre-flight
    keeps warning about a problem that no longer exists — and the crew learns to
    ignore it. If the reverse, the GUI stops warning about a real one."""
    from gui_backend.core.pico_state import ESTOP_FEEDBACK_ENABLED

    source = FIRMWARE.read_text()
    match = re.search(r"#define ESTOP_FEEDBACK_ENABLED (\d+)", source)
    assert match, "ESTOP_FEEDBACK_ENABLED not found in the sketch"
    assert bool(int(match.group(1))) is ESTOP_FEEDBACK_ENABLED


def test_the_esc_arm_delay_matches_the_firmware():
    from gui_backend.core.pico_state import ESC_ARM_DELAY_MS

    source = FIRMWARE.read_text()
    match = re.search(r"const unsigned long ESC_ARM_DELAY_MS = (\d+);", source)
    assert match
    assert int(match.group(1)) == ESC_ARM_DELAY_MS


def test_the_sbus_thresholds_match_the_firmware():
    """The GUI derives the RC-selected mode itself, so it has to threshold Ch8
    exactly as the firmware does or the two disagree about what the switch says."""
    from gui_backend.core import pico_state

    source = FIRMWARE.read_text()
    for name, expected in (
        ("MODE_LOW_MAX", pico_state.MODE_LOW_MAX),
        ("MODE_MID_MAX", pico_state.MODE_MID_MAX),
        ("ARM_THRESHOLD", pico_state.ARM_THRESHOLD),
    ):
        match = re.search(rf"const int {name}\s*=\s*(\d+);", source)
        assert match, f"{name} not found in the sketch"
        assert int(match.group(1)) == expected, name

    for name, expected in (
        ("SBUS_MIN", pico_state.SBUS_MIN),
        ("SBUS_MID", pico_state.SBUS_MID),
        ("SBUS_MAX", pico_state.SBUS_MAX),
    ):
        match = re.search(rf"const uint16_t {name}\s*=\s*(\d+);", source)
        assert match, f"{name} not found in the sketch"
        assert int(match.group(1)) == expected, name


def test_the_failsafe_timeouts_are_both_500ms():
    """Documented as 600 ms in several places for a long time. They are 500."""
    source = FIRMWARE.read_text()
    for name in ("SBUS_FAILSAFE_TIMEOUT_MS", "SERIAL_FAILSAFE_TIMEOUT_MS"):
        match = re.search(rf"const unsigned long {name} = (\d+);", source)
        assert match, name
        assert int(match.group(1)) == 500, name


def test_the_status_line_format_is_the_one_the_parser_expects():
    """The parser matches whole keys. If the firmware renames one, the panel
    goes quiet rather than wrong — but quiet is still a failure, so check the
    exact spellings appear in the sketch's status print."""
    from gui_backend.core.pico_state import _KEYS

    source = FIRMWARE.read_text()
    start = source.index('"[STAT] Mode:%d')
    end = source.index(");", start)
    printf = source[start:end]

    for key in _KEYS:
        assert f"{key}:" in printf, f"the firmware no longer prints {key}:"
