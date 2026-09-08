"""One value, one place.

A quantity rendered in two panels is a quantity that will eventually disagree
with itself. Heading did: the vessel panel read it from the ``vessel`` stream at
5 Hz and the heading panel from the ``heading`` stream at 2 Hz, so the two rows
sat next to each other reading 029° and 028°. An operator cannot resolve that,
so they stop trusting both — which is worse than either number being slightly
wrong.

These are source-level checks. They cannot prove the rule holds everywhere, but
they pin the cases that were actually wrong, so re-introducing one fails here
rather than on a beach.
"""

import re
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest

GUI = Path(__file__).resolve().parents[2] / "asket_gui" / "src"

pytestmark = pytest.mark.skipif(
    not GUI.is_dir(), reason="the frontend is not present"
)


def source(*parts) -> str:
    return (GUI.joinpath(*parts)).read_text()


def test_heading_is_not_displayed_in_the_vessel_panel():
    """The regression this whole module exists for."""
    text = source("panels", "VesselState.jsx")
    assert "heading_deg" not in text
    assert "heading_valid" not in text
    # And the reason is written down, so the next person does not helpfully
    # add it back.
    assert "Heading is deliberately not here" in text


def test_the_heading_panel_still_has_a_heading_when_the_link_is_at_its_worst():
    """Removing the duplicate must not lose the value on the beacon profile,
    where the heading stream is not carried at all."""
    text = source("panels", "HeadingPanel.jsx")
    assert "vessel.heading_deg" in text
    # ...and says so, because a bearing with no accuracy figure is worth less
    # than one with, and the operator has to know which they are looking at.
    assert "the heading stream is not carried on this link" in text


def test_hull_attitude_and_transducer_attitude_are_labelled_apart():
    """Both are real, both are roll and pitch, and they are different sensors
    reading different numbers. Identical labels would read as a contradiction."""
    assert 'label="Roll / pitch"' in source("panels", "VesselState.jsx")
    assert 'label="Transducer pitch / roll"' in source("panels", "SonarPanel.jsx")


def test_the_status_strip_reads_the_same_streams_as_the_panels_it_summarises():
    """The strip is a summary, not a second source. If it read anything the
    owning panel does not, the two could disagree — which is the exact failure
    this file exists to prevent, reproduced in the header."""
    strip = source("components", "StatusStrip.jsx")
    for stream in ("power", "mission", "link"):
        assert f"streamPayload(state, '{stream}')" in strip

    # The fields it shows are the ones those panels show.
    assert "state_of_charge" in strip and "state_of_charge" in source("panels", "PowerPanel.jsx")
    assert "endurance_s" in strip and "endurance_s" in source("panels", "PowerPanel.jsx")
    assert "elapsed_s" in strip and "elapsed_s" in source("panels", "MissionPanel.jsx")
    assert "active_link" in strip and "active_link" in source("panels", "LinkStatus.jsx")


def test_the_status_strip_degrades_stale_values_like_everything_else():
    """A value in the header is still a live value. One that has gone stale
    while pinned above the fold would be the most persuasive lie on screen."""
    strip = source("components", "StatusStrip.jsx")
    assert "staleness" in strip and "thresholdsForRate" in strip


def test_map_layer_colours_are_declared_once():
    """The legend names the colours the layers are painted with. Two literals
    for one colour is a legend that lies after the next edit."""
    text = source("panels", "MissionMap.jsx")
    body = text[text.index("const COLOURS"):]
    # Every paint reference goes through COLOURS, so no raw hex survives in a
    # paint block below the declaration.
    paints = re.findall(r"'(?:line|fill|circle)-color':\s*([^,}]+)", body)
    assert paints, "no layer paints found — has the file been restructured?"
    for value in paints:
        assert value.strip().startswith("COLOURS."), f"raw colour in a paint: {value}"


@pytest.mark.skipif(shutil.which("node") is None, reason="node is not installed")
def test_relay_and_esc_bytes_are_decoded_into_words():
    """`Relays · · · ·` and `ESCs e1 e1` are precise and unreadable. Somebody who
    did not write the firmware cannot tell whether e1 is normal or catastrophic."""
    out = subprocess.run(
        ["node", "--input-type=module", "-e", textwrap.dedent(f"""
            const m = await import('{GUI}/lib/hull.js');
            const armed = m.hullSummary({{
                armed: true, relay_states: [true, true, false, false], esc_status: [0, 3],
            }});
            const disarmed = m.hullSummary({{
                armed: false, relay_states: [false, false], esc_status: [1, 1],
            }});
            console.log(JSON.stringify({{ armed, disarmed }}));
        """)],
        capture_output=True, text=True, check=True,
    )
    import json
    data = json.loads(out.stdout)

    armed = data["armed"]
    assert armed["relays"][0]["text"] == "closed"
    assert armed["relays"][2]["text"] == "open"
    assert armed["escs"][0]["text"] == "running"
    # An undocumented code is reported, never interpreted.
    assert armed["escs"][1]["text"] == "code 3"
    assert armed["escs"][1]["unknown"] is True
    # A thruster that is not running while armed is promoted out of the
    # collapsed section: a fault is never hidden behind a disclosure triangle.
    assert armed["alert"] == "1 of 2 thrusters not running while armed"

    # Disarmed, a stopped thruster is the correct state. Calling it a fault
    # would train the crew to ignore the line that matters.
    assert data["disarmed"]["alert"] is None


def test_no_relay_is_given_a_name_nobody_has_confirmed():
    """Which load each relay drives lives in pico_bridge, which this repository
    must not modify (Q7). Inventing 'Bilge pump' would be a caption an operator
    would act on."""
    text = source("lib", "hull.js")
    assert "export const RELAY_LABELS = {};" in text
    assert "PROVISIONAL (Q7)" in text
