"""The mock's payloads must have the same shape as the real backend's.

Mock mode is how the GUI gets reviewed. If it sends a field the vessel never
sends, a panel comes to depend on it and works perfectly right up until the
first time it is pointed at a boat. If it omits one, a real bug goes unnoticed
because the mock never reproduces it.

So the JavaScript payload builders are run under Node, and their key sets are
compared with the Python ones, per stream and per detail level. The test skips
if Node is not installed — it is a cross-language check, not a build dependency.
"""

import json
import shutil
import subprocess
import textwrap
from pathlib import Path

import pytest
from asket_sim.core.world import SimWorld, WorldConfig
from gui_backend.core import payloads
from gui_backend.core.sim_source import SimSource

GUI = Path(__file__).resolve().parents[2] / "asket_gui"
DETAILS = ["full", "reduced", "minimal"]

pytestmark = pytest.mark.skipif(
    shutil.which("node") is None or not (GUI / "src" / "lib" / "mock").is_dir(),
    reason="node or the frontend mock is not present",
)


def mock_payload_keys() -> dict:
    """Run the JS payload builders under Node and return their key sets."""
    script = textwrap.dedent(
        f"""
        const base = '{GUI}/src/lib/mock';
        const {{ MockWorld }} = await import(base + '/world.js');
        const P = await import(base + '/payloads.js');

        const world = new MockWorld();
        // Run it far enough that the vessel is moving: several payloads only
        // carry their optional fields once there is a speed to divide by.
        for (let i = 0; i < 900; i += 1) world.step(0.1);

        const context = {{ profile: 'full', manual: false, rateBytesPerS: 1000 }};
        const out = {{}};
        for (const detail of ['full', 'reduced', 'minimal']) {{
          out[detail] = {{
            vessel: Object.keys(P.vesselPayload(world, detail)),
            heading: Object.keys(P.headingPayload(world, detail)),
            pico: Object.keys(P.picoPayload(world, detail)),
            power: Object.keys(P.powerPayload(world, detail)),
            sonar: Object.keys(P.sonarPayload(world, detail)),
            lidar: Object.keys(P.lidarPayload(world, detail)),
            link: Object.keys(P.linkPayload(world, detail, context)),
          }};
        }}
        console.log(JSON.stringify(out));
        """
    )
    result = subprocess.run(
        ["node", "--input-type=module", "-e", script],
        capture_output=True, text=True, timeout=120,
    )
    if result.returncode != 0:
        pytest.fail(f"node failed:\n{result.stderr}")
    return json.loads(result.stdout)


def python_payload_keys() -> dict:
    source = SimSource(SimWorld(WorldConfig()), time_scale=10.0)
    t = 0.0
    for _ in range(300):
        t += 0.1
        source.step(t)

    snapshot = source.world.snapshot()
    estimate = source._heading_estimate()
    sonar = source.snapshot("sonar", "full").payload

    class Health:
        def __init__(self, data):
            self.__dict__.update(data)
            self.clock_ok = data["clock_ok"]
            self.clock_compromised = data["clock_compromised"]
            self.ping_rate_ok = data["ping_rate_ok"]

    out = {}
    for detail in DETAILS:
        entry = {
            "vessel": list(payloads.vessel_payload(snapshot.vessel, detail)),
            "heading": list(payloads.heading_payload(estimate, detail)),
            "pico": list(payloads.pico_payload(snapshot.pico, detail)),
            "power": list(
                payloads.power_payload(
                    snapshot.battery, detail,
                    survey_remaining_m=source._survey_remaining_m(),
                    speed_ms=max(0.5, snapshot.vessel.sog_ms),
                )
            ),
            "sonar": list(payloads.sonar_payload(Health(sonar), detail)),
            "lidar": list(payloads.lidar_payload(snapshot.lidar, detail, 1)),
            "link": list(
                payloads.link_payload(
                    snapshot.link, profile="full", profile_manual=False,
                    clients=1, rate_bytes_per_s=1000.0, detail=detail,
                )
            ),
        }
        out[detail] = entry
    return out


@pytest.mark.parametrize("detail", DETAILS)
def test_mock_payload_keys_match_the_backend(detail):
    mock = mock_payload_keys()[detail]
    real = python_payload_keys()[detail]

    for stream in sorted(real):
        assert set(mock[stream]) == set(real[stream]), (
            f"{stream} at detail '{detail}' differs.\n"
            f"  only in mock: {sorted(set(mock[stream]) - set(real[stream]))}\n"
            f"  only in real: {sorted(set(real[stream]) - set(mock[stream]))}"
        )


def test_detail_levels_actually_shrink_the_mock_payload():
    """`minimal` must not be `full` with a smaller number in front of it."""
    keys = mock_payload_keys()
    for stream in ("vessel", "pico", "power"):
        full = set(keys["full"][stream])
        minimal = set(keys["minimal"][stream])
        assert minimal < full, f"{stream} does not shrink at minimal detail"
