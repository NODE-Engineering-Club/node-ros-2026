"""The pre-flight as it reaches the GUI in sim mode.

Sim mode exists so the pre-flight can be rehearsed before anyone is standing on
a beach. The mounting check is the one item on it a simulated vessel can answer
truthfully: the file either says somebody measured the boat, or it does not.
"""

from asket_sim.core.world import SimWorld, WorldConfig
from gui_backend.core.sim_source import SimSource


def source(**kwargs):
    return SimSource(SimWorld(WorldConfig()), time_scale=10.0, **kwargs)


def item(report, check_id):
    return next(i for i in report.items if i.id == check_id)


def test_sim_reads_the_real_mounting_file_and_warns_that_it_is_provisional():
    report = source().run_preflight()
    result = item(report, "sonar.mounting")
    assert result.status == "WARN"
    assert "PROVISIONAL" in result.message
    # It warns; it does not ground the boat.
    assert report.go


def test_the_warning_names_the_actual_file_on_disk():
    result = item(source().run_preflight(), "sonar.mounting")
    assert result.message.endswith("mounting.yaml says nobody measured it")
    assert "/" in result.message, "the operator needs the path, not just the name"


def test_a_measured_file_clears_the_warning(tmp_path):
    """Proves the check is reading the file rather than always warning."""
    path = tmp_path / "mounting.yaml"
    path.write_text(
        'measured: true\nmeasured_by: "NODE"\nmeasured_utc: "2026-04-01"\n'
        "tilt_deg: 32.0\nlever_x_m: -0.19\nlever_y_m: -0.41\nlever_z_m: -0.16\n"
    )
    result = item(source(mounting_path=path).run_preflight(), "sonar.mounting")
    assert result.status == "PASS"
    assert "NODE" in result.message


def test_a_missing_file_is_reported_rather_than_assumed_harmless(tmp_path):
    result = item(source(mounting_path=tmp_path / "absent.yaml").run_preflight(),
                  "sonar.mounting")
    assert result.status == "WARN"
    assert result.remedy
