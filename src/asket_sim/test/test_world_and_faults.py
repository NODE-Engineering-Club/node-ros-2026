"""The composed world, and every injectable fault doing what it claims."""

import pytest
from asket_sim.core.faults import FAULTS, FaultInjector
from asket_sim.core.pico import MODE_AUTONOMOUS, MODE_ESTOP, MODE_MANUAL
from asket_sim.core.world import SimWorld, WorldConfig


def run(world, seconds, dt=0.1):
    for _ in range(int(seconds / dt)):
        world.step(dt)


@pytest.fixture
def world():
    return SimWorld(WorldConfig(), start_utc_ms=1_700_000_000_000)


def test_unknown_fault_is_rejected_loudly():
    """A typo in a launch file must fail, not silently do nothing."""
    with pytest.raises(KeyError):
        FaultInjector().inject("sonar_droput")


def test_faults_expire():
    f = FaultInjector()
    f.inject("link_loss", duration_s=1.0)
    f.step(0.5)
    assert f.active("link_loss")
    f.step(0.6)
    assert not f.active("link_loss")


@pytest.mark.parametrize("name", sorted(FAULTS))
def test_every_documented_fault_can_be_injected_and_stepped(world, name):
    world.inject_fault(name)
    run(world, 2)
    assert name in world.snapshot().active_faults


def test_heading_invalid_marks_the_sample_not_just_an_alarm(world):
    run(world, 5)
    assert world.snapshot().vessel.heading_valid
    world.inject_fault("heading_invalid")
    run(world, 1)
    assert not world.snapshot().vessel.heading_valid


def test_clock_drift_accumulates(world):
    world.inject_fault("clock_drift")
    run(world, 10)
    first = world.snapshot().ping
    run(world, 20)
    assert world.cfg.sonar.clock_offset_ms > 500
    assert first is not None


def test_sonar_dropout_stops_pings_without_stopping_the_world(world):
    run(world, 5)
    assert world.take_ping() is not None
    world.inject_fault("sonar_dropout")
    run(world, 5)
    assert world.take_ping() is None
    world.clear_fault("sonar_dropout")
    run(world, 5)
    assert world.take_ping() is not None


def test_disk_full_leaves_almost_nothing(world):
    world.inject_fault("disk_full")
    run(world, 1)
    assert world.snapshot().disk_free_bytes < 16 * 1024**2


def test_link_loss_and_degradation(world):
    run(world, 5)
    assert world.snapshot().link.active_link == "wifi"
    world.inject_fault("link_degraded")
    run(world, 1)
    assert world.snapshot().link.active_link == "4g"
    world.clear_fault("link_degraded")
    world.inject_fault("link_loss")
    run(world, 1)
    snap = world.snapshot()
    assert snap.link.active_link == "none"
    assert snap.link.capacity_bytes_per_s == 0.0


def test_battery_fault_drains_faster(world):
    world.pico.request_mode(MODE_AUTONOMOUS)
    run(world, 60)
    healthy = world.snapshot().battery.consumed_wh

    fast = SimWorld(WorldConfig(), start_utc_ms=1_700_000_000_000)
    fast.pico.request_mode(MODE_AUTONOMOUS)
    fast.inject_fault("battery_fault")
    run(fast, 60)
    assert fast.snapshot().battery.consumed_wh > healthy * 1.5


def test_propulsion_only_draws_power_when_the_pico_says_armed(world):
    """Displayed and simulated behaviour both follow the Pico, not a request."""
    run(world, 30)
    assert not world.snapshot().pico.armed
    idle = world.snapshot().battery.power_w
    world.pico.request_mode(MODE_AUTONOMOUS)
    run(world, 60)
    assert world.snapshot().pico.armed
    assert world.snapshot().battery.power_w > idle
