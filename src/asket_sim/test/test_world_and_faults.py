"""The composed world, and every injectable fault doing what it claims."""

import pytest
from asket_sim.core.faults import FAULTS, FaultInjector
from asket_sim.core.pico import MODE_AUTONOMOUS
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


def test_obstacles_are_never_driven_through(world):
    """The simulator does not do obstacle avoidance — that is the navigation
    stack's job, and faking it here would test nothing. So obstacles must be
    placed clear of the track the vessel actually drives, which is not the same
    as the track it plans: the controller runs a few metres wide in a
    cross-current and wider through a turn.

    A demo in which the boat passes through a moored vessel is a demo nobody
    believes, and a lidar panel showing a 0.2 m return from inside a rock
    teaches an operator to distrust the panel."""
    import math

    closest = {o.name: float("inf") for o in world.cfg.obstacles}
    while not world.vessel.finished and world.t < 3000:
        world.step(0.1)
        for obstacle in world.cfg.obstacles:
            distance = math.hypot(
                world.vessel.east - obstacle.east_m, world.vessel.north - obstacle.north_m
            )
            closest[obstacle.name] = min(closest[obstacle.name], distance - obstacle.radius_m)

    for name, clearance in closest.items():
        assert clearance > 3.0, f"the vessel passes within {clearance:.1f} m of the {name}"


def test_at_least_one_obstacle_comes_within_lidar_range(world):
    """The other half of the same requirement: obstacles clear of the track are
    useless if they are also out of range, because the lidar panel then has
    nothing to show and cannot be demonstrated or reviewed."""
    max_range = world.cfg.lidar.max_range_m
    seen = 0
    while not world.vessel.finished and world.t < 3000:
        world.step(0.1)
        scan = world.take_lidar()
        if scan is not None and scan.nearest(use_filtered=True):
            if scan.nearest(use_filtered=True)[0] < max_range:
                seen += 1
    assert seen > 50, "obstacles are placed too far from the survey to ever be seen"


def test_sampling_the_link_does_not_change_it():
    """A read that mutates what it reads is a bug waiting to happen, and this
    one bit: the backend samples the link several times per tick, so a fade
    advanced inside sample() ran sixty times a second instead of once. Quality
    swung between 0.26 and 0.99 on a stationary vessel and the profile selector
    could never hold a candidate long enough to recover."""
    from asket_sim.core.link import LinkSim

    link = LinkSim()
    for _ in range(50):
        link.step(0.05)

    first = link.sample(0, 10.0, 20.0)
    for _ in range(20):
        again = link.sample(0, 10.0, 20.0)
        assert (again.quality, again.active_link, again.rtt_ms) == (
            first.quality,
            first.active_link,
            first.rtt_ms,
        )


def test_fading_is_the_same_whatever_the_step_size():
    """Parameterised as a standard deviation and a time constant rather than a
    per-step amplitude, so changing the tick rate does not silently change the
    weather."""
    import statistics

    from asket_sim.core.link import LinkSim

    def spread(dt, steps):
        link = LinkSim(seed=11)
        values = []
        for _ in range(steps):
            link.step(dt)
            values.append(link.sample(0, 0.0, 0.0).quality)
        return statistics.pstdev(values)

    fine = spread(0.01, 20000)     # 200 s at 100 Hz
    coarse = spread(0.5, 400)      # 200 s at 2 Hz
    assert abs(fine - coarse) < 0.03


def test_a_stationary_vessel_near_the_station_keeps_a_usable_link(world):
    """Fading must wander, not thrash. A link that flips between WiFi and LTE-M
    while the boat sits still makes the whole profile mechanism look broken."""
    links = set()
    for _ in range(600):
        world.step(0.1)
        world.vessel.east, world.vessel.north = 0.0, 0.0   # hold it in place
        links.add(world.snapshot().link.active_link)
    assert links == {"wifi"}


def test_a_dropout_does_not_un_send_the_hosts_ping_command(world):
    """A simulated dropout models the sonar failing, not the command being
    forgotten. Otherwise clearing the fault would leave the sonar stopped, and
    injecting one would look like the command had never been sent."""
    world.sonar.ping_enabled_by_command = True
    run(world, 2)
    assert world.sonar.pinging

    world.inject_fault("sonar_dropout")
    run(world, 2)
    assert not world.sonar.pinging
    assert world.sonar.ping_enabled_by_command, "the command must survive the fault"

    world.clear_fault("sonar_dropout")
    run(world, 2)
    assert world.sonar.pinging


def test_a_host_stop_survives_a_fault_being_cleared(world):
    """The other direction: clearing a fault must not start a sonar the host
    deliberately stopped."""
    world.sonar.ping_enabled_by_command = False
    world.inject_fault("sonar_dropout")
    run(world, 2)
    world.clear_fault("sonar_dropout")
    run(world, 2)
    assert not world.sonar.pinging
