"""The Pico is the authority, and the hardware outranks the Pico.

These tests encode safety rules 1 and 4 from docs/safety.md. If one of them ever
fails, something in this repository has started believing it can override the
killswitch, and that is not a bug to negotiate with.
"""

from asket_sim.core.pico import MODE_AUTONOMOUS, MODE_ESTOP, MODE_MANUAL, PicoConfig, PicoSim


def step(pico, seconds, dt=0.05):
    for _ in range(int(seconds / dt)):
        pico.step(dt)


def test_a_mode_request_is_not_immediately_confirmed():
    """The gap between request and confirmation is what the GUI must render."""
    pico = PicoSim(PicoConfig(confirm_delay_s=0.5))
    assert pico.mode == MODE_MANUAL
    pico.request_mode(MODE_AUTONOMOUS)
    step(pico, 0.2)
    assert pico.sample(0).mode == MODE_MANUAL, "must not report the requested mode"
    step(pico, 0.5)
    assert pico.sample(0).mode == MODE_AUTONOMOUS


def test_a_lost_request_never_confirms():
    """The GUI's command timeout path has to be reachable."""
    pico = PicoSim(PicoConfig(request_loss_probability=1.0))
    assert pico.request_mode(MODE_AUTONOMOUS) is True   # accepted...
    step(pico, 5.0)
    assert pico.sample(0).mode == MODE_MANUAL           # ...but never confirmed


def test_hardware_killswitch_forces_estop_and_software_cannot_leave_it():
    pico = PicoSim()
    pico.set_hardware_killswitch(True)
    assert pico.sample(0).mode == MODE_ESTOP

    pico.request_mode(MODE_AUTONOMOUS)
    step(pico, 5.0)
    assert pico.sample(0).mode == MODE_ESTOP, "software must not override the killswitch"
    assert not pico.sample(0).armed


def test_rc_channel_8_is_sovereign_too():
    pico = PicoSim()
    pico.request_mode(MODE_AUTONOMOUS)
    step(pico, 2.0)
    assert pico.sample(0).mode == MODE_AUTONOMOUS

    pico.set_rc_channel8(0)
    assert pico.sample(0).mode == MODE_ESTOP
    pico.request_mode(MODE_MANUAL)
    step(pico, 5.0)
    assert pico.sample(0).mode == MODE_ESTOP


def test_recovery_is_possible_once_the_hardware_releases():
    pico = PicoSim()
    pico.set_hardware_killswitch(True)
    step(pico, 1.0)
    pico.set_hardware_killswitch(False)
    pico.request_mode(MODE_MANUAL)
    step(pico, 2.0)
    assert pico.sample(0).mode == MODE_MANUAL


def test_invalid_mode_is_rejected():
    pico = PicoSim()
    assert pico.request_mode(99) is False
