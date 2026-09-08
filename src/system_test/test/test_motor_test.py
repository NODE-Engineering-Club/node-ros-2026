"""Safety rule 6, written as code: motor tests never run automatically."""

from system_test.core.motor_test import (
    CONSENT_PHRASE,
    CONSENT_VALIDITY_S,
    MAX_DURATION_S,
    MAX_THROTTLE,
    MotorTestRequest,
    evaluate_request,
)

NOW = 1_700_000_000_000


def request(**overrides):
    fields = dict(
        thruster=0, throttle=0.1, duration_s=1.0,
        consent_phrase=CONSENT_PHRASE, consent_utc_ms=NOW,
    )
    fields.update(overrides)
    return MotorTestRequest(**fields)


def test_a_fully_consented_request_is_allowed():
    decision = evaluate_request(request(), NOW, armed=True, in_water=False)
    assert decision.allowed
    assert decision.command["throttle"] == 0.1


def test_no_consent_means_no_test():
    decision = evaluate_request(request(consent_phrase=""), NOW, armed=True, in_water=False)
    assert not decision.allowed
    assert "out of the water" in decision.reason
    assert decision.command is None


def test_a_truthy_value_is_not_consent():
    """A checkbox can be clicked by muscle memory and a default-true boolean in
    a config file can arm this without anybody deciding to. The operator has to
    produce the exact phrase."""
    for pretend in ("yes", "true", "1", "OK", "vessel clear"):
        assert not evaluate_request(
            request(consent_phrase=pretend), NOW, armed=True, in_water=False
        ).allowed


def test_consent_goes_stale():
    """Confirmed twenty minutes ago is not confirmed now — the boat may be back
    in the water."""
    stale = NOW - int((CONSENT_VALIDITY_S + 10) * 1000)
    decision = evaluate_request(request(consent_utc_ms=stale), NOW, armed=True, in_water=False)
    assert not decision.allowed
    assert "Confirm again" in decision.reason


def test_a_vessel_reporting_itself_in_the_water_is_refused_despite_consent():
    decision = evaluate_request(request(), NOW, armed=True, in_water=True)
    assert not decision.allowed
    assert "in the water" in decision.reason


def test_throttle_is_capped_at_a_pulse():
    assert not evaluate_request(
        request(throttle=MAX_THROTTLE + 0.01), NOW, armed=True, in_water=False
    ).allowed
    assert not evaluate_request(request(throttle=1.0), NOW, armed=True, in_water=False).allowed
    assert not evaluate_request(request(throttle=0.0), NOW, armed=True, in_water=False).allowed


def test_duration_is_capped_at_a_pulse():
    assert not evaluate_request(
        request(duration_s=MAX_DURATION_S + 0.1), NOW, armed=True, in_water=False
    ).allowed
    assert not evaluate_request(request(duration_s=0.0), NOW, armed=True, in_water=False).allowed


def test_an_unarmed_vessel_is_refused_rather_than_silently_doing_nothing():
    """Otherwise an operator learns that pressing it does nothing, and presses
    it without thinking on the day the vessel is armed."""
    decision = evaluate_request(request(), NOW, armed=False, in_water=False)
    assert not decision.allowed
    assert "not armed" in decision.reason


def test_refusal_is_the_default():
    """Every path returning allowed=True has passed every gate explicitly."""
    decision = evaluate_request(
        MotorTestRequest(0, 0.1, 1.0, "", 0), NOW, armed=True, in_water=None
    )
    assert not decision.allowed
