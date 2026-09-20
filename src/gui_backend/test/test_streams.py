"""Bandwidth negotiation: the defence against a GUI that dies 200 m offshore."""

import pytest
from gui_backend.core.streams import (
    DETAIL_FULL,
    DETAIL_MINIMAL,
    PROFILE_FULL,
    PROFILE_MINIMAL,
    PROFILE_ORDER,
    PROFILE_REDUCED,
    PROFILES,
    STREAMS,
    estimate_bytes_per_s,
    resolve,
)


def test_an_unknown_stream_is_refused_with_a_reason():
    res = resolve("bathymetry", 5.0, DETAIL_FULL, PROFILE_FULL)
    assert not res.granted
    assert "no such stream" in res.reason


def test_a_client_cannot_exceed_a_streams_own_ceiling():
    res = resolve("vessel", 1000.0, DETAIL_FULL, PROFILE_FULL)
    assert res.rate_hz == STREAMS["vessel"].max_rate_hz
    assert "maximum" in res.reason


def test_the_profile_wins_over_the_client():
    res = resolve("vessel", 10.0, DETAIL_FULL, PROFILE_MINIMAL)
    assert res.rate_hz == 0.2
    assert "minimal profile" in res.reason


def test_a_refusal_says_why_rather_than_silently_starving_the_stream():
    """An operator who cannot tell 'nothing is happening' from 'I am not being
    sent it' will eventually act on the wrong one."""
    res = resolve("lidar", 5.0, DETAIL_FULL, PROFILE_REDUCED)
    assert not res.granted
    assert "not carried on the reduced link profile" in res.reason


def test_detail_is_reduced_and_the_client_is_told():
    res = resolve("vessel", 1.0, DETAIL_FULL, PROFILE_MINIMAL)
    assert res.detail == DETAIL_MINIMAL
    assert "detail reduced" in res.reason


def test_an_undemanding_request_on_a_good_link_is_granted_untouched():
    res = resolve("vessel", 2.0, DETAIL_FULL, PROFILE_FULL)
    assert res.granted and res.rate_hz == 2.0 and res.reason == ""


def test_on_change_streams_have_no_rate():
    res = resolve("plan", 10.0, DETAIL_FULL, PROFILE_FULL)
    assert res.granted and res.rate_hz == 0.0


@pytest.mark.parametrize("profile", PROFILE_ORDER)
def test_every_profile_fits_inside_its_own_budget(profile):
    """The whole point. If subscribing to everything a profile allows exceeds
    what that link can carry, the profile is a lie."""
    resolutions = [resolve(name, None, None, profile) for name in STREAMS]
    estimate = estimate_bytes_per_s(resolutions)
    assert estimate < PROFILES[profile].budget_bytes_per_s


def test_critical_streams_survive_into_the_beacon_profile():
    """On LTE-M an operator must still know where the boat is, what mode it is
    in, and whether it is about to run out of charge."""
    for name in ("vessel", "pico", "power", "alarms"):
        assert resolve(name, None, None, PROFILE_MINIMAL).granted


def test_profiles_are_ordered_from_richest_to_poorest():
    counts = [len(PROFILES[p].policies) for p in PROFILE_ORDER]
    assert counts == sorted(counts, reverse=True)
    budgets = [PROFILES[p].budget_bytes_per_s for p in PROFILE_ORDER]
    assert budgets == sorted(budgets, reverse=True)
