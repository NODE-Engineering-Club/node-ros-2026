"""Automatic profile selection: hysteresis, and never overriding a human."""

import pytest
from gui_backend.core.link_profile import ProfileSelector, ProfileThresholds
from gui_backend.core.streams import PROFILE_FULL, PROFILE_MINIMAL, PROFILE_REDUCED

T = ProfileThresholds(upgrade_hold_s=8.0, downgrade_hold_s=2.0)


def hold(selector, rtt, quality, seconds, start=0.0, step=0.5):
    t = start
    for _ in range(int(seconds / step)):
        t += step
        selector.update(rtt, quality, t)
    return t


def test_a_good_link_stays_on_full():
    sel = ProfileSelector(T)
    hold(sel, rtt=20.0, quality=0.9, seconds=30)
    assert sel.profile == PROFILE_FULL


def test_degradation_is_fast():
    """Being slow to degrade means the operator watches a frozen screen."""
    sel = ProfileSelector(T)
    t = hold(sel, 300.0, 0.5, seconds=3)
    assert sel.profile == PROFILE_REDUCED
    assert t <= 4.0


def test_recovery_is_slow():
    """A link that has just come back has not proved anything yet."""
    sel = ProfileSelector(T, profile=PROFILE_REDUCED)
    hold(sel, 20.0, 0.9, seconds=4)
    assert sel.profile == PROFILE_REDUCED      # not yet
    hold(sel, 20.0, 0.9, seconds=8, start=4.0)
    assert sel.profile == PROFILE_FULL


def test_a_brief_dip_does_not_change_anything():
    """Flapping between profiles is worse than being wrong: every change
    re-runs the client's subscriptions and panels appear and vanish."""
    sel = ProfileSelector(T)
    sel.update(300.0, 0.5, 1.0)
    sel.update(20.0, 0.9, 1.5)
    hold(sel, 20.0, 0.9, seconds=3, start=2.0)
    assert sel.profile == PROFILE_FULL


def test_a_lost_link_goes_straight_to_the_beacon_profile():
    sel = ProfileSelector(T)
    hold(sel, float("inf"), 0.0, seconds=3)
    assert sel.profile == PROFILE_MINIMAL


def test_a_flickering_link_still_climbs_out_of_minimal():
    """Wandering between two profiles that are both better than the current one
    must not restart the recovery timer forever — that would strand an operator
    on a beacon-grade feed with a usable link."""
    sel = ProfileSelector(T, profile=PROFILE_MINIMAL)
    t = 0.0
    for i in range(40):
        t += 0.5
        # Alternates between full-grade and reduced-grade measurements.
        sel.update((20.0 if i % 2 else 300.0), 0.9, t)
    assert sel.profile == PROFILE_REDUCED, "takes the more conservative of the two"


def test_a_manual_profile_ignores_every_measurement():
    sel = ProfileSelector(T)
    sel.force(PROFILE_MINIMAL)
    hold(sel, 5.0, 1.0, seconds=60)
    assert sel.profile == PROFILE_MINIMAL
    assert sel.manual
    assert "manually" in sel.reason


def test_releasing_restores_automatic_selection():
    sel = ProfileSelector(T)
    sel.force(PROFILE_MINIMAL)
    sel.release()
    hold(sel, 20.0, 0.9, seconds=20)
    assert sel.profile == PROFILE_FULL
    assert not sel.manual


def test_an_unknown_profile_is_rejected():
    with pytest.raises(ValueError):
        ProfileSelector().force("turbo")


def test_the_reason_is_a_measurement_not_a_bearer_name():
    """A 'WiFi' link at 300 m with 40% loss is a 4G link as far as the GUI is
    concerned. What the operator's laptop experiences is what counts."""
    sel = ProfileSelector(T)
    hold(sel, 300.0, 0.5, seconds=3)
    assert "round trip" in sel.reason
