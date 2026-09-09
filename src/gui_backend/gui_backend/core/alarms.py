"""Alarms.

**Keep them few.** An alarm that fires often is an alarm an operator learns to
dismiss, and then the one that mattered gets dismissed with it. So the rule
here is narrow: alarm only on conditions that **cannot self-correct** and that
require a human to do something.

Deliberately *not* alarms:

* obstacles — the navigation stack handles avoidance autonomously, so the lidar
  panel informs rather than alerts;
* momentary link quality dips — the profile selector handles those;
* a single late sonar packet — that is what ``packet_loss_ratio`` is for.

Every alarm carries a plain-language message and a remedy, because "CLOCK
DRIFT" on a beach in Namibia costs an hour and "sonar clock 1.2 s off the
Jetson — data recorded now cannot be georeferenced; restart the NTP service"
costs a minute.
"""

from __future__ import annotations

from dataclasses import dataclass

SEVERITY_WARN = "warn"
SEVERITY_ALARM = "alarm"


@dataclass(frozen=True)
class Alarm:
    key: str
    severity: str
    message: str
    remedy: str = ""

    def to_dict(self) -> dict:
        return {
            "key": self.key,
            "severity": self.severity,
            "message": self.message,
            "remedy": self.remedy,
        }


@dataclass
class AlarmThresholds:
    """Mirrors asket_bringup/config/mission_defaults.yaml."""

    battery_low_soc: float = 0.25
    battery_critical_soc: float = 0.12
    disk_low_bytes: int = 5 * 1024**3
    clock_offset_warn_ms: int = 250
    clock_offset_alarm_ms: int = 1000
    roll_warn_deg: float = 20.0
    roll_alarm_deg: float = 30.0
    link_lost_s: float = 10.0
    geofence_approach_m: float = 25.0
    sonar_silent_s: float = 10.0


def evaluate(state: dict, thresholds: AlarmThresholds | None = None) -> list[Alarm]:
    """Turn a state snapshot into the alarms that should currently be showing.

    Stateless on purpose: an alarm is present because the condition is present.
    Latching lives in the client, which is where an acknowledgement belongs.
    """
    t = thresholds or AlarmThresholds()
    out: list[Alarm] = []

    # -- link -------------------------------------------------------------
    link_age_s = state.get("link_age_s")
    if link_age_s is not None and link_age_s > t.link_lost_s:
        out.append(
            Alarm(
                "link_lost",
                SEVERITY_ALARM,
                f"No data from the vessel for {link_age_s:.0f} s",
                "Everything on screen is stale. Move towards the vessel or "
                "switch to the cellular link.",
            )
        )

    # -- power ------------------------------------------------------------
    soc = state.get("state_of_charge")
    if soc is not None:
        if soc <= t.battery_critical_soc:
            out.append(
                Alarm(
                    "battery_critical",
                    SEVERITY_ALARM,
                    f"Battery at {soc * 100:.0f}%",
                    "Return to shore now.",
                )
            )
        elif soc <= t.battery_low_soc:
            out.append(
                Alarm(
                    "battery_low",
                    SEVERITY_WARN,
                    f"Battery at {soc * 100:.0f}%",
                    "Check the remaining survey distance against the endurance "
                    "estimate in the power panel.",
                )
            )

    if state.get("can_finish_survey") is False:
        out.append(
            Alarm(
                "endurance_short",
                SEVERITY_WARN,
                "Not enough charge to finish the planned survey",
                "Shorten the pattern or plan a battery swap.",
            )
        )

    # -- recording --------------------------------------------------------
    disk_free = state.get("disk_free_bytes")
    if state.get("recording") and disk_free is not None and disk_free < t.disk_low_bytes:
        out.append(
            Alarm(
                "disk_low",
                SEVERITY_ALARM,
                f"{disk_free / 1024**3:.1f} GB of disk left while recording",
                "Stop the mission and export, or the recording will truncate.",
            )
        )

    # A recorder that has stopped because the disk filled is no longer
    # "recording", so the alarm above goes quiet at the exact moment it
    # matters most. This one does not depend on the state it is reporting on.
    if state.get("recording_error"):
        out.append(
            Alarm(
                "recording_stopped",
                SEVERITY_ALARM,
                f"Recording stopped: {state['recording_error']}",
                "The survey is no longer being logged. What was recorded is "
                "intact and listed. Free space, then start a new mission.",
            )
        )

    # -- sonar ------------------------------------------------------------
    if state.get("sonar_expected"):
        silent_s = state.get("sonar_seconds_since_data")
        if not state.get("sonar_connected") or (
            silent_s is not None and silent_s > t.sonar_silent_s
        ):
            out.append(
                Alarm(
                    "sonar_unresponsive",
                    SEVERITY_ALARM,
                    "Sonar is not sending data",
                    "Check the sonar's Ethernet cable and power. The survey is "
                    "not being recorded.",
                )
            )

        offset = state.get("clock_offset_ms")
        if offset is not None:
            if abs(offset) >= t.clock_offset_alarm_ms:
                out.append(
                    Alarm(
                        "clock_drift",
                        SEVERITY_ALARM,
                        f"Sonar clock is {offset} ms from the Jetson's",
                        "Data recorded now cannot be georeferenced afterwards. "
                        "Check the NTP server on the Jetson.",
                    )
                )
            elif abs(offset) >= t.clock_offset_warn_ms:
                out.append(
                    Alarm(
                        "clock_drift",
                        SEVERITY_WARN,
                        f"Sonar clock drifting ({offset} ms)",
                        f"Unusable past {t.clock_offset_alarm_ms} ms. Check NTP "
                        "before it gets there.",
                    )
                )

    # -- heading ----------------------------------------------------------
    if state.get("heading_valid") is False:
        out.append(
            Alarm(
                "heading_invalid",
                SEVERITY_ALARM,
                "Heading is invalid",
                "Sonar data recorded during this window is compromised. Note "
                "the time, and re-run these lines.",
            )
        )
    elif state.get("heading_divergence_suspicious"):
        divergence = state.get("heading_divergence_deg")
        out.append(
            Alarm(
                "heading_divergence",
                SEVERITY_WARN,
                f"Heading and course over ground differ by {abs(divergence):.0f} deg",
                "Either a strong cross-current or a bad heading. On calm water "
                "in a straight line they should agree.",
            )
        )

    # -- attitude ---------------------------------------------------------
    roll = state.get("roll_deg")
    if roll is not None:
        if abs(roll) >= t.roll_alarm_deg:
            out.append(
                Alarm(
                    "roll_excessive",
                    SEVERITY_ALARM,
                    f"Roll {abs(roll):.0f} deg",
                    "Sea state is beyond what this hull surveys in. Head for shelter.",
                )
            )
        elif abs(roll) >= t.roll_warn_deg:
            out.append(
                Alarm(
                    "roll_high",
                    SEVERITY_WARN,
                    f"Roll {abs(roll):.0f} deg",
                    "Sonar coverage degrades as roll increases.",
                )
            )

    # -- geofence ---------------------------------------------------------
    fence_m = state.get("geofence_distance_m")
    if fence_m is not None and fence_m < t.geofence_approach_m:
        out.append(
            Alarm(
                "geofence_approach",
                SEVERITY_WARN,
                f"{fence_m:.0f} m from the geofence",
                "The vessel will stop at the boundary. Turn it, or move the fence.",
            )
        )

    # -- RC ---------------------------------------------------------------
    if state.get("rc_link_ok") is False:
        out.append(
            Alarm(
                "rc_link_lost",
                SEVERITY_ALARM,
                "RC link lost",
                "The hardware killswitch is out of range. Command authority "
                "requires an operator within RC range.",
            )
        )

    return out


def diff(previous: list[Alarm], current: list[Alarm]) -> tuple[list[Alarm], list[str]]:
    """``(newly raised, keys newly cleared)``.

    Used to send transitions on the wire rather than the whole set every second,
    which matters on a link with a one-kilobyte budget.
    """
    prev_by_key = {a.key: a for a in previous}
    curr_by_key = {a.key: a for a in current}
    raised = [a for k, a in curr_by_key.items() if prev_by_key.get(k) != a]
    cleared = [k for k in prev_by_key if k not in curr_by_key]
    return raised, cleared
