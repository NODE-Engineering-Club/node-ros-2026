# system_test

Pre-flight built-in test.

**Purpose, stated plainly: when something is wrong on a Namibian beach, identify
the faulty link in thirty seconds rather than an hour.** Everything here follows
from that.

## What it does

Builds on the ROS 2 standard `diagnostic_msgs/DiagnosticArray` — every node
publishes its own health continuously — and adds the checks that need a view
across the whole system: is the sonar's clock disciplined, is there enough disk
and is it fast enough, does the heading agree with the course over ground.

Runs **automatically at boot**, after a delay so everything else has come up, and
publishes a latched report. A client connecting later still gets the boot-time
verdict; nobody is aboard the vessel, and an operator on the beach should not
have to ask for the first answer.

## The three rules the messages follow

1. **Plain language, with a remedy.** Not `GPS: ERROR` but
   *"4 satellites, 6 required — wait, or move the vessel clear of buildings"*.
   A red word is not an instruction.
2. **Name the link, not the symptom.** *"The sonar is on the network but not
   answering"* and *"nothing is answering at the sonar's address"* send you to
   different cables, so they are different messages.
3. **Unknown is not pass.** A check with no data reports `SKIPPED`, and where it
   is something we must be able to confirm, the verdict is **NO-GO**.
   *"GO — 14 checks could not run"* is the most dangerous sentence this panel
   could produce.

## Trends

Every run is timestamped and stored. **Gradual degradation shows up as a trend
before it becomes an outright failure** — a corroding connector does not fail on
the day it fails, it spends a month getting worse while every individual
pre-flight still says GO. Checks that are drifting are flagged in the GUI even
while they pass.

## Active tests

**Motor tests never run automatically** (safety rule 6). The gate is in
`core/motor_test.py` and requires, in order: an explicit request; an operator
acknowledgement typed as an exact phrase — not a checkbox, which can be clicked
by muscle memory, and not a boolean, which can default to true in a config file;
a consent no more than a minute old; the vessel not reporting itself in the
water; and a brief low-power pulse within hard-coded ceilings.

Driving the thrusters is `pico_bridge`'s job and this repository does not modify
it, so the gate is deliberately **not wired to a service**. `~/run` refuses
`include_active` outright, so there is no path through this node that can reach
a thruster. Wiring it up belongs to the day a human is standing next to the boat.

## Testing in sim

```bash
pytest src/system_test
```

Against the simulator, with faults:

```bash
python3 -m gui_backend.core.app --sim --port 8080
# GUI -> Pre-flight -> Run pre-flight, then inject gnss_degraded or clock_drift
```
