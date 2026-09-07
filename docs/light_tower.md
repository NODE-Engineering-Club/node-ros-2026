# Light tower colour codes

The light tower is the **only** visual feedback available from shore. Somebody
standing on a Namibian beach with no laptop must be able to read the vessel's
state from it.

Codes are defined in `src/asket_bringup/config/light_tower.yaml` so they can be
changed without touching code, and are surfaced in the GUI (Vessel state panel)
so an operator can confirm what the beach is seeing.

## Priority

Only one pattern shows at a time. Highest priority wins.

| Priority | Condition | Pattern |
|---|---|---|
| 1 | Propulsion cut (ESTOP latched) | **Red, solid** |
| 2 | Link lost (> 10 s without a GUI heartbeat) while autonomous | **Red, fast blink** (2 Hz) |
| 3 | Autonomous | **Blue, solid** |
| 4 | Manual | **Amber, solid** |
| 5 | Idle / disarmed | **Amber, slow blink** (0.5 Hz) |

## Overlay

Recording state is an *overlay* on the above, not a replacement, so mode is
never hidden by it:

| Condition | Overlay |
|---|---|
| Mission recording | **Green, single short flash every 2 s** |
| Recording error (disk full, write failure) | **Green, fast blink** (2 Hz) |

## Reading it from the beach

- Red at all = propulsion is cut. Safe to approach.
- Blue = under its own control. Do not approach.
- Amber = manual or idle.
- A green wink every couple of seconds = it is recording. No green while a
  mission is supposed to be running means the survey is not being logged.

> Driving the tower hardware is `pico_bridge`'s job and is out of scope here.
> This document defines the contract; `system_test` reports the state it
> believes the tower should be showing so it can be compared with reality
> during pre-flight.
