# Light tower colour codes

The light tower is the **only** visual feedback available from shore. Somebody
standing on a Namibian beach with no laptop must be able to read the vessel's
state from it.

> **This document describes what the firmware does.** It used to describe
> something else — blue, blink patterns, and a green recording flash, none of
> which exist — and the discrepancy pointed the dangerous way: green meant
> "recording in progress" here and "autonomous and armed, propellers may start"
> in the firmware. It has been rewritten to match
> `firmware/pico-node_v4/pico-node_v4.ino`, which is the only thing that
> actually drives the lamps.

## The codes

Three outputs, three colours, one lit at a time, always solid. This is the
whole of `set_light()`:

| Colour | Pin | Condition | What it means to someone on the beach |
|---|---|---|---|
| **Red** | `GP12` | Disarmed, **or** ESTOP | Propulsion is cut. Safe to approach. |
| **Yellow** | `GP11` | Armed, MANUAL | The pilot has it. Propellers may turn. |
| **Green** | `GP10` | Armed, AUTONOMOUS | It is driving itself. **Do not approach.** |

Read it as one question — *can the propellers turn, and who is deciding?*

- **Red** → no.
- **Yellow** → yes, and a human is holding the transmitter.
- **Green** → yes, and nobody is.

Red covers two quite different situations — disarmed at rest, and an ESTOP
after something went wrong — because from the beach they call for the same
action, which is that approaching is safe. Telling them apart is what the GUI
is for.

## What the tower does not tell you

**It says nothing about recording.** The Pico has no idea whether a mission is
being logged: that lives on the Jetson and nothing sends it down the serial
link. There is no green wink, and a green light means autonomy, not logging.
Confirm recording in the GUI's Recording panel, which is the only place that
knows.

**It says nothing about the shore link.** A Jetson that has lost Wi-Fi looks
exactly like one that has not. What *is* visible: if the link drops while
autonomous, the Pico stops hearing the heartbeat, revokes autonomy within
600 ms and falls back to MANUAL — so **green turning yellow with nobody
touching the transmitter means the Jetson stopped talking.**

**It does not blink, ever.** If a lamp appears to flicker, that is a loose
connection or a failing lamp, not a code.

## Startup

On power-up the tower cycles red → yellow → green twice, about 150 ms per step,
then settles to red. That sequence is a lamp test: if a colour does not appear
during it, that channel is dead, and you have just found out on the bench
rather than at 200 m.

## If you want more than three states

Adding a fourth condition — "recording", "link lost", "low battery" — means
adding a downlink command so the Jetson can tell the Pico something it does not
otherwise know, and that is a change to the one component that must stay small
enough to read in a sitting. It is not free, and it has not been done.

Blinking as a way to encode more states is a false economy at distance: a
2 Hz blink and a failing connection look identical across 200 m of glare, and
the failure mode is somebody reading "link lost" as "everything is fine".

> Driving the tower is the firmware's job and happens nowhere else. `system_test`
> reports the colour it believes the tower should be showing, so it can be
> compared with reality during pre-flight — which is worth doing once per
> deployment, because a burnt-out lamp is silent.
