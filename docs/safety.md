# Safety rules

These are non-negotiable and constrain every design decision in this repository.
They are reproduced here so nobody has to go looking for the original brief.

1. **The hardware killswitch and RC channel 8 are sovereign.** They cut
   propulsion independently of all software. Nothing in this GUI can override,
   delay or interfere with them.

2. **The GUI is never in the safety chain.** If the Jetson, ROS 2, the network
   or the browser fails, safety is unaffected. No code here may be the only
   thing standing between a fault and a stopped motor.

3. **Software ESTOP is a soft latch only.** In the UI it is labelled
   **"Cut propulsion"** — never "Emergency stop". The label matters: no operator
   should ever come to rely on it as a true emergency stop.
   Enforced by a test: `src/asket_gui/src/panels/ModeCommands.test-notes.md` and
   the label constant in `src/asket_gui/src/lib/labels.js`.

4. **Displayed state is always confirmed state.** The GUI shows what the Pico
   reports the vessel is actually doing, never what was requested. A pending
   command renders distinctly from a confirmed one.

5. **Mode commands require two-step confirmation** and must show Pico-confirmed
   feedback before the displayed mode changes.

6. **Motor tests never run automatically.** Two-step confirmation, an explicit
   operator acknowledgement that the vessel is out of the water or securely
   moored, and a brief low-power pulse only.

7. **Data age is always visible.** Every live value carries its age. Values
   older than a configurable threshold are visually degraded. On an intermittent
   link an operator must never mistake a stale position for a current one.

## How the code enforces these

| Rule | Mechanism |
|---|---|
| 3 | The string "Emergency stop" appears nowhere in the frontend; `labels.js` owns the wording and `gui_backend` names the command `cut_propulsion`. |
| 4, 5 | `gui_backend/core/commands.py` returns `pending` on send and only `confirmed` once the *status stream* reports the new mode. Timeout produces an explicit `failed`. |
| 7 | Every WebSocket data frame carries `source_utc_ms` (when the value was produced) alongside `server_utc_ms` (when it was sent). The client renders age from the former, and estimates clock skew from ping/pong so a wrong laptop clock cannot hide staleness. |
