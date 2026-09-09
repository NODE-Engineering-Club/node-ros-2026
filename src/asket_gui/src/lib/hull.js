/**
 * Decoding the Pico's relay and ESC state into something a club member can read
 * at 06:00 on a beach.
 *
 * `Relays · · · ·` and `ESCs e1 e1` were the raw wire values. They are precise
 * and completely opaque: somebody who did not write the firmware cannot tell
 * whether `e1` is normal or catastrophic, which makes them worse than useless
 * on site.
 *
 * There is one relay, and it is named
 * -----------------------------------
 *
 * The panel used to read `0/4 relays closed`. There is no fourth relay and there
 * never was: the `4` came from `num_relays` in the *simulator's* placeholder
 * config, and the GUI rendered whatever length of array arrived. Firmware v3 has
 * exactly one, `ESTOP_RELAY_PIN` on GPIO21, and it cuts ESC power.
 *
 * That relay is now named, because unlike the hull wiring its function is
 * confirmed in the firmware source. `RELAY_LABELS` is still the place to add
 * more if the hardware ever grows them.
 *
 * Two thrusters, on GPIO15 and GPIO16, bidirectional 1000/1500/2000 µs.
 *
 * What is still not known
 * -----------------------
 *
 * ESC status codes beyond 0. The firmware does not report an ESC code at all
 * over serial — `esc_status` arrives only from `asket_sim`. Nothing here invents
 * a meaning for a non-zero code.
 */

//: GPIO21. Confirmed in pico-node_v3.ino: HIGH = ESC power on, LOW = power cut.
export const RELAY_LABELS = { 0: 'ESC power' };

//: What the single relay does, for the operator who has to decide whether an
//: open relay is a problem right now.
export const RELAY_DESCRIPTIONS = {
  0: 'Cuts power to both ESCs. Open while disarmed, which is correct.',
};

//: ESC status codes beyond 0 are hull-specific. 0 is the only value this
//: repository can claim to know the meaning of.
export const ESC_STATUS_LABELS = { 0: 'running' };

//: The ESC arming window: the relay closes, then the firmware holds both
//: thrusters at neutral for this long before any setpoint is applied.
//: `ESC_ARM_DELAY_MS` in the firmware. Shown so the GUI does not look
//: unresponsive for two seconds after arming.
export const ESC_ARM_DELAY_MS = 2000;

export function relayName(index) {
  return RELAY_LABELS[index] || `Relay ${index + 1}`;
}

/** `{ name, closed, text, description }` per relay. */
export function decodeRelays(states) {
  return (states || []).map((closed, i) => ({
    index: i,
    name: relayName(i),
    closed: Boolean(closed),
    text: closed ? 'closed' : 'open',
    description: RELAY_DESCRIPTIONS[i] || '',
  }));
}

/**
 * `{ name, code, ok, text, unknown }` per ESC.
 *
 * `unknown` marks a code we cannot interpret. It renders as a warning rather
 * than an alarm: an uninterpretable code is not evidence of a fault, and
 * calling it one would send somebody looking for a problem that may not exist.
 */
export function decodeEscs(codes) {
  return (codes || []).map((code, i) => {
    const known = ESC_STATUS_LABELS[code];
    return {
      index: i,
      name: `Thruster ${i + 1}`,
      code,
      ok: code === 0,
      unknown: known === undefined,
      text: known || `code ${code}`,
    };
  });
}

/**
 * Whether the vessel is inside the ESC arming window.
 *
 * During it the thrusters are held at neutral no matter what is commanded. An
 * operator who does not know that reads two seconds of no response as a fault.
 */
export function armingWindow(pico, sinceRelayClosedMs) {
  if (pico?.armed !== true) return null;
  if (sinceRelayClosedMs === null || sinceRelayClosedMs === undefined) return null;
  if (sinceRelayClosedMs >= ESC_ARM_DELAY_MS) return null;
  return {
    remainingMs: Math.max(0, ESC_ARM_DELAY_MS - sinceRelayClosedMs),
    text: 'ESCs arming — thrusters held at neutral',
  };
}

/**
 * One line for the collapsed summary, and whether anything needs promoting out
 * of it.
 *
 * A fault must never be hidden behind a disclosure triangle, so the panel shows
 * a chip outside the collapse whenever an ESC is not running **while the vessel
 * is armed**. Disarmed, an ESC that is not running is the correct state and
 * saying so in red would train everyone to ignore it.
 */
export function hullSummary(pico) {
  const relays = decodeRelays(pico?.relay_states);
  const escs = decodeEscs(pico?.esc_status);
  const closed = relays.filter((r) => r.closed).length;
  const notRunning = escs.filter((e) => !e.ok);
  const armed = pico?.armed === true;

  // Singular when there is one, because "1/1 relays closed" reads as a count of
  // something that ought to have more members.
  const relayText = relays.length === 1
    ? `${relays[0].name} ${relays[0].text}`
    : `${closed}/${relays.length} relays closed`;

  const parts = [];
  if (relays.length) parts.push(relayText);
  if (escs.length) {
    parts.push(`${escs.length - notRunning.length}/${escs.length} thrusters running`);
  }

  return {
    relays,
    escs,
    summary: parts.join(', '),
    alert: armed && notRunning.length > 0
      ? `${notRunning.length} of ${escs.length} thrusters not running while armed`
      : null,
  };
}
