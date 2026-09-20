/**
 * Decoding the Pico's relay and ESC bytes into something a club member can read
 * at 06:00 on a beach.
 *
 * `Relays · · · ·` and `ESCs e1 e1` were the raw wire values. They are precise
 * and completely opaque: somebody who did not write the firmware cannot tell
 * whether `e1` is normal or catastrophic, which makes them worse than useless
 * on site.
 *
 * What is known and what is not
 * -----------------------------
 *
 * The **states** are known: a relay is closed or open, an ESC reports zero or
 * a non-zero code. Those are decoded below.
 *
 * **Which load each relay drives is not**, and neither is what each non-zero
 * ESC code means — both are hull-specific and live in `pico_bridge`, which this
 * repository must not modify (docs/open_questions.md Q7). So nothing here
 * invents a name. `RELAY_LABELS` is empty on purpose: fill it in once somebody
 * reads the wiring, and the panel starts saying "Bilge pump" instead of
 * "Relay 3" with no other change.
 */

//: PROVISIONAL (Q7): empty until the hull wiring is confirmed. Index -> name.
//: e.g. { 0: 'Main power', 1: 'Thrusters' }
export const RELAY_LABELS = {};

//: PROVISIONAL (Q7): ESC status codes beyond 0 are hull-specific. 0 is the
//: only value this repository can claim to know the meaning of.
export const ESC_STATUS_LABELS = { 0: 'running' };

export function relayName(index) {
  return RELAY_LABELS[index] || `Relay ${index + 1}`;
}

/** `{ name, closed, text }` per relay. */
export function decodeRelays(states) {
  return (states || []).map((closed, i) => ({
    index: i,
    name: relayName(i),
    closed: Boolean(closed),
    text: closed ? 'closed' : 'open',
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

  return {
    relays,
    escs,
    summary: relays.length
      ? `${closed}/${relays.length} relays closed, ${escs.length - notRunning.length}/${escs.length} thrusters running`
      : '',
    alert: armed && notRunning.length > 0
      ? `${notRunning.length} of ${escs.length} thrusters not running while armed`
      : null,
  };
}
