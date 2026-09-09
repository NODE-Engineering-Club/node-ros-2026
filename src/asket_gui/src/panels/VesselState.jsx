import { Panel, Row, Rows, Chip } from '../components/Panel.jsx';
import { PanelAge, Value } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { coordinate, int, num } from '../lib/format.js';
import { armingWindow, hullSummary } from '../lib/hull.js';
import { MODE_LABELS } from '../lib/labels.js';

/**
 * What the vessel is actually doing.
 *
 * Every value here comes from the Pico or from the GNSS, and every one carries
 * its age. Nothing in this panel ever reflects a command that was sent
 * (safety rule 4) — the mode shown is the mode the Pico reports, full stop.
 *
 * **Heading is deliberately not here.** It has its own panel, and it used to
 * appear in both — sampled from two streams at two rates, so the two rows
 * disagreed by a degree and an operator had no way to tell which to believe.
 * A value shown in two places is a value that will eventually contradict
 * itself. One source, one place.
 */
export function VesselState({ state }) {
  const vessel = streamPayload(state, 'vessel');
  const pico = streamPayload(state, 'pico');
  const vesselAge = streamAgeMs(state, 'vessel');
  const picoAge = streamAgeMs(state, 'pico');
  const vesselRate = state.subscriptions.vessel?.rate_hz;
  const picoRate = state.subscriptions.pico?.rate_hz;

  const mode = pico?.mode ?? 'UNKNOWN';
  const rcMode = pico?.rc_mode ?? null;
  const hull = hullSummary(pico);

  // The firmware sitting below what Ch8 selects means a software request is
  // holding the vessel down. That is not a fault, and an operator who cannot
  // see the difference will hunt for one.
  const clamped = pico?.software_clamp_active === true;
  const arming = armingWindow(pico, pico?.relay_closed_ms_ago);

  // Anything in the folded half that is off-nominal pulls the whole half open.
  // The RC link and the killswitch are the sovereign path: they are folded away
  // only while they are healthy.
  const forced = pico?.estop_latched
    ? 'propulsion is cut and latched'
    : pico?.hardware_killswitch_engaged
      ? 'the hardware killswitch is engaged'
      : pico?.rc_link_ok === false
        ? 'the RC link is lost'
        : clamped
          ? 'a software request is holding the vessel below the transmitter'
          : hull.alert
            ? hull.alert
            : vessel?.num_sats !== undefined && vessel.num_sats < 6
              ? `only ${vessel.num_sats} satellites`
              : '';

  return (
    <Panel
      title="Vessel"
      id="vessel"
      collapsible
      forceOpen={Boolean(forced)}
      forceReason={forced}
      aside={<PanelAge ageMs={vesselAge} rateHz={vesselRate} />}
      summary={
        <>
          <div style={{ display: 'flex', alignItems: 'center', gap: 10, marginBottom: 8 }}>
            <Value ageMs={picoAge} rateHz={picoRate}>
              <span className={`mode-badge mode-${mode}`}>{MODE_LABELS[mode] || mode}</span>
            </Value>
            {/* What the transmitter is selecting, shown only when it differs
                from what the vessel is actually in. Showing it always would be
                two mode readouts side by side agreeing, which teaches the crew
                to stop reading either. */}
            {rcMode && rcMode !== mode && (
              <Chip
                level="warn"
                title={
                  'The transmitter (channel 8) selects ' +
                  (MODE_LABELS[rcMode] || rcMode) +
                  '. The vessel is more restrictive than that because software ' +
                  'is requesting it. Software can only ever restrict, never extend.'
                }
              >
                Ch8 wants {MODE_LABELS[rcMode] || rcMode}
              </Chip>
            )}
            <div className="spacer" />
            {/* Disarmed is the correct state on a slipway, so it stays green
                and Armed is the one that warns. What changed here is the title:
                a disarmed vessel now says *why*, because with mode commands
                reaching the firmware the operator will otherwise read a
                confirmed AUTONOMOUS and a motionless boat as a fault.

                `armed` is three-state. Null is "not sent" and must never render
                as "Disarmed" — that would be a claim about the vessel nobody
                made. */}
            <Chip
              level={pico?.armed === undefined || pico?.armed === null
                ? ''
                : pico.armed ? 'warn' : 'ok'}
              title={pico?.arming_block || ''}
            >
              {pico?.armed === undefined || pico?.armed === null
                ? 'Arming not sent'
                : pico.armed ? 'Armed' : 'Disarmed'}
            </Chip>
          </div>
          <Rows>
            <Row label="Position">
              <Value ageMs={vesselAge} rateHz={vesselRate} showAge={false}>
                <span className="mono">{coordinate(vessel?.lat, vessel?.lon)}</span>
              </Value>
            </Row>
            <Row label="Speed">
              <Value ageMs={vesselAge} rateHz={vesselRate} showAge={false}>
                {num(vessel?.sog_ms, 2, ' m/s')}
              </Value>
            </Row>
          </Rows>
        </>
      }
    >
      <Rows>
        {/* Hull attitude from the IMU. The sonar panel shows the
            *transducer's* attitude, which is a different sensor and is
            labelled as such — two rows reading "roll / pitch" with different
            numbers is an inconsistency an operator cannot resolve. */}
        <Row label="Roll / pitch">
          <Value ageMs={vesselAge} rateHz={vesselRate} showAge={false}>
            {num(vessel?.roll_deg, 1, '°')} / {num(vessel?.pitch_deg, 1, '°')}
          </Value>
        </Row>
        <Row label="GNSS">
          <Value ageMs={vesselAge} rateHz={vesselRate} showAge={false}>
            {int(vessel?.num_sats)} sats, HDOP {num(vessel?.hdop, 1)}
          </Value>
        </Row>
      </Rows>

      <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', marginTop: 8 }}>
        {/* RC link and channel 8 are shown because they are the sovereign
            path. Software can observe them; it can never move them.

            Both are absent by design on the reduced and minimal profiles, and
            "not sent" must never render as "lost" — reporting a healthy RC
            link as lost because the shore link is narrow is precisely the kind
            of lie this GUI exists to avoid. */}
        <Chip level={rcLevel(pico?.rc_link_ok)} title={rcTitle(pico?.rc_link_ok)}>
          RC {pico?.rc_link_ok === undefined ? 'not sent' : pico.rc_link_ok ? 'linked' : 'lost'}
        </Chip>
        <Chip
          level={picoChannelLevel(pico?.rc_channel8_raw_pct)}
          title="RC channel 8 cuts propulsion in hardware. Nothing in this GUI can move it."
        >
          Ch8 {int(pico?.rc_channel8_raw_pct, '%')}
        </Chip>
        {/* The latch is the one an operator most needs told about. A latched
            vessel that will not re-arm looks broken, and somebody who does not
            know it is latched will go looking for a loose wire. Say what clears
            it, on the chip itself. */}
        {pico?.estop_latched && (
          <Chip
            level="alarm"
            title="Latched. Re-arming is blocked until the operator cycles the arm switch (Ch7) or selects ESTOP on channel 8. This is deliberate: the latch is cleared by a physical act, never by software."
          >
            Propulsion cut — latched
          </Chip>
        )}
        {pico?.hardware_killswitch_engaged && <Chip level="alarm">Killswitch engaged</Chip>}
        {clamped && (
          <Chip
            level="warn"
            title="A software mode request is holding the vessel more restrictive than channel 8 alone would. Stop refreshing the request and it expires, handing authority back to the transmitter."
          >
            Software clamp
          </Chip>
        )}
        {arming && (
          <Chip
            level="warn"
            title="The relay has just closed. The firmware holds both thrusters at neutral for 2 s so the ESCs can boot. The vessel is not unresponsive; it is arming."
          >
            {arming.text}
          </Chip>
        )}
      </div>

      {/* A fault is never hidden behind a disclosure triangle. Anything
          off-nominal is promoted out of the collapse; the rest stays folded
          away, because relay positions are debugging detail on a normal day. */}
      {hull.alert && (
        <p className="errline" style={{ marginBottom: 0, marginTop: 8 }}>
          {hull.alert}
        </p>
      )}

      {hull.relays.length > 0 && (
        <details className="advanced">
          <summary>Advanced — {hull.summary}</summary>
          <Rows>
            {hull.relays.map((r) => (
              <Row key={`r${r.index}`} label={r.name}>
                <span className={r.closed ? '' : 'dim-value'}>{r.text}</span>
              </Row>
            ))}
            {hull.escs.map((e) => (
              <Row key={`e${e.index}`} label={e.name}>
                <span
                  className={e.ok ? '' : e.unknown ? 'warnline' : 'errline'}
                  title={
                    e.unknown
                      ? 'This status code is hull-specific and not documented here (open question Q7). It is reported, not interpreted.'
                      : ''
                  }
                >
                  {e.text}
                </span>
                <span className="raw-code"> ({e.code})</span>
              </Row>
            ))}
          </Rows>
          <p className="hint" style={{ marginBottom: 0 }}>
            Which load each relay drives is not confirmed here — see
            docs/open_questions.md Q7. The states are real; the names are not
            claimed.
          </p>
        </details>
      )}
    </Panel>
  );
}

function picoChannelLevel(pct) {
  // No level at all when the value was not sent: an uncoloured chip reads as
  // "unknown", which is what it is. Green would claim it is fine and red would
  // claim it is not, and neither is known.
  if (pct === null || pct === undefined) return '';
  return pct < 25 ? 'alarm' : 'ok';
}

function rcLevel(ok) {
  if (ok === undefined || ok === null) return '';
  return ok ? 'ok' : 'alarm';
}

function rcTitle(ok) {
  if (ok === undefined || ok === null) {
    return 'RC link state is not carried on the current link profile. This says nothing about the RC link itself.';
  }
  if (ok === false) {
    // The two link losses are different events with different consequences, and
    // an operator told only "link lost" cannot tell which one just happened.
    return (
      'RC link lost. Outside AUTONOMOUS this latches an e-stop after 500 ms: ' +
      'ESC power is cut and re-arming is blocked until the arm switch is cycled. ' +
      'This is not the same as losing the shore link to this GUI, which holds ' +
      'the thrusters at neutral without cutting power.'
    );
  }
  return 'RC channel 8 cuts propulsion in hardware. Nothing in this GUI can move it.';
}
