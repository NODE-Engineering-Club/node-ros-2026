import { Panel, Row, Rows, Chip } from '../components/Panel.jsx';
import { PanelAge, Value } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { bearing, coordinate, int, num } from '../lib/format.js';
import { MODE_LABELS } from '../lib/labels.js';

/**
 * What the vessel is actually doing.
 *
 * Every value here comes from the Pico or from the GNSS, and every one carries
 * its age. Nothing in this panel ever reflects a command that was sent
 * (safety rule 4) — the mode shown is the mode the Pico reports, full stop.
 */
export function VesselState({ state }) {
  const vessel = streamPayload(state, 'vessel');
  const pico = streamPayload(state, 'pico');
  const vesselAge = streamAgeMs(state, 'vessel');
  const picoAge = streamAgeMs(state, 'pico');
  const vesselRate = state.subscriptions.vessel?.rate_hz;
  const picoRate = state.subscriptions.pico?.rate_hz;

  const mode = pico?.mode ?? 'UNKNOWN';

  return (
    <Panel title="Vessel" aside={<PanelAge ageMs={vesselAge} rateHz={vesselRate} />}>
      <div style={{ display: 'flex', alignItems: 'center', gap: 10, marginBottom: 10 }}>
        <Value ageMs={picoAge} rateHz={picoRate}>
          <span className={`mode-badge mode-${mode}`}>{MODE_LABELS[mode] || mode}</span>
        </Value>
        <div className="spacer" />
        <Chip level={pico?.armed ? 'warn' : 'ok'}>{pico?.armed ? 'Armed' : 'Disarmed'}</Chip>
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
        <Row label="Heading">
          <Value ageMs={vesselAge} rateHz={vesselRate} showAge={false}>
            {vessel?.heading_valid === false ? (
              <span className="errline">invalid</span>
            ) : (
              bearing(vessel?.heading_deg)
            )}
          </Value>
        </Row>
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

      <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', marginTop: 10 }}>
        {/* RC link and channel 8 are shown because they are the sovereign
            path. Software can observe them; it can never move them. */}
        <Chip level={pico?.rc_link_ok ? 'ok' : 'alarm'}>
          RC {pico?.rc_link_ok ? 'linked' : 'lost'}
        </Chip>
        <Chip
          level={picoChannelLevel(pico?.rc_channel8_raw_pct)}
          title="RC channel 8 cuts propulsion in hardware. Nothing in this GUI can move it."
        >
          Ch8 {int(pico?.rc_channel8_raw_pct, '%')}
        </Chip>
        {pico?.estop_latched && <Chip level="alarm">Propulsion cut</Chip>}
        {pico?.hardware_killswitch_engaged && <Chip level="alarm">Killswitch engaged</Chip>}
      </div>

      {pico?.relay_states?.length > 0 && (
        <Rows>
          <Row label="Relays">
            <span className="mono">
              {pico.relay_states.map((on, i) => (on ? `${i + 1}` : '·')).join(' ')}
            </span>
          </Row>
          <Row label="ESCs">
            <span className="mono">
              {pico.esc_status.map((s) => (s === 0 ? 'ok' : `e${s}`)).join(' ')}
            </span>
          </Row>
        </Rows>
      )}
    </Panel>
  );
}

function picoChannelLevel(pct) {
  if (pct === null || pct === undefined) return '';
  return pct < 25 ? 'alarm' : 'ok';
}
