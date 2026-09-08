import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { duration, pct } from '../lib/format.js';
import { LINK_LABELS, PROFILE_LABELS } from '../lib/labels.js';
import { staleness, thresholdsForRate } from '../lib/staleness.js';

/**
 * Three answers that must never require scrolling.
 *
 * The cockpit column scrolls, and the panels carrying "is there battery left",
 * "am I recording" and "what is the link doing" sat below the fold. Finding out
 * whether there is charge to finish should not cost a scroll on a beach, so
 * those three live in the top bar and stay there whatever else is on screen.
 *
 * It is a *summary*, not a second source. Every value here reads the same
 * payload as the panel that owns it — battery from `power`, recording from
 * `mission`, link from `link` — so the strip and the panel cannot disagree.
 * The panels keep the detail; this keeps the headline.
 */
export function StatusStrip({ state }) {
  const power = streamPayload(state, 'power');
  const mission = streamPayload(state, 'mission');
  const link = streamPayload(state, 'link');
  const profile = state.profile || {};

  const soc = power?.state_of_charge;
  const recording = mission?.state === 'RECORDING';
  const errored = mission?.state === 'ERROR';

  return (
    <div className="status-strip">
      <Item
        label="Battery"
        level={soc === undefined ? '' : soc < 0.12 ? 'alarm' : soc < 0.25 ? 'warn' : 'ok'}
        age={ageLevel(state, 'power')}
        value={soc === undefined ? 'not sent' : pct(soc)}
        detail={power?.endurance_s ? duration(power.endurance_s) : ''}
      />
      <Item
        label="Recording"
        level={errored ? 'alarm' : recording ? 'rec' : ''}
        age={ageLevel(state, 'mission')}
        value={mission?.state ? labelFor(mission.state) : 'not sent'}
        detail={recording && mission?.elapsed_s ? duration(mission.elapsed_s) : ''}
      />
      <Item
        label="Link"
        level={
          !state.connected || link?.active_link === 'none'
            ? 'alarm'
            : profile.profile === 'minimal'
              ? 'warn'
              : 'ok'
        }
        age={ageLevel(state, 'link')}
        value={state.connected ? LINK_LABELS[link?.active_link] || 'connected' : 'disconnected'}
        detail={PROFILE_LABELS[profile.profile] || ''}
      />
    </div>
  );
}

/**
 * A strip value that has gone stale must not read as current. It carries the
 * same staleness treatment as everything else on screen rather than a quieter
 * one just because it lives in the header.
 */
function ageLevel(state, stream) {
  const ageMs = streamAgeMs(state, stream);
  return staleness(ageMs, ...thresholdsForRate(state.subscriptions[stream]?.rate_hz));
}

function Item({ label, value, detail, level, age }) {
  return (
    <div className={`strip-item ${level} ${age}`}>
      <span className="strip-label">{label}</span>
      <span className="strip-value">{value}</span>
      {detail && <span className="strip-detail">{detail}</span>}
    </div>
  );
}

function labelFor(state) {
  return { IDLE: 'Idle', RECORDING: 'Recording', ERROR: 'Error' }[state] || state;
}
