import { Panel, Row, Rows, Chip } from '../components/Panel.jsx';
import { streamPayload } from '../lib/connection.js';
import { bytes, int, num } from '../lib/format.js';
import { LINK_LABELS, PROFILE_LABELS } from '../lib/labels.js';

/**
 * The shore link, always visible.
 *
 * Shows what is actually being carried and, when a subscription has been cut
 * back, why. An operator who cannot tell "nothing is happening" from "I am not
 * being sent it" will eventually act on the wrong one.
 */
export function LinkStatus({ state, connection }) {
  const link = streamPayload(state, 'link');
  const profile = state.profile || {};
  const degraded = Object.values(state.subscriptions).filter((s) => s.reason);

  const quality = link?.quality;
  const level =
    !state.connected || link?.active_link === 'none'
      ? 'alarm'
      : quality !== undefined && quality < 0.35
        ? 'warn'
        : 'ok';

  return (
    <Panel
      title="Link"
      aside={
        <Chip level={level}>
          {state.connected ? LINK_LABELS[link?.active_link] || 'connected' : 'disconnected'}
        </Chip>
      }
    >
      <Rows>
        <Row label="Quality">{quality === undefined ? '—' : num(quality * 100, 0, '%')}</Row>
        <Row label="Latency">{num(link?.rtt_ms, 0, ' ms')}</Row>
        <Row label="Carrying">{bytes(link?.rate_bytes_per_s)}/s</Row>
        <Row label="This client">≈ {bytes(state.estimatedBytesPerS)}/s</Row>
        <Row label="Detail level">
          {PROFILE_LABELS[profile.profile] || profile.profile}
          {profile.manual ? ' (manual)' : ''}
        </Row>
      </Rows>

      <p className="hint" style={{ marginTop: 6, marginBottom: 6 }}>
        {profile.reason}
      </p>

      <div className="button-row">
        {['auto', 'full', 'reduced', 'minimal'].map((option) => (
          <button
            key={option}
            onClick={() => connection.setProfile(option)}
            disabled={
              option === 'auto' ? !profile.manual : profile.manual && profile.profile === option
            }
          >
            {option === 'auto' ? 'Auto' : PROFILE_LABELS[option]}
          </button>
        ))}
      </div>

      {degraded.length > 0 && (
        <details className="debug" style={{ marginTop: 8 }}>
          <summary>{degraded.length} stream(s) cut back by the link</summary>
          <Rows>
            {degraded.map((s) => (
              <Row key={s.name} label={s.name}>
                <span className="hint">
                  {s.granted ? `${num(s.rate_hz, 1)} Hz — ` : 'not sent — '}
                  {s.reason}
                </span>
              </Row>
            ))}
          </Rows>
        </details>
      )}

      {!state.connected && (
        <p className="errline" style={{ marginBottom: 0 }}>
          No connection to the vessel. Everything on screen is the last value
          received{state.attempts ? `; retrying (attempt ${state.attempts})` : ''}.
        </p>
      )}
    </Panel>
  );
}
