import { Panel, Row, Rows, Chip } from '../components/Panel.jsx';
import { PanelAge, Value } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { bearing, num } from '../lib/format.js';
import { HEADING_SOURCE_LABELS } from '../lib/labels.js';

/**
 * Heading, with its provenance.
 *
 * Heading error is the dominant error source in the whole survey — roughly
 * 90 cm of seabed position error per degree at 50 m range — so this gets its
 * own panel rather than a line in the vessel panel.
 *
 * The row that earns its place is **heading vs course over ground**. In a
 * straight line on calm water they should agree; a persistent divergence is
 * either a strong current or a bad heading, and either way the operator wants
 * to know before the survey is finished rather than after.
 */
export function HeadingPanel({ state }) {
  const heading = streamPayload(state, 'heading');
  const ageMs = streamAgeMs(state, 'heading');
  const rateHz = state.subscriptions.heading?.rate_hz;

  const valid = heading?.valid;
  const divergence = heading?.divergence_deg;

  return (
    <Panel
      title="Heading"
      aside={<PanelAge ageMs={ageMs} rateHz={rateHz} />}
    >
      <div className="big">
        <Value ageMs={ageMs} rateHz={rateHz} showAge={false}>
          {valid ? bearing(heading?.heading_deg) : <span className="errline">invalid</span>}
        </Value>
      </div>

      <Rows>
        <Row label="Source">
          {HEADING_SOURCE_LABELS[heading?.source] || heading?.source || '—'}
        </Row>
        <Row label="Accuracy">
          {heading?.accuracy_deg !== null && heading?.accuracy_deg !== undefined
            ? `± ${num(heading.accuracy_deg, 1, '°')}`
            : '—'}
        </Row>
        <Row label="Seabed error at 50 m">
          {/* The number that makes heading matter. */}
          {num(heading?.seabed_error_at_50m_m, 2, ' m')}
        </Row>
        <Row label="Course over ground">{bearing(heading?.cog_deg)}</Row>
        <Row label="Heading − COG">
          {heading?.divergence_meaningful ? (
            <span className={heading.divergence_suspicious ? 'warnline' : ''}>
              {num(divergence, 1, '°')}
            </span>
          ) : (
            <span className="hint">too slow to compare</span>
          )}
        </Row>
      </Rows>

      {heading?.divergence_suspicious && (
        <p className="warnline" style={{ marginBottom: 0 }}>
          Heading and course disagree. On calm water in a straight line they should
          match — check for a cross-current, or a bad heading.
        </p>
      )}
      {valid === false && (
        <p className="errline" style={{ marginBottom: 0 }}>
          Sonar data recorded while heading is invalid is compromised. Note the time
          and re-run these lines.
        </p>
      )}
      {heading?.source === 'magnetometer' && (
        <div style={{ marginTop: 8 }}>
          <Chip level="warn" title="A dual-antenna GNSS compass would be about 0.2°.">
            Magnetometer heading
          </Chip>
        </div>
      )}
    </Panel>
  );
}
