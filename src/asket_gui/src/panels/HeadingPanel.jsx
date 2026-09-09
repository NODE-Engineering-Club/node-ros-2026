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
 * It is also the *only* place heading is displayed. It used to appear here and
 * in the vessel panel, from two streams sampled at two rates, so the two rows
 * disagreed by a degree — and an operator seeing that has no way to know which
 * one to trust, so they stop trusting both.
 *
 * The row that earns its place is **heading vs course over ground**. In a
 * straight line on calm water they should agree; a persistent divergence is
 * either a strong current or a bad heading, and either way the operator wants
 * to know before the survey is finished rather than after.
 */
export function HeadingPanel({ state }) {
  const full = streamPayload(state, 'heading');
  const vessel = streamPayload(state, 'vessel');

  // On the beacon profile the heading stream is not carried at all, but the
  // vessel stream still carries a heading. Falling back to it keeps the number
  // on screen when the link is at its worst — which is when knowing which way
  // the boat is pointing matters most — and the panel says where it came from,
  // because a bearing with no accuracy figure is worth much less than one with.
  const fallback = !full && vessel?.heading_deg !== undefined;
  const heading = full ?? (fallback
    ? { heading_deg: vessel.heading_deg, valid: vessel.heading_valid, source: vessel.heading_source }
    : null);

  const stream = full ? 'heading' : 'vessel';
  const ageMs = streamAgeMs(state, stream);
  const rateHz = state.subscriptions[stream]?.rate_hz;

  const valid = heading?.valid;
  const divergence = heading?.divergence_deg;

  // Held open whenever the number cannot be trusted or cannot be checked: an
  // invalid heading, a heading that disagrees with the course, or a heading
  // arriving without the accuracy figure that says how much it is worth.
  const forced = valid === false
    ? 'heading is invalid'
    : heading?.divergence_suspicious
      ? 'heading and course over ground disagree'
      : fallback
        ? 'reduced heading — no accuracy figure on this link profile'
        : '';

  return (
    <Panel
      title="Heading"
      id="heading"
      collapsible
      forceOpen={Boolean(forced)}
      forceReason={forced}
      aside={<PanelAge ageMs={ageMs} rateHz={rateHz} />}
      summary={
        <>
          <div className="big">
            <Value ageMs={ageMs} rateHz={rateHz} showAge={false}>
              {valid ? bearing(heading?.heading_deg) : <span className="errline">invalid</span>}
            </Value>
          </div>
          <div style={{ display: 'flex', gap: 6, flexWrap: 'wrap', marginTop: 4 }}>
            <Chip
              level={heading?.source === 'magnetometer' ? 'warn' : valid ? 'ok' : 'alarm'}
              title="A dual-antenna GNSS compass would be about 0.2°."
            >
              {HEADING_SOURCE_LABELS[heading?.source] || heading?.source || 'no source'}
            </Chip>
          </div>
        </>
      }
    >
      <Rows>
        <Row label="Accuracy">
          {/* A figure the receiver reported and one we assumed for its class of
              hardware are worth very different amounts. A UM982 that has lost
              an antenna reports a degraded accuracy; the nominal 0.2° would
              hide precisely that. So the two never look the same. */}
          {heading?.accuracy_deg !== null && heading?.accuracy_deg !== undefined
            ? (
              <>
                {`± ${num(heading.accuracy_deg, 1, '°')}`}
                {heading.accuracy_reported === false ? (
                  <span className="hint" title="The receiver did not report an accuracy. This is the typical figure for this source, not a measurement of this one."> assumed</span>
                ) : null}
              </>
            )
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
      {fallback && (
        <p className="warnline" style={{ marginBottom: 0 }}>
          From the vessel stream — the heading stream is not carried on this link
          profile, so there is no accuracy figure and no comparison with course.
        </p>
      )}
    </Panel>
  );
}
