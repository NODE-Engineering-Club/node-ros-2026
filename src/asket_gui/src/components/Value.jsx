import { age as formatAge } from '../lib/format.js';
import { MISSING, staleness, thresholdsForRate } from '../lib/staleness.js';

/**
 * A live value with its age.
 *
 * Safety rule 7 in one component: nothing on this screen shows a number
 * without saying how old it is. Values past the threshold are degraded
 * loudly — struck through and red — because an operator glancing at a laptop
 * in sunlight will not notice a slightly different shade of grey.
 *
 * `showAge={false}` suppresses the label on values that appear in a group with
 * one shared age, not on values that have no age.
 */
export function Value({ children, ageMs, rateHz, showAge = true, thresholds }) {
  const [staleMs, veryStaleMs] = thresholds || thresholdsForRate(rateHz);
  const level = staleness(ageMs, staleMs, veryStaleMs);
  return (
    <span className={`value ${level}`}>
      <span className="value-text">{level === MISSING ? '—' : children}</span>
      {showAge && level !== 'fresh' && <span className="age">{formatAge(ageMs)}</span>}
    </span>
  );
}

/** A whole panel's shared age, shown once in the header. */
export function PanelAge({ ageMs, rateHz }) {
  const [staleMs, veryStaleMs] = thresholdsForRate(rateHz);
  const level = staleness(ageMs, staleMs, veryStaleMs);
  if (level === MISSING) return <span className="age missing">no data</span>;
  return <span className={`age ${level === 'fresh' ? '' : level}`}>{formatAge(ageMs)}</span>;
}
