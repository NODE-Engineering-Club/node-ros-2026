// Data age thresholds and how they render.
//
// Safety rule 7: every live value carries its age, and values older than a
// threshold are visually degraded. On an intermittent link an operator must
// never mistake a stale position for a current one.
//
// Degradation is deliberately not subtle. "Slightly greyer" is not a signal a
// person notices in bright sunlight on a beach; a struck-through value with an
// explicit age is.

export const FRESH = 'fresh';
export const STALE = 'stale';
export const VERY_STALE = 'very-stale';
export const MISSING = 'missing';

// Defaults; overridden from the server's alarm thresholds where it sends them.
export const DEFAULT_STALE_MS = 3000;
export const DEFAULT_VERY_STALE_MS = 10000;

export function staleness(ageMs, staleMs = DEFAULT_STALE_MS, veryStaleMs = DEFAULT_VERY_STALE_MS) {
  if (ageMs === null || ageMs === undefined) return MISSING;
  if (ageMs >= veryStaleMs) return VERY_STALE;
  if (ageMs >= staleMs) return STALE;
  return FRESH;
}

// A stream's rate determines what "stale" means for it. A 0.2 Hz stream on the
// beacon profile is not stale at four seconds old — expecting otherwise would
// paint the whole screen red on a link that is working exactly as negotiated.
export function thresholdsForRate(rateHz) {
  if (!rateHz || rateHz <= 0) return [DEFAULT_STALE_MS, DEFAULT_VERY_STALE_MS];
  const period = 1000 / rateHz;
  return [Math.max(DEFAULT_STALE_MS, period * 3), Math.max(DEFAULT_VERY_STALE_MS, period * 10)];
}
