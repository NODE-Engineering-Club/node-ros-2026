// Stream negotiation, mirroring the backend.
//
// The table itself is GENERATED from `gui_backend/core/streams.py` into
// `streamPolicy.json`, and `test_mock_policy.py` fails if the two drift. That
// matters more than it looks: a mock that negotiates differently from the real
// backend is not a preview of this system, it is a demonstration of a system
// that does not exist — and it gets reviewed and believed.

// Import attribute required by Node's ESM loader; Vite honours it too. Keeping
// the table as JSON rather than JS is what lets the Python test compare it.
import policy from './streamPolicy.json' with { type: 'json' };

export const STREAMS = policy.streams;
export const PROFILES = policy.profiles;
export const PROFILE_ORDER = policy.profile_order;

const DETAIL_ORDER = ['full', 'reduced', 'minimal'];

/** What a subscription actually gets, and why — the reason ends up on screen. */
export function resolve(stream, requestedRateHz, requestedDetail, profileName) {
  const spec = STREAMS[stream];
  if (!spec) {
    return { stream, granted: false, rate_hz: 0, detail: 'minimal',
             reason: `no such stream '${stream}'` };
  }

  const profile = PROFILES[profileName];
  const streamPolicy = profile.policies[stream];
  if (!streamPolicy) {
    return {
      stream,
      granted: false,
      rate_hz: 0,
      detail: 'minimal',
      reason: `not carried on the ${profileName} link profile (${profile.description})`,
    };
  }

  const reasons = [];
  let rate = 0;
  if (!spec.on_change_only) {
    rate = requestedRateHz ?? spec.default_rate_hz;
    if (rate > spec.max_rate_hz) {
      rate = spec.max_rate_hz;
      reasons.push(`capped at the stream's own maximum of ${spec.max_rate_hz} Hz`);
    }
    if (rate > streamPolicy.max_rate_hz) {
      rate = streamPolicy.max_rate_hz;
      reasons.push(`limited to ${streamPolicy.max_rate_hz} Hz by the ${profileName} profile`);
    }
  }

  let detail = requestedDetail || streamPolicy.detail;
  if (DETAIL_ORDER.indexOf(detail) < DETAIL_ORDER.indexOf(streamPolicy.detail)) {
    detail = streamPolicy.detail;
    reasons.push(`detail reduced to '${streamPolicy.detail}' by the ${profileName} profile`);
  }

  return { stream, granted: true, rate_hz: rate, detail, reason: reasons.join('; ') };
}

const DETAIL_SCALE = { full: 1, reduced: 0.4, minimal: 0.15 };

export function estimateBytesPerS(resolutions) {
  let total = 0;
  for (const res of resolutions) {
    if (!res.granted) continue;
    const spec = STREAMS[res.stream];
    const rate = res.rate_hz > 0 ? res.rate_hz : 0.05;
    total += spec.typical_bytes * DETAIL_SCALE[res.detail] * rate;
  }
  return total;
}
