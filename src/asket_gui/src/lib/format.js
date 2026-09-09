// Formatting helpers.
//
// One rule runs through all of them: a value that is not available renders as
// an em dash, never as zero. On a boat, a zero is a measurement.

export const DASH = '—';

export function num(value, digits = 1, unit = '') {
  if (value === null || value === undefined || Number.isNaN(value)) return DASH;
  return `${value.toFixed(digits)}${unit}`;
}

export function int(value, unit = '') {
  if (value === null || value === undefined || Number.isNaN(value)) return DASH;
  return `${Math.round(value)}${unit}`;
}

export function pct(value, digits = 0) {
  if (value === null || value === undefined || Number.isNaN(value)) return DASH;
  return `${(value * 100).toFixed(digits)}%`;
}

export function bearing(value) {
  if (value === null || value === undefined || Number.isNaN(value)) return DASH;
  return `${value.toFixed(0).padStart(3, '0')}°`;
}

export function duration(seconds) {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return DASH;
  if (seconds < 0) return DASH;
  const s = Math.round(seconds);
  const h = Math.floor(s / 3600);
  const m = Math.floor((s % 3600) / 60);
  if (h > 0) return `${h} h ${String(m).padStart(2, '0')} min`;
  if (m > 0) return `${m} min ${String(s % 60).padStart(2, '0')} s`;
  return `${s} s`;
}

export function bytes(value) {
  if (value === null || value === undefined) return DASH;
  const units = ['B', 'kB', 'MB', 'GB', 'TB'];
  let v = value;
  let i = 0;
  while (v >= 1024 && i < units.length - 1) {
    v /= 1024;
    i += 1;
  }
  return `${v.toFixed(v < 10 && i > 0 ? 1 : 0)} ${units[i]}`;
}

export function coordinate(lat, lon) {
  if (lat === null || lat === undefined || lon === null || lon === undefined) return DASH;
  const ns = lat >= 0 ? 'N' : 'S';
  const ew = lon >= 0 ? 'E' : 'W';
  return `${Math.abs(lat).toFixed(5)}° ${ns}  ${Math.abs(lon).toFixed(5)}° ${ew}`;
}

// Data age, rendered the way an operator needs to read it at a glance.
export function age(ms) {
  if (ms === null || ms === undefined || !Number.isFinite(ms)) return DASH;
  if (ms < 0) return 'now';
  if (ms < 1500) return 'now';
  if (ms < 60000) return `${(ms / 1000).toFixed(0)} s ago`;
  return `${Math.floor(ms / 60000)} min ago`;
}
