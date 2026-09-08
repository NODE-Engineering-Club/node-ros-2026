// Small geographic helpers for drawing on the map.
//
// These mirror asket_common/geo.py deliberately: the same conventions, the
// same tangent-plane approximation. A survey box is a few kilometres across, so
// the error is far below anything we can measure.

const METRES_PER_DEG_LAT = (Math.PI * 6371008.8) / 180;

export function metresPerDegLon(lat) {
  return METRES_PER_DEG_LAT * Math.cos((lat * Math.PI) / 180);
}

/** Offset a lat/lon by a distance along a compass bearing. */
export function offset(lat, lon, bearingDeg, distanceM) {
  const rad = (bearingDeg * Math.PI) / 180;
  const east = distanceM * Math.sin(rad);
  const north = distanceM * Math.cos(rad);
  return [lon + east / metresPerDegLon(lat), lat + north / METRES_PER_DEG_LAT];
}

/**
 * The coverage ribbon.
 *
 * Built as a series of quads between consecutive samples rather than one filled
 * polygon, so a gap in the data renders as a gap in the ribbon. With one sonar
 * covering one side only, a gap that renders as filled is the single most
 * expensive way this GUI could mislead an operator.
 */
export function coverageRibbon(segments, side = 'starboard') {
  const features = [];
  let run = [];

  const flush = () => {
    for (let i = 0; i + 1 < run.length; i += 1) {
      const a = run[i];
      const b = run[i + 1];
      features.push({
        type: 'Feature',
        properties: {},
        geometry: { type: 'Polygon', coordinates: [[a.near, a.far, b.far, b.near, a.near]] },
      });
    }
    run = [];
  };

  // Wire format: [lat, lon, heading_deg, half_width_m, inner_gap_m], or null
  // for a gap — a stretch where the sonar was not ensonifying because the
  // vessel was turning, the heading was invalid, or the sonar had dropped out.
  const bearingOffset = side === 'port' ? -90 : 90;
  for (const segment of segments) {
    if (!segment) {
      flush();
      continue;
    }
    const [lat, lon, heading, halfWidth, innerGap] = segment;
    run.push({
      near: offset(lat, lon, heading + bearingOffset, innerGap || 0),
      far: offset(lat, lon, heading + bearingOffset, halfWidth),
    });
  }
  flush();

  return { type: 'FeatureCollection', features };
}

/** Lidar returns, vessel-relative bearings, placed geographically. */
export function lidarPoints(pairs, lat, lon, headingDeg) {
  if (lat === null || lat === undefined) return { type: 'FeatureCollection', features: [] };
  return {
    type: 'FeatureCollection',
    features: (pairs || []).map(([bearing, range]) => ({
      type: 'Feature',
      properties: { range },
      geometry: { type: 'Point', coordinates: offset(lat, lon, headingDeg + bearing, range) },
    })),
  };
}
