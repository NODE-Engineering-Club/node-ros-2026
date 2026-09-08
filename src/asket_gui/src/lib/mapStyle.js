// MapLibre styles, built locally.
//
// There is no internet in the field, so no style is fetched from a URL and no
// glyph or sprite server is referenced. Everything the map needs is either in
// the .mbtiles file on the Jetson or drawn from GeoJSON we produce ourselves.
//
// When there are no tiles the map does not silently show a grey rectangle. It
// draws a coordinate graticule with labels, so an operator can still read
// position, track and coverage against a scale — degraded, and visibly so.

export function rasterStyle(tileUrl) {
  return {
    version: 8,
    sources: {
      offline: {
        type: 'raster',
        tiles: [tileUrl || `${window.location.origin}/tiles/{z}/{x}/{y}.png`],
        tileSize: 256,
        minzoom: 0,
        maxzoom: 20,
      },
    },
    layers: [
      { id: 'background', type: 'background', paint: { 'background-color': '#0d1117' } },
      { id: 'offline', type: 'raster', source: 'offline', paint: { 'raster-opacity': 1 } },
    ],
  };
}

export function blankStyle() {
  return {
    version: 8,
    sources: {},
    layers: [
      { id: 'background', type: 'background', paint: { 'background-color': '#0d1117' } },
    ],
  };
}

/**
 * A latitude/longitude graticule as GeoJSON.
 *
 * Drawn when there are no tiles. Spacing adapts to the view so the grid stays
 * readable from a whole survey box down to a single line.
 */
export function graticule(bounds, targetLines = 8) {
  const west = bounds.getWest();
  const east = bounds.getEast();
  const south = bounds.getSouth();
  const north = bounds.getNorth();

  const step = niceStep(Math.max(east - west, north - south) / targetLines);
  const features = [];

  for (let lon = Math.ceil(west / step) * step; lon <= east; lon += step) {
    features.push({
      type: 'Feature',
      properties: { label: `${lon.toFixed(decimalsFor(step))}°` },
      geometry: { type: 'LineString', coordinates: [[lon, south], [lon, north]] },
    });
  }
  for (let lat = Math.ceil(south / step) * step; lat <= north; lat += step) {
    features.push({
      type: 'Feature',
      properties: { label: `${lat.toFixed(decimalsFor(step))}°` },
      geometry: { type: 'LineString', coordinates: [[west, lat], [east, lat]] },
    });
  }
  return { type: 'FeatureCollection', features };
}

function niceStep(raw) {
  const magnitude = 10 ** Math.floor(Math.log10(raw));
  for (const multiple of [1, 2, 5, 10]) {
    if (raw <= magnitude * multiple) return magnitude * multiple;
  }
  return magnitude * 10;
}

function decimalsFor(step) {
  return Math.max(0, Math.min(6, Math.ceil(-Math.log10(step)) + 1));
}
