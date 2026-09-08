import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
import { useEffect, useRef, useState } from 'react';

import { coverageRibbon, lidarPoints } from '../lib/geometry.js';
import { blankStyle, graticule, rasterStyle } from '../lib/mapStyle.js';
import { streamPayload } from '../lib/connection.js';

/**
 * The primary panel.
 *
 * Vessel, heading, track, planned survey lines, coverage, lidar, geofence.
 *
 * Offline tiles are mandatory — there is no internet in the field. If the
 * .mbtiles file is missing or does not cover the survey area, the map falls
 * back to a coordinate graticule and says so in plain words. It never silently
 * shows an empty rectangle, because an operator would read that as "no map
 * today" rather than "fix the tile file".
 */
export function MissionMap({ state, follow, onFollowChange, showRawLidar = false }) {
  const container = useRef(null);
  const map = useRef(null);
  const [tiles, setTiles] = useState(null);
  const [ready, setReady] = useState(false);

  const vessel = streamPayload(state, 'vessel');
  const track = streamPayload(state, 'track');
  const plan = streamPayload(state, 'plan');
  const coverage = streamPayload(state, 'coverage');
  const lidar = streamPayload(state, 'lidar');

  // -- tile availability ------------------------------------------------

  useEffect(() => {
    let cancelled = false;
    fetch('/api/tiles/info')
      .then((r) => r.json())
      .then((info) => !cancelled && setTiles(info))
      .catch(() =>
        !cancelled &&
        setTiles({ available: false, message: 'Could not ask the backend about map tiles.' }),
      );
    return () => {
      cancelled = true;
    };
  }, []);

  // -- map creation -----------------------------------------------------

  useEffect(() => {
    if (!container.current || tiles === null || map.current) return;

    const instance = new maplibregl.Map({
      container: container.current,
      style: tiles.available ? rasterStyle() : blankStyle(),
      center: [14.5053, -22.9576],
      zoom: 14,
      attributionControl: false,
      // No glyph server exists offline, so nothing may try to render text from
      // a font the style would have to fetch.
      localIdeographFontFamily: false,
    });
    instance.addControl(new maplibregl.NavigationControl({ showCompass: true }), 'top-right');
    instance.addControl(new maplibregl.ScaleControl({ unit: 'metric' }), 'bottom-right');
    instance.on('dragstart', () => onFollowChange(false));

    instance.on('load', () => {
      addLayers(instance, !tiles.available);
      map.current = instance;
      setReady(true);
    });

    return () => {
      instance.remove();
      map.current = null;
      setReady(false);
    };
  }, [tiles, onFollowChange]);

  // -- data into layers -------------------------------------------------

  useEffect(() => {
    const instance = map.current;
    if (!instance || !ready) return;

    setData(instance, 'plan', {
      type: 'FeatureCollection',
      features: (plan?.lines || []).map((line) => ({
        type: 'Feature',
        properties: { index: line.index },
        geometry: { type: 'LineString', coordinates: line.coords },
      })),
    });

    setData(instance, 'geofence', {
      type: 'FeatureCollection',
      features: plan?.geofence?.length
        ? [{ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: [...plan.geofence, plan.geofence[0]] } }]
        : [],
    });

    setData(
      instance,
      'coverage',
      coverageRibbon(coverage?.segments || [], coverage?.side),
    );

    setData(instance, 'track', {
      type: 'FeatureCollection',
      features: track?.points?.length
        ? [{ type: 'Feature', properties: {}, geometry: { type: 'LineString', coordinates: track.points } }]
        : [],
    });

    setData(
      instance,
      'lidar',
      lidarPoints(
        showRawLidar ? lidar?.raw || lidar?.filtered : lidar?.filtered,
        vessel?.lat,
        vessel?.lon,
        vessel?.heading_deg ?? 0,
      ),
    );

    const vesselFeatures = vessel?.lat
      ? [
          {
            type: 'Feature',
            properties: {
              heading: vessel.heading_deg ?? 0,
              // A vessel whose heading is not trustworthy must not be drawn
              // with a confident arrow.
              headingValid: vessel.heading_valid ? 1 : 0,
            },
            geometry: { type: 'Point', coordinates: [vessel.lon, vessel.lat] },
          },
        ]
      : [];
    setData(instance, 'vessel', { type: 'FeatureCollection', features: vesselFeatures });

    if (follow && vessel?.lat) {
      instance.easeTo({ center: [vessel.lon, vessel.lat], duration: 400 });
    }
  }, [ready, vessel, track, plan, coverage, lidar, follow, showRawLidar]);

  // -- graticule --------------------------------------------------------

  useEffect(() => {
    const instance = map.current;
    if (!instance || !ready || tiles?.available) return;
    const redraw = () => setData(instance, 'graticule', graticule(instance.getBounds()));
    redraw();
    instance.on('moveend', redraw);
    return () => instance.off('moveend', redraw);
  }, [ready, tiles]);

  return (
    <div className="map-area">
      <div ref={container} className="map" />
      <div className="map-overlay">
        <button onClick={() => onFollowChange(!follow)}>
          {follow ? 'Following vessel' : 'Follow vessel'}
        </button>
        {/* Judging coverage means seeing the whole box at once. Following the
            vessel keeps you at a zoom where a gap two lines away is off
            screen. */}
        <button
          onClick={() => {
            onFollowChange(false);
            fitSurvey(map.current, plan, coverage);
          }}
          disabled={!plan?.lines?.length}
          title="Zoom out to the whole survey box, so gaps in coverage are visible"
        >
          Fit survey
        </button>
      </div>
      {tiles && !tiles.available && (
        <div className="map-note">
          <strong className="warnline">No offline map tiles.</strong> Showing a coordinate
          grid only. {tiles.message}
        </div>
      )}
      {tiles?.available && tiles.bounds && vessel?.lat &&
        !withinBounds(tiles.bounds, vessel.lat, vessel.lon) && (
          <div className="map-note">
            <strong className="warnline">The vessel is outside the tiled area.</strong> The
            map will be blank here. Pre-download tiles covering the survey box.
          </div>
        )}
    </div>
  );
}

/** Zoom to the planned survey box plus whatever has been covered so far. */
function fitSurvey(map, plan, coverage) {
  if (!map || !plan?.lines?.length) return;
  const points = plan.lines.flatMap((line) => line.coords);
  for (const segment of coverage?.segments || []) {
    if (segment) points.push([segment[1], segment[0]]);
  }
  if (!points.length) return;

  const lons = points.map((p) => p[0]);
  const lats = points.map((p) => p[1]);
  map.fitBounds(
    [
      [Math.min(...lons), Math.min(...lats)],
      [Math.max(...lons), Math.max(...lats)],
    ],
    { padding: 48, duration: 600 },
  );
}

function withinBounds(bounds, lat, lon) {
  const [west, south, east, north] = bounds;
  return lon >= west && lon <= east && lat >= south && lat <= north;
}

function setData(map, id, data) {
  const source = map.getSource(id);
  if (source) source.setData(data);
}

function empty() {
  return { type: 'FeatureCollection', features: [] };
}

function addLayers(map, withGraticule) {
  for (const id of ['graticule', 'geofence', 'plan', 'coverage', 'track', 'lidar', 'vessel']) {
    map.addSource(id, { type: 'geojson', data: empty() });
  }

  if (withGraticule) {
    map.addLayer({
      id: 'graticule',
      type: 'line',
      source: 'graticule',
      paint: { 'line-color': '#2b3440', 'line-width': 1 },
    });
  }

  // Coverage sits under everything else, but it is not background: with a
  // single sonar covering one side only, spotting a gap is the whole reason
  // this map exists, and a 28%-opacity green on a near-black basemap is
  // effectively invisible in sunlight. Opaque enough to read at a glance,
  // still translucent enough that overlapping passes show as a brighter band —
  // which is how an operator spots a line flown twice.
  map.addLayer({
    id: 'coverage',
    type: 'fill',
    source: 'coverage',
    paint: { 'fill-color': '#2fd07a', 'fill-opacity': 0.55 },
  });

  map.addLayer({
    id: 'plan',
    type: 'line',
    source: 'plan',
    paint: { 'line-color': '#4aa3ff', 'line-width': 2, 'line-dasharray': [3, 2], 'line-opacity': 0.8 },
  });

  map.addLayer({
    id: 'geofence',
    type: 'line',
    source: 'geofence',
    paint: { 'line-color': '#f2b134', 'line-width': 2, 'line-dasharray': [1, 2] },
  });

  map.addLayer({
    id: 'track',
    type: 'line',
    source: 'track',
    paint: { 'line-color': '#e8eef5', 'line-width': 1.6, 'line-opacity': 0.75 },
  });

  map.addLayer({
    id: 'lidar',
    type: 'circle',
    source: 'lidar',
    paint: { 'circle-radius': 2.5, 'circle-color': '#ff8a4a', 'circle-opacity': 0.85 },
  });

  map.addLayer({
    id: 'vessel-halo',
    type: 'circle',
    source: 'vessel',
    paint: { 'circle-radius': 9, 'circle-color': '#4aa3ff', 'circle-opacity': 0.25 },
  });

  // The heading arrow is drawn as a triangle rotated by heading. When heading
  // is invalid it turns red and stops rotating — pointing an arrow confidently
  // in a direction we do not trust is exactly the lie this GUI must not tell.
  map.addLayer({
    id: 'vessel',
    type: 'symbol',
    source: 'vessel',
    layout: {
      'icon-image': 'vessel-arrow',
      'icon-rotate': ['case', ['==', ['get', 'headingValid'], 1], ['get', 'heading'], 0],
      'icon-rotation-alignment': 'map',
      'icon-allow-overlap': true,
      'icon-size': 1,
    },
  });

  map.addImage('vessel-arrow', arrowImage(), { pixelRatio: 2 });
}

/** A 32x32 arrow, generated so nothing has to be fetched. */
function arrowImage() {
  const size = 32;
  const data = new Uint8ClampedArray(size * size * 4);
  const put = (x, y, r, g, b, a) => {
    const i = (y * size + x) * 4;
    data[i] = r;
    data[i + 1] = g;
    data[i + 2] = b;
    data[i + 3] = a;
  };
  // Triangle pointing up (north at rotation 0).
  for (let y = 4; y < 28; y += 1) {
    const halfWidth = ((y - 4) / 24) * 9;
    for (let x = Math.round(16 - halfWidth); x <= Math.round(16 + halfWidth); x += 1) {
      const edge = Math.abs(x - 16) > halfWidth - 2 || y > 25;
      put(x, y, edge ? 13 : 74, edge ? 17 : 163, edge ? 23 : 255, 255);
    }
  }
  return { width: size, height: size, data };
}
