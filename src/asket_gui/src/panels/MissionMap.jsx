import maplibregl from 'maplibre-gl';
import 'maplibre-gl/dist/maplibre-gl.css';
import { useEffect, useRef, useState } from 'react';

import { coverageRibbon, lidarPoints } from '../lib/geometry.js';
import { blankStyle, graticule, rasterStyle } from '../lib/mapStyle.js';
import { registerMockTileProtocol } from '../lib/mock/tiles.js';
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
export function MissionMap({ state, connection, follow, onFollowChange, showRawLidar = false }) {
  const container = useRef(null);
  const map = useRef(null);
  const [tiles, setTiles] = useState(null);
  const [ready, setReady] = useState(false);
  //: The survey box is framed once, when the plan first arrives. Opening on a
  //: fixed zoom put a 300 m survey on screen as a 30 px smudge, which is not a
  //: map anyone can judge coverage from. Once only: re-framing under an
  //: operator who has zoomed in to look at a gap would be worse than the smudge.
  const framed = useRef(false);

  const vessel = streamPayload(state, 'vessel');
  const track = streamPayload(state, 'track');
  const plan = streamPayload(state, 'plan');
  const coverage = streamPayload(state, 'coverage');
  const lidar = streamPayload(state, 'lidar');

  // -- tile availability ------------------------------------------------

  useEffect(() => {
    let cancelled = false;
    connection
      .fetchTileInfo()
      .then((info) => !cancelled && setTiles(info))
      .catch(() =>
        !cancelled &&
        setTiles({ available: false, message: 'Could not ask about map tiles.' }),
      );
    return () => {
      cancelled = true;
    };
  }, [connection, state.tileGeneration]);

  // -- map creation -----------------------------------------------------

  useEffect(() => {
    if (!container.current || tiles === null || map.current) return;

    // In mock mode the tiles are generated in the browser through a custom
    // protocol, so the tiled rendering path can be reviewed with no .mbtiles
    // file and no internet.
    if (connection.isMock) registerMockTileProtocol(maplibregl);
    const tileUrl = connection.isMock ? 'mocktiles://{z}/{x}/{y}' : undefined;

    const instance = new maplibregl.Map({
      container: container.current,
      style: tiles.available ? rasterStyle(tileUrl) : blankStyle(),
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
  }, [tiles, onFollowChange, connection]);

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
    if (!framed.current && plan?.lines?.length) {
      framed.current = true;
      // Open on the whole job. Following is switched off for the same reason
      // the Fit survey button switches it off: centred on the vessel at survey
      // zoom, half the box is off screen, and judging coverage is the thing
      // this map is for. One press of Following vessel gets it back.
      onFollowChange(false);
      // Instant, not animated: the follow ease runs on every vessel sample and
      // cancels an in-flight fitBounds, leaving the survey a smudge.
      fitSurvey(instance, plan, coverage, 0);
    }
  }, [ready, vessel, track, plan, coverage, lidar, follow, showRawLidar, onFollowChange]);

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
      {/* Five overlays in five colours is four too many to hold in your head at
          06:00. The legend is small, permanent, and uses the same colours the
          layers are painted with — they are declared once, below. */}
      <div className="map-legend">
        {LEGEND.map((item) => (
          <div key={item.label}>
            <span
              className={`swatch ${item.fill ? 'fill' : ''}`}
              style={{ background: item.color, opacity: item.opacity ?? 1 }}
            />
            {item.label}
          </div>
        ))}
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
function fitSurvey(map, plan, coverage, duration = 600) {
  if (!map || !plan?.lines?.length) return;
  const points = plan.lines.flatMap((line) => line.coords);
  points.push(...(plan.geofence || []));
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
    // maxZoom keeps the camera inside the tile set's own range. A short survey
    // otherwise frames to a zoom past the deepest tile available, and MapLibre
    // upscales — a blurred basemap that looks like a rendering fault rather
    // than the edge of the data.
    { padding: 48, duration, maxZoom: 18 },
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

//: The map's palette, declared once. `addLayers` paints from it and the legend
//: reads from it, so a colour cannot be changed in one place and not the other.
const COLOURS = {
  coverage: '#00a651',
  plan: '#0b57d0',
  geofence: '#a25a00',
  track: '#000000',
  lidar: '#d34500',
  graticule: '#aeaeae',
};

const LEGEND = [
  { label: 'Covered', color: COLOURS.coverage, fill: true, opacity: 0.42 },
  { label: 'Planned line', color: COLOURS.plan },
  { label: 'Track', color: COLOURS.track },
  { label: 'Geofence', color: COLOURS.geofence },
  { label: 'Obstacle', color: COLOURS.lidar, fill: true },
];

function addLayers(map, withGraticule) {
  for (const id of ['graticule', 'geofence', 'plan', 'coverage', 'track', 'lidar', 'vessel']) {
    map.addSource(id, { type: 'geojson', data: empty() });
  }

  if (withGraticule) {
    map.addLayer({
      id: 'graticule',
      type: 'line',
      source: 'graticule',
      // Grey on purpose: the graticule is a reference frame, not data, and it
      // must not compete with the track drawn on top of it.
      paint: { 'line-color': COLOURS.graticule, 'line-width': 1 },
    });
  }

  // Coverage sits under everything else, but it is not background: with a
  // single sonar covering one side only, spotting a gap is the whole reason
  // this map exists. On white it has to be a saturated green rather than a
  // pale wash — and translucent enough that overlapping passes darken into a
  // deeper band, which is how an operator spots a line flown twice.
  map.addLayer({
    id: 'coverage',
    type: 'fill',
    source: 'coverage',
    paint: { 'fill-color': COLOURS.coverage, 'fill-opacity': 0.42 },
  });

  map.addLayer({
    id: 'plan',
    type: 'line',
    source: 'plan',
    paint: { 'line-color': COLOURS.plan, 'line-width': 2, 'line-dasharray': [3, 2] },
  });

  map.addLayer({
    id: 'geofence',
    type: 'line',
    source: 'geofence',
    paint: { 'line-color': COLOURS.geofence, 'line-width': 2, 'line-dasharray': [1, 2] },
  });

  map.addLayer({
    id: 'track',
    type: 'line',
    source: 'track',
    // Where the boat actually went, in black: it must read over coverage,
    // over the plan, and over the graticule, at any zoom.
    paint: { 'line-color': COLOURS.track, 'line-width': 1.6 },
  });

  map.addLayer({
    id: 'lidar',
    type: 'circle',
    source: 'lidar',
    paint: { 'circle-radius': 2.5, 'circle-color': COLOURS.lidar },
  });

  map.addLayer({
    id: 'vessel-halo',
    type: 'circle',
    source: 'vessel',
    paint: { 'circle-radius': 9, 'circle-color': COLOURS.plan, 'circle-opacity': 0.2 },
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
      // Black outline, blue body — legible on white, on the green coverage
      // ribbon, and on a satellite tile alike.
      put(x, y, edge ? 0 : 11, edge ? 0 : 87, edge ? 0 : 208, 255);
    }
  }
  return { width: size, height: size, data };
}
