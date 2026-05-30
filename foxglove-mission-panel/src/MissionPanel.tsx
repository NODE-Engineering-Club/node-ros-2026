// Njord Mission panel.
//
// Provides the operator interface for the Njord 2026 ASV:
//   - A Leaflet map centred on the boat's live GPS position (~1 km view).
//   - The boat shown as a marker.
//   - GPS waypoint management: add/delete/clear waypoints, numbered markers,
//     a mission-path polyline, and a "Launch Mission" action that publishes the
//     waypoint list to a ROS 2 topic.
//
// The panel is split into:
//   - `MissionPanel`     : the React component (UI + Leaflet lifecycle)
//   - `initMissionPanel` : the Foxglove mount/unmount entry point
// and ROS message handling lives in ./ros/messages, so further features can be
// layered on without rewriting the mounting or serialization logic.

import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import * as L from "leaflet";
import { ReactElement, StrictMode, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot, Root } from "react-dom/client";

import { NavSatFix, POSE_ARRAY_SCHEMA, Waypoint, waypointsToPoseArray } from "./ros/messages";

import "leaflet/dist/leaflet.css";

// GPS topic published by the Njord sensor stack (sensor_msgs/NavSatFix).
const GPS_TOPIC = "/gps_driver/gps_raw";

// Topic the mission waypoints are published to (geometry_msgs/PoseArray).
const WAYPOINTS_TOPIC = "/mission/waypoints";

// Initial map zoom. At mid latitudes Leaflet zoom 15 shows roughly a 1 km-wide
// view, matching the requested ~1 km radius around the boat.
const DEFAULT_ZOOM = 15;

const PATH_COLOR = "#1e88e5";

/** A GPS position usable by Leaflet. */
interface BoatPosition {
  lat: number;
  lon: number;
}

type StatusKind = "info" | "error" | "success";
interface Status {
  kind: StatusKind;
  text: string;
}

// ── Marker icons ─────────────────────────────────────────────────────────────

// Boat marker. Leaflet's default icon relies on image asset URLs that do not
// resolve inside the bundled extension, so we use a divIcon instead. This is the
// only emoji used in the UI, by design.
const BOAT_ICON = L.divIcon({
  className: "njord-boat-marker",
  html: '<div style="font-size:22px;line-height:22px;transform:translate(-50%,-50%)">🚤</div>',
  iconSize: [22, 22],
  iconAnchor: [11, 11],
});

/** Numbered waypoint marker icon. */
function waypointIcon(n: number): L.DivIcon {
  return L.divIcon({
    className: "njord-wp-marker",
    html:
      `<div style="display:flex;align-items:center;justify-content:center;` +
      `width:22px;height:22px;border-radius:50%;background:${PATH_COLOR};` +
      `color:#fff;font:bold 12px sans-serif;border:2px solid #fff;` +
      `box-shadow:0 0 2px rgba(0,0,0,0.6)">${n}</div>`,
    iconSize: [22, 22],
    iconAnchor: [11, 11],
  });
}

// ── Styling ──────────────────────────────────────────────────────────────────

const styles: Record<string, React.CSSProperties> = {
  root: { display: "flex", flexDirection: "column", height: "100%", width: "100%", font: "13px sans-serif" },
  header: { display: "flex", justifyContent: "space-between", padding: "4px 8px", fontSize: 12, opacity: 0.85 },
  map: { flex: 1, minHeight: 160, width: "100%" },
  controls: { display: "flex", flexWrap: "wrap", gap: 6, alignItems: "center", padding: "6px 8px", borderTop: "1px solid rgba(127,127,127,0.3)" },
  input: { width: 110, padding: "3px 6px", boxSizing: "border-box" },
  labelInput: { width: 130, padding: "3px 6px", boxSizing: "border-box" },
  button: { padding: "4px 10px", cursor: "pointer" },
  launch: { padding: "4px 12px", cursor: "pointer", fontWeight: "bold", marginLeft: "auto" },
  list: { maxHeight: 160, overflowY: "auto", padding: "0 8px 8px" },
  row: { display: "flex", alignItems: "center", gap: 8, padding: "3px 0", borderBottom: "1px solid rgba(127,127,127,0.2)" },
  num: { display: "inline-block", minWidth: 18, textAlign: "center", fontWeight: "bold", color: PATH_COLOR },
  coords: { flex: 1, fontFamily: "monospace" },
  del: { cursor: "pointer", padding: "1px 6px" },
};

function statusColor(kind: StatusKind): string {
  return kind === "error" ? "#e53935" : kind === "success" ? "#43a047" : "inherit";
}

// ── Component ────────────────────────────────────────────────────────────────

function MissionPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const boatMarkerRef = useRef<L.Marker | null>(null);
  const wpLayerRef = useRef<L.LayerGroup | null>(null);
  const hasCentredRef = useRef(false);
  const nextIdRef = useRef(1);

  const [boatPos, setBoatPos] = useState<BoatPosition | undefined>();
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [latInput, setLatInput] = useState("");
  const [lonInput, setLonInput] = useState("");
  const [labelInput, setLabelInput] = useState("");
  const [status, setStatus] = useState<Status>({ kind: "info", text: `Waiting for GPS on ${GPS_TOPIC}…` });
  const [publishReady, setPublishReady] = useState(false);

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // ── Foxglove data subscription ────────────────────────────────────────────
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      const frame = renderState.currentFrame as readonly MessageEvent[] | undefined;
      if (frame && frame.length > 0) {
        const last = frame[frame.length - 1];
        const fix = last?.message as NavSatFix | undefined;
        if (fix && Number.isFinite(fix.latitude) && Number.isFinite(fix.longitude)) {
          setBoatPos({ lat: fix.latitude, lon: fix.longitude });
        }
      }
    };

    context.watch("currentFrame");
    context.subscribe([{ topic: GPS_TOPIC }]);

    return () => {
      context.onRender = undefined;
    };
  }, [context]);

  // Signal render completion after every commit.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // ── Advertise the waypoints topic for publishing ──────────────────────────
  useEffect(() => {
    if (context.advertise == null || context.publish == null) {
      setPublishReady(false);
      return;
    }
    try {
      context.advertise(WAYPOINTS_TOPIC, POSE_ARRAY_SCHEMA);
      setPublishReady(true);
    } catch (err) {
      setPublishReady(false);
      setStatus({ kind: "error", text: `Cannot advertise ${WAYPOINTS_TOPIC}: ${String(err)}` });
    }
    return () => {
      try {
        context.unadvertise?.(WAYPOINTS_TOPIC);
      } catch {
        // ignore — data source may already be gone
      }
    };
  }, [context]);

  // ── Leaflet map lifecycle ─────────────────────────────────────────────────
  useEffect(() => {
    if (mapContainerRef.current == null || mapRef.current != null) {
      return;
    }

    const map = L.map(mapContainerRef.current, { center: [0, 0], zoom: DEFAULT_ZOOM, zoomControl: true });
    L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19,
      attribution: "© OpenStreetMap contributors",
    }).addTo(map);
    wpLayerRef.current = L.layerGroup().addTo(map);
    mapRef.current = map;

    const observer = new ResizeObserver(() => map.invalidateSize());
    observer.observe(mapContainerRef.current);

    return () => {
      observer.disconnect();
      map.remove();
      mapRef.current = null;
      boatMarkerRef.current = null;
      wpLayerRef.current = null;
      hasCentredRef.current = false;
    };
  }, []);

  // ── Update boat marker + recentre on new GPS fixes ────────────────────────
  useEffect(() => {
    const map = mapRef.current;
    if (map == null || boatPos == null) {
      return;
    }
    const latlng: L.LatLngExpression = [boatPos.lat, boatPos.lon];
    if (boatMarkerRef.current == null) {
      boatMarkerRef.current = L.marker(latlng, { icon: BOAT_ICON }).addTo(map);
    } else {
      boatMarkerRef.current.setLatLng(latlng);
    }
    if (!hasCentredRef.current) {
      map.setView(latlng, DEFAULT_ZOOM);
      hasCentredRef.current = true;
    } else {
      map.panTo(latlng, { animate: true });
    }
  }, [boatPos]);

  // ── Redraw waypoint markers + mission path on change ──────────────────────
  useEffect(() => {
    const group = wpLayerRef.current;
    if (group == null) {
      return;
    }
    group.clearLayers();
    const latlngs: L.LatLngExpression[] = waypoints.map((w) => [w.lat, w.lon]);
    waypoints.forEach((w, i) => {
      L.marker([w.lat, w.lon], { icon: waypointIcon(i + 1) })
        .bindTooltip(w.label ? w.label : `WP ${i + 1}`)
        .addTo(group);
    });
    if (latlngs.length >= 2) {
      L.polyline(latlngs, { color: PATH_COLOR, weight: 3, opacity: 0.8 }).addTo(group);
    }
  }, [waypoints]);

  // ── Actions ────────────────────────────────────────────────────────────────
  const addWaypoint = useCallback(() => {
    const lat = Number(latInput);
    const lon = Number(lonInput);
    if (!Number.isFinite(lat) || lat < -90 || lat > 90) {
      setStatus({ kind: "error", text: "Latitude must be a number between -90 and 90." });
      return;
    }
    if (!Number.isFinite(lon) || lon < -180 || lon > 180) {
      setStatus({ kind: "error", text: "Longitude must be a number between -180 and 180." });
      return;
    }
    const label = labelInput.trim();
    setWaypoints((prev) => [...prev, { id: nextIdRef.current++, lat, lon, label: label ? label : undefined }]);
    setLatInput("");
    setLonInput("");
    setLabelInput("");
    setStatus({ kind: "info", text: "Waypoint added." });
  }, [latInput, lonInput, labelInput]);

  const deleteWaypoint = useCallback((id: number) => {
    setWaypoints((prev) => prev.filter((w) => w.id !== id));
  }, []);

  const clearAll = useCallback(() => {
    setWaypoints([]);
    setStatus({ kind: "info", text: "Waypoints cleared." });
  }, []);

  const launchMission = useCallback(() => {
    if (waypoints.length === 0) {
      setStatus({ kind: "error", text: "Add at least one waypoint before launching." });
      return;
    }
    if (!publishReady || context.publish == null) {
      setStatus({ kind: "error", text: "Publishing not available — connect to a writable data source." });
      return;
    }
    try {
      context.publish(WAYPOINTS_TOPIC, waypointsToPoseArray(waypoints));
      setStatus({ kind: "success", text: `Mission published: ${waypoints.length} waypoint(s) → ${WAYPOINTS_TOPIC}` });
    } catch (err) {
      setStatus({ kind: "error", text: `Publish failed: ${String(err)}` });
    }
  }, [waypoints, publishReady, context]);

  // Add waypoint on Enter from any input field.
  const onInputKeyDown = useCallback(
    (e: React.KeyboardEvent) => {
      if (e.key === "Enter") {
        addWaypoint();
      }
    },
    [addWaypoint],
  );

  const boatText = useMemo(
    () => (boatPos ? `Boat: ${boatPos.lat.toFixed(6)}, ${boatPos.lon.toFixed(6)}` : "Boat: no GPS fix"),
    [boatPos],
  );

  // ── Render ───────────────────────────────────────────────────────────────
  return (
    <div style={styles.root}>
      <div style={styles.header}>
        <span>{boatText}</span>
        <span style={{ color: statusColor(status.kind) }}>{status.text}</span>
      </div>

      <div ref={mapContainerRef} style={styles.map} />

      <div style={styles.controls}>
        <input
          style={styles.input}
          type="number"
          step="any"
          placeholder="Latitude"
          value={latInput}
          onChange={(e) => setLatInput(e.target.value)}
          onKeyDown={onInputKeyDown}
        />
        <input
          style={styles.input}
          type="number"
          step="any"
          placeholder="Longitude"
          value={lonInput}
          onChange={(e) => setLonInput(e.target.value)}
          onKeyDown={onInputKeyDown}
        />
        <input
          style={styles.labelInput}
          type="text"
          placeholder="Label (optional)"
          value={labelInput}
          onChange={(e) => setLabelInput(e.target.value)}
          onKeyDown={onInputKeyDown}
        />
        <button style={styles.button} onClick={addWaypoint}>
          Add waypoint
        </button>
        <button style={styles.button} onClick={clearAll} disabled={waypoints.length === 0}>
          Clear all
        </button>
        <button style={styles.launch} onClick={launchMission} disabled={waypoints.length === 0 || !publishReady}>
          Launch Mission
        </button>
      </div>

      <div style={styles.list}>
        {waypoints.length === 0 ? (
          <div style={{ opacity: 0.6, padding: "6px 0" }}>No waypoints yet.</div>
        ) : (
          waypoints.map((w, i) => (
            <div key={w.id} style={styles.row}>
              <span style={styles.num}>{i + 1}</span>
              <span style={styles.coords}>
                {w.lat.toFixed(6)}, {w.lon.toFixed(6)}
                {w.label ? `  —  ${w.label}` : ""}
              </span>
              <button style={styles.del} onClick={() => deleteWaypoint(w.id)} title="Delete waypoint">
                Delete
              </button>
            </div>
          ))
        )}
      </div>
    </div>
  );
}

export function initMissionPanel(context: PanelExtensionContext): () => void {
  const root: Root = createRoot(context.panelElement);
  root.render(
    <StrictMode>
      <MissionPanel context={context} />
    </StrictMode>,
  );
  return () => {
    root.unmount();
  };
}
