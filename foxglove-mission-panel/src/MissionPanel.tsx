// Njord Mission panel.
//
// Displays a Leaflet map centred on the boat's live GPS position with the boat
// shown as a marker. The map keeps a ~1 km view around the boat and re-centres
// as new GPS fixes arrive.
//
// The panel is intentionally split into:
//   - `MissionPanel`     : the React component (UI + Leaflet lifecycle)
//   - `initMissionPanel` : the Foxglove mount/unmount entry point
// so that future features (waypoint entry, mission control) can be added as
// additional components without rewriting the mounting logic.

import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import * as L from "leaflet";
import { ReactElement, StrictMode, useEffect, useLayoutEffect, useMemo, useRef, useState } from "react";
import { createRoot, Root } from "react-dom/client";

import "leaflet/dist/leaflet.css";

// GPS topic published by the Njord sensor stack (sensor_msgs/NavSatFix).
const GPS_TOPIC = "/gps_driver/gps_raw";

// Initial map zoom. At mid latitudes Leaflet zoom 15 shows roughly a 1 km-wide
// view, matching the requested ~1 km radius around the boat.
const DEFAULT_ZOOM = 15;

// Minimal shape of the sensor_msgs/NavSatFix fields we care about.
type NavSatFix = {
  latitude: number;
  longitude: number;
  // status.status >= 0 means a valid fix in the NavSatStatus convention.
  status?: { status: number };
};

/** A GPS position usable by Leaflet. */
type BoatPosition = {
  lat: number;
  lon: number;
};

function MissionPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const mapContainerRef = useRef<HTMLDivElement>(null);
  const mapRef = useRef<L.Map | null>(null);
  const boatMarkerRef = useRef<L.Marker | null>(null);

  // Latest boat position derived from the GPS topic.
  const [boatPos, setBoatPos] = useState<BoatPosition | undefined>();

  // Whether we have already centred the map on the first fix.
  const hasCentredRef = useRef(false);

  // Foxglove render callback; invoked after each commit so the panel can be
  // throttled correctly.
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Boat marker icon. Leaflet's default icon relies on image asset URLs that do
  // not resolve inside the bundled extension, so we use a simple divIcon instead.
  const boatIcon = useMemo(
    () =>
      L.divIcon({
        className: "njord-boat-marker",
        html: '<div style="font-size:22px;line-height:22px;transform:translate(-50%,-50%)">🚤</div>',
        iconSize: [22, 22],
        iconAnchor: [11, 11],
      }),
    [],
  );

  // ── Foxglove data subscription ────────────────────────────────────────────
  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);

      // Pull the most recent NavSatFix message from the GPS topic, if any.
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

  // ── Leaflet map lifecycle ─────────────────────────────────────────────────
  useEffect(() => {
    if (mapContainerRef.current == null || mapRef.current != null) {
      return;
    }

    const map = L.map(mapContainerRef.current, {
      center: [0, 0],
      zoom: DEFAULT_ZOOM,
      zoomControl: true,
    });

    L.tileLayer("https://tile.openstreetmap.org/{z}/{x}/{y}.png", {
      maxZoom: 19,
      attribution: "© OpenStreetMap contributors",
    }).addTo(map);

    mapRef.current = map;

    return () => {
      map.remove();
      mapRef.current = null;
      boatMarkerRef.current = null;
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
      boatMarkerRef.current = L.marker(latlng, { icon: boatIcon }).addTo(map);
    } else {
      boatMarkerRef.current.setLatLng(latlng);
    }

    // Centre on the first fix; afterwards keep the boat in view by panning.
    if (!hasCentredRef.current) {
      map.setView(latlng, DEFAULT_ZOOM);
      hasCentredRef.current = true;
    } else {
      map.panTo(latlng, { animate: true });
    }
  }, [boatPos, boatIcon]);

  // Leaflet needs an explicit size invalidation once its container is laid out.
  useEffect(() => {
    const map = mapRef.current;
    if (map == null) {
      return;
    }
    const observer = new ResizeObserver(() => map.invalidateSize());
    if (mapContainerRef.current) {
      observer.observe(mapContainerRef.current);
    }
    return () => observer.disconnect();
  }, []);

  return (
    <div style={{ display: "flex", flexDirection: "column", height: "100%", width: "100%" }}>
      <div style={{ padding: "4px 8px", fontSize: 12, opacity: 0.8 }}>
        {boatPos
          ? `Boat: ${boatPos.lat.toFixed(6)}, ${boatPos.lon.toFixed(6)}`
          : `Waiting for GPS on ${GPS_TOPIC}…`}
      </div>
      <div ref={mapContainerRef} style={{ flex: 1, minHeight: 0, width: "100%" }} />
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

  // Cleanup handler invoked by Foxglove when the panel is removed.
  return () => {
    root.unmount();
  };
}
