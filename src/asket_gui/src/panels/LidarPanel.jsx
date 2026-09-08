import { useEffect, useRef } from 'react';

import { Chip, Panel, Row, Rows } from '../components/Panel.jsx';
import { PanelAge } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { bearing, num } from '../lib/format.js';

// PROVISIONAL (Q1): harbour versus open coast changes these substantially.
// Mirrors lidar.warn_radius_m / alarm_radius_m in mission_defaults.yaml.
const WARN_RADIUS_M = 15;
const ALARM_RADIUS_M = 8;

/**
 * Top-down, vessel-centred obstacle view.
 *
 * Note what this panel does **not** do: it does not raise alarms. Obstacle
 * avoidance is handled autonomously by the navigation stack, so the GUI informs
 * rather than alerts. An alarm that fires every time the vessel passes a buoy
 * is an alarm the operator learns to dismiss, and then the one that mattered
 * gets dismissed with it.
 *
 * The raw/filtered toggle exists because during debugging it must be possible
 * to tell whether the sensor or the filter is at fault. It drives the map
 * overlay too, so the two never disagree about what is being looked at.
 */
export function LidarPanel({ state, showRaw, onShowRawChange }) {
  const canvas = useRef(null);
  const lidar = streamPayload(state, 'lidar');
  const ageMs = streamAgeMs(state, 'lidar');
  const rateHz = state.subscriptions.lidar?.rate_hz;
  const subscription = state.subscriptions.lidar;

  const points = (showRaw ? lidar?.raw : lidar?.filtered) || [];
  const nearest = lidar?.nearest_range_m;

  useEffect(() => {
    draw(canvas.current, points, showRaw);
  }, [points, showRaw]);

  const level =
    nearest === null || nearest === undefined
      ? ''
      : nearest < ALARM_RADIUS_M
        ? 'alarm'
        : nearest < WARN_RADIUS_M
          ? 'warn'
          : 'ok';

  return (
    <Panel title="Obstacles" aside={<PanelAge ageMs={ageMs} rateHz={rateHz} />}>
      {subscription && !subscription.granted ? (
        <p className="hint" style={{ margin: 0 }}>
          {/* The server's reason is a full sentence already; prefixing it with
              another one produced "Not carried on this link. not carried on
              the minimal link profile". */}
          Obstacle returns are {subscription.reason}. Avoidance is handled
          on board by the navigation stack and is unaffected.
        </p>
      ) : (
        <>
          <div style={{ display: 'flex', alignItems: 'baseline', gap: 10 }}>
            <span className="big">{num(nearest, 1, ' m')}</span>
            <Chip level={level}>
              {nearest === null || nearest === undefined
                ? 'nothing in range'
                : `nearest, ${bearing(lidar?.nearest_bearing_deg)}`}
            </Chip>
          </div>

          <canvas
            ref={canvas}
            width={340}
            height={340}
            style={{ width: '100%', maxWidth: 340, display: 'block', margin: '8px auto' }}
          />

          <Rows>
            <Row label="Returns">
              {points.length} of {lidar?.points_per_revolution ?? '—'} per revolution
            </Row>
            <Row label="Rotation">{num(lidar?.rotation_hz, 1, ' Hz')}</Row>
          </Rows>

          <div className="button-row" style={{ marginTop: 8 }}>
            <button onClick={() => onShowRawChange(false)} disabled={!showRaw}>
              Filtered
            </button>
            <button
              onClick={() => onShowRawChange(true)}
              disabled={showRaw || lidar?.raw === undefined}
              title={
                lidar?.raw === undefined
                  ? 'Raw returns are only sent at full detail'
                  : 'Show every return, including wave clutter'
              }
            >
              Raw
            </button>
          </div>
          {showRaw && (
            <p className="hint" style={{ marginBottom: 0 }}>
              Raw returns include wave clutter. The navigation stack uses the
              filtered set.
            </p>
          )}
        </>
      )}
    </Panel>
  );
}

/** Vessel-centred plan view: rings at 10 m, hull silhouette, returns. */
function draw(canvas, points, showRaw) {
  if (!canvas) return;
  const ctx = canvas.getContext('2d');
  const { width, height } = canvas;
  const cx = width / 2;
  const cy = height / 2;
  const maxRangeM = 30;
  const scale = (Math.min(width, height) / 2 - 12) / maxRangeM;

  ctx.clearRect(0, 0, width, height);

  // Range rings, labelled. An unlabelled ring is decoration.
  ctx.strokeStyle = '#2b3440';
  ctx.fillStyle = '#93a3b5';
  ctx.font = '10px ui-sans-serif, system-ui, sans-serif';
  ctx.lineWidth = 1;
  for (const ring of [10, 20, 30]) {
    ctx.beginPath();
    ctx.arc(cx, cy, ring * scale, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillText(`${ring} m`, cx + 3, cy - ring * scale + 11);
  }

  // The alarm radius, so distance can be judged rather than read.
  ctx.strokeStyle = '#f2b134';
  ctx.setLineDash([3, 3]);
  ctx.beginPath();
  ctx.arc(cx, cy, ALARM_RADIUS_M * scale, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);

  // Hull silhouette, bow up. Gives every bearing a reference.
  const halfLength = 1.6 * scale * 2;
  const halfBeam = 0.9 * scale * 2;
  ctx.fillStyle = '#4aa3ff';
  ctx.beginPath();
  ctx.moveTo(cx, cy - halfLength);
  ctx.lineTo(cx + halfBeam, cy - halfLength * 0.2);
  ctx.lineTo(cx + halfBeam, cy + halfLength);
  ctx.lineTo(cx - halfBeam, cy + halfLength);
  ctx.lineTo(cx - halfBeam, cy - halfLength * 0.2);
  ctx.closePath();
  ctx.fill();

  ctx.fillStyle = showRaw ? '#ffb37a' : '#ff8a4a';
  for (const [bearingDeg, range] of points) {
    const rad = (bearingDeg * Math.PI) / 180;
    const x = cx + Math.sin(rad) * range * scale;
    const y = cy - Math.cos(rad) * range * scale;
    ctx.beginPath();
    ctx.arc(x, y, 2, 0, Math.PI * 2);
    ctx.fill();
  }
}
