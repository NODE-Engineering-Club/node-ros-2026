import { useEffect, useRef } from 'react';

import { Chip, Panel, Row, Rows } from '../components/Panel.jsx';
import { PanelAge } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { bearing, int, num } from '../lib/format.js';

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

  // Stalled is not the same as clear water, and only one of them needs a person.
  const stalled = lidar?.rotation_hz !== undefined && lidar.rotation_hz < 1;
  const forced = stalled
    ? 'the lidar is not turning'
    : level === 'alarm'
      ? `an obstacle is inside ${ALARM_RADIUS_M} m`
      : level === 'warn'
        ? `an obstacle is inside ${WARN_RADIUS_M} m`
        : '';

  return (
    <Panel
      title="Obstacles"
      id="obstacles"
      collapsible
      forceOpen={Boolean(forced)}
      forceReason={forced}
      aside={<PanelAge ageMs={ageMs} rateHz={rateHz} />}
      summary={
        subscription && !subscription.granted ? null : (
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
              style={{ width: '100%', maxWidth: 300, display: 'block', margin: '6px auto' }}
            />
          </>
        )
      }
    >
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
          <Rows>
            {/* Returns against beams *swept*. Both used to be the return count,
                so the row always read "N of N" and open water was
                indistinguishable from a blind sensor. */}
            <Row label="Returns">
              {int(lidar?.points_per_revolution)} of {int(lidar?.beams_per_revolution)} beams
            </Row>
            <Row label="Rotation">
              <span className={stalled ? 'errline' : ''}>
                {num(lidar?.rotation_hz, 1, ' Hz')}
              </span>
            </Row>
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
  //
  // These were still the dark theme's colours: a #93a3b5 label on white is a
  // label nobody can read, which is why the rings looked unlabelled and the
  // alarm radius looked absent.
  ctx.strokeStyle = '#b0b0b0';
  ctx.fillStyle = '#000000';
  ctx.font = '10px ui-sans-serif, system-ui, sans-serif';
  ctx.lineWidth = 1;
  for (const ring of [10, 20, 30]) {
    ctx.beginPath();
    ctx.arc(cx, cy, ring * scale, 0, Math.PI * 2);
    ctx.stroke();
    ctx.fillText(`${ring} m`, cx + 3, cy - ring * scale + 11);
  }

  // The alarm radius, so distance can be judged rather than read. Drawn
  // whether or not anything is inside it — an operator has to be able to see
  // where the boundary is *before* something crosses it.
  ctx.strokeStyle = '#a25a00';
  ctx.lineWidth = 2;
  ctx.setLineDash([4, 3]);
  ctx.beginPath();
  ctx.arc(cx, cy, ALARM_RADIUS_M * scale, 0, Math.PI * 2);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.lineWidth = 1;

  // Hull silhouette, bow up. Gives every bearing a reference.
  const halfLength = 1.6 * scale * 2;
  const halfBeam = 0.9 * scale * 2;
  ctx.fillStyle = '#0b57d0';
  ctx.beginPath();
  ctx.moveTo(cx, cy - halfLength);
  ctx.lineTo(cx + halfBeam, cy - halfLength * 0.2);
  ctx.lineTo(cx + halfBeam, cy + halfLength);
  ctx.lineTo(cx - halfBeam, cy + halfLength);
  ctx.lineTo(cx - halfBeam, cy - halfLength * 0.2);
  ctx.closePath();
  ctx.fill();

  ctx.fillStyle = showRaw ? '#e88a4a' : '#d34500';
  for (const [bearingDeg, range] of points) {
    const rad = (bearingDeg * Math.PI) / 180;
    const x = cx + Math.sin(rad) * range * scale;
    const y = cy - Math.cos(rad) * range * scale;
    ctx.beginPath();
    ctx.arc(x, y, 2, 0, Math.PI * 2);
    ctx.fill();
  }
}
