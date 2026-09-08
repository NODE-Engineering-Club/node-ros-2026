import { useEffect, useState } from 'react';

import { Chip, Panel, Row, Rows } from '../components/Panel.jsx';
import { ConfirmButton } from '../components/ConfirmButton.jsx';
import { PanelAge } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { int, num, pct } from '../lib/format.js';

/**
 * Sonar health, and the controls for it.
 *
 * The row that matters most is the least obvious one. `clock_offset_ms` is the
 * difference between the sonar's own clock and the Jetson's, and the entire
 * post-mission fusion strategy — record the raw sonar stream and the trajectory
 * separately, pair them afterwards on that timestamp — rests on it staying
 * small. If it drifts and nobody notices, the survey is not degraded, it is
 * worthless, and nobody finds out until the data is opened back home. So it is
 * given a line of its own with an explicit consequence, not a number in a list.
 *
 * Parameter changes are commands like any other: they show as pending until the
 * **sonar** reports the new setting. A range that never took effect must not
 * read as applied.
 */
export function SonarPanel({ state, connection }) {
  const sonar = streamPayload(state, 'sonar');
  const ageMs = streamAgeMs(state, 'sonar');
  const rateHz = state.subscriptions.sonar?.rate_hz;

  const [rangeM, setRangeM] = useState(30);
  const [gain, setGain] = useState(4);
  const [pingRate, setPingRate] = useState(5);
  const [touched, setTouched] = useState(false);

  // Track the vessel's settings until the operator starts editing, so the
  // controls do not show a stale draft as if it were current.
  useEffect(() => {
    if (touched || !sonar) return;
    if (sonar.range_setting_m !== undefined && sonar.range_setting_m !== null) {
      setRangeM(sonar.range_setting_m);
      setGain(sonar.gain_setting ?? 4);
      setPingRate(sonar.commanded_ping_rate_hz ?? 5);
    }
  }, [sonar, touched]);

  const pending = Object.values(state.commands).some(
    (c) => c.name === 'set_ping_parameters' && c.status === 'pending',
  );
  const latest = Object.values(state.commands)
    .filter((c) => c.name === 'set_ping_parameters')
    .pop();

  const connected = sonar?.connected;
  const clockLevel = sonar?.clock_compromised ? 'alarm' : sonar?.clock_ok ? 'ok' : 'warn';

  return (
    <Panel
      title="Sonar"
      aside={
        <>
          <Chip level={connected ? 'ok' : 'alarm'}>
            {connected ? 'connected' : 'no data'}
          </Chip>
          <PanelAge ageMs={ageMs} rateHz={rateHz} />
        </>
      }
    >
      <Rows>
        <Row label="Ping rate">
          <span
            className={sonar?.ping_rate_ok === false ? 'warnline' : ''}
            title={
              sonar?.rate_from_device
                ? "The sonar's own figure, from END_PING_INFO."
                : 'Measured from arrival times here, which also measures the network.'
            }
          >
            {num(sonar?.actual_ping_rate_hz, 1)} / {num(sonar?.commanded_ping_rate_hz, 1, ' Hz')}
            {sonar?.rate_from_device === false && <span className="hint"> (measured)</span>}
          </span>
        </Row>
        <Row label="Points per ping">
          {int(sonar?.valid_points_per_ping)} of {int(sonar?.points_per_ping)}
        </Row>
        <Row label="Speed of sound">{num(sonar?.speed_of_sound, 0, ' m/s')}</Row>
        <Row label="Packet loss">
          <span className={(sonar?.packet_loss_ratio ?? 0) > 0.05 ? 'warnline' : ''}>
            {pct(sonar?.packet_loss_ratio, 1)}
          </span>
        </Row>
        <Row label="Pitch / roll">
          {num(sonar?.pitch_deg, 1, '°')} / {num(sonar?.roll_deg, 1, '°')}
        </Row>
        <Row label="Framing errors">
          {int(sonar?.checksum_errors)} bad, {int(sonar?.bytes_discarded)} B lost
        </Row>
      </Rows>

      <div style={{ marginTop: 8 }}>
        <Chip level={clockLevel} title="Sonar clock versus the Jetson's">
          Clock {int(sonar?.clock_offset_ms, ' ms')}
        </Chip>
        {sonar?.clock_compromised && (
          <p className="errline" style={{ marginBottom: 0, marginTop: 4 }}>
            Data recorded now cannot be georeferenced afterwards. Check the NTP
            server on the Jetson before continuing the survey.
          </p>
        )}
        {!sonar?.clock_ok && !sonar?.clock_compromised && (
          <p className="warnline" style={{ marginBottom: 0, marginTop: 4 }}>
            Clock drifting. Post-mission fusion pairs the sonar stream and the
            trajectory on this timestamp.
          </p>
        )}
      </div>

      <div style={{ marginTop: 10 }}>
        <Rows>
          <Row label={`Range ${num(rangeM, 0, ' m')}`}>
            <input
              type="range"
              min="5"
              max="100"
              step="1"
              value={rangeM}
              onChange={(e) => {
                setTouched(true);
                setRangeM(Number(e.target.value));
              }}
            />
          </Row>
          <Row label={`Gain ${gain < 0 ? 'auto' : gain}`}>
            <input
              type="range"
              min="-1"
              max="10"
              step="1"
              value={gain}
              onChange={(e) => {
                setTouched(true);
                setGain(Number(e.target.value));
              }}
            />
          </Row>
          <Row label={`Ping rate ${num(pingRate, 1, ' Hz')}`}>
            <input
              type="range"
              min="1"
              max="20"
              step="0.5"
              value={pingRate}
              onChange={(e) => {
                setTouched(true);
                setPingRate(Number(e.target.value));
              }}
            />
          </Row>
        </Rows>

        <div className="button-row" style={{ marginTop: 6 }}>
          <ConfirmButton
            label="Apply to sonar"
            prompt={`Set ${rangeM} m, gain ${gain}, ${pingRate} Hz?`}
            pending={pending}
            disabled={!state.connected}
            onConfirm={() => {
              setTouched(false);
              connection.command('set_ping_parameters', {
                range_m: rangeM,
                gain,
                ping_rate_hz: pingRate,
              });
            }}
          />
        </div>

        {latest && latest.status === 'failed' && (
          <p className="errline" style={{ marginBottom: 0 }}>
            The sonar did not report the new settings. {latest.detail}
          </p>
        )}
        <p className="hint" style={{ marginBottom: 0, marginTop: 6 }}>
          A longer range forces a lower ping rate. Watch the actual rate above:
          the sonar reduces it on its own when the range demands it, and reports
          what it achieved. Gain −1 is auto, which Cerulean recommend.
        </p>
      </div>
    </Panel>
  );
}
