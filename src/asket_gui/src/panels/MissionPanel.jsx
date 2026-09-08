import { useState } from 'react';

import { Chip, Panel, Row, Rows } from '../components/Panel.jsx';
import { ConfirmButton } from '../components/ConfirmButton.jsx';
import { PanelAge } from '../components/Value.jsx';
import { streamAgeMs, streamPayload } from '../lib/connection.js';
import { bytes, duration } from '../lib/format.js';

/**
 * Mission recording, and the missions already on the disk.
 *
 * "Recording" here means the recorder reports bytes landing on the disk, not
 * that a start request was accepted — same rule as vessel mode.
 *
 * Export is only offered to a detected fast path. When there is none, the
 * operator is told what to plug in rather than shown a dead button: a disabled
 * control with no explanation is the thing that wastes an hour on a beach.
 */
export function MissionPanel({ state, connection }) {
  const mission = streamPayload(state, 'mission');
  const ageMs = streamAgeMs(state, 'mission');
  const rateHz = state.subscriptions.mission?.rate_hz;

  const [name, setName] = useState('');
  const [selected, setSelected] = useState(null);

  const recording = mission?.state === 'RECORDING';
  const errored = mission?.state === 'ERROR';
  const destinations = mission?.destinations || [];
  const missions = mission?.missions || [];

  const pending = (command) =>
    Object.values(state.commands).some((c) => c.name === command && c.status === 'pending');

  const latest = Object.values(state.commands)
    .filter((c) => c.name?.endsWith('_mission'))
    .pop();

  // The verdict a collapsed recording panel has to carry: not "12.4 GB written"
  // but how long it can keep going. Disk that runs out mid-line is the failure
  // this row exists to prevent.
  const remaining = mission?.estimated_remaining_s;
  const verdict = errored
    ? mission.error_message
    : recording
      ? (remaining === null || remaining === undefined
        ? `Recording ${mission.name}.`
        : `Recording ${mission.name} — room for ${duration(remaining)} more.`)
      : 'Not recording.';

  const forced = errored
    ? 'the recorder has stopped with an error'
    : recording && remaining !== null && remaining !== undefined && remaining < 900
      ? 'less than 15 minutes of recording space left'
      : '';

  return (
    <Panel
      title="Recording"
      id="recording"
      collapsible
      forceOpen={Boolean(forced)}
      forceReason={forced}
      aside={
        <>
          <Chip level={recording ? 'warn' : errored ? 'alarm' : ''}>
            {mission?.state || '—'}
          </Chip>
          <PanelAge ageMs={ageMs} rateHz={rateHz} />
        </>
      }
      summary={
        <p className={errored ? 'errline' : 'hint'} style={{ margin: '0 0 6px' }}>
          {verdict}
        </p>
      }
    >
      {recording ? (
        <>
          <Rows>
            <Row label="Mission">{mission.name}</Row>
            <Row label="Elapsed">{duration(mission.elapsed_s)}</Row>
            <Row label="Written">{bytes(mission.bytes_written)}</Row>
            <Row label="Trajectory">{mission.trajectory_records} samples</Row>
            <Row label="Disk free">{bytes(mission.disk_free_bytes)}</Row>
            <Row label="Recording left">
              {mission.estimated_remaining_s === null
                ? '—'
                : duration(mission.estimated_remaining_s)}
            </Row>
          </Rows>
          <div className="button-row" style={{ marginTop: 8 }}>
            <ConfirmButton
              label="Stop recording"
              prompt="Stop and close the mission files?"
              pending={pending('stop_mission')}
              onConfirm={() => connection.command('stop_mission', {})}
            />
          </div>
        </>
      ) : (
        <>
          <input
            type="text"
            value={name}
            placeholder="Mission name"
            onChange={(e) => setName(e.target.value)}
          />
          <div className="button-row" style={{ marginTop: 8 }}>
            <ConfirmButton
              label="Start recording"
              prompt={`Start recording "${name || 'mission'}"?`}
              pending={pending('start_mission')}
              disabled={!state.connected}
              onConfirm={() => connection.command('start_mission', { name: name || 'mission' })}
            />
          </div>
        </>
      )}

      {errored && (
        <p className="errline" style={{ marginBottom: 0 }}>{mission.error_message}</p>
      )}
      {latest && latest.status === 'failed' && (
        <p className="errline" style={{ marginBottom: 0 }}>{latest.detail}</p>
      )}

      <h2 style={{ marginTop: 14 }}>
        <span>Stored missions</span>
        <span className="age">{missions.length}</span>
      </h2>

      {missions.length === 0 && (
        <p className="hint" style={{ margin: 0 }}>Nothing recorded yet.</p>
      )}

      {missions.map((m) => (
        <div
          key={m.path}
          style={{
            borderTop: '1px solid var(--line)',
            padding: '6px 0',
            cursor: 'pointer',
            opacity: selected === m.path ? 1 : 0.85,
          }}
          onClick={() => setSelected(selected === m.path ? null : m.path)}
        >
          <div style={{ display: 'flex', justifyContent: 'space-between', gap: 8 }}>
            <strong>{m.name}</strong>
            <span className="mono">{bytes(m.size_bytes)}</span>
          </div>
          <div className="hint">
            {duration(m.duration_s)}
            {!m.complete && (
              <>
                {' · '}
                <span className="warnline">interrupted</span>
              </>
            )}
          </div>

          {selected === m.path && (
            <div style={{ marginTop: 6 }}>
              {destinations.length === 0 ? (
                <p className="warnline" style={{ margin: '0 0 6px' }}>
                  {mission?.no_fast_path_message}
                </p>
              ) : (
                <div className="button-row">
                  {destinations.map((d) => (
                    <ConfirmButton
                      key={d.path}
                      label={`Export to ${d.label}`}
                      prompt={`Copy ${bytes(m.size_bytes)} to ${d.label} and verify?`}
                      pending={pending('export_mission')}
                      disabled={recording}
                      onConfirm={() =>
                        connection.command('export_mission', {
                          path: m.path,
                          destination: d.path,
                        })
                      }
                    />
                  ))}
                </div>
              )}
              {recording && (
                <p className="hint" style={{ margin: '4px 0 0' }}>
                  Export is blocked while recording — copying gigabytes competes
                  with the recorder for the disk.
                </p>
              )}
              <div className="button-row" style={{ marginTop: 6 }}>
                <ConfirmButton
                  danger
                  label="Delete"
                  prompt={`Permanently delete ${m.name}? This cannot be undone.`}
                  pending={pending('delete_mission')}
                  onConfirm={() =>
                    connection.command('delete_mission', { path: m.path, confirm: true })
                  }
                />
              </div>
            </div>
          )}
        </div>
      ))}
    </Panel>
  );
}
