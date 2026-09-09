import { Panel } from '../components/Panel.jsx';
import { ConfirmButton } from '../components/ConfirmButton.jsx';
import { streamPayload } from '../lib/connection.js';
import { COMMAND_LABELS, COMMAND_STATUS_LABELS, CONFIRM_PROMPTS } from '../lib/labels.js';

/**
 * The three mode commands.
 *
 * Safety rules 3, 4 and 5 all land here:
 *
 * - the soft ESTOP is labelled **Cut propulsion**, never "Emergency stop";
 * - each command is two-step: click to arm, click to send;
 * - the *displayed* mode comes from the vessel and lives in the Vessel panel.
 *   Nothing in here changes it. A command in flight shows as an outlined,
 *   pulsing button and a "waiting for the vessel" line, and it either becomes
 *   a confirmation from the Pico or an explicit failure.
 */
export function ModeCommands({ state, connection }) {
  const pico = streamPayload(state, 'pico');
  const mode = pico?.mode;

  const recent = Object.values(state.commands)
    .filter((c) => c.name === 'set_mode' || c.name === 'cut_propulsion')
    .sort((a, b) => (a.issued_utc_ms || 0) - (b.issued_utc_ms || 0));
  const latest = recent[recent.length - 1];
  const inFlight = recent.filter((c) => c.status === 'pending');

  const pendingFor = (name, argMode) =>
    inFlight.some((c) => c.name === name && (!argMode || c.args?.mode === argMode));

  const offline = !state.connected;

  return (
    <Panel title="Mode">
      <div className="button-row">
        <ConfirmButton
          label={COMMAND_LABELS.set_mode_MANUAL}
          prompt={CONFIRM_PROMPTS.set_mode_MANUAL}
          pending={pendingFor('set_mode', 'MANUAL')}
          disabled={offline || mode === 'MANUAL'}
          onConfirm={() => connection.command('set_mode', { mode: 'MANUAL' })}
        />
        <ConfirmButton
          label={COMMAND_LABELS.set_mode_AUTONOMOUS}
          prompt={CONFIRM_PROMPTS.set_mode_AUTONOMOUS}
          pending={pendingFor('set_mode', 'AUTONOMOUS')}
          disabled={offline || mode === 'AUTONOMOUS'}
          onConfirm={() => connection.command('set_mode', { mode: 'AUTONOMOUS' })}
        />
      </div>

      <div className="button-row" style={{ marginTop: 8 }}>
        <ConfirmButton
          danger
          label={COMMAND_LABELS.cut_propulsion}
          prompt={CONFIRM_PROMPTS.cut_propulsion}
          pending={pendingFor('cut_propulsion')}
          disabled={offline}
          onConfirm={() => connection.command('cut_propulsion', {})}
        />
      </div>

      {latest && (
        <p
          className={
            latest.status === 'failed' ? 'errline' : latest.status === 'pending' ? 'hint' : 'hint'
          }
          style={{ marginBottom: 0 }}
        >
          {describe(latest)}
        </p>
      )}

      <p className="hint" style={{ marginBottom: 0, marginTop: 8 }}>
        This is a software latch. The hardware killswitch and RC channel&nbsp;8 cut
        propulsion independently and always work.
      </p>
    </Panel>
  );
}

function describe(command) {
  const what =
    command.name === 'cut_propulsion'
      ? COMMAND_LABELS.cut_propulsion
      : `Mode → ${command.args?.mode ?? ''}`;
  const status = COMMAND_STATUS_LABELS[command.status] || command.status;
  return command.status === 'failed'
    ? `${what}: ${status}. ${command.detail || ''}`
    : `${what}: ${status}`;
}
