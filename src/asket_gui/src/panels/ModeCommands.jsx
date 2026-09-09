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

  // Which modes this vessel will accept from software, declared by the backend
  // rather than assumed. On a downward-only firmware build AUTONOMOUS is not
  // requestable: the way back up is to release the software clamp and let
  // channel 8 decide, which is what the Autonomous button does there.
  const requests = state.hello?.source?.mode_requests;
  const upwardAllowed = requests?.upward_allowed ?? false;
  const clamped = pico?.software_clamp_active === true;
  const rcMode = pico?.rc_mode ?? null;

  // The vessel declares what a propulsion cut can do here; the GUI does not
  // assume it. Falls back to the honest minimum before the hello arrives.
  const declared = state.hello?.source?.estop;
  const estop = {
    available: declared?.available ?? false,
    label: declared?.label || COMMAND_LABELS.cut_propulsion,
    effect: declared?.effect || '',
    prompt: declared?.effect
      ? `${declared.label || COMMAND_LABELS.cut_propulsion}? ${declared.effect}`
      : CONFIRM_PROMPTS.cut_propulsion,
  };

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
          prompt={
            upwardAllowed
              ? CONFIRM_PROMPTS.set_mode_AUTONOMOUS
              : CONFIRM_PROMPTS.release_clamp
          }
          pending={
            pendingFor('set_mode', 'AUTONOMOUS') || pendingFor('set_mode', 'RELEASE')
          }
          disabled={
            offline || (upwardAllowed ? mode === 'AUTONOMOUS' : !clamped)
          }
          onConfirm={() =>
            connection.command('set_mode', {
              mode: upwardAllowed ? 'AUTONOMOUS' : 'RELEASE',
            })
          }
        />
      </div>

      <div className="button-row" style={{ marginTop: 8 }}>
        <ConfirmButton
          danger
          label={estop.label}
          prompt={estop.prompt}
          pending={pendingFor('cut_propulsion')}
          disabled={offline || !estop.available}
          onConfirm={() => connection.command('cut_propulsion', {})}
        />
      </div>

      {/* On a downward-only build the Autonomous button cannot request
          AUTONOMOUS — the firmware refuses it. What it can do is release the
          software clamp, after which channel 8 decides. Say which of the two is
          on offer rather than leaving a button whose meaning has quietly
          changed. */}
      {!upwardAllowed && (
        <p className="hint" style={{ marginBottom: 0, marginTop: 6 }}>
          {clamped
            ? `Autonomous releases the software clamp. The transmitter then decides${
                rcMode ? `, and channel 8 currently selects ${rcMode}` : ''
              }.`
            : 'No software clamp is held. Channel 8 alone decides the mode; ' +
              'this build does not let the GUI request AUTONOMOUS.'}
        </p>
      )}

      {/* What this button does depends on the vessel, and the vessel says so.
          A button labelled "Cut propulsion" that quietly dropped the mode
          instead would be the single most dangerous thing on this screen, so it
          is labelled with what it actually does and the effect is spelled out
          underneath. */}
      {estop.effect && (
        <p className="warnline" style={{ marginBottom: 0, marginTop: 6 }}>
          {estop.effect}
        </p>
      )}

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
