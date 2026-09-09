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
  // rather than assumed.
  const requests = state.hello?.source?.mode_requests;
  const upwardAllowed = requests?.upward_allowed ?? false;
  const clamped = pico?.software_clamp_active === true;
  const rcMode = pico?.rc_mode ?? null;

  // Why the propellers cannot turn, computed by the backend so this panel, the
  // command line below and the logs all say the same sentence. Null when
  // nothing is blocking, and null when we simply have not been told — not
  // knowing is not evidence of a block.
  //
  // This is the state most likely to be misread: a mode request can be
  // accepted, arbitrated and confirmed while the vessel moves nothing at all,
  // because arming is RC channel 7 and no software request can raise it.
  const armingBlock = pico?.arming_block ?? null;

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
          // The prompt carries the arming state, because this is the click
          // where it matters: the mode will change and the boat will not move.
          prompt={
            armingBlock
              ? `${CONFIRM_PROMPTS.set_mode_AUTONOMOUS} Note: ${armingBlock}.`
              : CONFIRM_PROMPTS.set_mode_AUTONOMOUS
          }
          pending={pendingFor('set_mode', 'AUTONOMOUS')}
          // NOT disabled when arming blocks. Setting AUTONOMOUS from the GUI
          // and then arming on the transmitter is the normal launch sequence —
          // somebody is standing next to the boat. Disabling the button would
          // break the sequence; annotating it makes the order legible.
          disabled={offline || !upwardAllowed || mode === 'AUTONOMOUS'}
          onConfirm={() => connection.command('set_mode', { mode: 'AUTONOMOUS' })}
        />
      </div>

      {/* Release is its own action with its own label. It hands authority back
          to the transmitter rather than asking for a mode, and the two should
          never have shared a button. Only offered when there is a clamp to
          release: otherwise it would do nothing and say nothing. */}
      <div className="button-row" style={{ marginTop: 8 }}>
        <ConfirmButton
          label={COMMAND_LABELS.set_mode_RELEASE}
          prompt={CONFIRM_PROMPTS.set_mode_RELEASE}
          pending={pendingFor('set_mode', 'RELEASE')}
          disabled={offline || !clamped}
          onConfirm={() => connection.command('set_mode', { mode: 'RELEASE' })}
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

      {/* The one line that stops an operator guessing whether the boat ignored
          them or the switch did. Shown whenever propulsion is blocked, not only
          after a command: the state exists before anybody presses anything, and
          the answer to "why is nothing happening" should already be on screen.

          Deliberately a warning, not an error. A disarmed boat on the slipway
          is the correct and expected state — this says what to do next, it does
          not report a fault. */}
      {armingBlock && (
        <p className="warnline" style={{ marginBottom: 0, marginTop: 6 }}>
          {armingBlock}. Arming is RC channel 7 and cannot be commanded from
          here — a mode set now takes effect when somebody arms the boat.
        </p>
      )}

      {/* A build that refuses upward requests would leave the Autonomous button
          permanently disabled. Say so rather than letting it look broken. */}
      {!upwardAllowed && (
        <p className="hint" style={{ marginBottom: 0, marginTop: 6 }}>
          This build does not let the GUI request AUTONOMOUS
          (SOFTWARE_UPWARD_REQUESTS_ALLOWED is 0). Release to RC and channel 8
          decides
          {rcMode ? `; it currently selects ${rcMode}` : ''}.
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
