import { Chip, Panel } from '../components/Panel.jsx';
import { ConfirmButton } from '../components/ConfirmButton.jsx';
import { streamPayload } from '../lib/connection.js';

/**
 * Pre-flight built-in test.
 *
 * The purpose is narrow and worth restating: when something is wrong on a
 * Namibian beach, identify the faulty link in thirty seconds rather than an
 * hour. So every failing item shows its **remedy**, not just its status — a red
 * word is not an instruction.
 *
 * A check that could not run shows as "unknown", never as a pass. Where the
 * unknown check is one we must be able to confirm, the overall verdict is
 * NO-GO: "GO, 14 checks could not run" is the most dangerous sentence this
 * panel could produce.
 */
const STATUS_LEVEL = { PASS: 'ok', WARN: 'warn', FAIL: 'alarm', SKIPPED: '' };
const STATUS_LABEL = { PASS: 'ok', WARN: 'warn', FAIL: 'fail', SKIPPED: 'unknown' };

export function DiagnosticsPanel({ state, connection }) {
  const report = streamPayload(state, 'diagnostics');
  const pending = Object.values(state.commands).some(
    (c) => c.name === 'run_system_test' && c.status === 'pending',
  );

  const items = report?.items || [];
  const degrading = report?.history?.degrading || [];

  return (
    <Panel
      title="Pre-flight"
      aside={
        report ? (
          <Chip level={report.go ? 'ok' : 'alarm'}>{report.go ? 'GO' : 'NO-GO'}</Chip>
        ) : null
      }
    >
      {report ? (
        <p
          className={report.go ? '' : 'errline'}
          style={{ margin: '0 0 8px', fontWeight: 600 }}
        >
          {report.summary}
        </p>
      ) : (
        <p className="hint" style={{ margin: '0 0 8px' }}>
          No pre-flight has run yet in this session.
        </p>
      )}

      <div className="button-row">
        <ConfirmButton
          label="Run pre-flight"
          prompt="Run all passive checks?"
          pending={pending}
          disabled={!state.connected}
          onConfirm={() => connection.command('run_system_test', {})}
        />
      </div>
      <p className="hint" style={{ marginTop: 6, marginBottom: 0 }}>
        Passive checks only. Motor tests never run automatically and are not
        available from this panel.
      </p>

      {items.length > 0 && (
        <div style={{ marginTop: 10 }}>
          {items.map((item) => (
            <div
              key={item.id}
              style={{ borderTop: '1px solid #2b3440', padding: '6px 0' }}
            >
              <div style={{ display: 'flex', justifyContent: 'space-between', gap: 8 }}>
                <span>{item.name}</span>
                <Chip level={STATUS_LEVEL[item.status]}>{STATUS_LABEL[item.status]}</Chip>
              </div>
              <div className="hint">{item.message}</div>
              {item.remedy && item.status !== 'PASS' && (
                <div className={item.status === 'FAIL' ? 'errline' : 'warnline'}>
                  {item.remedy}
                </div>
              )}
              {degrading.includes(item.id) && (
                <div className="warnline">
                  Drifting across recent runs — still passing, but getting worse.
                </div>
              )}
            </div>
          ))}
        </div>
      )}
    </Panel>
  );
}
