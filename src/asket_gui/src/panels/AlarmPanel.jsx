import { Panel } from '../components/Panel.jsx';

/**
 * Active alarms.
 *
 * Deliberately few. Only conditions that cannot self-correct and that need a
 * person to do something appear here; obstacle detection, for instance, does
 * not, because avoidance is the navigation stack's job and an alarm an operator
 * learns to dismiss is worse than no alarm.
 *
 * Every entry carries a remedy, because a red word is not an instruction.
 */
export function AlarmPanel({ state }) {
  const alarms = state.alarms || [];
  if (alarms.length === 0) {
    return (
      <Panel title="Alarms">
        <p className="hint" style={{ margin: 0 }}>Nothing needs attention.</p>
      </Panel>
    );
  }

  return (
    <Panel title={`Alarms (${alarms.length})`}>
      <div className="alarm-list">
        {alarms.map((alarm) => (
          <div key={alarm.key} className={`alarm-item ${alarm.severity}`}>
            <div className="message">{alarm.message}</div>
            {alarm.remedy && <div className="remedy">{alarm.remedy}</div>}
          </div>
        ))}
      </div>
    </Panel>
  );
}
