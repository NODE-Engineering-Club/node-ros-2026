import { useEffect, useMemo, useState } from 'react';

import { AlarmPanel } from './panels/AlarmPanel.jsx';
import { HeadingPanel } from './panels/HeadingPanel.jsx';
import { LinkStatus } from './panels/LinkStatus.jsx';
import { MissionMap } from './panels/MissionMap.jsx';
import { ModeCommands } from './panels/ModeCommands.jsx';
import { VesselState } from './panels/VesselState.jsx';
import { Chip } from './components/Panel.jsx';
import { streamPayload } from './lib/connection.js';
import { useStore } from './lib/useStore.js';
import { MODE_LABELS } from './lib/labels.js';

/**
 * What this client asks for.
 *
 * These are *requests*, not entitlements. The server decides what it will
 * actually serve, based on the link profile, and tells us what we are getting
 * and why. Nothing arrives that is not listed here — the backend pushes
 * nothing by default.
 */
const SUBSCRIPTIONS = [
  { name: 'vessel', rate_hz: 5 },
  { name: 'pico', rate_hz: 2 },
  { name: 'heading', rate_hz: 2 },
  { name: 'link', rate_hz: 1 },
  { name: 'track', rate_hz: 1 },
  { name: 'coverage', rate_hz: 1 },
  { name: 'lidar', rate_hz: 5 },
  { name: 'plan' },
];

export function App({ connection }) {
  const state = useStore(connection);
  const [follow, setFollow] = useState(true);

  useEffect(() => {
    connection.subscribe(SUBSCRIPTIONS);
  }, [connection]);

  const pico = streamPayload(state, 'pico');
  const worstAlarm = useMemo(() => {
    const alarms = state.alarms || [];
    if (alarms.some((a) => a.severity === 'alarm')) return 'alarm';
    if (alarms.length) return 'warn';
    return 'ok';
  }, [state.alarms]);

  return (
    <div className="app">
      <header className="topbar">
        <strong>Asket</strong>
        <Chip level={state.connected ? 'ok' : 'alarm'}>
          {state.connected ? 'Connected' : state.connecting ? 'Reconnecting…' : 'Offline'}
        </Chip>
        <span className={`mode-badge mode-${pico?.mode ?? 'UNKNOWN'}`}>
          {MODE_LABELS[pico?.mode] ?? 'Unknown'}
        </span>
        <Chip level={worstAlarm}>
          {state.alarms?.length ? `${state.alarms.length} alarm(s)` : 'No alarms'}
        </Chip>
        <div className="spacer" />
        {state.hello?.source?.mode === 'sim' && (
          <Chip level="warn" title="Every value on this screen is simulated.">
            Simulation
          </Chip>
        )}
      </header>

      <MissionMap state={state} follow={follow} onFollowChange={setFollow} />

      <aside className="sidebar">
        <AlarmPanel state={state} />
        <VesselState state={state} />
        <ModeCommands state={state} connection={connection} />
        <HeadingPanel state={state} />
        <LinkStatus state={state} connection={connection} />
      </aside>
    </div>
  );
}
