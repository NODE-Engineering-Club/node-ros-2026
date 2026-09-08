import { useEffect, useMemo, useState } from 'react';

import { AlarmPanel } from './panels/AlarmPanel.jsx';
import { DiagnosticsPanel } from './panels/DiagnosticsPanel.jsx';
import { HeadingPanel } from './panels/HeadingPanel.jsx';
import { LidarPanel } from './panels/LidarPanel.jsx';
import { LinkStatus } from './panels/LinkStatus.jsx';
import { MissionMap } from './panels/MissionMap.jsx';
import { MissionPanel } from './panels/MissionPanel.jsx';
import { ModeCommands } from './panels/ModeCommands.jsx';
import { PowerPanel } from './panels/PowerPanel.jsx';
import { SonarPanel } from './panels/SonarPanel.jsx';
import { VesselState } from './panels/VesselState.jsx';
import { Chip } from './components/Panel.jsx';
import { streamPayload } from './lib/connection.js';
import { useStore } from './lib/useStore.js';
import { MODE_LABELS } from './lib/labels.js';
import { duration } from './lib/format.js';

//: A socket can stay open long after the link behind it has gone. TCP takes
//: tens of seconds to notice, and for all of that time `connected` is true
//: while nothing is arriving. So the header reports what has actually been
//: RECEIVED, not what the socket believes about itself.
const NO_DATA_AFTER_MS = 6000;

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
  { name: 'power', rate_hz: 1 },
  { name: 'sonar', rate_hz: 1 },
  { name: 'mission', rate_hz: 1 },
  { name: 'diagnostics', rate_hz: 0.2 },
  { name: 'plan' },
];

export function App({ connection }) {
  const state = useStore(connection);
  const [follow, setFollow] = useState(true);
  // One toggle drives both the lidar panel and the map overlay, so the two can
  // never disagree about which point set is being looked at.
  const [showRawLidar, setShowRawLidar] = useState(false);

  useEffect(() => {
    connection.subscribe(SUBSCRIPTIONS);
  }, [connection]);

  const pico = streamPayload(state, 'pico');
  const sinceMessageMs = state.lastMessageAt ? Date.now() - state.lastMessageAt : null;
  const receiving = sinceMessageMs !== null && sinceMessageMs < NO_DATA_AFTER_MS;
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
        <Chip
          level={receiving ? 'ok' : 'alarm'}
          title={
            state.connected && !receiving
              ? 'The socket is still open but nothing is arriving. Everything on screen is old.'
              : ''
          }
        >
          {receiving
            ? 'Receiving'
            : state.connected
              ? `No data for ${duration((sinceMessageMs ?? 0) / 1000)}`
              : state.connecting
                ? 'Reconnecting…'
                : 'Offline'}
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

      <MissionMap
        state={state}
        follow={follow}
        onFollowChange={setFollow}
        showRawLidar={showRawLidar}
      />

      <aside className="sidebar">
        <AlarmPanel state={state} />
        <VesselState state={state} />
        <ModeCommands state={state} connection={connection} />
        <HeadingPanel state={state} />
        <LidarPanel
          state={state}
          showRaw={showRawLidar}
          onShowRawChange={setShowRawLidar}
        />
        <PowerPanel state={state} />
        <SonarPanel state={state} connection={connection} />
        <MissionPanel state={state} connection={connection} />
        <DiagnosticsPanel state={state} connection={connection} />
        <LinkStatus state={state} connection={connection} />
      </aside>
    </div>
  );
}
