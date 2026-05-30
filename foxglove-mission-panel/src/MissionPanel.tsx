// Njord Mission panel.
//
// Minimal scaffold: renders an empty panel and wires up the Foxglove render
// lifecycle. Real features (map view, GPS-centred view, waypoint management)
// are added on top of this skeleton in later steps. Keeping the panel mounting
// logic (`initMissionPanel`) separate from the React component (`MissionPanel`)
// makes the component independently testable and easy to extend.

import { PanelExtensionContext } from "@foxglove/extension";
import { ReactElement, StrictMode, useEffect, useLayoutEffect, useState } from "react";
import { createRoot, Root } from "react-dom/client";

function MissionPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  // `renderDone` must be called once the panel has finished rendering a frame so
  // Foxglove can throttle updates correctly. We store it in state and invoke it
  // after each commit.
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  useLayoutEffect(() => {
    // Foxglove drives the panel by calling `onRender` whenever new data is
    // available. For now we only capture `done`; data subscriptions are added
    // when the map/waypoint features land.
    context.onRender = (_renderState, done) => {
      setRenderDone(() => done);
    };
  }, [context]);

  // Signal render completion after every commit.
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ padding: "1rem", height: "100%", boxSizing: "border-box" }}>
      <h2 style={{ margin: 0 }}>Njord Mission</h2>
      <p style={{ opacity: 0.7 }}>Panel scaffold — map and waypoints coming soon.</p>
    </div>
  );
}

export function initMissionPanel(context: PanelExtensionContext): () => void {
  const root: Root = createRoot(context.panelElement);
  root.render(
    <StrictMode>
      <MissionPanel context={context} />
    </StrictMode>,
  );

  // Cleanup handler invoked by Foxglove when the panel is removed.
  return () => {
    root.unmount();
  };
}
