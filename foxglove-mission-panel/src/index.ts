// Extension entry point.
//
// Foxglove Studio loads this module and calls `activate()` once. We register the
// Njord Mission panel here. Keeping registration in this thin entry file (and the
// panel implementation in its own module) makes it easy to add more panels later
// without touching the panel code itself.

import { ExtensionContext } from "@foxglove/extension";

import { initMissionPanel } from "./MissionPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Njord Mission",
    initPanel: initMissionPanel,
  });
}
