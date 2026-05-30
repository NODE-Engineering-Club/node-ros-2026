#!/usr/bin/env bash
# Build (and optionally package) the Njord Mission Foxglove extension.
#
# Wraps the npm steps so the team can build the panel without knowing the npm
# internals. Run from anywhere in the repo.
#
# Usage:
#   scripts/build-foxglove-panel.sh           # install deps + build
#   scripts/build-foxglove-panel.sh package   # install deps + build + .foxe
set -euo pipefail

# Resolve the extension directory relative to this script, so the command works
# regardless of the current working directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PANEL_DIR="$SCRIPT_DIR/../foxglove-mission-panel"

if ! command -v npm >/dev/null 2>&1; then
  echo "error: npm is required but not found. Install Node.js (>=18)." >&2
  exit 1
fi

cd "$PANEL_DIR"

echo "==> Installing dependencies"
npm install

echo "==> Building extension"
npm run build

if [[ "${1:-}" == "package" ]]; then
  echo "==> Packaging .foxe"
  npm run package
  echo "==> Done. Drag the generated .foxe onto Foxglove Studio to install."
else
  echo "==> Build complete. Run with 'package' to produce an installable .foxe."
fi
