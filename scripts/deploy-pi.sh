#!/usr/bin/env bash
set -euo pipefail

# Usage: GHCR_TOKEN=<pat> bash scripts/deploy-pi.sh <pi-host>
PI_HOST="${1:-}"

if [[ -z "$PI_HOST" ]]; then
  echo "Usage: GHCR_TOKEN=<pat> bash scripts/deploy-pi.sh <pi-host>" >&2
  exit 1
fi

if [[ -z "${GHCR_TOKEN:-}" ]]; then
  echo "GHCR_TOKEN is required" >&2
  exit 1
fi

echo "==> Deploying to $PI_HOST"
ssh "$PI_HOST" "sudo GHCR_TOKEN='$GHCR_TOKEN' bash -s" < "$(dirname "$0")/init.sh"
