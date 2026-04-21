#!/usr/bin/env bash
set -euo pipefail

# Usage: bash scripts/deploy-pi.sh <pi-host>
PI_HOST="${1:-}"

if [[ -z "$PI_HOST" ]]; then
  echo "Usage: bash scripts/deploy-pi.sh <pi-host>" >&2
  exit 1
fi

echo "==> Deploying to $PI_HOST"
ssh "$PI_HOST" "sudo bash -s" < "$(dirname "$0")/init.sh"
