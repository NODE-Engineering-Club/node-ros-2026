#!/usr/bin/env bash
set -euo pipefail

PI_HOST="${1:-pi@boat.local}"

read -sp "Password for $PI_HOST: " PASS
echo

if ! command -v sshpass &>/dev/null; then
  echo "sshpass is required but not installed." >&2
  echo "  sudo apt install sshpass" >&2
  exit 1
fi

echo "==> Deploying to $PI_HOST"

SCRIPT_B64=$(base64 -w0 "$(dirname "$0")/init.sh")

sshpass -p "$PASS" ssh -o StrictHostKeyChecking=no "$PI_HOST" \
  "echo '$PASS' | sudo -S bash -c \"\$(echo $SCRIPT_B64 | base64 -d)\""
