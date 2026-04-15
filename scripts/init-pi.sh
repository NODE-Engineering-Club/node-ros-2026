#!/usr/bin/env bash
set -euo pipefail

# Usage: sudo bash init-pi.sh [hostname] [username]
HOSTNAME=${1:-njord}
USERNAME=${2:-pi}

# ── System update ────────────────────────────────────────────────────────────
echo "==> Updating system packages"
apt-get update -qq
apt-get upgrade -y -qq
apt-get autoremove -y -qq

# ── Hostname ─────────────────────────────────────────────────────────────────
echo "==> Setting hostname to $HOSTNAME"
hostnamectl set-hostname "$HOSTNAME"
sed -i "s/127\.0\.1\.1.*/127.0.1.1\t$HOSTNAME/" /etc/hosts

# ── User setup ───────────────────────────────────────────────────────────────
if ! id "$USERNAME" &>/dev/null; then
  echo "==> Creating user $USERNAME"
  useradd -m -s /bin/bash "$USERNAME"
fi

# ── Docker ───────────────────────────────────────────────────────────────────
echo "==> Installing Docker"
curl -fsSL https://get.docker.com | sh
usermod -aG docker "$USERNAME"

# ── cgroups / boot config ────────────────────────────────────────────────────
CMDLINE=/boot/firmware/cmdline.txt
# Fall back to legacy path (pre-bookworm)
[[ -f $CMDLINE ]] || CMDLINE=/boot/cmdline.txt

if ! grep -q "cgroup_memory=1" "$CMDLINE"; then
  echo "==> Enabling cgroup memory in $CMDLINE"
  sed -i 's/$/ cgroup_enable=cpuset cgroup_enable=memory cgroup_memory=1/' "$CMDLINE"
fi

CONFIG=/boot/firmware/config.txt
[[ -f $CONFIG ]] || CONFIG=/boot/config.txt

if ! grep -q "^gpu_mem=" "$CONFIG"; then
  echo "==> Setting gpu_mem=16 in $CONFIG"
  echo "gpu_mem=16" >> "$CONFIG"
fi

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "Done. Reboot to apply cgroup and hostname changes."
echo "  sudo reboot"
