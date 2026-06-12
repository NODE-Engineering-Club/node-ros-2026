#!/usr/bin/env bash
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

# Usage: sudo bash init.sh
if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo: sudo GHCR_TOKEN=<pat> bash init-pi.sh" >&2
  exit 1
fi
GHCR_TOKEN="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"
USERNAME="${SUDO_USER:-$(logname 2>/dev/null || echo root)}"

# ── System update ────────────────────────────────────────────────────────────
echo "==> Updating system packages"
apt-get update -qq
apt-get upgrade -y -qq
apt-get autoremove -y -qq

# ── Podman ───────────────────────────────────────────────────────────────────
echo "==> Installing Podman"
apt-get install -y podman
systemctl enable --now podman.socket
usermod -aG dialout "$USERNAME"
usermod -aG video "$USERNAME"

# ── BlueOS setup ────────────────────────────────────────────────────────────
echo "==> Pulling BlueOS image"
podman pull docker.io/bluerobotics/blueos-core:1.4.3

echo "==> Setting up BlueOS"
mkdir -p /usr/blueos/userdata
chown -R 1000:1000 /usr/blueos/userdata

cat > /etc/systemd/system/blueos.service << 'BLUEOS_EOF'
[Unit]
Description=BlueOS - ArduPilot Companion
After=network.target podman.socket

[Service]
Restart=on-failure
ExecStartPre=-/usr/bin/podman rm -f blueos
ExecStart=/usr/bin/podman run --rm \
  --name blueos \
  --privileged \
  --network host \
  -v /run/udev:/run/udev:ro \
  -v /run/podman/podman.sock:/var/run/docker.sock \
  -v /usr/blueos/userdata:/usr/blueos/userdata \
  -e BLUEOS_UID=1000 \
  docker.io/bluerobotics/blueos-core:1.4.3
ExecStop=/usr/bin/podman stop blueos

[Install]
WantedBy=multi-user.target
BLUEOS_EOF

systemctl daemon-reload
systemctl enable blueos.service

# ── Udev rules for USB devices ──────────────────────────────────────────────
echo "==> Setting up udev rules for LIDAR and other USB devices"
cat > /etc/udev/rules.d/99-njord.rules << 'UDEV_EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"
SUBSYSTEM=="video4linux", GROUP="video", MODE="0666"
UDEV_EOF

# ── cgroups / boot config ────────────────────────────────────────────────────
CMDLINE=/boot/firmware/cmdline.txt
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

# ── Njord container ──────────────────────────────────────────────────────────
IMAGE="ghcr.io/node-engineering-club/node-ros-2026:latest"

echo "==> Pulling Njord image"
echo "$GHCR_TOKEN" | podman login ghcr.io -u x-access-token --password-stdin
podman pull "$IMAGE"

cat > /etc/systemd/system/njord-update.service << UPDATE_EOF
[Unit]
Description=Pull latest Njord image
Before=njord.service
After=network-online.target
Wants=network-online.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c 'echo "$GHCR_TOKEN" | /usr/bin/podman login ghcr.io -u x-access-token --password-stdin && /usr/bin/podman pull $IMAGE'

[Install]
WantedBy=multi-user.target
UPDATE_EOF

cat > /etc/systemd/system/njord.service << NJORD_EOF
[Unit]
Description=Njord ROS2 Stack
After=network.target blueos.service njord-update.service
Requires=blueos.service

[Service]
Restart=on-failure
SuccessExitStatus=143
ExecStartPre=-/usr/bin/podman rm -f njord
ExecStart=/usr/bin/podman run --rm \
  --name njord \
  --init \
  --privileged \
  --network host \
  --ipc host \
  --pid host \
  $IMAGE
ExecStop=/usr/bin/podman stop njord

[Install]
WantedBy=multi-user.target
NJORD_EOF

systemctl daemon-reload
systemctl enable njord-update.service njord.service

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "Initialization complete. Rebooting..."
reboot
