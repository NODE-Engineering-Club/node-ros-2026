#!/usr/bin/env bash
set -euo pipefail

# Usage: sudo bash init-pi.sh
if [[ -z "${SUDO_USER:-}" ]]; then
  echo "Run with sudo: sudo bash init-pi.sh" >&2
  exit 1
fi
USERNAME="$SUDO_USER"
REBOOT_AFTER=${REBOOT_AFTER:-false}

# ── System update ────────────────────────────────────────────────────────────
echo "==> Updating system packages"
apt-get update -qq
apt-get upgrade -y -qq
apt-get autoremove -y -qq

# ── Podman ───────────────────────────────────────────────────────────────────
echo "==> Installing Podman"
apt-get install -y podman
usermod -aG dialout "$USERNAME"
usermod -aG video "$USERNAME"

# ── BlueOS setup ────────────────────────────────────────────────────────────
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
  docker.io/bluerobotics/blueos-core:1.4
ExecStop=/usr/bin/podman stop blueos

[Install]
WantedBy=multi-user.target
BLUEOS_EOF

systemctl daemon-reload
systemctl enable blueos.service

# ── ROS2 environment ────────────────────────────────────────────────────────
echo "==> Setting up ROS2 environment"
cat >> "/home/$USERNAME/.bashrc" << 'ROS_EOF'

# ROS2 setup
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_EOF

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

# ── Done ─────────────────────────────────────────────────────────────────────
echo ""
echo "Initialization complete!"
echo ""
echo "Next steps:"
echo "1. Reboot: sudo reboot"
echo "2. After reboot, start services:"
echo "   sudo systemctl start blueos.service"
echo "3. Pull and run Njord container manually or set up as service"

if [ "$REBOOT_AFTER" = true ]; then
  echo "Rebooting in 5 seconds..."
  sleep 5
  reboot
else
  echo "Manual reboot required to apply all changes"
fi
