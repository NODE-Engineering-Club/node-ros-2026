#!/usr/bin/env bash
set -euo pipefail

# Usage: sudo bash init-pi.sh [hostname] [username]
HOSTNAME=${1:-njord}
USERNAME=${2:-pi}
REBOOT_AFTER=${REBOOT_AFTER:-false}

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

# ── Podman (for Njord container) ────────────────────────────────────────────
echo "==> Installing Podman"
apt-get install -y podman podman-compose
usermod -aG docker "$USERNAME"  # For BlueOS compatibility

# ── Docker (for BlueOS compatibility) ──────────────────────────────────────
echo "==> Installing Docker for BlueOS"
apt-get install -y docker.io
usermod -aG docker "$USERNAME"

# ── BlueOS setup ────────────────────────────────────────────────────────────
echo "==> Setting up BlueOS directories"
mkdir -p /usr/blueos/userdata
chown -R 1000:1000 /usr/blueos/userdata

# Create BlueOS service (will be started manually or via systemd)
cat > /etc/systemd/system/blueos.service << 'BLUEOS_EOF'
[Unit]
Description=BlueOS - ArduPilot Companion
After=network.target docker.service
Requires=docker.service

[Service]
Restart=unless-stopped
ExecStart=/usr/bin/docker run --rm \\
  --name blueos \\
  --privileged \\
  --network host \\
  -v /run/udev:/run/udev:ro \\
  -v /var/run/docker.sock:/var/run/docker.sock \\
  -v /usr/blueos/userdata:/usr/blueos/userdata \\
  -e BLUEOS_UID=1000 \\
  docker.io/bluerobotics/blueos-core:1.4

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
# LIDAR (RPLIDAR)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", GROUP="dialout", MODE="0666"
# Camera
SUBSYSTEM=="video4linux", GROUP="video", MODE="0666"
UDEV_EOF
usermod -aG dialout "$USERNAME"
usermod -aG video "$USERNAME"

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
