#!/usr/bin/env bash
set -e

# Source ROS 2
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source the workspace overlay if it has been built
if [ -f "/workspace/install/setup.bash" ]; then
  source /workspace/install/setup.bash
fi

# Auto-detect Tailscale interface if present and export it so the CycloneDDS
# config (which references ${A5_NETIF}) binds discovery to the VPN.
if ip link show tailscale0 >/dev/null 2>&1; then
  export A5_NETIF="tailscale0"
else
  export A5_NETIF="${A5_NETIF:-eth0}"
fi

echo "[a5] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}  NETIF=${A5_NETIF}"
if [ -n "${GITHUB_USER:-}" ]; then
  echo "[a5] GITHUB_USER=${GITHUB_USER} (use this as your ROS namespace)"
fi

exec "$@"
