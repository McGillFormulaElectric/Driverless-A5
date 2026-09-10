#!/usr/bin/env bash
set -e

# Source ROS 2
source "/opt/ros/${ROS_DISTRO}/setup.bash"

# Source the workspace overlay if it has been built
if [ -f "/workspace/install/setup.bash" ]; then
  source /workspace/install/setup.bash
fi

# ---------------------------------------------------------------------------
# Tailscale — join the MFE class tailnet automatically.
#
# TS_AUTHKEY comes from docker/.env (see docker/.env.example).
# GITHUB_USER becomes your Tailscale hostname so Neil can see who is
# connected in the admin console at tailscale.com/admin.
#
# How to get started:
#   1. Copy docker/.env.example → docker/.env
#   2. Paste the auth key Neil shared into docker/.env
#   3. docker compose run --rm new_member
# ---------------------------------------------------------------------------
if [ -n "${TS_AUTHKEY:-}" ]; then
  if command -v tailscale >/dev/null 2>&1; then
    echo "[a5] Joining MFE tailnet as '${GITHUB_USER:-a5-member}'..."
    tailscale up \
      --authkey="${TS_AUTHKEY}" \
      --hostname="${GITHUB_USER:-a5-member}" \
      --accept-routes 2>&1 || true
    echo "[a5] Tailscale: $(tailscale status --peers=false 2>&1 | head -1)"
  else
    echo "[a5] WARNING: TS_AUTHKEY set but 'tailscale' binary not found."
    echo "[a5]   Install Tailscale on your host: https://tailscale.com/download"
    echo "[a5]   Then re-run the container. With network_mode: host the host"
    echo "[a5]   tailscale daemon is used automatically."
  fi
else
  echo "[a5] TS_AUTHKEY not set — Tailscale join skipped."
  echo "[a5]   Copy docker/.env.example → docker/.env and add the key Neil sent."
fi

# ---------------------------------------------------------------------------
# Auto-detect Tailscale interface for CycloneDDS unicast peer binding.
# ---------------------------------------------------------------------------
if ip link show tailscale0 >/dev/null 2>&1; then
  export A5_NETIF="tailscale0"
else
  export A5_NETIF="${A5_NETIF:-eth0}"
fi

echo "[a5] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}  RMW=${RMW_IMPLEMENTATION}  NETIF=${A5_NETIF}"
if [ -n "${GITHUB_USER:-}" ]; then
  echo "[a5] GITHUB_USER=${GITHUB_USER}  →  your ROS namespace is /${GITHUB_USER}"
fi

exec "$@"
