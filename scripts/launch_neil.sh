#!/usr/bin/env bash
# ---------------------------------------------------------------------------
# A5 — Neil's side launcher
# Starts the bicycle sim + grader. A per-student sim instance is spawned
# automatically when Neil's node discovers /<user>/cmd on the network.
#
# Usage:  bash scripts/launch_neil.sh [--build]
# ---------------------------------------------------------------------------
set -e

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COMPOSE="docker compose -f $REPO_ROOT/docker/docker-compose.yml"

if [[ "${1:-}" == "--build" ]]; then
  echo "[neil] Building image..."
  $COMPOSE build neil
fi

echo "[neil] Starting A5 bicycle sim + grader..."
echo "[neil] Publishing: /neil/cone_map (latched), /<user>/state (per member)"
echo "[neil] Subscribes: /<user>/cmd (discovered automatically)"
echo "[neil] Feedback → /neil/feedback"
echo "[neil] Sim stats → /neil/sim_stats (JSON, per user)"
echo ""
echo "[neil] TIP: open a second terminal and run:"
echo "       bash scripts/watch_feedback.sh"
echo "       bash scripts/watch_sim_stats.sh"
echo ""

$COMPOSE run --rm neil bash -c "
  source /opt/ros/humble/setup.bash
  cd /workspace
  if [ -f install/setup.bash ]; then source install/setup.bash; fi
  colcon build --packages-select a5_neil --symlink-install --quiet
  source install/setup.bash
  echo '[neil] Launching sim_node + grader...'
  ros2 launch a5_neil neil.launch.py
"
