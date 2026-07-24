#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ZHIHANG_ROS_SETUP}"
TIMEOUT="${1:-30}"
DEADLINE=$((SECONDS + TIMEOUT))
while ((SECONDS < DEADLINE)); do
  mapfile -t pids < <(pgrep -f '[p]ython3 .*model_state\.py|[p]ython .*model_state\.py' || true)
  if ((${#pids[@]} > 0)); then
    if timeout 4 rostopic echo -n 1 /gazebo/set_model_states >/dev/null 2>&1; then
      echo "[OK] model_state.py running PID=${pids[*]} and /gazebo/set_model_states is publishing"
      exit 0
    fi
  fi
  echo '[WAIT] target-motion process/topic not ready yet'
  sleep 0.5
done
echo "[ERROR] target motion not ready within ${TIMEOUT}s" >&2
exit 1
