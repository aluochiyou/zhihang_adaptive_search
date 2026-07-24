#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TIMEOUT="${1:-180}"
source "${ROOT}/source_zhihang_ros_env.sh"
DEADLINE=$((SECONDS + TIMEOUT))
required_topics=(
  /clock /gazebo/model_states
  /standard_vtol_0/mavros/state /standard_vtol_1/mavros/state /standard_vtol_2/mavros/state
  /standard_vtol_0/mavros/local_position/pose /standard_vtol_1/mavros/local_position/pose /standard_vtol_2/mavros/local_position/pose
  /standard_vtol_0/camera/image_raw /standard_vtol_1/camera/image_raw /standard_vtol_2/camera/image_raw
)
TMP="/tmp/zhihang_v6_2_topics.$$"
trap 'rm -f "${TMP}"' EXIT
while (( SECONDS < DEADLINE )); do
  if rostopic list >"${TMP}" 2>/dev/null; then
    missing=()
    for topic in "${required_topics[@]}"; do grep -qx "${topic}" "${TMP}" || missing+=("${topic}"); done
    if ((${#missing[@]} == 0)); then
      all_connected=1
      for id in 0 1 2; do
        state="$(timeout 4 rostopic echo -n 1 "/standard_vtol_${id}/mavros/state" 2>/dev/null || true)"
        grep -q 'connected: True' <<<"${state}" || all_connected=0
      done
      mapfile -t comm_pids < <(pgrep -f '[v]tol_communication\.py' || true)
      if ((all_connected)) && ((${#comm_pids[@]} >= 3)); then
        echo '[OK] base simulation, three communication processes, MAVROS, pose and camera topics are ready'
        exit 0
      fi
      echo "[WAIT] topics exist; MAVROS_connected=${all_connected} communication=${#comm_pids[@]}/3"
    else
      echo "[WAIT] missing topics: ${missing[*]}"
    fi
  else
    echo '[WAIT] ROS master not available yet'
  fi
  sleep 2
done
echo "[ERROR] base environment not ready within ${TIMEOUT}s" >&2
exit 1
