#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
rospack find zhihang_adaptive_search_v6
for p in $((ZHIHANG_YOLO_PORT_BASE)) $((ZHIHANG_YOLO_PORT_BASE+1)) $((ZHIHANG_YOLO_PORT_BASE+2)); do
  ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${p}$" || {
    echo "[ERROR] YOLO port ${p} not listening" >&2
    exit 2
  }
done
for id in 0 1 2; do
  rostopic type "/standard_vtol_${id}/camera/image_raw" >/dev/null || {
    echo "[ERROR] camera v${id} missing" >&2
    exit 2
  }
  rostopic type "/standard_vtol_${id}/mavros/state" >/dev/null || {
    echo "[ERROR] MAVROS v${id} missing" >&2
    exit 2
  }
done
rostopic type /gazebo/model_states >/dev/null || {
  echo '[ERROR] /gazebo/model_states missing' >&2
  exit 2
}
if rosnode list | grep -Eq 'vehicle_flight_agent_v[0-2]|vehicle_search_v[0-2]'; then
  echo '[ERROR] old flight nodes already running' >&2
  exit 2
fi
echo '[OK] V6.7.19 preflight passed. Actual >=10 Hz is enforced by manager before arming.'
