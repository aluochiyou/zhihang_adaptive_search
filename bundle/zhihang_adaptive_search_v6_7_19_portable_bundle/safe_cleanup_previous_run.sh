#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
for id in 0 1 2; do
  state="$(timeout 3 rostopic echo -n 1 "/standard_vtol_${id}/mavros/state" 2>/dev/null || true)"
  if grep -q 'armed: True' <<<"${state}"; then
    echo "[ERROR] standard_vtol_${id} is armed; refusing cleanup" >&2
    exit 1
  fi
done
nodes="$(rosnode list 2>/dev/null || true)"
for node in /mission_manager_v6 \
  /vehicle_flight_agent_v0 /vehicle_perception_agent_v0 \
  /vehicle_flight_agent_v1 /vehicle_perception_agent_v1 \
  /vehicle_flight_agent_v2 /vehicle_perception_agent_v2 \
  /vision_target_state_estimator_v6 /validation_target_state_relay_v6; do
  if grep -qx "${node}" <<<"${nodes}"; then
    echo "[CLEAN] rosnode kill ${node}"
    rosnode kill "${node}" >/dev/null 2>&1 || true
  fi
done
pkill -TERM -f 'zhihang_adaptive_search_v6/.*/yolo26_single_worker.py' 2>/dev/null || true
pkill -TERM -f 'zhihang_adaptive_search_v6/scripts/yolo26_single_worker.py' 2>/dev/null || true
sleep 2
for port in $((ZHIHANG_YOLO_PORT_BASE)) $((ZHIHANG_YOLO_PORT_BASE+1)) $((ZHIHANG_YOLO_PORT_BASE+2)); do
  if ss -ltn | awk '{print $4}' | grep -Eq "(^|:)${port}$"; then
    echo "[ERROR] port ${port} remains occupied; inspect: ss -ltnp | grep :${port}" >&2
    exit 1
  fi
done
echo '[OK] no stale V6 manager, agent or YOLO worker remains'
