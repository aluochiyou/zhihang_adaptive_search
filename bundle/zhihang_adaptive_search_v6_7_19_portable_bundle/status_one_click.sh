#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
echo '=== V6.7.19 nodes ==='
rosnode list 2>/dev/null | grep -E 'mission_manager_v6|vehicle_(flight|perception)_agent_v|validation_target_state_relay|vision_target_state_estimator' || true
echo '=== target motion ==='
pgrep -af '[m]odel_state\.py' || echo 'model_state.py not running'
echo '=== YOLO endpoints ==='
for port in $((ZHIHANG_YOLO_PORT_BASE)) $((ZHIHANG_YOLO_PORT_BASE+1)) $((ZHIHANG_YOLO_PORT_BASE+2)); do
  ss -ltn | grep -E ":${port}\b" || true
done
echo '=== manager status ==='
timeout 5 rostopic echo -n 1 /zhihang/search_v6/manager/status 2>/dev/null || true
