#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/source_zhihang_ros_env.sh"
echo '[CHECK] ROS master'
rostopic list >/dev/null
echo '[CHECK] V6 ROS package'
rospack find zhihang_adaptive_search_v6
echo '[CHECK] no duplicate V6 application nodes'
OLD_RE='^/(mission_manager_v6|vehicle_flight_agent_v[0-2]|vehicle_perception_agent_v[0-2])$'
FOUND="$(rosnode list 2>/dev/null | grep -E "${OLD_RE}" || true)"
if [[ -n "${FOUND}" ]]; then
  echo '[ERROR] conflicting V6 nodes are already running:' >&2
  echo "${FOUND}" >&2
  exit 1
fi
python3 - <<'PY'
from zhihang_adaptive_search_v6.common import build_plan, validate_packet
print('[OK] V6 shared module import')
PY
echo '[OK] manager preflight passed'
