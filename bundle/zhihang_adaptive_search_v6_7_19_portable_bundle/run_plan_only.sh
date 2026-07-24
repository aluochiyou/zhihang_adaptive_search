#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
SCENE="${1:-adaptive_v6_7_19_plan}"
SEED="${2:-20260723}"
roslaunch zhihang_adaptive_search_v6 manager.launch \
  config:="${ZHIHANG_FORMAL_CONFIG}" \
  scene_id:="${SCENE}" \
  plan_only:=true \
  validation_truth_relay:=false \
  random_seed:="${SEED}"
RUN="$(find "${HOME}/zhihang_search_runs_v6" -mindepth 1 -maxdepth 1 -type d -name "${SCENE}_*" -printf '%T@ %p\n' | sort -nr | head -n1 | cut -d' ' -f2-)"
[[ -n "${RUN}" ]] && python3 - "${RUN}/search_plan.json" <<'PY'
import json,sys,math
p=json.load(open(sys.argv[1]))
print('[PLAN] model analysis:',p['model_state_analysis'])
print('[PLAN] search altitudes:',p['search_altitudes_m'])
for vid,fp in p['camera_footprint_search_per_vehicle'].items():
 print(f'[PLAN] v{vid} camera footprint:',fp)
for vid,r in p['routes'].items():
 pts=[x['point'] for x in r['waypoints']]
 d=sum(math.dist(a,b) for a,b in zip(pts,pts[1:]))
 print(f"[PLAN] v{vid} role={r['role']} route={r['route_id']} alt={r['altitude_m']:.1f}m wp={len(pts)} distance={d:.1f}m ideal@18m/s={d/18:.1f}s")
PY
