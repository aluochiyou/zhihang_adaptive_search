#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
SCENE="${1:-${ZHIHANG_DEFAULT_SCENE}}"
SEED="${2:-${ZHIHANG_DEFAULT_SEED}}"
rosparam set /zhihang_search_v6/validation/fov_proxy_enabled false || true
rosparam set /zhihang_search_v6/validation/truth_target_relay_enabled false || true
CFG="${ZHIHANG_FORMAL_CONFIG}"
[[ -f "${CFG}" ]] || {
  echo "[ERROR] formal config missing: ${CFG}" >&2
  exit 36
}
echo "[OK] formal manager package=${ZHIHANG_PKG_DIR}"
echo "[OK] formal manager config=${CFG}"
echo '[FORMAL] Gazebo target truth relay is disabled; planning uses YOLO localization only.'
exec roslaunch zhihang_adaptive_search_v6 manager.launch \
  config:="${CFG}" \
  scene_id:="${SCENE}" \
  random_seed:="${SEED}" \
  plan_only:=false \
  validation_truth_relay:=false
