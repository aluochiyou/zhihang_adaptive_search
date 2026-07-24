#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
SCENE="${1:-scene_001_adaptive_v6_7_19_validation}"
SEED="${2:-${ZHIHANG_DEFAULT_SEED}}"
echo "[MANAGER-VALIDATION] scene=${SCENE} seed=${SEED}"
echo "[MANAGER-VALIDATION] package=${ZHIHANG_PKG_DIR}"
echo '[WARNING] validation mode may use the independent truth relay for state-machine testing.'
exec roslaunch zhihang_adaptive_search_v6 manager.launch \
  config:="${ZHIHANG_VALIDATION_CONFIG}" \
  scene_id:="${SCENE}" \
  random_seed:="${SEED}" \
  plan_only:=false \
  validation_truth_relay:=true
