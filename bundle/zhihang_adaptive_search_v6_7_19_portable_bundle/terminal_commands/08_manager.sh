#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/source_zhihang_ros_env.sh"
SCENE="${1:-scene_001_adaptive_v6_7_19_validation}"
SEED="${2:-${ZHIHANG_DEFAULT_SEED}}"
echo "[08] V6.7.19 validation manager scene=${SCENE} seed=${SEED}"
cd "${ROOT}"
bash preflight_manager.sh
sleep 2
exec bash run_manager.sh "${SCENE}" "${SEED}"
