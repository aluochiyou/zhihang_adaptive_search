#!/usr/bin/env bash
# Compatibility validation launcher. Formal competition use must call
# launch_portable_formal_one_click.sh.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${ROOT}/load_machine_profile.sh"
source "${ROOT}/shell_arg_utils.sh"
SCENE="$(zh_trim_arg "${1:-scene_001_adaptive_v6_7_19_validation}")"
MODEL="$(zh_expand_user_path "${2:-${ZHIHANG_YOLO_MODEL}}")"
RUNTIME_ENV="$(zh_trim_arg "${3:-${ZHIHANG_YOLO_ENV}}")"
SEED="$(zh_trim_arg "${4:-${ZHIHANG_DEFAULT_SEED}}")"
RECORD_BAG=0
[[ "$(zh_trim_arg "${5:-}")" == '--record-bag' ]] && RECORD_BAG=1
echo '[WARNING] launching validation mode, not formal competition mode.'
bash "${ROOT}/wait_base_ready.sh" 30
exec bash "${ROOT}/launch_mission_four_terminals.sh" \
  "${SCENE}" "${MODEL}" "${RUNTIME_ENV}" "${SEED}" "${RECORD_BAG}"
